#include "fw_update.h"
#include "qspi_flash.h"
#include "../IO_config.h"                    /* UartComm_Ctx, UART_PS_*        */
#include "../APP_COM/interpreteur.h"         /* Get_Param_u32                 */
#include "../TIMER_MANAGER/timer_manager.h"  /* Timer_ms1                     */
#include "xstatus.h"
#include <string.h>

/* Taille de bloc annoncee au host. La reception vide le ring buffer RX en
 * continu (ISR RX + drain serre), donc l'occupation reste faible meme si ce
 * bloc depasse la taille du ring buffer. */
#define FW_CHUNK            8192U
#define FW_MAX_IMAGE_SIZE   0x1000000U   /* 16 Mo (limite adressage 3 octets) */
#define FW_RX_TIMEOUT_MS    3000         /* abandon si plus rien ne vient      */

static u8 s_block[FW_CHUNK];

/* -------------------------------------------------------------------------
 * TX pendant la commande : ATTENTION, xil_printf/outbyte de ce projet
 * n'ecrit PAS en polled -- il empile dans le ring buffer TX, vide seulement
 * par UART_PS_Update() DANS LA BOUCLE PRINCIPALE. Or ici la boucle est
 * bloquee dans la commande. On empile donc dans le ring PUIS on vide nous-
 * memes (on joue le role de la boucle principale). Sinon les reponses
 * (+READY/+ACK/+DONE) ne partiraient qu'apres la fin de la commande.
 * ------------------------------------------------------------------------- */
static void fw_flush(void) {
    while (UART_PS_TxFreeSpace(&UartComm_Ctx) != (UART_PS_RING_BUFFER_SIZE - 1)) {
        UART_PS_Update(&UartComm_Ctx);
    }
}

static void fw_send(const char *s) {
    UART_PS_SendBuffer(&UartComm_Ctx, (const u8 *)s, (u16)strlen(s));
    fw_flush();
}

/* Envoie "<prefix><valeur decimale>\n" (ex: "+ACK 4096\n") */
static void fw_send_num(const char *prefix, u32 v) {
    UART_PS_SendBuffer(&UartComm_Ctx, (const u8 *)prefix, (u16)strlen(prefix));
    char b[12];
    int i = 0;
    if (v == 0) {
        UART_PS_SendByte(&UartComm_Ctx, '0');
    } else {
        while (v) { b[i++] = (char)('0' + (v % 10)); v /= 10; }
        while (i) { UART_PS_SendByte(&UartComm_Ctx, (u8)b[--i]); }
    }
    UART_PS_SendByte(&UartComm_Ctx, '\n');
    fw_flush();
}

/* CRC32 "standard" (identique a zlib.crc32 / cote host python) : polynome
 * reflechi 0xEDB88320, init 0xFFFFFFFF, XOR final 0xFFFFFFFF. Accumulation
 * multi-appels : garder le crc "brut" entre les appels, inverser a la fin. */
static u32 crc32_step(u32 crc, const u8 *data, u32 len) {
    for (u32 i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return crc;
}

/* Recoit exactement 'want' octets dans s_block, avec timeout inter-octet.
 * La boucle principale etant bloquee, la reception repose sur l'ISR RX qui
 * remplit le ring buffer ; on le vide ici en boucle serree. */
static int fw_recv_block(u32 want) {
    u32 got = 0;
    int last_ms = Timer_ms1;
    while (got < want) {
        u8 c;
        if (UART_PS_GetByte(&UartComm_Ctx, &c)) {
            s_block[got++] = c;
            last_ms = Timer_ms1;
        } else if ((Timer_ms1 - last_ms) > FW_RX_TIMEOUT_MS) {
            return 0;
        }
    }
    return 1;
}

uint8_t FW_Update_Cmd(void) {
    u32 size = 0, expected_crc = 0;

    /* 1. Parametres : taille (decimal) + crc32 (decimal, non signe) */
    if (Get_Param_u32(&size) != 0 || Get_Param_u32(&expected_crc) != 0) {
        fw_send("-ERR params\n");
        return 1;
    }
    if (size == 0U || size > FW_MAX_IMAGE_SIZE) {
        fw_send("-ERR size\n");
        return 1;
    }

    /* 2. Init QSPI + effacement de la zone (lent : fait AVANT +READY) */
    if (QspiFlash_Init() != XST_SUCCESS) {
        fw_send("-ERR qspi_init\n");
        return 1;
    }
    if (QspiFlash_EraseRange(0, size) != XST_SUCCESS) {
        fw_send("-ERR erase\n");
        return 1;
    }

    /* 3. Purge toute reception residuelle (le host n'envoie le binaire
     *    qu'apres +READY, donc rien d'utile n'est jete ici). */
    { u8 c; while (UART_PS_GetByte(&UartComm_Ctx, &c)) { } }

    fw_send_num("+READY ", FW_CHUNK);

    /* 4. Reception + ecriture bloc par bloc, avec ACK (flow-control) */
    u32 total = 0;
    u32 crc   = 0xFFFFFFFFU;
    while (total < size) {
        u32 blk = (size - total < FW_CHUNK) ? (size - total) : FW_CHUNK;

        if (!fw_recv_block(blk)) {
            fw_send("-ERR timeout\n");
            return 1;
        }
        if (QspiFlash_Write(total, s_block, blk) != XST_SUCCESS) {
            fw_send("-ERR write\n");
            return 1;
        }
        crc = crc32_step(crc, s_block, blk);
        total += blk;
        fw_send_num("+ACK ", total);
    }
    crc ^= 0xFFFFFFFFU;

    /* 5a. CRC des donnees recues (detecte une corruption de transfert) */
    if (crc != expected_crc) {
        fw_send("-ERR crc_rx\n");
        return 1;
    }

    /* 5b. CRC par RELECTURE de la flash (detecte une ecriture QSPI ratee) */
    u32 rcrc = 0xFFFFFFFFU;
    u32 off  = 0;
    while (off < size) {
        u32 n = (size - off < FW_CHUNK) ? (size - off) : FW_CHUNK;
        if (QspiFlash_Read(off, s_block, n) != XST_SUCCESS) {
            fw_send("-ERR readback\n");
            return 1;
        }
        rcrc = crc32_step(rcrc, s_block, n);
        off += n;
    }
    rcrc ^= 0xFFFFFFFFU;
    if (rcrc != expected_crc) {
        fw_send("-ERR crc_flash\n");
        return 1;
    }

    /* 6. Succes : le host peut envoyer REBOOT pour booter la nouvelle image
     *    (a condition que la carte soit strappee QSPI). */
    fw_send("+DONE\n");
    return 0;
}
