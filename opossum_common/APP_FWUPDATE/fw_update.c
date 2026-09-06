#include "fw_update.h"
#include "qspi_flash.h"
#include "../IO_config.h"                    /* UartComm_Ctx, UART_PS_GetByte */
#include "../APP_COM/interpreteur.h"         /* Get_Param_u32                 */
#include "../TIMER_MANAGER/timer_manager.h"  /* Timer_ms1                     */
#include "xil_printf.h"
#include "xstatus.h"

/* Taille de bloc annoncee au host. La reception vide le ring buffer RX en
 * continu (bien plus vite que 115200 bauds), donc l'occupation reste ~0 et
 * ce bloc peut depasser la taille du ring buffer sans risque d'overflow. */
#define FW_CHUNK            2048U
#define FW_MAX_IMAGE_SIZE   0x1000000U   /* 16 Mo (limite adressage 3 octets) */
#define FW_RX_TIMEOUT_MS    3000         /* abandon si plus rien ne vient      */

static u8 s_block[FW_CHUNK];

/* CRC32 "standard" (identique a zlib.crc32 / cote host python) : polynome
 * reflechi 0xEDB88820, init 0xFFFFFFFF, XOR final 0xFFFFFFFF. Accumulation
 * possible en plusieurs appels : garder le crc "brut" entre les appels, ne
 * l'inverser (^0xFFFFFFFF) qu'a la toute fin. */
static u32 crc32_step(u32 crc, const u8 *data, u32 len) {
    for (u32 i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88820U & (0U - (crc & 1U)));
        }
    }
    return crc;
}

/* Recoit exactement 'want' octets dans s_block, avec timeout inter-octet.
 * Retourne 1 si OK, 0 si timeout. */
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
        xil_printf("-ERR params\n");
        return 1;
    }
    if (size == 0U || size > FW_MAX_IMAGE_SIZE) {
        xil_printf("-ERR size\n");
        return 1;
    }

    /* 2. Init QSPI + effacement de la zone (lent : fait AVANT +READY) */
    if (QspiFlash_Init() != XST_SUCCESS) {
        xil_printf("-ERR qspi_init\n");
        return 1;
    }
    if (QspiFlash_EraseRange(0, size) != XST_SUCCESS) {
        xil_printf("-ERR erase\n");
        return 1;
    }

    /* 3. Purge toute reception residuelle (ex: '\n' de fin de ligne) : le
     *    host n'envoie le binaire qu'apres +READY, donc rien d'utile n'est
     *    jete ici. */
    { u8 c; while (UART_PS_GetByte(&UartComm_Ctx, &c)) { } }

    /* xil_printf ecrit en polled directement sur la console (UART_COMM), donc
     * les reponses partent immediatement meme si la boucle principale (qui
     * vide le ring buffer TX) est bloquee dans cette commande. */
    xil_printf("+READY %u\n", (unsigned)FW_CHUNK);

    /* 4. Reception + ecriture bloc par bloc, avec ACK (flow-control) */
    u32 total = 0;
    u32 crc   = 0xFFFFFFFFU;
    while (total < size) {
        u32 blk = (size - total < FW_CHUNK) ? (size - total) : FW_CHUNK;

        if (!fw_recv_block(blk)) {
            xil_printf("-ERR timeout\n");
            return 1;
        }
        if (QspiFlash_Write(total, s_block, blk) != XST_SUCCESS) {
            xil_printf("-ERR write\n");
            return 1;
        }
        crc = crc32_step(crc, s_block, blk);
        total += blk;
        xil_printf("+ACK %u\n", (unsigned)total);
    }
    crc ^= 0xFFFFFFFFU;

    /* 5a. CRC des donnees recues (detecte une corruption de transfert) */
    if (crc != expected_crc) {
        xil_printf("-ERR crc_rx r=%08x a=%08x\n", (unsigned)crc, (unsigned)expected_crc);
        return 1;
    }

    /* 5b. CRC par RELECTURE de la flash (detecte une ecriture QSPI ratee) */
    u32 rcrc = 0xFFFFFFFFU;
    u32 off  = 0;
    while (off < size) {
        u32 n = (size - off < FW_CHUNK) ? (size - off) : FW_CHUNK;
        if (QspiFlash_Read(off, s_block, n) != XST_SUCCESS) {
            xil_printf("-ERR readback\n");
            return 1;
        }
        rcrc = crc32_step(rcrc, s_block, n);
        off += n;
    }
    rcrc ^= 0xFFFFFFFFU;
    if (rcrc != expected_crc) {
        xil_printf("-ERR crc_flash r=%08x a=%08x\n", (unsigned)rcrc, (unsigned)expected_crc);
        return 1;
    }

    /* 6. Succes : le host peut envoyer REBOOT pour booter la nouvelle image
     *    (a condition que la carte soit strappee QSPI). */
    xil_printf("+DONE\n");
    return 0;
}
