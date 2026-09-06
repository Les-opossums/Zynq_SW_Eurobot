#include "qspi_flash.h"
#include "xqspips.h"
#include "xparameters.h"

/* --- Jeu de commandes NOR single-lane (adressage 3 octets) --------------- */
#define CMD_WRITE_ENABLE  0x06U
#define CMD_READ_STATUS   0x05U
#define CMD_SECTOR_ERASE  0xD8U   /* efface 64 KiB              */
#define CMD_PAGE_PROGRAM  0x02U   /* programme jusqu'a 256 o    */
#define CMD_READ_DATA     0x03U   /* lecture normale (robuste)  */

#define SR_WIP_MASK       0x01U   /* Write In Progress (busy)   */

static XQspiPs Qspi;

/* Tampons de travail : 4 octets (commande + adresse) + une page de donnees */
static u8 WrBuf[4U + QSPI_PAGE_SIZE];
static u8 RdBuf[4U + QSPI_PAGE_SIZE];

/* --- Helpers internes ---------------------------------------------------- */
static void FlashWriteEnable(void) {
    u8 cmd = CMD_WRITE_ENABLE;
    XQspiPs_PolledTransfer(&Qspi, &cmd, NULL, 1);
}

/* Attend la fin d'une ecriture/effacement (polling du bit WIP du status reg) */
static void FlashWaitReady(void) {
    u8 tx[2] = { CMD_READ_STATUS, 0U };
    u8 rx[2] = { 0U, 0U };
    do {
        XQspiPs_PolledTransfer(&Qspi, tx, rx, 2);
    } while (rx[1] & SR_WIP_MASK);
}

/* --- API ----------------------------------------------------------------- */
int QspiFlash_Init(void) {
    XQspiPs_Config *cfg = XQspiPs_LookupConfig(XPAR_XQSPIPS_0_DEVICE_ID);
    if (cfg == NULL) {
        return XST_FAILURE;
    }
    if (XQspiPs_CfgInitialize(&Qspi, cfg, cfg->BaseAddress) != XST_SUCCESS) {
        return XST_FAILURE;
    }
    /* Mode manuel + CS force + Hold : configuration classique flasheur QSPI */
    XQspiPs_SetOptions(&Qspi, XQSPIPS_MANUAL_START_OPTION |
                              XQSPIPS_FORCE_SSELECT_OPTION |
                              XQSPIPS_HOLD_B_DRIVE_OPTION);
    XQspiPs_SetClkPrescaler(&Qspi, XQSPIPS_CLK_PRESCALE_8); /* 200 MHz / 8 = 25 MHz */
    XQspiPs_SetSlaveSelect(&Qspi);
    return XST_SUCCESS;
}

int QspiFlash_EraseRange(u32 addr, u32 len) {
    u32 start = addr & ~(QSPI_SECTOR_SIZE - 1U);
    u32 end   = addr + len;
    for (u32 a = start; a < end; a += QSPI_SECTOR_SIZE) {
        FlashWriteEnable();
        WrBuf[0] = CMD_SECTOR_ERASE;
        WrBuf[1] = (u8)((a >> 16) & 0xFFU);
        WrBuf[2] = (u8)((a >> 8)  & 0xFFU);
        WrBuf[3] = (u8)( a        & 0xFFU);
        if (XQspiPs_PolledTransfer(&Qspi, WrBuf, NULL, 4) != XST_SUCCESS) {
            return XST_FAILURE;
        }
        FlashWaitReady();
    }
    return XST_SUCCESS;
}

int QspiFlash_Write(u32 addr, const u8 *src, u32 len) {
    u32 done = 0;
    while (done < len) {
        u32 a        = addr + done;
        u32 page_off = a & (QSPI_PAGE_SIZE - 1U);
        u32 n        = QSPI_PAGE_SIZE - page_off;      /* ne franchit pas une frontiere de page */
        if (n > (len - done)) {
            n = len - done;
        }
        FlashWriteEnable();
        WrBuf[0] = CMD_PAGE_PROGRAM;
        WrBuf[1] = (u8)((a >> 16) & 0xFFU);
        WrBuf[2] = (u8)((a >> 8)  & 0xFFU);
        WrBuf[3] = (u8)( a        & 0xFFU);
        for (u32 i = 0; i < n; i++) {
            WrBuf[4 + i] = src[done + i];
        }
        if (XQspiPs_PolledTransfer(&Qspi, WrBuf, NULL, 4 + n) != XST_SUCCESS) {
            return XST_FAILURE;
        }
        FlashWaitReady();
        done += n;
    }
    return XST_SUCCESS;
}

int QspiFlash_Read(u32 addr, u8 *dst, u32 len) {
    u32 done = 0;
    while (done < len) {
        u32 a = addr + done;
        u32 n = QSPI_PAGE_SIZE;
        if (n > (len - done)) {
            n = len - done;
        }
        WrBuf[0] = CMD_READ_DATA;
        WrBuf[1] = (u8)((a >> 16) & 0xFFU);
        WrBuf[2] = (u8)((a >> 8)  & 0xFFU);
        WrBuf[3] = (u8)( a        & 0xFFU);
        /* Les donnees lues arrivent apres les 4 octets commande+adresse */
        if (XQspiPs_PolledTransfer(&Qspi, WrBuf, RdBuf, 4 + n) != XST_SUCCESS) {
            return XST_FAILURE;
        }
        for (u32 i = 0; i < n; i++) {
            dst[done + i] = RdBuf[4 + i];
        }
        done += n;
    }
    return XST_SUCCESS;
}
