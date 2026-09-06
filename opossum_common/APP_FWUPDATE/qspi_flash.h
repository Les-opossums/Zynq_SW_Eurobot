#ifndef QSPI_FLASH_H
#define QSPI_FLASH_H

#include "xil_types.h"

/*
 * Acces bas niveau a la flash QSPI (XQspiPs, single-lane, adressage 3 octets).
 * Commandes NOR generiques compatibles Winbond / Micron / Spansion. Suffisant
 * pour reflasher un BOOT.bin a l'offset 0 (cf APP_FWUPDATE/fw_update.c).
 *
 * Adressage 3 octets => couvre les 16 premiers Mo de la flash, ce qui suffit
 * largement pour un BOOT.bin (~2-3 Mo). Pour un boot au-dela de 16 Mo il
 * faudrait passer en adressage 4 octets (commandes 0x13/0x12/0xDC + EN4B).
 */

#define QSPI_PAGE_SIZE    256U       /* taille max d'un PAGE PROGRAM (0x02)   */
#define QSPI_SECTOR_SIZE  0x10000U   /* taille d'un SECTOR ERASE (0xD8) = 64K */

int QspiFlash_Init(void);
int QspiFlash_EraseRange(u32 addr, u32 len);          /* efface les secteurs 64K couvrant [addr, addr+len) */
int QspiFlash_Write(u32 addr, const u8 *src, u32 len);/* ecrit len octets (decoupage en pages gere)          */
int QspiFlash_Read(u32 addr, u8 *dst, u32 len);       /* relit len octets                                    */

#endif /* QSPI_FLASH_H */
