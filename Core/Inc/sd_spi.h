#ifndef SD_SPI_H
#define SD_SPI_H

#include "diskio.h"

DSTATUS SD_SPI_Initialize(void);
DSTATUS SD_SPI_Status(void);

DRESULT SD_SPI_Read(BYTE *buffer,DWORD sector,UINT count);
DRESULT SD_SPI_Write(const BYTE *buffer,DWORD sector,UINT count);
DRESULT SD_SPI_Ioctl(BYTE command,void *buffer);

#endif
