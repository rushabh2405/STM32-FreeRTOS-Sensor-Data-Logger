#include "sd_spi.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi2;

static volatile DSTATUS sdStatus = STA_NOINIT;
static uint8_t sdCardType = 0;

/* Card type flags */
#define CT_MMC      0x01U
#define CT_SD1      0x02U
#define CT_SD2      0x04U
#define CT_BLOCK    0x08U

/* SD commands */
#define CMD0        0U
#define CMD1        1U
#define CMD8        8U
#define CMD9        9U
#define CMD12       12U
#define CMD16       16U
#define CMD17       17U
#define CMD18       18U
#define CMD24       24U
#define CMD25       25U
#define CMD55       55U
#define CMD58       58U
#define ACMD41      (0x80U + 41U)

static uint8_t SD_SPI_Transfer(uint8_t data);
static void SD_SPI_Select(void);
static void SD_SPI_Deselect(void);
static uint8_t SD_SPI_WaitReady(uint32_t timeout);
static uint8_t SD_SPI_SendCommand(uint8_t command, uint32_t argument);

static uint8_t SD_SPI_ReceiveDataBlock(uint8_t *buffer,uint32_t length);

#if _USE_WRITE == 1
static uint8_t SD_SPI_TransmitDataBlock(const uint8_t *buffer,uint8_t token);
#endif

static uint8_t SD_SPI_Transfer(uint8_t data)
{
    uint8_t received = 0xFF;

    if (HAL_SPI_TransmitReceive(&hspi2,&data,&received,1,100) != HAL_OK)
    {
        return 0xFF;
    }

    return received;
}

static void SD_SPI_Select(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port,SD_CS_Pin,GPIO_PIN_RESET);
}

static void SD_SPI_Deselect(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port,SD_CS_Pin,GPIO_PIN_SET);

    /* Extra clock after releasing the card */
    SD_SPI_Transfer(0xFF);
}

static uint8_t SD_SPI_WaitReady(uint32_t timeout)
{
    uint32_t startTime = HAL_GetTick();

    do
    {
        if (SD_SPI_Transfer(0xFF) == 0xFF)
        {
            return 1;
        }
    }
    while ((HAL_GetTick() - startTime) < timeout);

    return 0;
}

static uint8_t SD_SPI_SendCommand(uint8_t command, uint32_t argument)
{
    uint8_t response;
    uint8_t crc = 0x01;

    /* ACMD commands require CMD55 first */
    if ((command & 0x80U) != 0U)
    {
        command &= 0x7FU;

        response = SD_SPI_SendCommand(CMD55, 0);

        if (response > 1U)
        {
            return response;
        }
    }

    SD_SPI_Deselect();
    SD_SPI_Select();

    if (SD_SPI_WaitReady(500) == 0U)
    {
        SD_SPI_Deselect();
        return 0xFF;
    }

    /* Send command byte */
    SD_SPI_Transfer(0x40U | command);

    /* Send 32-bit argument, most significant byte first */
    SD_SPI_Transfer((uint8_t)(argument >> 24));
    SD_SPI_Transfer((uint8_t)(argument >> 16));
    SD_SPI_Transfer((uint8_t)(argument >> 8));
    SD_SPI_Transfer((uint8_t)argument);

    /* Correct CRC is required during card initialization */
    if (command == CMD0)
    {
        crc = 0x95;
    }
    else if (command == CMD8)
    {
        crc = 0x87;
    }

    SD_SPI_Transfer(crc);

    if (command == CMD12)
    {
        SD_SPI_Transfer(0xFF);
    }

    /* Wait for the card response */
    for (uint8_t attempt = 0; attempt < 10U; attempt++)
    {
        response = SD_SPI_Transfer(0xFF);

        if ((response & 0x80U) == 0U)
        {
            return response;
        }
    }

    return 0xFF;
}

DSTATUS SD_SPI_Initialize(void)
{
    uint8_t response;
    uint8_t cardType = 0;
    uint8_t ocr[4];
    uint32_t startTime;
    char debugMessage[50];

    sdStatus = STA_NOINIT;
    sdCardType = 0;

    SD_SPI_Deselect();

    /* Send at least 80 clock pulses while CS is HIGH */
    for (uint8_t i = 0; i < 10U; i++)
    {
        SD_SPI_Transfer(0xFF);
    }

    /* Retry CMD0 until the card enters SPI idle mode */
    response = 0xFF;
    startTime = HAL_GetTick();

    do
    {
        response = SD_SPI_SendCommand(CMD0, 0U);
    }
    while ((response != 0x01U) &&
           ((HAL_GetTick() - startTime) < 1000U));

    snprintf(debugMessage, sizeof(debugMessage),"CMD0 response: 0x%02X\r\n", response);

    HAL_UART_Transmit(&huart2, (uint8_t *)debugMessage,strlen(debugMessage), 1000);

    if (response != 0x01U)
    {
        SD_SPI_Deselect();
        return STA_NOINIT;
    }

    /* Check whether this is an SD version 2 card */
    response = SD_SPI_SendCommand(CMD8, 0x1AAU);

    if (response == 0x01U)
    {
        for (uint8_t i = 0; i < 4U; i++)
        {
            ocr[i] = SD_SPI_Transfer(0xFF);
        }

        if ((ocr[2] == 0x01U) && (ocr[3] == 0xAAU))
        {
            startTime = HAL_GetTick();

            do
            {
                response = SD_SPI_SendCommand(ACMD41, 0x40000000U);
            }
            while ((response != 0x00U) &&
                   ((HAL_GetTick() - startTime) < 2000U));

            if ((response == 0x00U) &&
                (SD_SPI_SendCommand(CMD58, 0U) == 0x00U))
            {
                for (uint8_t i = 0; i < 4U; i++)
                {
                    ocr[i] = SD_SPI_Transfer(0xFF);
                }

                cardType = CT_SD2;

                if ((ocr[0] & 0x40U) != 0U)
                {
                    cardType |= CT_BLOCK;
                }
            }
        }
    }
    else
    {
        /* Older SD version 1 or MMC card */
        if (SD_SPI_SendCommand(ACMD41, 0U) <= 0x01U)
        {
            cardType = CT_SD1;
        }
        else
        {
            cardType = CT_MMC;
        }

        startTime = HAL_GetTick();

        do
        {
            if (cardType == CT_SD1)
            {
                response = SD_SPI_SendCommand(ACMD41, 0U);
            }
            else
            {
                response = SD_SPI_SendCommand(CMD1, 0U);
            }
        }
        while ((response != 0x00U) &&
               ((HAL_GetTick() - startTime) < 2000U));

        if ((response != 0x00U) ||
            (SD_SPI_SendCommand(CMD16, 512U) != 0x00U))
        {
            cardType = 0;
        }
    }

    sdCardType = cardType;
    SD_SPI_Deselect();

    if (cardType != 0U)
    {
        sdStatus &= (DSTATUS)~STA_NOINIT;
    }
    else
    {
        sdStatus = STA_NOINIT;
    }

    return sdStatus;
}

DSTATUS SD_SPI_Status(void)
{
    return sdStatus;
}

static uint8_t SD_SPI_ReceiveDataBlock(
    uint8_t *buffer,
    uint32_t length)
{
    uint8_t token;
    uint32_t startTime = HAL_GetTick();

    /* Wait for start token 0xFE */
    do
    {
        token = SD_SPI_Transfer(0xFF);
    }
    while ((token == 0xFF) && ((HAL_GetTick() - startTime) < 200));

    if (token != 0xFE)
    {
        return 0;
    }

    for (uint32_t i = 0; i < length; i++)
    {
        buffer[i] = SD_SPI_Transfer(0xFF);
    }

    /* Ignore two CRC bytes */
    SD_SPI_Transfer(0xFF);
    SD_SPI_Transfer(0xFF);

    return 1;
}

#if _USE_WRITE == 1

static uint8_t SD_SPI_TransmitDataBlock(
    const uint8_t *buffer,
    uint8_t token)
{
    uint8_t response;

    if (SD_SPI_WaitReady(500) == 0)
    {
        return 0;
    }

    /* Send start token */
    SD_SPI_Transfer(token);

    /* 0xFD is only the multi-block stop token */
    if (token != 0xFD)
    {
        for (uint32_t i = 0; i < 512; i++)
        {
            SD_SPI_Transfer(buffer[i]);
        }

        /* Dummy CRC */
        SD_SPI_Transfer(0xFF);
        SD_SPI_Transfer(0xFF);

        response = SD_SPI_Transfer(0xFF);

        /* 0x05 means data accepted */
        if ((response & 0x1F) != 0x05)
        {
            return 0;
        }

        if (SD_SPI_WaitReady(500) == 0)
        {
            return 0;
        }
    }

    return 1;
}

#endif

DRESULT SD_SPI_Read(BYTE *buffer, DWORD sector, UINT count)
{
    uint32_t address;

    if ((buffer == NULL) || (count == 0U))
    {
        return RES_PARERR;
    }

    if ((sdStatus & STA_NOINIT) != 0U)
    {
        return RES_NOTRDY;
    }

    address = sector;

    /* Older cards use byte addressing */
    if ((sdCardType & CT_BLOCK) == 0U)
    {
        address *= 512U;
    }

    while (count > 0U)
    {
        if ((SD_SPI_SendCommand(CMD17, address) != 0x00U) || (SD_SPI_ReceiveDataBlock(buffer, 512U) == 0U))
        {
            SD_SPI_Deselect();
            return RES_ERROR;
        }

        SD_SPI_Deselect();

        buffer += 512U;
        address += ((sdCardType & CT_BLOCK) != 0U) ? 1U : 512U;
        count--;
    }

    return RES_OK;
}

#if _USE_WRITE == 1
DRESULT SD_SPI_Write(const BYTE *buffer, DWORD sector, UINT count)
{
    uint32_t address;

    if ((buffer == NULL) || (count == 0U))
    {
        return RES_PARERR;
    }

    if ((sdStatus & STA_NOINIT) != 0U)
    {
        return RES_NOTRDY;
    }

    address = sector;

    if ((sdCardType & CT_BLOCK) == 0U)
    {
        address *= 512U;
    }

    while (count > 0U)
    {
        if ((SD_SPI_SendCommand(CMD24, address) != 0x00U) || (SD_SPI_TransmitDataBlock(buffer, 0xFEU) == 0U))
        {
            SD_SPI_Deselect();
            return RES_ERROR;
        }

        SD_SPI_Deselect();

        buffer += 512U;
        address += ((sdCardType & CT_BLOCK) != 0U) ? 1U : 512U;
        count--;
    }

    return RES_OK;
}
#endif

#if _USE_IOCTL == 1

DRESULT SD_SPI_Ioctl(BYTE command, void *buffer)
{
    DRESULT result = RES_ERROR;
    uint8_t csd[16];

    if ((sdStatus & STA_NOINIT) != 0U)
    {
        return RES_NOTRDY;
    }

    if ((buffer == NULL) && (command != CTRL_SYNC))
    {
        return RES_PARERR;
    }

    switch (command)
    {
        case CTRL_SYNC:
        {
            SD_SPI_Select();

            if (SD_SPI_WaitReady(500U) != 0U)
            {
                result = RES_OK;
            }

            SD_SPI_Deselect();
            break;
        }

        case GET_SECTOR_SIZE:
        {
            *(WORD *)buffer = 512U;
            result = RES_OK;
            break;
        }

        case GET_SECTOR_COUNT:
        {
            if ((SD_SPI_SendCommand(CMD9, 0U) == 0x00U) && (SD_SPI_ReceiveDataBlock(csd, 16U) != 0U))
            {
                uint32_t sectorCount;

                /* CSD version 2: SDHC or SDXC */
                if ((csd[0] & 0xC0U) == 0x40U)
                {
                    uint32_t cardSize =
                        ((uint32_t)(csd[7] & 0x3FU) << 16) |
                        ((uint32_t)csd[8] << 8) |
                        csd[9];

                    sectorCount = (cardSize + 1U) << 10;
                }
                else
                {
                    /* CSD version 1: SDSC or MMC */
                    uint32_t cardSize =
                        ((uint32_t)(csd[6] & 0x03U) << 10) |
                        ((uint32_t)csd[7] << 2) |
                        ((csd[8] & 0xC0U) >> 6);

                    uint8_t readBlockLength = csd[5] & 0x0FU;

                    uint8_t sizeMultiplier =
                        ((csd[9] & 0x03U) << 1) |
                        ((csd[10] & 0x80U) >> 7);

                    uint8_t shift =
                        readBlockLength + sizeMultiplier + 2U;

                    cardSize++;

                    if (shift >= 9U)
                    {
                        sectorCount = cardSize << (shift - 9U);
                    }
                    else
                    {
                        sectorCount = cardSize >> (9U - shift);
                    }
                }

                *(DWORD *)buffer = sectorCount;
                result = RES_OK;
            }

            SD_SPI_Deselect();
            break;
        }

        case GET_BLOCK_SIZE:
        {
            /* Safe default: one 512-byte sector */
            *(DWORD *)buffer = 1U;
            result = RES_OK;
            break;
        }

        default:
        {
            result = RES_PARERR;
            break;
        }
    }

    return result;
}

#endif
