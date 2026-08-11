#include "bme280_app.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

static struct bme280_dev bme280Dev;
static uint8_t bme280Address = 0x77U;

static BME280_INTF_RET_TYPE BME280_I2C_Read(uint8_t regAddr,uint8_t *data,uint32_t length,void *intfPtr);
static BME280_INTF_RET_TYPE BME280_I2C_Write(uint8_t regAddr,const uint8_t *data,uint32_t length,void *intfPtr);

static void BME280_DelayUs(uint32_t period, void *intfPtr);


static BME280_INTF_RET_TYPE BME280_I2C_Read(uint8_t regAddr,uint8_t *data,uint32_t length,void *intfPtr)
{
    uint8_t address = *(uint8_t *)intfPtr;

    if (HAL_I2C_Mem_Read(&hi2c1,address << 1,regAddr,I2C_MEMADD_SIZE_8BIT,data,(uint16_t)length,100) == HAL_OK)
    {
        return BME280_OK;
    }

    return BME280_E_COMM_FAIL;
}


static BME280_INTF_RET_TYPE BME280_I2C_Write(uint8_t regAddr,const uint8_t *data,uint32_t length,void *intfPtr)
{
    uint8_t address = *(uint8_t *)intfPtr;

    if (HAL_I2C_Mem_Write(&hi2c1,address << 1,regAddr,I2C_MEMADD_SIZE_8BIT,(uint8_t *)data,(uint16_t)length,100) == HAL_OK)
    {
        return BME280_OK;
    }

    return BME280_E_COMM_FAIL;
}


static void BME280_DelayUs(uint32_t period, void *intfPtr)
{
    (void)intfPtr;

    if (period < 1000U)
    {
        period = 1000U;
    }

    HAL_Delay((period + 999U) / 1000U);
}


int8_t BME280_App_Init(void)
{
    int8_t result;
    struct bme280_settings settings = {0};

    /* Confirm BME280 is responding at 0x77 */
    if (HAL_I2C_IsDeviceReady(&hi2c1,0x77U << 1,3,100) != HAL_OK)
    {
        return BME280_E_COMM_FAIL;
    }

    bme280Address = 0x77U;

    bme280Dev.intf = BME280_I2C_INTF;
    bme280Dev.read = BME280_I2C_Read;
    bme280Dev.write = BME280_I2C_Write;
    bme280Dev.delay_us = BME280_DelayUs;
    bme280Dev.intf_ptr = &bme280Address;

    result = bme280_init(&bme280Dev);

    if (result != BME280_OK)
    {
        return result;
    }

    settings.filter = BME280_FILTER_COEFF_2;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.standby_time = BME280_STANDBY_TIME_1000_MS;

    result = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS,&settings,&bme280Dev);

    if (result != BME280_OK)
    {
        return result;
    }

    return bme280_set_sensor_mode(BME280_POWERMODE_NORMAL,&bme280Dev);
}


int8_t BME280_App_Read(struct bme280_data *sensorData)
{
    if (sensorData == NULL)
    {
        return BME280_E_NULL_PTR;
    }

    return bme280_get_sensor_data(BME280_ALL,sensorData,&bme280Dev);
}
