#ifndef BME280_APP_H
#define BME280_APP_H

#include "bme280.h"

int8_t BME280_App_Init(void);
int8_t BME280_App_Read(struct bme280_data *sensorData);

#endif
