/*
 * scd4x.h
 *
 *  Created on: Mar 28, 2022
 *      Author: Administrator
 */

#ifndef INC_SCD4X_H_
#define INC_SCD4X_H_

#include "stm32f0xx.h"
#include <stdbool.h>

#define SCD4x_I2C_ADDR 		(0x62 << 1)

#define CRC8_POLYNOMIAL 	0x31
#define CRC8_INIT 			0xFF


typedef struct measurements {

	uint16_t CO2;
	float Humidity;
	float Temperature;

	uint8_t CO2_crc;
	uint8_t Humidity_crc;
	uint8_t Temperature_crc;

} Measurements_TypeDef;

typedef struct settings {

	uint16_t temperatureOffset;
	uint8_t	temperatureOffset_crc;
	uint16_t Altitude;
	uint8_t Altitude_crc;
	bool AutoSelfCalib;
	uint8_t AutoSelfCalib_crc;

} Settings_TypeDef;

typedef struct sensor_id {

	uint64_t sensorID;
	uint8_t sensorID_Data_0_crc;
	uint8_t sensorID_Data_1_crc;
	uint8_t sensorID_Data_2_crc;

} SensorID_TypeDef;


void SCD4x_Start_Periodic_Measurement(void);
void SCD4x_Read_Measurement(void);
void SCD4x_Stop_Periodic_Measurement(void);
void SCD4x_Set_Temperature_Offset(uint16_t offset);
void SCD4x_Get_Temperature_Offset(void);
void SCD4x_Set_Sensor_Altitude(uint16_t altitude); // One can set altitude to get higher CO2 accuracy, default altitude is 0.
void SCD4x_Get_Sensor_Altitude(void);
void SCD4x_Set_Ambient_Pressure(uint16_t ambientP);
void SCD4x_Set_Automatic_Self_Calibration(bool autoCalibMode);
void SCD4x_Get_Automatic_Self_Calibration(void);
void SCD4x_Start_Low_Power_Periodic_Measurement(void);
bool SCD4x_Get_Data_Ready_Status(void);
void SCD4x_Persist_Settings(void);
void SCD4x_Get_Serial_Number(void);
bool SCD4x_Perform_Self_Test(void);
void SCD4x_Perform_Factory_Reset(void);
void SCD4x_Reinit(void);
void SCD41x_Measure_Single_Shot(void); // SCD41 only.
void SCD41x_Measure_Single_Shot_RHT_Only(void); // SCD41 only. Use read_measurement command, CO2 is returned as 0 ppm.

void SCD4x_Error_Handler(void);

static uint8_t SCD4x_Generate_CRC(const uint8_t* data, uint16_t count);

#endif /* INC_SCD4X_H_ */
