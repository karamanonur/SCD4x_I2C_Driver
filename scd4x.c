/*
 * scd4x.c
 *
 *  Created on: Mar 28, 2022
 *      Author: Onur Karaman
 */


#include "scd4x.h"
#include "i2c.h"
#include "math.h"

Measurements_TypeDef Measurements;
Settings_TypeDef Settings;
SensorID_TypeDef Sensor;

/***************************************************************
 *
 While a periodic measurement is running no other commands
 must issued with the exceptionf of;

 "read_measurement",
 "get_data_ready_status",
 "stop_periodic_measurement",
 "set_ambient_pressure".

 ***************************************************************/

void SCD4x_Start_Periodic_Measurement(void)
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x21;
	txBuff[1] = 0xB1;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(5000);
}

void SCD4x_Read_Measurement(void)
{
	bool isDataReady;
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[9] = {0};

	isDataReady = SCD4x_Get_Data_Ready_Status();

	int i = 0;
	while(i < 10)
	{
		if(isDataReady == 0)
			isDataReady = SCD4x_Get_Data_Ready_Status();
		else
			break;
	}

	if(!(isDataReady))
	{
		SCD4x_Stop_Periodic_Measurement();
		SCD4x_Reinit();
		SCD4x_Start_Periodic_Measurement();
	}

	txBuff[0] = 0xEC;
	txBuff[1] = 0x05;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c2, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);
	HAL_Delay(1);

	Measurements.CO2 = (rxBuff[0] << 8) | rxBuff[1];
	Measurements.CO2_crc = rxBuff[2];

	Measurements.Temperature = (float)(-45 + ((((rxBuff[3] << 8) | rxBuff[4]) * 175) / pow(2, 16)));
	Measurements.Temperature_crc = rxBuff[5];

	Measurements.Humidity = (float)(100 * ((rxBuff[6] << 8) | rxBuff[7]) / pow(2, 16));
	Measurements.Humidity_crc = rxBuff[8];

}

void SCD4x_Stop_Periodic_Measurement(void) // idle mode.
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x3F;
	txBuff[1] = 0x86;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(500);
}

void SCD4x_Set_Temperature_Offset(uint16_t offset)
{
	uint8_t txBuff[5] = {0};
	uint8_t offsetBuff[2] = {0};

	offsetBuff[0] = offset >> 8;
	offsetBuff[1] = offset & 0xFF;

	txBuff[0] = 0x24;
	txBuff[1] = 0x1D;
	txBuff[2] = offset >> 8;
	txBuff[3] = offset & 0xFF;
	txBuff[4] = SCD4x_Generate_CRC(offsetBuff, sizeof(offsetBuff));

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1);
}

void SCD4x_Get_Temperature_Offset(void)
{
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[3] = {0};

	txBuff[0] = 0x23;
	txBuff[1] = 0x18;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c2, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	Settings.temperatureOffset = (rxBuff[0] << 8) | rxBuff[1];
	Settings.temperatureOffset_crc = rxBuff[3];

}

void SCD4x_Set_Sensor_Altitude(uint16_t altitude) // One can set altitude to get higher CO2 accuracy, default altitude is 0.
{
	uint8_t txBuff[5] = {0};
	uint8_t altitudeBuff[2] = {0};

	altitudeBuff[0] = altitude >> 8;
	altitudeBuff[1] = altitude & 0xFF;

	txBuff[0] = 0x24;
	txBuff[1] = 0x27;
	txBuff[2] = altitude & 0xFF;
	txBuff[3] = altitude >> 8;
	txBuff[4] = SCD4x_Generate_CRC(altitudeBuff, sizeof(altitudeBuff));

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1);
}

void SCD4x_Get_Sensor_Altitude(void)
{
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[3] = {0};

	txBuff[0] = 0x23;
	txBuff[1] = 0x22;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c2, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	Settings.Altitude = (rxBuff[0] << 8) | rxBuff[1];
	Settings.Altitude_crc = rxBuff[3];
}

void SCD4x_Set_Ambient_Pressure(uint16_t ambientP)
{
	uint8_t txBuff[5] = {0};
	uint8_t ambientPBuff[2] = {0};

	ambientPBuff[0] = ambientP >> 8;
	ambientPBuff[1] = ambientP & 0xFF;

	txBuff[0] = 0x0E;
	txBuff[1] = 0x00;
	txBuff[2] = ambientP & 0xFF;
	txBuff[3] = ambientP >> 8;
	txBuff[4] = SCD4x_Generate_CRC(ambientPBuff, sizeof(ambientPBuff));

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1);
}

void SCD4x_Set_Automatic_Self_Calibration(bool autoCalibMode)
{
	uint8_t txBuff[5] = {0};
	uint8_t autoCalibModeBuff[2] = {0};

	if(!(autoCalibMode))
	{
		autoCalibModeBuff[0] = 0x00;
		autoCalibModeBuff[1] = 0x00;
	}
	else if(autoCalibMode)
	{
		autoCalibModeBuff[0] = 0x00;
		autoCalibModeBuff[1] = 0x01;
	}
	else
	{
		autoCalibModeBuff[0] = 0x00;
		autoCalibModeBuff[1] = 0x01;
	}

	txBuff[0] = 0x24;
	txBuff[1] = 0x16;
	txBuff[2] = autoCalibMode & 0xFF;
	txBuff[3] = autoCalibMode >> 8;
	txBuff[4] = SCD4x_Generate_CRC(autoCalibModeBuff, sizeof(autoCalibModeBuff));

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1);

}

void SCD4x_Get_Automatic_Self_Calibration(void)
{
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[3] = {0};

	txBuff[0] = 0x23;
	txBuff[1] = 0x13;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c2, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	if(((rxBuff[0] << 8) | rxBuff[1]) == 0 && rxBuff[3] == 0x81)
		Settings.AutoSelfCalib = false;
	else if(((rxBuff[0] << 8) | rxBuff[1]) == 1 && rxBuff[3] == 0xB0)
		Settings.AutoSelfCalib = true;
	else
		SCD4x_Error_Handler();

	Settings.AutoSelfCalib_crc = rxBuff[3];
}

void SCD4x_Start_Low_Power_Periodic_Measurement(void)
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x21;
	txBuff[1] = 0xAC;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1);
}

bool SCD4x_Get_Data_Ready_Status(void)
{
	bool dataReadyFlag;
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[3] = {0};

	txBuff[0] = 0xE4;
	txBuff[1] = 0xB8;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c2, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	if((((rxBuff[0] << 8) | rxBuff[1]) & 0x07FF) != 0)
		dataReadyFlag = true;
	else
		dataReadyFlag = false;

	return dataReadyFlag;
}

void SCD4x_Persist_Settings(void)
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x36;
	txBuff[1] = 0x15;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1000);
}

void SCD4x_Get_Serial_Number(void)
{
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[9] = {0};

	txBuff[0] = 0x36;
	txBuff[1] = 0x82;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c2, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	Sensor.sensorID = (rxBuff[0] << 40) | (rxBuff[1] << 32) | (rxBuff[3] << 24) |
					  (rxBuff[4] << 16) | (rxBuff[6] << 8) | rxBuff[7];

	Sensor.sensorID_Data_0_crc = rxBuff[2];
	Sensor.sensorID_Data_1_crc = rxBuff[5];
	Sensor.sensorID_Data_2_crc = rxBuff[8];
}

bool SCD4x_Perform_Self_Test(void)
{
	bool malfunctionFlag;
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[3] = {0};

	txBuff[0] = 0x36;
	txBuff[1] = 0x39;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(10000);
	HAL_I2C_Master_Receive(&hi2c2, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	if(((rxBuff[0] << 8) | (rxBuff[1])) != 0)
		malfunctionFlag = true;
	else
		malfunctionFlag = false;

	return malfunctionFlag;
}

void SCD4x_Perform_Factory_Reset(void)
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x36;
	txBuff[1] = 0x32;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1200);
}

void SCD4x_Reinit(void)
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x36;
	txBuff[1] = 0x46;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(20);
}

void SCD41x_Measure_Single_Shot(void) // SCD41 only.
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x21;
	txBuff[1] = 0x9D;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(5000);
}

void SCD41x_Measure_Single_Shot_RHT_Only(void) // SCD41 only. Use read_measurement command, CO2 is returned as 0 ppm.
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x21;
	txBuff[1] = 0x96;

	HAL_I2C_Master_Transmit(&hi2c2, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(50);
}

static uint8_t SCD4x_Generate_CRC(const uint8_t* data, uint16_t count)
{
	uint16_t current_byte;
	uint8_t crc = CRC8_INIT;
	uint8_t crc_bit;

	/* calculates 8-Bit checksum with given polynomial */
	for (current_byte = 0; current_byte < count; ++current_byte)
	{
		crc ^= (data[current_byte]);

		for (crc_bit = 8; crc_bit > 0; --crc_bit)
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ CRC8_POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}
	return crc;
}

void SCD4x_Error_Handler(void) {}

