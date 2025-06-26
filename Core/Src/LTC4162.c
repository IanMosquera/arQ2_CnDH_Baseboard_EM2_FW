/*
 * LTC4162.c
 *
 *  Created on: Jun 26, 2025
 *      Author: IanCMosquera
 */

// Includes____________________________________________________________________
#include "LTC4162.h"

#include "main.h"
#include "math.h"

// Private Variables__________________________________________________________
// Assumed that these device and I2C handles are declared on main.c
extern I2C_HandleTypeDef hi2c1;
extern LTC4162 ltc;
extern uint8_t txBuf[128];

// High Level Functions
void LTC_Init(LTC4162 *ltc)
{
  uint8_t size = 2;
  uint16_t data;

  // LTC4162_Initialize_Device(&ltc, &hi2c1);
  SetConfigBitsReg(ltc, 0);
  SetChargerStateAlertRegister(ltc, bat_short_fault);
  SetChargeStatusAlertRegister(ltc, thermal_reg_active);
  SetInputCurrentLimit(ltc, 59); // 59+1 = 3A

  // Set Alert Limits
  LTC4162_WriteAlertLimits(ltc, VBAT_LO_ALERT_LIMIT, 11.80f);
  LTC4162_WriteAlertLimits(ltc, VBAT_HI_ALERT_LIMIT, 14.50f);
  LTC4162_WriteAlertLimits(ltc, VIN_LO_ALERT_LIMIT,  11.90f);
  LTC4162_WriteAlertLimits(ltc, VIN_HI_ALERT_LIMIT,  24.80f);
  LTC4162_WriteAlertLimits(ltc, VOUT_LO_ALERT_LIMIT, 11.80f);
  LTC4162_WriteAlertLimits(ltc, VOUT_HI_ALERT_LIMIT, 18.80f);

  data = 16000;
  LTC4162_WriteRegisters(ltc, DIE_TEMP_HI_ALERT_LIMIT, (uint8_t *)&data, size);

  HAL_Delay(100);

  SetLimitAlertNotif(ltc, die_temp_hi);
  LTC4162_ReadChemCellReg(ltc);
}

//Initiate LTC4162 Device
HAL_StatusTypeDef LTC4162_Initialize_Device(LTC4162 *dev, I2C_HandleTypeDef *i2cHandle)
{
  HAL_StatusTypeDef status;

  dev->i2cHandle = i2cHandle;

  // Wait to ensure that the device is ready
  while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

  status = HAL_I2C_IsDeviceReady(dev->i2cHandle, LTC4162_I2C_ADDR, 5, 1000);
  HAL_Delay(100);

  if(status != HAL_OK)
    return status;
  else
    return HAL_OK;
}


uint8_t LTC_Assign_I2C(LTC4162 *ltc, I2C_HandleTypeDef *i2CHandle)
{
  uint8_t status = 0;
  ltc->i2cHandle = i2CHandle;

  // See if the device is ready
  while(HAL_I2C_GetState(ltc->i2cHandle) != HAL_I2C_STATE_READY){}

  status = HAL_I2C_IsDeviceReady(ltc->i2cHandle, LTC4162_I2C_ADDR, 5, 1000);
  HAL_Delay(100);

  if(status != 0)
    return status;
  else
    return 0;
}

HAL_StatusTypeDef SetChargerStateAlertRegister(LTC4162 *dev, ChargerState chargerState)
{
	HAL_StatusTypeDef ret;

	// Read inital value
	ret = LTC4162_ReadEnChargerStateAlertsReg(dev);
	if (ret == HAL_OK){
		HAL_Delay(100);

		// Set value
		ret = LTC4162_WriteEnChargerStateAlertReg(dev, chargerState);
		if (ret == HAL_OK){
			HAL_Delay(100);
			ret = LTC4162_ReadEnChargerStateAlertsReg(dev);
			if (ret == HAL_OK){
			}
			else ret = 1;
		}
		else ret = 1;
	}
	else ret = 1;

	return ret;
}

HAL_StatusTypeDef SetConfigBitsReg(LTC4162 *dev, ConfigBitsVal value)
{
  HAL_StatusTypeDef ret;

  // Read inital value first
  ret = LTC4162_ReadConfigBitsReg(dev);

  if (ret == HAL_OK)
  {
    HAL_Delay(100);

    ret = LTC4162_WriteConfigBitsReg(dev, value);
    if (ret == HAL_OK){
	    HAL_Delay(100);
	    ret = LTC4162_ReadConfigBitsReg(dev);
	    if (ret == HAL_OK){
	    }
	    else ret = 1;
    }
    else ret = 1;
  }
  else ret = 1;

  return ret;
}

HAL_StatusTypeDef SetChargeStatusAlertRegister(LTC4162 *dev, ChargeStatus chargeStatus){
	HAL_StatusTypeDef ret;

	// Read inital value
	ret = LTC4162_ReadEnChargeStatusAlertsReg(dev);

	if (ret == HAL_OK){
		HAL_Delay(100);

		ret = LTC4162_WriteEnChargeStatusAlertReg(dev, chargeStatus);
		if (ret == HAL_OK){
			HAL_Delay(100);

			ret = LTC4162_ReadEnChargeStatusAlertsReg(dev);
			if (ret == HAL_OK){
			}
			else ret = 1;
		}
		else ret = 1;
	}
	else ret = 1;

	return ret;
}

HAL_StatusTypeDef SetInputCurrentLimit(LTC4162 *dev, uint8_t data){
	HAL_StatusTypeDef ret;

	// Read inital value
	ret = LTC4162_ReadIinLimitTarget(dev);

	if (ret == HAL_OK){
		HAL_Delay(100);

		// Set Value
		ret = LTC4162_WriteIinLimitTarget(dev, data);
		if (ret == HAL_OK){
			HAL_Delay(100);
			ret = LTC4162_ReadIinLimitTarget(dev);
			if (ret == HAL_OK){
			}
			else ret = 1;
		}
		else ret = 1;
	}
	else ret = 1;

	return ret;
}

HAL_StatusTypeDef SetLimitAlertNotif(LTC4162 *dev, uint16_t data){
	HAL_StatusTypeDef ret;

	// Read inital value
	ret = LTC4162_ReadEnLimitAlertsReg(dev);

	if (ret == HAL_OK){
		HAL_Delay(100);

		// Set Value
		ret = LTC4162_WriteENLimitAlertsReg(dev, data);
		if (ret == HAL_OK){
			HAL_Delay(100);
			ret = LTC4162_ReadEnLimitAlertsReg(dev);
			if (ret == HAL_OK){
			}
			else ret = 1;
		}
		else ret = 1;
	}
	else ret = 1;

	return ret;
}



HAL_StatusTypeDef LTC4162_ReadIinLimitTarget(LTC4162 *dev){
	uint8_t raw;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegister(dev, LTC_REG_IIN_LIMIT_TARGET, &raw);

	if (status != HAL_OK) status = 1;
	else dev->iinLimitTarget = (raw & 0x3F) * 0.05f;
	return status;
}

HAL_StatusTypeDef LTC4162_ReadEnChargerStateAlertsReg(LTC4162 * dev){
	uint8_t val[2];
	uint16_t temp;

	// Wait for I2C device to be ready
	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	HAL_StatusTypeDef status =  LTC4162_ReadRegisters(dev, LTC_REG_EN_CHARGER_STATE_ALERTS_REG, val, 2);
	if (status != HAL_OK) status = 1;
	else{
		temp = ((val[1] & 0x1F) << 8) | (val[0] & 0x43);
		dev->enChargerStateAlertsReg = temp;
	}

	return status;
}

HAL_StatusTypeDef LTC4162_ReadEnChargeStatusAlertsReg(LTC4162 *dev){
	uint8_t val, temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegister(dev, LTC_REG_EN_CHARGE_STATUS_ALERTS_REG, &val);
	if (status != HAL_OK) status = 1;
	else{
		temp = val & 0x3F;
		dev->enChargeStatusAlertsReg = temp;
	}
	return status;
}




HAL_StatusTypeDef LTC4162_ReadConfigBitsReg(LTC4162 *dev){
	uint8_t raw;
	HAL_StatusTypeDef status;

	// Wait for I2C device to be ready
	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegister(dev, LTC_REG_CONFIGBIT, &raw);
	if (status != HAL_OK) status = 1;
	else{
		dev->configBitsReg = raw & 0x3E;
		//  Save value to ConfigBits
		// dev->confBits.equalize_req 				= getBits(0, raw);
		dev->confBits.mppt_en 						= getBits(1, raw);
		dev->confBits.force_telemetry_on 	= getBits(2, raw);
		dev->confBits.telemetry_speed 		= getBits(3, raw);
		dev->confBits.run_bsr 						= getBits(4, raw);
		dev->confBits.suspend_charger 		= getBits(5, raw);
	}
	return status;
}

HAL_StatusTypeDef LTC4162_ReadChargeStatus(LTC4162 *dev){
	uint8_t chargerStatus;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}


	status = LTC4162_ReadRegister(dev, LTC_REG_CHG_STATUS, &chargerStatus);
	if (status != HAL_OK) status = 1;
	else{
		/**
		  * Extracting Charge Status Data from Raw Data
		  * Refer ro Page 39 of Datasheet
		  * NOTE: mutually exclusive [default = 0, 1, 2, 4, 8, 16, 32]
		  * 1. Mask the byte with [XX11 1111] = 0x3F
		  * */
		dev->chargeStatus = chargerStatus & 0x3F;

		/*Get Enums*/
		switch (dev->chargeStatus){
		case 0:
			dev->chargeStatusStr = "cgof";
			break;
		case 1:
			dev->chargeStatusStr = "_cv_";
			break;
		case 2:
			dev->chargeStatusStr = "_cc_";
			break;
		case 4:
			dev->chargeStatusStr = "iinL";
			break;
		case 8:
			dev->chargeStatusStr = "v_uv";
			break;
		case 16:
			dev->chargeStatusStr = "thrm";
			break;
		case 32:
			dev->chargeStatusStr = "ilim";
			break;
		default:
			dev->chargeStatusStr = "_err";
			break;
		}
	}
	return status;
}

HAL_StatusTypeDef LTC4162_ReadChargerState(LTC4162 *dev){
	uint8_t chargerState[2];
	uint16_t temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status = LTC4162_ReadRegisters(dev, LTC_REG_CHG_STATE, chargerState, 2);
	if (status != HAL_OK) status = 1;
	else{
	 	/**
		  * Extracting Charger State Data from Raw Data
		  * Refer ro Page 39 of Datasheet
		  * NOTE: mutually exclusive [1, 2, 64, default=256, 512, 1024, 2048, 4096]
		  * 1. Mask byte[1] with [XXX1 1111] = 0x1F
		  * 2. Left shift byte[1] by 8 then store to 16bit register
		  * 3. Mask byte[0] with [X1XX XXX1] = 0x41
		  * 4. OR with 16 bit register
		  * */
		temp = ((chargerState[1] & 0x1F) << 8) | (chargerState[0] & 0x43);
		dev->chargerState = temp;

		/*Get Enums*/
		switch (temp){
		case 1:
			dev->chargerStateStr = "btsF";
			break;
		case 2:
			dev->chargerStateStr = "btmF";
			break;
		case 4:
			dev->chargerStateStr = "mctF";
			break;
		case 8:
			dev->chargerStateStr = "coxt";
			break;
		case 16:
			dev->chargerStateStr = "tmrt";
			break;
		case 32:
			dev->chargerStateStr = "ntcP";
			break;
		case 64:
			dev->chargerStateStr = "cccv";
			break;
		case 256:
			dev->chargerStateStr = "csus";
			break;
		case 512:
			dev->chargerStateStr = "absC";
			break;
		case 1024:
			dev->chargerStateStr = "eqcg";
			break;
		case 2048:
			dev->chargerStateStr = "batd";
			break;
		case 4096:
			dev->chargerStateStr = "bdff";
			break;
		default:
			dev->chargerStateStr = "_err";
			break;
		}
	}
	return status;
}

HAL_StatusTypeDef LTC4162_ReadChemCellReg(LTC4162 *dev){
	uint8_t chemCell[2];
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegisters(dev, LTC_REG_CHEM_CELL, chemCell, 2);
	if (status != HAL_OK) status = 1;
	else{
		dev->Chem = chemCell[1] & 0x0F;
		dev->cellCount = chemCell[0] & 0x0F;
	}
	return status;
}

HAL_StatusTypeDef LTC4162_ReadDieTemp(LTC4162 *dev){
	uint8_t dieTemp[2];
	uint16_t temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegisters(dev, LTC_REG_DIE_TEMP, dieTemp, 2);
	if (status != HAL_OK) status = 1;
	else{
		temp  = (dieTemp[1] << 8) | dieTemp[0];
		dev->dieTemp = (temp  * 0.0215f) - 264.4f;
	}
	return status;
}

HAL_StatusTypeDef LTC4162_ReadIBAT(LTC4162 *dev){
	uint8_t iBAT[2];
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegisters(dev, LTC_REG_IBAT, iBAT, 2);
	if (status != HAL_OK) status = 1;
	else{
		int16_t temp = (iBAT[1] <<8 | iBAT[0]);
		dev->iBAT = temp * 0.0001466f;
	}
	return status;
}

HAL_StatusTypeDef LTC4162_ReadIIN(LTC4162 *dev){
	uint8_t raw[2];
	int16_t temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status = LTC4162_ReadRegisters(dev, LTC_REG_IIN, raw, 2);

	if (status != HAL_OK) status = 1;
	else{
		temp = (raw[1] <<8 | raw[0]);		/**< Combine registers to get raw data */
		dev->iIN = temp *  0.0001466f;	/**< Converts raw data to Volts */
	}

	return status;
}

HAL_StatusTypeDef LTC4162_ReadLimitAlertReg(LTC4162 *dev){
	uint8_t raw[2];
	int16_t temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status = LTC4162_ReadRegisters(dev, LTC_REG_LIM_ALERTS, raw, 2);
	if (status != HAL_OK) status = 1;
	else{
		temp = (raw[1] <<8 | raw[0]);		/**< Combine registers to get raw data */
		dev->limitAlert = temp;

		dev->limAlertbits.thm_vol_low		= getBits(0, raw[0]);
		dev->limAlertbits.thm_vol_hi		= getBits(1, raw[0]);
		dev->limAlertbits.bsr_hi				= getBits(2, raw[0]);
		dev->limAlertbits.die_temp_hi		= getBits(3, raw[0]);
		dev->limAlertbits.ibat_low			= getBits(4, raw[0]);
		dev->limAlertbits.iin_hi				= getBits(5, raw[0]);
		dev->limAlertbits.vout_hi				= getBits(6, raw[0]);
		dev->limAlertbits.vout_low			= getBits(7, raw[0]);

		dev->limAlertbits.vin_hi				= getBits(0, raw[1]); //8
		dev->limAlertbits.vin_low				= getBits(1, raw[1]);	//9
		dev->limAlertbits.vbat_hi				= getBits(2, raw[1]);	//10
		dev->limAlertbits.vbat_low			= getBits(3, raw[1]);	//11


		dev->limAlertbits.bsr_done			= getBits(6, raw[1]);	//14
		dev->limAlertbits.tel_valid			= getBits(7, raw[1]);	//15
	}
	return status;
}

HAL_StatusTypeDef LTC4162_ReadNTC(LTC4162 *dev){
	uint8_t ntc[2];
	float rntc, tc;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegisters(dev, LTC_REG_THERMISTOR_VOLTAGE, ntc, 2);
	if (status != HAL_OK) status = 1;
	else{
		dev->NTC = (ntc[1] <<8 | ntc[0]);
		// Convert to degrees
		// NTCtoTempC(ltc.NTC, 3694, 10000));

		rntc = 10000 * (dev->NTC / (21829.2f - dev->NTC));
		tc = (3694 / (log(rntc/10000) + (3694/298.15f))) - 273.15;
		dev->NTCDegrees = tc;
	}
	return status;
}

HAL_StatusTypeDef LTC4162_ReadSystemStatusReg(LTC4162 *dev){
	uint8_t raw[2];
	int16_t temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status = LTC4162_ReadRegisters(dev, LTC_REG_SYS_STAT, raw, 2);
	if (status != HAL_OK) status = 1;
	else{
		temp = (raw[1] <<8 | raw[0]);		/**< Combine registers to get raw data */
		dev->systemStatusReg = temp;

		dev->ssReg.intvcc_gt_2p8v 	= getBits(0, raw[0]);
		dev->ssReg.vin_gt_4p2v 			= getBits(1, raw[0]);
		dev->ssReg.vin_gt_vbat 			= getBits(2, raw[0]);
		dev->ssReg.vin_ovlo					= getBits(3, raw[0]);
		dev->ssReg.thermal_shutdown = getBits(4, raw[0]);
		dev->ssReg.no_rt						= getBits(5, raw[0]);
		dev->ssReg.cell_count_err		= getBits(7, raw[0]);
		dev->ssReg.en_chg						= getBits(0, raw[1]);
	}
	return status;
}

HAL_StatusTypeDef LTC4162_ReadVBAT(LTC4162 *dev){
	uint8_t vBAT[2];
	int16_t temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegisters(dev, LTC_REG_VBAT, vBAT, 2);
	if (status != HAL_OK) status = 1;
	else{
		temp = (vBAT[1] <<8 | vBAT[0]);
		if (dev->Chem == 6){
			dev->vBAT = (temp * 0.0001924f) * 4;
		}
		else if(dev->Chem == 9){
			dev->vBAT = (temp  * 0.0003848f)  * (dev->cellCount/2);
		}
	}
	return status;
}


HAL_StatusTypeDef LTC4162_ReadVIN(LTC4162 *dev){
	uint8_t vIN[2];
	uint16_t temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegisters(dev, LTC_REG_VIN, vIN, 2);
	if (status != HAL_OK) status = 1;
	else{
		temp = (vIN[1] <<8 | vIN[0]);	/**< Combine registers to get raw data */
		dev->vIN = temp * 0.001649f;	/**< Converts raw data to Volts */
	}
	return status;
}

HAL_StatusTypeDef LTC4162_ReadVOUT(LTC4162 *dev){
	uint8_t vOUT[2];
	uint16_t temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegisters(dev, LTC_REG_VOUT, vOUT, 2);
	if (status != HAL_OK) status = 1;
	else{
		temp = (vOUT[1] <<8 | vOUT[0]);	/**< Combine registers to get raw data */
		dev->vOUT = temp * 0.001653f;		/**< Converts raw data to Volts */
	}
	return status;
}

HAL_StatusTypeDef LTC4162_WriteAlertLimits(LTC4162 *dev, uint8_t AlertLimit, float value){
	uint8_t bitValue[2];
	uint16_t tempInt;
	double temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	switch (AlertLimit){
		case 1:
			temp = value /(0.0003848f * 2);
			break;
		case 2:
			temp = value /(0.0003848f * 2);
			break;
		case 3:
			temp = value /0.001649f;
			break;
		case 4:
			temp = value /0.001649f;
			break;
		case 5:
			temp = value /0.001653f;
			break;
		case 6:
			temp = value /0.001653f;
			break;
		default:
			break;
	}

	tempInt = temp;
	bitValue[0] = tempInt & 0x00FF;
	bitValue[1] = (tempInt & 0xFF00) >> 8;
	status = HAL_I2C_Mem_Write(dev->i2cHandle, LTC4162_I2C_ADDR, AlertLimit, 1, bitValue, 2, HAL_MAX_DELAY);

	return status;
}

HAL_StatusTypeDef LTC4162_WriteIinLimitTarget(LTC4162 *dev, uint8_t value){
	HAL_StatusTypeDef status;
	uint8_t inp = value;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status = LTC4162_WriteRegister(dev, LTC_REG_IIN_LIMIT_TARGET, &inp);
	if (status != HAL_OK) status = 1;
	return status;
}

HAL_StatusTypeDef LTC4162_WriteConfigBitsReg(LTC4162 *dev, uint8_t value){
	HAL_StatusTypeDef status;
	uint8_t temp[2];

	// Wait for I2C device to be ready
	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	temp[0] = value;
	temp[1] = 0;

//	status = LTC4162_WriteRegisters(dev, LTC_REG_CONFIGBIT, value);
	status = HAL_I2C_Mem_Write(&hi2c1, LTC4162_I2C_ADDR, LTC_REG_CONFIGBIT, 1, &temp[0], 2, HAL_MAX_DELAY);
	if (status != HAL_OK) status = 1;

	return status;
}

HAL_StatusTypeDef LTC4162_ReadEnLimitAlertsReg(LTC4162 *dev){
	uint8_t val[2];
	uint16_t temp;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status =  LTC4162_ReadRegisters(dev, LTC_REG_EN_LIMIT_ALERTS_REG, val, 2);
	if (status != HAL_OK) status = 1;
	else{
		temp = ((val[1] & 0xCF) << 8) | (val[0] & 0xFF);
		dev->enLimitAlertReg = temp;
	}
	return status;
}

HAL_StatusTypeDef LTC4162_WriteEnChargeStatusAlertReg(LTC4162 *dev, ChargeStatus value){
	uint8_t inp = value;
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status = LTC4162_WriteRegister(dev, LTC_REG_EN_CHARGE_STATUS_ALERTS_REG, &inp);
	if (status != HAL_OK) status = 1;
	return status;
}

HAL_StatusTypeDef LTC4162_WriteEnChargerStateAlertReg(LTC4162 *dev, ChargerState value){
	uint8_t inp[2];
	HAL_StatusTypeDef status;

	// < Clear Flags
	inp[0] = value & 0x00FF;
	inp[1] = value >> 8;

	// Wait for I2C device to be ready
	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	status = HAL_I2C_Mem_Write(dev->i2cHandle, LTC4162_I2C_ADDR, LTC_REG_EN_CHARGER_STATE_ALERTS_REG, 1, inp, 2, HAL_MAX_DELAY);
	if (status != HAL_OK) status = 1;
	return status;
}

HAL_StatusTypeDef LTC4162_WriteENLimitAlertsReg(LTC4162 *dev, uint16_t value){
	uint8_t inp[2];
	HAL_StatusTypeDef status;

	while(HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY){}

	inp[0] = value & 0x00FF;
	inp[1] = value >> 8;

	status = LTC4162_WriteRegisters(dev, LTC_REG_EN_LIMIT_ALERTS_REG, inp, 2);

	if (status != HAL_OK) status = 1;
	return status;
}

// Low Level Functions
HAL_StatusTypeDef LTC4162_ReadRegister(LTC4162 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read(dev->i2cHandle, LTC4162_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef LTC4162_ReadRegisters(LTC4162 *dev, uint8_t reg, uint8_t *data, uint8_t length){
	return HAL_I2C_Mem_Read(dev->i2cHandle, LTC4162_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef LTC4162_WriteRegister(LTC4162 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev->i2cHandle, LTC4162_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef LTC4162_WriteRegisters(LTC4162 *dev, uint8_t reg, uint8_t *data, uint8_t size){
	return HAL_I2C_Mem_Write(dev->i2cHandle, LTC4162_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

// Other Functions
uint8_t getBits(uint8_t bitPlace, uint8_t num){
	uint8_t bit = (num & ( 1 << bitPlace )) >> bitPlace;
	return bit;
}

uint8_t hexCharToInt(char hex){
	uint8_t digit;

	if (hex>='0' && hex<='9'){
		digit = hex - 48;
	}else if (hex>='a' && hex<='f'){
		digit = hex - 97 + 10;
	}else if (hex>='A' && hex<='F'){
		digit = hex - 65 + 10;}

	return digit;
}
