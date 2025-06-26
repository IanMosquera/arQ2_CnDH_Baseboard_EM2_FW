/*
 * LTC4162.h
 *
 *  Created on: Jun 26, 2025
 *      Author: IanCMosquera
 */

#ifndef INC_LTC4162_H_
#define INC_LTC4162_H_

// Includes____________________________________________________________________
#include "main.h"

#include "arQ_CnDH_Baseboard.h"

#include "stdbool.h"
#include "stm32wbxx_hal_def.h"
#include "stm32wbxx_hal_i2c.h"

// Exported types______________________________________________________________
typedef enum{
	bat_short_fault					= 1,
	bat_missing_fault				= 2,
	cc_cv_charge						= 64,
	charger_suspended				= 256,
	absorb_charge						= 512,
	equalize_charge					= 1024,
	battery_detection				= 2048,
	bat_detect_failed_fault	= 4096
}ChargerState;

typedef enum{
	charger_off							= 0,
	constant_voltage				= 1,
	constant_current				= 2,
	iin_limit_active				= 4,
	vin_uvcl_active					= 8,
	thermal_reg_active			= 16,
	ilim_reg_active					= 32
}ChargeStatus;

typedef enum{
	thermistor_voltage_lo		= 1,
	thermistor_voltage_hi 	= 2,
	bsr_hi									= 4,
	die_temp_hi							= 8,
	ibat_lo									= 16,
	iin_hi									= 32,
	vout_hi									= 64,
	vout_lo									= 128,
	vin_hi									= 256,
	vin_lo									= 512,
	vbat_hi									= 1024,
	vbat_lo									= 2048,
	bsr_done								= 16384,
	telemetry_valid					= 32768
}LimitAlerts;

typedef enum{
	zero_cfg								= 0,
	equalize_req						= 1,
	mppt_en									= 2,
	force_telemetry_on			= 4,
	telemetry_speed					= 8,
	run_bsr									= 16,
	suspend_charger					= 32
}ConfigBitsVal;

typedef enum{
	tel_low_speed 	= 0,
	tel_high_speed 	= 1
}Telemetry_Speed;

typedef struct {
	bool equalize_req;				/*	Runs an equalization phase upon completion of an absorption phase. */
	bool mppt_en; 						/*	Run MPPT algorithm*/
	bool force_telemetry_on;	/*	Causes the telemetry system to operate at all times.*/
	bool telemetry_speed;			/*	1 = HS, 0 = LS*/
	bool run_bsr;							/*	Causes the BSR measurement to be made ASA charge cycle starts*/
	bool suspend_charger;			/*	Causes the battery charging to be suspended.*/
}ConfigBits;

typedef struct {
	bool intvcc_gt_2p8v;			/*	[0]IntVCC greater than Telemetry system lockout level*/
	bool vin_gt_4p2v;					/*	[1]VIN pin voltage is at least greater than the
								 	 	 	 	 	 	 *	switching regulator undervoltage lockout level*/
	bool vin_gt_vbat;					/*	[2]VIN pin voltage is sufficiently above the
								 	 	 	 	 	 	 *	battery voltage to begin a charge cycle*/
	bool vin_ovlo;						/*	[3]Input voltage shutdown protection is active*/
	bool thermal_shutdown;		/*	[4]High die temperature*/
	bool no_rt;								/*	[5]Missing RT resistor*/
	bool cell_count_err;			/*	[7]*/
	bool en_chg;							/*	[8]Battery charger is active*/
}SystemStatus;

typedef struct {
	bool thm_vol_low;
	bool thm_vol_hi;
	bool bsr_hi;
	bool die_temp_hi;
	bool ibat_low;
	bool iin_hi;
	bool vout_hi;
	bool vout_low;
	bool vin_hi;
	bool vin_low;
	bool vbat_hi;
	bool vbat_low;
	bool bsr_done;
	bool tel_valid;
}LimitsAlert;

typedef struct {
	I2C_HandleTypeDef *i2cHandle;		/**< I2C Handler */
	float vBATLowAlertLimit;				/*	Lower Limit of VBat						[0x01]	*/
	float vBATHighAlertLimit;				/*	Upper Limit of VBat						[0x02]	*/
	float vINLowAlertLimit;					/*	Lower Limit of VIN						[0x03]	*/
	float vINHightAlertLimit;				/*	Upper Limit of VIN						[0x04]	*/
	float vOUTLowAlertLimit;				/*	Lower Limit of VOUT						[0x05]	*/
	float vOUTHighAlertLimit;				/*	Lower Limit of VBat						[0x06]	*/
	uint16_t enLimitAlertReg;				/*	Enable limit and alert notif	[0x0D]	*/
	uint16_t enChargerStateAlertsReg;	/*	Enable chg state alert			[0x0E]	*/
	uint16_t enChargeStatusAlertsReg;	/*	Enable chg state alert			[0x0F]	*/
	uint8_t configBitsReg;					/*  System Configuration Settings	[0x14]	*/
	ConfigBits confBits;
	float iinLimitTarget;						/*  Input Current Limit Target		[0x15]	*/
	float inputUVSetting;						/*  Input Undervoltage settings		[0x16]	*/
	uint8_t vChargeSetting;					/*  															[0x1B]	*/
	uint16_t maxAbsorbTime;					/*  Max Absorb Time								[0x2B]	*/
	uint16_t tAbsorbTimer;					/*  Elapsed time in seconds				[0x32]	*/
	uint16_t chargerState;					/*	Charger State 								[0x34]	*/
	char *chargerStateStr;					/*	Enumerated Values 						------	*/
	uint8_t chargeStatus;						/*	Charger Status 								[0x35]  */
	uint16_t limitAlert;						/*	Limits Alert Register 				[0x36]  */
	LimitsAlert limAlertbits;				/*	bits 													[0x36]  */
	char *chargeStatusStr;					/*	Enumerated Values 						------	*/
	uint8_t chargeStatusAlertsReg;	/*																[0x38]	*/
	uint16_t systemStatusReg;				/*	Realtime system status ind		[0x39]	*/
	SystemStatus ssReg;
	uint8_t Chem;
	uint8_t cellCount;
	float vBAT;					/*	Battery Voltage					[0x3A] 	*/
	float vIN;					/*	Input Voltage						[0x3B]	*/
	float vOUT;					/*	Output Voltage					[0x3C]	*/
	float iBAT;					/*	Battery Current					[0x3D]	*/
	float iIN;					/*	Input Current						[0x3E]	*/
	float iOUT;					/*  Computed Output Current 				*/
	float dieTemp;			/*	Die Temp data in degC 	[0x3F]	*/
	uint16_t NTC;				/*	Thermistor voltage 			[0x40]  */
	float NTCDegrees;
	float vChargeDAC;		/*	Actual BAT Voltage		 	[0x40]  */
	float InputUVDAC;		/*	Input udervoltage DAC		[0x4B]  */
	char LTC_MSG[100];
}LTC4162;

// Exported Constants__________________________________________________________
#define LTC4162_I2C_ADDR 										(0b1101000 << 1)

#define VBAT_LO_ALERT_LIMIT									0x01
#define VBAT_HI_ALERT_LIMIT									0x02
#define VIN_LO_ALERT_LIMIT									0x03
#define VIN_HI_ALERT_LIMIT									0x04
#define VOUT_LO_ALERT_LIMIT									0x05
#define VOUT_HI_ALERT_LIMIT									0x06
#define IIN_HI_ALERT_LIMIT									0x07
#define IBAT_LO_ALERT_LIMIT									0x08
#define DIE_TEMP_HI_ALERT_LIMIT							0x09
#define BSR_HI_ALERT_LIMIT									0x0A
#define THERMISTOR_VOLTAGE_HI_ALERT_LIMIT		0x0B
#define THERMISTOR_VOLTAGE_LO_ALERT_LIMIT		0x0C
#define LTC_REG_EN_LIMIT_ALERTS_REG					0x0D
#define LTC_REG_EN_CHARGER_STATE_ALERTS_REG	0x0E
#define LTC_REG_EN_CHARGE_STATUS_ALERTS_REG	0x0F
#define LTC_REG_INPUT_UV_SETTING						0x16
#define LTC_REG_CONFIGBIT										0x14
#define LTC_REG_IIN_LIMIT_TARGET						0x15
#define LTC_REG_CHG_CUR_SET									0x1A
#define LTC_REG_VCHARGE_SETTINGS						0x1B
#define LTC_REG_MAX_ABSORB_TIME							0x2B
#define LTC_REG_TABSORBTIMER								0x32
#define LTC_REG_CHG_STATE 									0x34
#define LTC_REG_CHG_STATUS									0x35
#define LTC_REG_LIM_ALERTS									0x36
#define LTC_REG_CHG_STATUS_ALERTS						0x38
#define LTC_REG_SYS_STAT										0x39
#define LTC_REG_VBAT												0x3A
#define LTC_REG_VIN													0x3B
#define LTC_REG_VOUT												0x3C
#define LTC_REG_IBAT												0x3D
#define LTC_REG_IIN													0x3E
#define LTC_REG_DIE_TEMP										0x3F
#define LTC_REG_THERMISTOR_VOLTAGE					0x40
#define LTC_REG_CHEM_CELL										0x43
#define LTC_REG_VCHARGE_DAC									0x45
#define LTC_REG_INPUT_UV_DAC								0x4B




// Exported Macros



/**
 * Define Limit Alerts Constants
 */

// Exported functions prototypes_______________________________________________
// Device Initialization
uint8_t LTC_Assign_I2C(LTC4162 *ltc, I2C_HandleTypeDef *i2CHandle);

HAL_StatusTypeDef LTC4162_Initialize_Device(LTC4162 *dev, I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef SetChargerStateAlertRegister(LTC4162 *dev, ChargerState chargerState);
HAL_StatusTypeDef SetChargeStatusAlertRegister(LTC4162 *dev, ChargeStatus chargeStatus);
HAL_StatusTypeDef SetConfigBitsReg(LTC4162 *dev, ConfigBitsVal value);
HAL_StatusTypeDef SetInputCurrentLimit(LTC4162 *dev, uint8_t data);
HAL_StatusTypeDef SetLimitAlertNotif(LTC4162 *dev, uint16_t data);

// Data Writing Functions
HAL_StatusTypeDef LTC4162_WriteEnChargerStateAlertReg(LTC4162 *dev, ChargerState value);
HAL_StatusTypeDef LTC4162_WriteEnChargeStatusAlertReg(LTC4162 *dev, ChargeStatus value);
HAL_StatusTypeDef LTC4162_WriteAlertLimits(LTC4162 *dev, uint8_t AlertLimit, float value);
HAL_StatusTypeDef LTC4162_WriteENLimitAlertsReg(LTC4162 *dev, uint16_t value);
HAL_StatusTypeDef LTC4162_WriteIinLimitTarget(LTC4162 *dev, uint8_t value);
HAL_StatusTypeDef LTC4162_WriteConfigBitsReg(LTC4162 *dev, uint8_t value);

// Data Acquisition Functions
HAL_StatusTypeDef LTC4162_ReadAlertLimits(LTC4162 *dev, uint8_t AlertLimit);
HAL_StatusTypeDef LTC4162_ReadEnLimitAlertsReg(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadEnChargerStateAlertsReg(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadEnChargeStatusAlertsReg(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadConfigBitsReg(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadChargerState(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadChargeStatus(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadChargeStatusAlertsReg(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadChemCellReg(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadDieTemp(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadIBAT(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadIIN(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadIinLimitTarget(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadInputUVSetting(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadInputUVDAC(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadLimitAlertReg(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadMaxAbsorbTime(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadNTC(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadSystemStatusReg(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadTAbsorbTimer(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadVChargeDAC(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadVChargeSetting(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadVBAT(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadVIN(LTC4162 *dev);
HAL_StatusTypeDef LTC4162_ReadVOUT(LTC4162 *dev);

// High Level Functions
void LTC_Init(LTC4162 *ltc);

// Low Level Function
HAL_StatusTypeDef LTC4162_WriteRegister(LTC4162 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef LTC4162_WriteRegisters(LTC4162 *dev, uint8_t reg, uint8_t *data, uint8_t size);
HAL_StatusTypeDef LTC4162_ReadRegister(LTC4162 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef LTC4162_ReadRegisters(LTC4162 *dev, uint8_t reg, uint8_t *data, uint8_t length);

// Other Functions
uint8_t getBits(uint8_t bitPlace, uint8_t num);
uint8_t hexCharToInt(char hex);

#endif /* INC_LTC4162_H_ */
