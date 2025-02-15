/*****************************************************/
/* Author  : Mohamed Abdel Hamid                     */
/* Date    : 31 / 10 / 2023                          */
/* Version : V01                                     */
/*****************************************************/
#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include "main.h"
#include "stm32f4xx.h"
#include "Legacy/stm32_hal_legacy.h"
#include <stddef.h>

/* Config */
typedef struct{
	uint8_t u8Address           ;
	uint8_t u8OperationMode     ;
	I2C_HandleTypeDef * I2cId   ;
} IMU_structCfg;

/* Vector Struct */
typedef struct {
  double x;
  double y;
  double z;
} IMU_tstructVector;


typedef struct {
  uint8_t x;
  uint8_t x_sign;
  uint8_t y;
  uint8_t y_sign;
  uint8_t z;
  uint8_t z_sign;
} IMU_tAxisMap;

/* Remapping Values */
typedef enum {
  IMU_AXIS_X = 0x00,
  IMU_AXIS_Y = 0x01,
  IMU_AXIS_Z = 0x02
}IMU_enuAxis;

typedef enum  {
  IMU_AXIS_SIGN_POSITIVE = 0x00,
  IMU_AXIS_SIGN_NEGATIVE = 0x01
}IMU_enuAxisSign;

/* Calibration */
typedef struct {
  IMU_tstructVector gyro;
  IMU_tstructVector mag;
  IMU_tstructVector accel;
} IMU_structCalibOffset;

typedef struct {
  uint16_t mag;
  uint16_t accel;
} IMU_structCalibRadius;

typedef struct {
  IMU_structCalibOffset offset;
  IMU_structCalibRadius radius;
} IMU_structCalibData;


/* u8OperationMode Options */
#define	IMU_OPERATION_MODE_NDOF  					12

/* u8PowerMode Options */
#define	IMU_POWER_MODE_NORMAL    					0
#define	IMU_POWER_MODE_LOW_POWER 					1
#define	IMU_POWER_MODE_SUSPEND   					2 

/* Prototypes */

void IMU_voidInit(IMU_structCfg * Add_structCfg);
void IMU_voidSetAxisMap(IMU_structCfg * Add_structCfg,IMU_tAxisMap * axis);
HAL_StatusTypeDef IMU_enumReset(IMU_structCfg * Add_structCfg);
IMU_tstructVector IMU_structGetVectorAccelerometer(IMU_structCfg * Add_structCfg);
IMU_tstructVector IMU_structGetVectorMagnetometer(IMU_structCfg * Add_structCfg);
IMU_tstructVector IMU_structGetVectorGyroscope(IMU_structCfg * Add_structCfg);
IMU_tstructVector IMU_structGetVectorEuler(IMU_structCfg * Add_structCfg) ;
IMU_tstructVector IMU_structGetVectorLinearAccel(IMU_structCfg * Add_structCfg);
IMU_tstructVector IMU_structGetVectorGravity(IMU_structCfg * Add_structCfg);
IMU_tstructVector IMU_structGetVectorQuaternion(IMU_structCfg * Add_structCfg);
IMU_structCalibData IMU_structGetCalibrationData(IMU_structCfg * Add_structCfg);
void IMU_voidSetCalibrationData(IMU_structCfg * Add_structCfg);


#endif