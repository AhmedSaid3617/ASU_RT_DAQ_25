/**********************************************************/
/* Author  : Mohamed Abdel Hamid                          */
/* Date    : 31 / 10 / 2023                               */
/* Version : V01                                          */
/**********************************************************/
#ifndef IMU_PRIV_H
#define IMU_PRIV_H


/* Register Addresses */
#define IMU_ID 			        (0xA0)
#define IMU_CHIP_ID 		     0x00   // value: 0xA0
#define IMU_ACC_ID 			     0x01  // value: 0xFB
#define IMU_MAG_ID 			     0x02  // value: 0x32
#define IMU_GYRO_ID 		     0x03  // value: 0x0F
#define IMU_SW_REV_ID_LSB 	 0x04  // value: 0x08
#define IMU_SW_REV_ID_MSB 	 0x05  // value: 0x03
#define IMU_BL_REV_ID 		   0x06      // N/A
#define IMU_PAGE_ID 		     0x07
#define IMU_ACC_DATA_X_LSB 	 0x08
#define IMU_ACC_DATA_X_MSB 	 0x09
#define IMU_ACC_DATA_Y_LSB 	 0x0A
#define IMU_ACC_DATA_Y_MSB 	 0x0B
#define IMU_ACC_DATA_Z_LSB   0x0C
#define IMU_ACC_DATA_Z_MSB   0x0D
#define IMU_MAG_DATA_X_LSB   0x0E
#define IMU_MAG_DATA_X_MSB   0x0F
#define IMU_MAG_DATA_Y_LSB   0x10
#define IMU_MAG_DATA_Y_MSB   0x11
#define IMU_MAG_DATA_Z_LSB   0x12
#define IMU_MAG_DATA_Z_MSB   0x13
#define IMU_GYR_DATA_X_LSB   0x14
#define IMU_GYR_DATA_X_MSB   0x15
#define IMU_GYR_DATA_Y_LSB   0x16
#define IMU_GYR_DATA_Y_MSB   0x17
#define IMU_GYR_DATA_Z_LSB   0x18
#define IMU_GYR_DATA_Z_MSB   0x19
#define IMU_EUL_HEADING_LSB  0x1A
#define IMU_EUL_HEADING_MSB  0x1B
#define IMU_EUL_ROLL_LSB     0x1C
#define IMU_EUL_ROLL_MSB     0x1D
#define IMU_EUL_PITCH_LSB    0x1E
#define IMU_EUL_PITCH_MSB    0x1F
#define IMU_QUA_DATA_W_LSB   0x20
#define IMU_QUA_DATA_W_MSB   0x21
#define IMU_QUA_DATA_X_LSB   0x22
#define IMU_QUA_DATA_X_MSB   0x23
#define IMU_QUA_DATA_Y_LSB   0x24
#define IMU_QUA_DATA_Y_MSB   0x25
#define IMU_QUA_DATA_Z_LSB   0x26
#define IMU_QUA_DATA_Z_MSB   0x27
#define IMU_LIA_DATA_X_LSB   0x28
#define IMU_LIA_DATA_X_MSB   0x29
#define IMU_LIA_DATA_Y_LSB   0x2A
#define IMU_LIA_DATA_Y_MSB   0x2B
#define IMU_LIA_DATA_Z_LSB   0x2C
#define IMU_LIA_DATA_Z_MSB   0x2D
#define IMU_GRV_DATA_X_LSB   0x2E
#define IMU_GRV_DATA_X_MSB   0x2F
#define IMU_GRV_DATA_Y_LSB   0x30
#define IMU_GRV_DATA_Y_MSB   0x31
#define IMU_GRV_DATA_Z_LSB   0x32
#define IMU_GRV_DATA_Z_MSB   0x33
#define IMU_TEMP             0x34
#define IMU_CALIB_STAT       0x35
#define IMU_ST_RESULT        0x36
#define IMU_INT_STATUS       0x37
#define IMU_SYS_CLK_STATUS   0x38
#define IMU_SYS_STATUS       0x39
#define IMU_SYS_ERR          0x3A
#define IMU_UNIT_SEL         0x3B
#define IMU_OPR_MODE         0x3D
#define IMU_PWR_MODE         0x3E
#define IMU_SYS_TRIGGER      0x3F
#define IMU_TEMP_SOURCE      0x40
#define IMU_AXIS_MAP_CONFIG  0x41
#define IMU_AXIS_MAP_SIGN    0x42
#define IMU_ACC_OFFSET_X_LSB 0x55
#define IMU_ACC_OFFSET_X_MSB 0x56
#define IMU_ACC_OFFSET_Y_LSB 0x57
#define IMU_ACC_OFFSET_Y_MSB 0x58
#define IMU_ACC_OFFSET_Z_LSB 0x59
#define IMU_ACC_OFFSET_Z_MSB 0x5A
#define IMU_MAG_OFFSET_X_LSB 0x5B
#define IMU_MAG_OFFSET_X_MSB 0x5C
#define IMU_MAG_OFFSET_Y_LSB 0x5D
#define IMU_MAG_OFFSET_Y_MSB 0x5E
#define IMU_MAG_OFFSET_Z_LSB 0x5F
#define IMU_MAG_OFFSET_Z_MSB 0x60
#define IMU_GYR_OFFSET_X_LSB 0x61
#define IMU_GYR_OFFSET_X_MSB 0x62
#define IMU_GYR_OFFSET_Y_LSB 0x63
#define IMU_GYR_OFFSET_Y_MSB 0x64
#define IMU_GYR_OFFSET_Z_LSB 0x65
#define IMU_GYR_OFFSET_Z_MSB 0x66
#define IMU_ACC_RADIUS_LSB   0x67
#define IMU_ACC_RADIUS_MSB   0x68
#define IMU_MAG_RADIUS_LSB   0x69
#define IMU_MAG_RADIUS_MSB   0x6A


/* IMU Registers Base of each reading : 
   used in Get functions for more readability
*/
#define  IMU_VECTOR_ACCELEROMETER   0x08   // Default: m/s²
#define  IMU_VECTOR_MAGNETOMETER    0x0E   // Default: uT
#define  IMU_VECTOR_GYROSCOPE       0x14   // Default: rad/s
#define  IMU_VECTOR_EULER           0x1A   // Default: degrees
#define  IMU_VECTOR_QUATERNION      0x20   // No units
#define  IMU_VECTOR_LINEARACCEL     0x28   // Default: m/s²
#define  IMU_VECTOR_GRAVITY         0x2E   // Default: m/s²

/* priv functions */
static void IMU_voidDelay(int time);
static HAL_StatusTypeDef IMU_enumWriteData(IMU_structCfg * Add_structCfg, uint8_t Copy_u8Reg, uint8_t Copy_u8Data);
static HAL_StatusTypeDef IMU_enumReadData(IMU_structCfg * Add_structCfg, uint8_t Copy_u8Reg, uint8_t * Add_u8Data,uint8_t Copy_u8Length);
static IMU_tstructVector IMU_structGetVector(IMU_structCfg * Add_structCfg ,uint8_t Copy_u8VecType);

/* Prescales [constants] */
const uint8_t accelScale = 100;
const uint8_t tempScale = 1;
const uint8_t angularRateScale = 16;
const uint8_t eulerScale = 16;
const uint8_t magScale = 16;
const uint16_t quaScale = (1<<14);    // 2^14

#endif  
