/**********************************************************/
/* Author  : Mohamed Abdel Hamid                          */
/* Date    : 31 / 10 / 2023                               */
/* Version : V01                                          */
/**********************************************************/

#include "stm32f4xx.h"
#include <stdio.h>
#include "string.h"
#include "IMU.h"
#include "IMU_priv.h"
#include <stdint.h>

/**
 * @brief Writes data to a register of an IMU device.
 *
 * This function writes data to a specified register of an IMU device using I2C communication.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @param Copy_u8Reg The register address to write the data to.
 * @param Copy_u8Data The data to be written to the register.
 * @return HAL_StatusTypeDef The status of the write operation (HAL_OK if successful).
 */
HAL_StatusTypeDef IMU_enumWriteData(IMU_structCfg * Add_structCfg, uint8_t Copy_u8Reg, uint8_t Copy_u8Data){
    // Prepare the data to be transmitted
    uint8_t Loc_u8TxData[2] = {Copy_u8Reg, Copy_u8Data};

    // Transmit data via I2C
    uint8_t Loc_u8Status = HAL_I2C_Master_Transmit(Add_structCfg->I2cId, Add_structCfg->u8Address << 1,
                                                    Loc_u8TxData, sizeof(Loc_u8TxData), HAL_MAX_DELAY);

    // Return the status of the write operation
    return Loc_u8Status;
}

/**
 * @brief Reads data from a register of an IMU device.
 *
 * This function reads data from a specified register of an IMU device using I2C communication.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @param Copy_u8Reg The register address to read the data from.
 * @param Add_u8Data Pointer to the buffer where the read data will be stored.
 * @param Copy_u8Length The number of bytes to read.
 * @return HAL_StatusTypeDef The status of the read operation (HAL_OK if successful).
 */
HAL_StatusTypeDef IMU_enumReadData(IMU_structCfg * Add_structCfg, uint8_t Copy_u8Reg, uint8_t * Add_u8Data,uint8_t Copy_u8Length){
    uint8_t Loc_u8States;

    // Transmit register address to initiate read operation
    HAL_I2C_Master_Transmit(Add_structCfg->I2cId, Add_structCfg->u8Address << 1, &Copy_u8Reg, 1, HAL_MAX_DELAY);

    // Receive data from the IMU device
    Loc_u8States = HAL_I2C_Master_Receive(Add_structCfg->I2cId, Add_structCfg->u8Address << 1, Add_u8Data, Copy_u8Length, HAL_MAX_DELAY);

    // Return the status of the read operation
    return Loc_u8States;
}


/**
 * @brief Resets an IMU device.
 *
 * This function resets an IMU device by writing a reset command to a specific register.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @return HAL_StatusTypeDef The status of the reset operation (HAL_OK if successful).
 */
HAL_StatusTypeDef IMU_enumReset(IMU_structCfg * Add_structCfg) {
    uint8_t Loc_u8Status;

    // Send reset command to the IMU device
    Loc_u8Status = IMU_enumWriteData(Add_structCfg, IMU_SYS_TRIGGER, 0x20);

    // Delay for reset completion
    HAL_Delay(700);

    // Return the status of the reset operation
    return Loc_u8Status;
}

/**
 * @brief Initializes an IMU device.
 *
 * This function initializes an IMU device by performing the following steps:
 * 1. Reset the device.
 * 2. Set the page register.
 * 3. Set the internal oscillator.
 * 4. Set the operation mode.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 */
void IMU_voidInit(IMU_structCfg * Add_structCfg) {
    /* 1- Reset */
    IMU_enumReset(Add_structCfg);

    /* 2- Set Page */
    IMU_enumWriteData(Add_structCfg, IMU_PAGE_ID, 0);

    /* 3- Set internal oscillator */
    IMU_enumWriteData(Add_structCfg, IMU_SYS_TRIGGER, 0x0);

    /* 4- Set Operation Mode */
    IMU_enumWriteData(Add_structCfg, IMU_OPR_MODE, Add_structCfg->u8OperationMode);
    HAL_Delay(10);
}


/**
 * @brief Retrieves a vector from the IMU device.
 *
 * This function retrieves a vector of a specified type from the IMU device.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @param Copy_u8VecType The type of vector to retrieve.
 * @return IMU_tstructVector The retrieved vector.
 */
IMU_tstructVector IMU_structGetVector(IMU_structCfg * Add_structCfg, uint8_t Copy_u8VecType) {

    /* Buffer to receive data */
    uint8_t Loc_u8Buffer[8] = {0};

    /* Variable to set Loc_doubleScale */
    double Loc_doubleScale = 1;

    /* RX Vector */
    IMU_tstructVector xyz = {.x = 10, .y = 10, .z = 10};

    /* 1- Read Data */
    uint8_t Loc_u8State;
    Loc_u8State = IMU_enumReadData(Add_structCfg, Copy_u8VecType, Loc_u8Buffer, 8);

    /* 2- Set Scale */
    if (Copy_u8VecType == IMU_VECTOR_MAGNETOMETER) {
        Loc_doubleScale = magScale;
    } else if (Copy_u8VecType == IMU_VECTOR_ACCELEROMETER ||
               Copy_u8VecType == IMU_VECTOR_LINEARACCEL || Copy_u8VecType == IMU_VECTOR_GRAVITY) {
        Loc_doubleScale = accelScale;
    } else if (Copy_u8VecType == IMU_VECTOR_GYROSCOPE) {
        Loc_doubleScale = angularRateScale;
    } else if (Copy_u8VecType == IMU_VECTOR_EULER) {
        Loc_doubleScale = eulerScale;
    } else if (Copy_u8VecType == IMU_VECTOR_QUATERNION) {
        Loc_doubleScale = quaScale;
    }

    /* 3- Determine x, y, z */
    xyz.x = (int16_t)((Loc_u8Buffer[1] << 8) | Loc_u8Buffer[0]) / Loc_doubleScale;
    xyz.y = (int16_t)((Loc_u8Buffer[3] << 8) | Loc_u8Buffer[2]) / Loc_doubleScale;
    xyz.z = (int16_t)((Loc_u8Buffer[5] << 8) | Loc_u8Buffer[4]) / Loc_doubleScale;

    return xyz;
}


/**
 * @brief Retrieves calibration data from the IMU device.
 *
 * This function retrieves calibration data from the IMU device, including offsets and radii.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @return IMU_structCalibData The retrieved calibration data.
 */
IMU_structCalibData IMU_structGetCalibrationData(IMU_structCfg * Add_structCfg) {
    IMU_structCalibData calibData;
    uint8_t buffer[22];

    // Read calibration data from the IMU device
    IMU_enumReadData(Add_structCfg, IMU_ACC_OFFSET_X_LSB, buffer, 22);

    // Copy calibration data into calibData structure
    memcpy(&calibData.offset.accel, buffer, 6);
    memcpy(&calibData.offset.mag, buffer + 6, 6);
    memcpy(&calibData.offset.gyro, buffer + 12, 6);
    memcpy(&calibData.radius.accel, buffer + 18, 2);
    memcpy(&calibData.radius.mag, buffer + 20, 2);

    return calibData;
}


/**
 * @brief Sets calibration data for the IMU device.
 *
 * This function sets calibration data for the IMU device by writing zeros to the corresponding registers.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 */
void IMU_voidSetCalibrationData(IMU_structCfg * Add_structCfg) {
    for (uint8_t i = 0; i < 22; i++) {
        // TODO: Implement multibyte write
        IMU_enumWriteData(Add_structCfg, IMU_ACC_OFFSET_X_LSB + i, 0x00);
    }
}

/**
 * @brief Sets axis mapping for the IMU device.
 *
 * This function sets axis mapping for the IMU device based on the provided axis mapping configuration.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @param axis Pointer to the axis mapping configuration.
 */
void IMU_voidSetAxisMap(IMU_structCfg * Add_structCfg, IMU_tAxisMap * axis) {
    uint8_t loc_u8AxisRemap = (axis->z << 4) | (axis->y << 2) | (axis->x);
    uint8_t loc_u8AxisMapSign = (axis->x_sign << 2) | (axis->y_sign << 1) | (axis->z_sign);
    IMU_enumWriteData(Add_structCfg, IMU_AXIS_MAP_CONFIG, loc_u8AxisRemap);
    IMU_enumWriteData(Add_structCfg, IMU_AXIS_MAP_SIGN, loc_u8AxisMapSign);
}

/**
 * @brief Retrieves accelerometer vector from the IMU device.
 *
 * This function retrieves the accelerometer vector from the IMU device.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @return IMU_tstructVector The accelerometer vector.
 */
IMU_tstructVector IMU_structGetVectorAccelerometer(IMU_structCfg * Add_structCfg) {
    return IMU_structGetVector(Add_structCfg, IMU_VECTOR_ACCELEROMETER);
}

/**
 * @brief Retrieves magnetometer vector from the IMU device.
 *
 * This function retrieves the magnetometer vector from the IMU device.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @return IMU_tstructVector The magnetometer vector.
 */
IMU_tstructVector IMU_structGetVectorMagnetometer(IMU_structCfg * Add_structCfg) {
    return IMU_structGetVector(Add_structCfg, IMU_VECTOR_MAGNETOMETER);
}

/**
 * @brief Retrieves gyroscope vector from the IMU device.
 *
 * This function retrieves the gyroscope vector from the IMU device.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @return IMU_tstructVector The gyroscope vector.
 */
IMU_tstructVector IMU_structGetVectorGyroscope(IMU_structCfg * Add_structCfg) {
    return IMU_structGetVector(Add_structCfg, IMU_VECTOR_GYROSCOPE);
}

/**
 * @brief Retrieves Euler angles vector from the IMU device.
 *
 * This function retrieves the Euler angles vector from the IMU device.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @return IMU_tstructVector The Euler angles vector.
 */
IMU_tstructVector IMU_structGetVectorEuler(IMU_structCfg * Add_structCfg) {
    return IMU_structGetVector(Add_structCfg, IMU_VECTOR_EULER);
}

/**
 * @brief Retrieves linear acceleration vector from the IMU device.
 *
 * This function retrieves the linear acceleration vector from the IMU device.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @return IMU_tstructVector The linear acceleration vector.
 */
IMU_tstructVector IMU_structGetVectorLinearAccel(IMU_structCfg * Add_structCfg) {
    return IMU_structGetVector(Add_structCfg, IMU_VECTOR_LINEARACCEL);
}

/**
 * @brief Retrieves gravity vector from the IMU device.
 *
 * This function retrieves the gravity vector from the IMU device.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @return IMU_tstructVector The gravity vector.
 */
IMU_tstructVector IMU_structGetVectorGravity(IMU_structCfg * Add_structCfg) {
    return IMU_structGetVector(Add_structCfg, IMU_VECTOR_GRAVITY);
}

/**
 * @brief Retrieves quaternion vector from the IMU device.
 *
 * This function retrieves the quaternion vector from the IMU device.
 *
 * @param Add_structCfg Pointer to the configuration structure of the IMU device.
 * @return IMU_tstructVector The quaternion vector.
 */
IMU_tstructVector IMU_structGetVectorQuaternion(IMU_structCfg * Add_structCfg) {
    return IMU_structGetVector(Add_structCfg, IMU_VECTOR_QUATERNION);
}

