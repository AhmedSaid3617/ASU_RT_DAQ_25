#include "IMU.h"
#include "COMM.h"
#include "config.h"

// IMU_structCfg* imu1;

static IMU_structCfg * imu;

void IMU_init(IMU_structCfg *imu_handle)
{
    imu = imu_handle;
    /* 01 - Init */
    IMU_configure(imu_handle);

    /* 02 - Mapping */
    IMU_tAxisMap loc_structMapping = {
        .x = IMU_AXIS_X,
        .y = IMU_AXIS_Y,
        .z = IMU_AXIS_Z,
        .x_sign = IMU_AXIS_SIGN_POSITIVE,
        .y_sign = IMU_AXIS_SIGN_POSITIVE,
        .z_sign = IMU_AXIS_SIGN_POSITIVE};
    IMU_voidSetAxisMap(imu_handle, &loc_structMapping);
}

void IMU_task()
{
    COMM_can_message_t can_message = {};

    COMM_message_IMU_t imu_acceleration_message = {};
    COMM_message_IMU_t imu_angles_message = {};

    IMU_tstructVector acceleration_vector = {};
    IMU_tstructVector angles_vector = {};

    can_message.size = 6;
    while (1)
    {
        // TODO: needs so much testing.
        acceleration_vector = IMU_structGetVectorAccelerometer(imu);
        angles_vector = IMU_structGetVectorEuler(imu);

        can_message.id = COMM_CAN_ID_IMU_ACCEL;
        imu_acceleration_message.x = (int16_t)(acceleration_vector.x * CONFIG_IMU_ACCELERATION_ACCURACY);
        imu_acceleration_message.y = (int16_t)(acceleration_vector.y * CONFIG_IMU_ACCELERATION_ACCURACY);
        imu_acceleration_message.z = (int16_t)(acceleration_vector.z * CONFIG_IMU_ACCELERATION_ACCURACY);
        can_message.data = *((uint64_t*)(&imu_acceleration_message));
        COMM_can_enqueue(&can_message);

        can_message.id = COMM_CAN_ID_IMU_ANGLE;
        imu_angles_message.x = (int16_t)(angles_vector.x * CONFIG_IMU_ANGLES_ACCURACY);
        imu_angles_message.y = (int16_t)(angles_vector.y * CONFIG_IMU_ANGLES_ACCURACY);
        imu_angles_message.z = (int16_t)(angles_vector.z * CONFIG_IMU_ANGLES_ACCURACY);
        can_message.data = *((uint64_t*)(&imu_angles_message));
        COMM_can_enqueue(&can_message);

        vTaskDelay(1000);
    }
}
