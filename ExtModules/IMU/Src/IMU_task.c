#include "IMU.h"
#include "COMM.h"

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
    IMU_tstructVector imu_test_vector = {};
    COMM_can_message_t can_message = {};
    can_message.id = COMM_CAN_ID_IMU;
    can_message.size = 4;
    while (1)
    {
        imu_test_vector = IMU_structGetVectorAccelerometer(imu);
        can_message.data = (int32_t)imu_test_vector.x;
        COMM_can_enqueue(&can_message);
        vTaskDelay(1000);
    }
}
