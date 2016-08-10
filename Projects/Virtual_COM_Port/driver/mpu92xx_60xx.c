#include "mpu92xx_60xx.h"
#include "mltypes.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "eMPL_outputs.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "i2c.h"
#include "log.h"
#include "stm32f10x.h"
#include <math.h>

void run_self_test(void);

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif

struct int_param_s int_param;

void set_exit_int(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
    /* Configure EXTI4 line */

    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

int inv_mpu_init(void) {
    inv_error_t result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
#ifdef COMPASS_ENABLED
    unsigned short compass_fsr;
#endif

    I2C_init();
    set_exit_int();
    st_hw_msdelay(20);
    result = mpu_init(&int_param);
    if (result) {
        MPL_LOGI("Could not initialize gyro.%d\n",result);
        return -1;
    }

    result = inv_init_mpl();
    if (result) {
        MPL_LOGI("Could not initialize MPL.\n");
        return -1;
    }

    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();

    inv_enable_fast_nomot();
    inv_enable_gyro_tc();

#ifdef COMPASS_ENABLED
    /* Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
#endif

    inv_enable_eMPL_outputs();

    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED) {
        while (1) {
            MPL_LOGI("Not authorized.\n");
        }
    }
    if (result) {
        MPL_LOGI("Could not start the MPL.\n");
    }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    /* Read back configuration in case it was set improperly. */
//    mpu_get_sample_rate(&gyro_rate);
//    mpu_get_gyro_fsr(&gyro_fsr);
//    mpu_get_accel_fsr(&accel_fsr);
//#ifdef COMPASS_ENABLED
//    mpu_get_compass_fsr(&compass_fsr);
//#endif
    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
//    inv_set_gyro_sample_rate(1000000L / gyro_rate);
//    inv_set_accel_sample_rate(1000000L / gyro_rate);
//#ifdef COMPASS_ENABLED
//    /* The compass rate is independent of the gyro and accel rates. As long as
//     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
//     * fusion algorithm's compass correction gain will work properly.
//     */
//    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
//#endif
//    /* Set chip-to-body orientation matrix.
//     * Set hardware units to dps/g's/degrees scaling factor.
//     */
    inv_set_gyro_orientation_and_scale(
	    inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
	    (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
	    inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
	    (long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
	    inv_orientation_matrix_to_scalar(compass_pdata.orientation),
	    (long)compass_fsr<<15);
#endif

    dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
//    
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		       DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
		       DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    //    dmp_set_fifo_rate(20);

    //    run_self_test();
    mpu_set_dmp_state(1);
    log_printf("mpu init OK\r\n");
    return 0;
}

void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 1);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
	MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif
    }
    else {
            if (!(result & 0x1))
                MPL_LOGI("Gyro failed.\n");
            if (!(result & 0x2))
                MPL_LOGI("Accel failed.\n");
            if (!(result & 0x4))
                MPL_LOGI("Compass failed.\n");
     }

}

#define q30  1073741824.0f

static uint8_t int_flag = 0;
void get_senser(void) {
    short accel_data[3],gyro_data[3],sensors;
    float  accel[3];
    float  gyro[3];
    unsigned char more;
    long quat[4];
    unsigned long timestamp;
    uint32_t timest;
    unsigned short accel_sens = 0;
    float gyro_sens = 0;
    float Pitch,Roll,Yaw;
    static float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

    if(int_flag) {
        int_flag = 0;
        log_printf("\r\n");
//        mpu_get_gyro_reg(gyro_data,&timestamp);
//        mpu_get_accel_reg(accel_data,&timestamp);
        dmp_read_fifo(gyro_data,accel_data,quat,&timestamp,&sensors,&more);
        log_printf("t:%d\r\n",timestamp);
        mpu_get_accel_sens(&accel_sens);
        accel[0] = ((float)accel_data[0]/accel_sens);
	accel[1] = ((float)accel_data[1]/accel_sens);
	accel[2] = ((float)accel_data[2]/accel_sens);

        mpu_get_gyro_sens(&gyro_sens);
        gyro[0] = gyro_data[0]/gyro_sens;
        gyro[1] = gyro_data[1]/gyro_sens;
        gyro[2] = gyro_data[2]/gyro_sens;

        if(sensors & INV_WXYZ_QUAT) {
            q0=quat[0] / q30;
            q1=quat[1] / q30;
            q2=quat[2] / q30;
            q3=quat[3] / q30;
            Pitch  = asin(2 * q1 * q3 - 2 * q0* q2)* 57.3; // pitch
            Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
            Yaw =  atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
            log_printf("pi:%3.5f,ro:%3.5f,ya:%3.5f\r\n",Pitch,Roll,Yaw);
        }
        log_printf("a X:%2.5f,Y:%2.5f,Z:%2.5f\r\n",accel[0],accel[1],accel[2]);
        log_printf("g X:%3.5f,Y:%3.5f,Z:%3.5f\r\n",gyro[0],gyro[1],gyro[2]);

        //       MPL_LOGI("\r\n====");
        //        MPL_LOGI("t:%d",get_timer());
        //        MPL_LOGI("accel X:%d,Y:%d,Z:%d",accel_data[0],accel_data[1],accel_data[2]);
        //        MPL_LOGI("gyro X:%d,Y:%d,Z:%d",gyro_data[0],gyro_data[1],gyro_data[2]);

//        log_printf("quat :q1%ld ,q2%ld, q3%ld ,q4%ld\r\n",quat[0],quat[1],quat[2],quat[3]);
//        timest = get_timer();
//        log_printf("t2%d\r\n",timest);
    }

}


void EXTI9_5_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
        int_flag = 1;
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}
