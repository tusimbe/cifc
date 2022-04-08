/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      mpu6000.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月5日星期二
 * \brief     MPU6000驱动
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月5日星期二
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "usbcfg.h"
#include "system.h"
#include "shell.h"
#include "scheduler.h"
#include "drv_spi.h"
#include "mpu6000.h"
#include "mpu6000_reg.h"
#include "lpf2.h"
#include "imu.h"
#include <string.h>
#include <stdlib.h>

#define MPU6000_LOW_FREQ    (2*MHZ)
#define MPU6000_HIGH_FREQ   (8*MHZ)

#define MPU_SAMPLE_SIZE       14
#define MPU_FIFO_BUFFER_LEN   16

#define M_PI      (3.141592653589793f)

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS     9.80665f
#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

#define MPU6000_SAMPLE_HZ      (1000)
#define MPU6000_SAMPLE_US      ((1000 * 1000) / MPU6000_SAMPLE_HZ)

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

static void *g_drvp;
static float _accel_scale;
static float _gyro_scale;
static int16_t _raw_temp;

bool _data_ready(void);

typedef struct stru_mpu6k_s {
    float temp_sensitivity; // degC/LSB
    float temp_zero;        // degC
    void *temp_lfp2;
    uint8_t fifo_buf[MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE];
} stru_mpu6k_t;

static stru_mpu6k_t mpu6k;

static inline float radians(float deg)
{
    return deg * DEG_TO_RAD;
}

void _fifo_reset(void);
void _read_fifo(void);
static bool _sample_thread_start(void);


int32_t mpu6k_init(uint32_t spi_id)
{   
    uint8_t chip_id;

    mpu6k.temp_lfp2 = lpf2_create(1000, 1);
    if (NULL == mpu6k.temp_lfp2) {
        printk("[%s, %d] lpf2 create failed.\r\n", __func__, __LINE__);
        return -1;
    }

    g_drvp = drv_spi_create(spi_id);
    if (!g_drvp) {
        printk("[%s, %d] spi driver create failed.\r\n", __func__, __LINE__);
        return -1;
    }

    drv_spi_start(g_drvp, MPU6000_LOW_FREQ);

    chip_id = drv_spi_register_read(g_drvp, MPUREG_WHOAMI);
    if (MPU_WHOAMI_6000 != chip_id) {
        printk("[%s, %d] spi driver create failed.\r\n", __func__, __LINE__);
        return -1;
    }
    
    // Chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {

        drv_spi_write_register(g_drvp, MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        scheduler_delay(100);
        
        // reset the device signal paths
        drv_spi_write_register(g_drvp, MPUREG_SIGNAL_PATH_RESET, 
                                MPUREG_SIGNAL_PATH_GYRO | MPUREG_SIGNAL_PATH_ACCEL | MPUREG_SIGNAL_PATH_TEMP);
        scheduler_delay(100);  // datasheet specifies a 100ms delay after signal path reset    

        /* disable i2c interface */
        drv_spi_write_register(g_drvp, MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_IF_DIS);

        // Wake up device and select GyroZ clock. Note that the
        // Invensense starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        drv_spi_write_register(g_drvp, MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        scheduler_delay(5);

        // check it has woken up
        if (drv_spi_register_read(g_drvp, MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }

        scheduler_delay(10);
        if (_data_ready()) {
            break;
        }
    }

    if (tries == 5) {
        printk("[%s, %d] boot mpu6000 failed.\r\n", __func__, __LINE__);
        return -1;
    }

    _fifo_reset();

    mpu6k.temp_zero = 36.53f;
    mpu6k.temp_sensitivity = 1.0f/340;

    drv_spi_set_speed(g_drvp, MPU6000_LOW_FREQ);

    // set sample rate to 1000Hz and apply a software filter
    // In this configuration, the gyro sample rate is 8kHz
    drv_spi_write_register(g_drvp, MPUREG_SMPLRT_DIV, 0);
    scheduler_delay(1);

    // Gyro scale 2000º/s
    drv_spi_write_register(g_drvp, MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);
    scheduler_delay(1);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    uint8_t product_id = drv_spi_register_read(g_drvp, MPUREG_PRODUCT_ID);

    if ((product_id == MPU6000ES_REV_C4) ||
         (product_id == MPU6000ES_REV_C5) ||
         (product_id == MPU6000_REV_C4)   ||
         (product_id == MPU6000_REV_C5)) {
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        drv_spi_write_register(g_drvp, MPUREG_ACCEL_CONFIG, 1<<3);
        _accel_scale = GRAVITY_MSS / 4096.f;
        _gyro_scale = (radians(1) / 16.4f);
    } else {
        // Accel scale 16g (2048 LSB/g)
        drv_spi_write_register(g_drvp, MPUREG_ACCEL_CONFIG, 3<<3);
        _accel_scale = GRAVITY_MSS / 2048.f;
        _gyro_scale = (radians(1) / 16.4f);
    }
    
    scheduler_delay(1);
    
    // configure interrupt to fire when new data arrives
    drv_spi_write_register(g_drvp, MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    scheduler_delay(1);

    drv_spi_set_speed(g_drvp, MPU6000_HIGH_FREQ);

    if (!_sample_thread_start()) {
        printk("[%s, %d] mpu6000 thread start failed.\r\n", __func__, __LINE__);
        return -1;
    }

    return 0;
}

/*
  fetch temperature in order to detect FIFO sync errors
*/
bool _check_raw_temp(int16_t t2)
{
    if (abs(t2 - _raw_temp) < 400) {
        // cached copy OK
        return true;
    }
    uint8_t trx[2];
    if (drv_spi_read_registers(g_drvp, MPUREG_TEMP_OUT_H, trx, 2)) {
        _raw_temp = int16_val(trx, 0);
    }
    return (abs(t2 - _raw_temp) < 800);
}

bool _data_ready(void)
{
    uint8_t status = drv_spi_register_read(g_drvp, MPUREG_INT_STATUS);
    return ((status & BIT_RAW_RDY_INT) != 0);
}

void _fifo_reset(void)
{
    uint8_t user_ctrl;

    drv_spi_set_speed(g_drvp, MPU6000_LOW_FREQ);
    user_ctrl = drv_spi_register_read(g_drvp, MPUREG_USER_CTRL);
    user_ctrl &= ~(BIT_USER_CTRL_FIFO_RESET | BIT_USER_CTRL_FIFO_EN);
    drv_spi_write_register(g_drvp, MPUREG_FIFO_EN, 0);
    drv_spi_write_register(g_drvp, MPUREG_USER_CTRL, user_ctrl);
    drv_spi_write_register(g_drvp, MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_RESET);
    drv_spi_write_register(g_drvp, MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_EN);
    drv_spi_write_register(g_drvp, MPUREG_FIFO_EN, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
                    BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN);
    scheduler_delay(1);
    drv_spi_set_speed(g_drvp, MPU6000_HIGH_FREQ);
}

bool _accumulate(uint8_t *samples, uint8_t n_samples)
{
    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + MPU_SAMPLE_SIZE * i;
        float accel[3], gyro[3];
        
        accel[0] = int16_val(data, 0) * _accel_scale;
        accel[1] = int16_val(data, 1) * _accel_scale;
        accel[2] = int16_val(data, 2) * _accel_scale;

        int16_t t2 = int16_val(data, 3);

        if (!_check_raw_temp(t2)) {
            printk("temp reset IMU (%d %d).\r\n", _raw_temp, t2);
            _fifo_reset();
            return false;
        }

        float temp = t2 * mpu6k.temp_sensitivity + mpu6k.temp_zero;
        
        gyro[0] = int16_val(data, 4) * _gyro_scale;
        gyro[1] = int16_val(data, 5) * _gyro_scale;
        gyro[2] = int16_val(data, 6) * _gyro_scale;

        imu_notify_new_accel_raw_sample(accel, gyro, 0);
        imu_notify_new_gyro_raw_sample(gyro, 0);

        lpf2_apply(mpu6k.temp_lfp2, temp);
#if 0
        _rotate_and_correct_accel(_accel_instance, accel);
        _rotate_and_correct_gyro(_gyro_instance, gyro);



        _temp_filtered = _temp_filter.apply(temp);
#endif
    }
    return true;
}

static THD_FUNCTION(mpu6k_sample_thd, arg) {

    (void)arg;
    while (true) {
        chThdSleepMicroseconds(MPU6000_SAMPLE_US);
        _read_fifo();
    }
}

void _read_fifo(void)
{
    uint8_t n_samples;
    uint16_t bytes_read;
    uint8_t *rx = mpu6k.fifo_buf;
    bool need_reset = false;

    if (!drv_spi_read_registers(g_drvp, MPUREG_FIFO_COUNTH, rx, 2)) {
        printk("read fifo counter failed.\r\n");
        goto check_registers;
    }

    bytes_read = uint16_val(rx, 0);
    //printk("read fifo counter %u.\r\n", bytes_read);
    n_samples = bytes_read / MPU_SAMPLE_SIZE;

    if (n_samples == 0) {
        /* Not enough data in FIFO */
        goto check_registers;
    }

    /*
      testing has shown that if we have more than 32 samples in the
      FIFO then some of those samples will be corrupt. It always is
      the ones at the end of the FIFO, so clear those with a reset
      once we've read the first 24. Reading 24 gives us the normal
      number of samples for fast sampling at 400Hz

      On I2C with the much lower clock rates we need a lower threshold
      or we may never catch up
     */
    if (n_samples > 32) {
        need_reset = true;
        n_samples = 24;
    }
    
    while (n_samples > 0) {
        uint8_t n = MIN(n_samples, MPU_FIFO_BUFFER_LEN);

        if (!drv_spi_read_registers(g_drvp, MPUREG_FIFO_R_W, rx, n * MPU_SAMPLE_SIZE)) {
            goto check_registers;
        }

        _accumulate(rx, n);
        n_samples -= n;
    }

    if (need_reset) {
        printk("fifo reset n_samples %u.\r\n", bytes_read/MPU_SAMPLE_SIZE);
        _fifo_reset();
    }
    
check_registers:
    return;
}

static bool _sample_thread_start(void)
{
    thread_t* thread_ctx;

	thread_ctx = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1024), "mpu6k",
				      SCHE_SPI_PRIORITY, mpu6k_sample_thd, NULL);
    if (thread_ctx == NULL) {
        return false;
    }

    return true;
}

void mpu6k_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	int32_t cmd_type;
	uint8_t chip_id;

	if (argc != 1) {
		shellUsage(chp, "mpu6k [id | init | fifo]");
		return;
	} else {
		if (strcmp(argv[0], "id") == 0) {
			cmd_type = 1;
		} else if (strcmp(argv[0], "init") == 0){
			cmd_type = 2;
		} else if (strcmp(argv[0], "fifo") == 0){
			cmd_type = 3;
		} else {
			cmd_type = -1;
			shellUsage(chp, "mpu6k [id | init | fifo]");
			return;
		}
	}

    if (cmd_type == 1) {
        drv_spi_set_speed(g_drvp, MPU6000_LOW_FREQ);
        chip_id = drv_spi_register_read(g_drvp, MPUREG_WHOAMI);
        printk("mpu6k chip id:0x%x.\r\n", chip_id);
        chip_id = drv_spi_register_read(g_drvp, MPUREG_PRODUCT_ID);
        printk("mpu6k product id:0x%x.\r\n", chip_id);
        drv_spi_set_speed(g_drvp, MPU6000_HIGH_FREQ);
    } else if (cmd_type == 2) {
        int ret = mpu6k_init(0);
        printk("mpu6k_init ret %d", ret);
    } else if (cmd_type == 3) {
        _read_fifo();
    }

    return;
}



