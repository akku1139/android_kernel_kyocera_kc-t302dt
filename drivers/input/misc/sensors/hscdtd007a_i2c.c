/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */
/* drivers/input/misc/hscdtd007a_i2c.c
 *
 * GeoMagneticField device driver for I2C (HSCDTD007/HSCDTD008)
 *
 * Copyright (C) 2011-2014 ALPS ELECTRIC CO., LTD. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef ALPS_MAG_DEBUG
#define DEBUG 1
#endif

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/version.h>

#define I2C_RETRIES		5

#define HSCDTD_DRIVER_NAME	"hscdtd007a"
#define HSCDTD_LOG_TAG		"[HSCDTD], "

#define HSCDTD_CHIP_ID		0x1511

#define HSCDTD_STBA		0x0C
#define HSCDTD_INFO		0x0D
#define HSCDTD_XOUT		0x10
#define HSCDTD_YOUT		0x12
#define HSCDTD_ZOUT		0x14
#define HSCDTD_XOUT_H		0x11
#define HSCDTD_XOUT_L		0x10
#define HSCDTD_YOUT_H		0x13
#define HSCDTD_YOUT_L		0x12
#define HSCDTD_ZOUT_H		0x15
#define HSCDTD_ZOUT_L		0x14

#define HSCDTD_STATUS		0x18
#define HSCDTD_CTRL1		0x1B
#define HSCDTD_CTRL2		0x1C
#define HSCDTD_CTRL3		0x1D
#define HSCDTD_CTRL4		0x1E

#define HSCDTD_TCS_TIME		10000	/* Measure temp. of every 10 sec */
#define HSCDTD_DATA_ACCESS_NUM	6
#define HSCDTD_3AXIS_NUM	3
#define HSCDTD_INITIALL_DELAY	20
#define STBB_OUTV_THR		3838

#define HSCDTD_DELAY(us)	usleep_range(us, us)

/* Self-test resiter value */
#define HSCDTD_ST_REG_DEF	0x55
#define HSCDTD_ST_REG_PASS	0xAA
#define HSCDTD_ST_REG_X		0x01
#define HSCDTD_ST_REG_Y		0x02
#define HSCDTD_ST_REG_Z		0x04
#define HSCDTD_ST_REG_XYZ	0x07

/* Self-test error number */
#define HSCDTD_ST_OK		0x00
#define HSCDTD_ST_ERR_I2C	0x01
#define HSCDTD_ST_ERR_INIT	0x02
#define HSCDTD_ST_ERR_1ST	0x03
#define HSCDTD_ST_ERR_2ND	0x04
#define HSCDTD_ST_ERR_VAL	0x10
#define HSCDTD_ST_ERR_VAL_X	(HSCDTD_ST_REG_X | HSCDTD_ST_ERR_VAL)
#define HSCDTD_ST_ERR_VAL_Y	(HSCDTD_ST_REG_Y | HSCDTD_ST_ERR_VAL)
#define HSCDTD_ST_ERR_VAL_Z	(HSCDTD_ST_REG_Z | HSCDTD_ST_ERR_VAL)

#define HSCDTD_X_AXIS_COEF	1
#define HSCDTD_Y_AXIS_COEF	1
#define HSCDTD_Z_AXIS_COEF	1

static struct i2c_client *client_hscdtd;

static atomic_t flgEna;
static atomic_t delay;
static atomic_t flgSuspend;

struct magsns_regulator_data {
    struct regulator* vdd_reg;
    uint32_t min_uV;
    uint32_t max_uV;
    uint32_t on_load_uA;
    uint32_t off_load_uA;
};

static struct magsns_regulator_data reg_data;

/*--------------------------------------------------------------------------
 * i2c read/write function
 *--------------------------------------------------------------------------*/
static int hscdtd_i2c_read(char *rxData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= client_hscdtd->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxData,
		},
		{
			.addr	= client_hscdtd->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxData,
		 },
	};

	do {
		err = i2c_transfer(client_hscdtd->adapter,
			msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&client_hscdtd->adapter->dev,
			"read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int hscdtd_i2c_write(char *txData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= client_hscdtd->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txData,
		},
	};

	do {
		err = i2c_transfer(client_hscdtd->adapter,
			msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&client_hscdtd->adapter->dev,
			"write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

/*--------------------------------------------------------------------------
 * hscdtd function
 *--------------------------------------------------------------------------*/
static int hscdtd_soft_reset(void)
{
	int rc;
	u8 buf[2];

	dev_dbg(&client_hscdtd->adapter->dev,
		HSCDTD_LOG_TAG "Software Reset\n");

	buf[0] = HSCDTD_CTRL3;
	buf[1] = 0x80;
	rc = hscdtd_i2c_write(buf, 2);
	HSCDTD_DELAY(5000);
	return rc;
}

int hscdtd_get_magnetic_field_data(int *xyz)
{
	int err = -1;
	int i;
	u8  sx[HSCDTD_DATA_ACCESS_NUM];

	if (atomic_read(&flgSuspend) == 1)
		return err;

	sx[0] = HSCDTD_XOUT;
	err = hscdtd_i2c_read(sx, HSCDTD_DATA_ACCESS_NUM);
	if (err < 0)
		return err;
	for (i = 0; i < HSCDTD_3AXIS_NUM; i++)
		xyz[i] = (int) ((short)((sx[2*i+1] << 8) | (sx[2*i])));

	dev_dbg(&client_hscdtd->adapter->dev,
		HSCDTD_LOG_TAG "org x:%d,y:%d,z:%d\n", xyz[0], xyz[1], xyz[2]);

	xyz[0] *= HSCDTD_X_AXIS_COEF;
	xyz[1] *= HSCDTD_Y_AXIS_COEF;
	xyz[2] *= HSCDTD_Z_AXIS_COEF;

	dev_dbg(&client_hscdtd->adapter->dev,
		HSCDTD_LOG_TAG "x:%d,y:%d,z:%d\n", xyz[0], xyz[1], xyz[2]);

	return err;
}
EXPORT_SYMBOL(hscdtd_get_magnetic_field_data);

void hscdtd_activate(int flgatm, int flg, int dtime)
{
	u8 buf[2];
	int err = -1;

	if (flg != 0)
		flg = 1;

    if (flg == 1) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
        err = regulator_set_load(reg_data.vdd_reg, reg_data.on_load_uA);
        if( err < 0 ) {
            dev_err(&client_hscdtd->adapter->dev, "regulator_set_load fail. err=%d\n", err);
            return;
        }
#else
        err = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.on_load_uA);
        if( err < 0 ) {
            dev_err(&client_hscdtd->adapter->dev, "regulator_set_optimum_mode fail. err=%d\n", err);
            return;
        }
#endif /* LINUX_VERSION_CODE */
        usleep_range(1000,1000);
    }

    if (!flg) {
        buf[0] = HSCDTD_CTRL1;
        buf[1] = 0x0A;
        hscdtd_i2c_write(buf, 2);
    } else {
		buf[0] = HSCDTD_CTRL1;
		if (dtime >= 100) {
			buf[1] = 0x88;
		}
		else if (dtime >= 50) {
			buf[1] = 0x90;
		}
		else {
			buf[1] = 0x98;
		}
		hscdtd_i2c_write(buf, 2);
		HSCDTD_DELAY(10);

		buf[0] = HSCDTD_CTRL4;
		buf[1] = 0x90;
		hscdtd_i2c_write(buf, 2);
	}

	if (flgatm) {
		atomic_set(&flgEna, flg);
		atomic_set(&delay, dtime);
	}

    if (flg == 0) {
        usleep_range(1000,1000);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
        err = regulator_set_load(reg_data.vdd_reg, reg_data.off_load_uA);
        if( err < 0 ) {
            dev_err(&client_hscdtd->adapter->dev, "regulator_set_load fail. err=%d\n", err);
            return;
        }
#else
        err = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.off_load_uA);
        if( err < 0 ) {
            dev_err(&client_hscdtd->adapter->dev, "regulator_set_optimum_mode fail. err=%d\n", err);
            return;
        }
#endif /* LINUX_VERSION_CODE */
    }
}
EXPORT_SYMBOL(hscdtd_activate);

int hscdtd_get_hardware_data(int *xyz)
{
	int ret = 0;

	if (atomic_read(&flgSuspend) == 1)
		return -1;
	hscdtd_activate(0, 1, 10);
	HSCDTD_DELAY(5000);
	ret = hscdtd_get_magnetic_field_data(xyz);
	hscdtd_activate(0, atomic_read(&flgEna), atomic_read(&delay));
	return ret;
}
EXPORT_SYMBOL(hscdtd_get_hardware_data);

int hscdtd_self_test_A(void)
{
	int rc = HSCDTD_ST_OK;
	u8 sx[2], cr1[1];

	if (atomic_read(&flgSuspend) == 1)
		return -1;

	/* Control resister1 backup  */
	cr1[0] = HSCDTD_CTRL1;
	if (hscdtd_i2c_read(cr1, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&client_hscdtd->adapter->dev,
		HSCDTD_LOG_TAG "Control resister1 value, %02X\n", cr1[0]);


	/* Move active mode (normal state)  */
	sx[0] = HSCDTD_CTRL1;
	sx[1] = 0x88;
	if (hscdtd_i2c_write(sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(10);

	/* Get inital value of self-test-A register  */
	sx[0] = HSCDTD_STBA;
	hscdtd_i2c_read(sx, 1);
	sx[0] = HSCDTD_STBA;
	if (hscdtd_i2c_read(sx, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&client_hscdtd->adapter->dev,
		HSCDTD_LOG_TAG "STBA reg. initial value, %02X\n", sx[0]);
	if (sx[0] != HSCDTD_ST_REG_DEF) {
		dev_err(&client_hscdtd->adapter->dev, HSCDTD_LOG_TAG
			"Err: Initial value of STBA reg. is %02X\n", sx[0]);
		rc = HSCDTD_ST_ERR_INIT;
		goto err_STBA;
	}

	/* do self-test-A  */
	sx[0] = HSCDTD_CTRL3;
	sx[1] = 0x10;
	if (hscdtd_i2c_write(sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(3000);

	/* Get 1st value of self-test-A register  */
	sx[0] = HSCDTD_STBA;
	if (hscdtd_i2c_read(sx, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&client_hscdtd->adapter->dev,
		HSCDTD_LOG_TAG "STBA reg. 1st value, %02X\n", sx[0]);
	if (sx[0] != HSCDTD_ST_REG_PASS) {
		dev_err(&client_hscdtd->adapter->dev, HSCDTD_LOG_TAG
			"Err: 1st value of STBA reg. is %02X\n", sx[0]);
		rc = HSCDTD_ST_ERR_1ST;
		goto err_STBA;
	}
	HSCDTD_DELAY(3000);

	/* Get 2nd value of self-test-A register  */
	sx[0] = HSCDTD_STBA;
	if (hscdtd_i2c_read(sx, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&client_hscdtd->adapter->dev,
		HSCDTD_LOG_TAG "STBA reg. 2nd value, %02X\n", sx[0]);
	if (sx[0] != HSCDTD_ST_REG_DEF) {
		dev_err(&client_hscdtd->adapter->dev, HSCDTD_LOG_TAG
			"Err: 2nd value of STBA reg. is %02X\n", sx[0]);
		rc = HSCDTD_ST_ERR_2ND;
	}

err_STBA:
	/* Resume */
	sx[0] = HSCDTD_CTRL1;
	sx[1] = cr1[0];
	if (hscdtd_i2c_write(sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(10);

	return rc;
}
EXPORT_SYMBOL(hscdtd_self_test_A);

int hscdtd_self_test_B(void)
{
	int rc = HSCDTD_ST_OK, xyz[3];

	if (atomic_read(&flgSuspend) == 1)
		return -1;

	/* Measurement sensor value */
	if (hscdtd_get_hardware_data(xyz))
		return HSCDTD_ST_ERR_I2C;

	/* Check output value */
	if ((xyz[0] <= -STBB_OUTV_THR) || (xyz[0] >= STBB_OUTV_THR))
		rc |= HSCDTD_ST_REG_X;
	if ((xyz[1] <= -STBB_OUTV_THR) || (xyz[1] >= STBB_OUTV_THR))
		rc |= HSCDTD_ST_REG_Y;
	if ((xyz[2] <= -STBB_OUTV_THR) || (xyz[2] >= STBB_OUTV_THR))
		rc |= HSCDTD_ST_REG_Z;
	if (rc)
		rc |= HSCDTD_ST_ERR_VAL;

	return rc;
}
EXPORT_SYMBOL(hscdtd_self_test_B);

static int hscdtd_register_init(void)
{
	int ret = 0;
	u8  buf[2];

	dev_dbg(&client_hscdtd->adapter->dev,
		HSCDTD_LOG_TAG "%s\n", __func__);

	if (hscdtd_soft_reset()) {
		dev_err(&client_hscdtd->adapter->dev, HSCDTD_LOG_TAG
		    "Err. Can't execute software reset");
		return -1;
	}

    buf[0] = HSCDTD_CTRL4;
    buf[1] = 0x90;
    hscdtd_i2c_write(buf, 2);

	return ret;
}


/*--------------------------------------------------------------------------
 * suspend/resume function
 *--------------------------------------------------------------------------*/
static int hscdtd_suspend(struct device *dev)
{
	dev_dbg(dev, HSCDTD_LOG_TAG "%s\n", __func__);
	atomic_set(&flgSuspend, 1);
	hscdtd_activate(0, 0, atomic_read(&delay));
	return 0;
}

static int hscdtd_resume(struct device *dev)
{
	dev_dbg(dev, HSCDTD_LOG_TAG "%s\n", __func__);
	atomic_set(&flgSuspend, 0);
	hscdtd_activate(0, atomic_read(&flgEna), atomic_read(&delay));
	return 0;
}



/*--------------------------------------------------------------------------
 * i2c device
 *--------------------------------------------------------------------------*/
static int hscdtd_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
    int err = -1;
	dev_dbg(&client->adapter->dev,
		HSCDTD_LOG_TAG "%s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->adapter->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	client_hscdtd = client;

	atomic_set(&flgEna, 0);
	atomic_set(&delay, HSCDTD_INITIALL_DELAY);
	atomic_set(&flgSuspend, 0);

    of_property_read_u32(client->dev.of_node, "mag-vdd-min-voltage", &reg_data.min_uV);
    of_property_read_u32(client->dev.of_node, "mag-vdd-max-voltage", &reg_data.max_uV);
    of_property_read_u32(client->dev.of_node, "mag-vdd-on-load-current", &reg_data.on_load_uA);
    of_property_read_u32(client->dev.of_node, "mag-vdd-off-load-current", &reg_data.off_load_uA);
    dev_info(&client->adapter->dev, "regulator min_uV = %d, max_uV = %d, on_load_uA = %d, off_load_uA = %d\n",
        reg_data.min_uV, reg_data.max_uV, reg_data.on_load_uA, reg_data.off_load_uA);

    reg_data.vdd_reg = regulator_get(&client->dev, "mag-vdd");
    if( IS_ERR(reg_data.vdd_reg) ) {
        dev_err(&client->adapter->dev, "failed regulator_get \n");
        return -EIO;
    }

    err = regulator_set_voltage(reg_data.vdd_reg, reg_data.min_uV, reg_data.max_uV);
    if( err ) {
        dev_err(&client->adapter->dev, "regulator_set_voltage fail. err=%d\n", err);
        return -EIO;
    }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    err = regulator_set_load(reg_data.vdd_reg, reg_data.on_load_uA);
    if( err < 0 ) {
        dev_err(&client->adapter->dev, "regulator_set_load fail. err=%d\n", err);
        return -EIO;
    }
#else
    err = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.on_load_uA);
    if( err < 0 ) {
        dev_err(&client->adapter->dev, "regulator_set_optimum_mode fail. err=%d\n", err);
        return -EIO;
    }
#endif /* LINUX_VERSION_CODE */

    err = regulator_enable(reg_data.vdd_reg);
    if( err ) {
        dev_err(&client->adapter->dev, "regulator_enable fail. err=%d\n", err);
        return -EIO;
    }
    usleep_range(3000,3000);

	if (hscdtd_register_init()) {
		dev_err(&client->adapter->dev,
			"failed to initialize sensor\n");
		return -EIO;
	}

	dev_info(&client->adapter->dev,
		"detected HSCDTD007/008 geomagnetic field sensor\n");

	return 0;
}

static int hscdtd_remove(struct i2c_client *client)
{
	dev_dbg(&client->adapter->dev,
		HSCDTD_LOG_TAG "%s\n", __func__);
	hscdtd_activate(0, 0, atomic_read(&delay));
	client_hscdtd = NULL;
	return 0;
}


/*--------------------------------------------------------------------------
 * module
 *--------------------------------------------------------------------------*/
static const struct i2c_device_id ALPS_id[] = {
	{ HSCDTD_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ALPS_id);

static struct of_device_id hscdtd_of_match[] = {
	{ .compatible = HSCDTD_DRIVER_NAME,},
	{ },
};
MODULE_DEVICE_TABLE(of, hscdtd_of_match);

static SIMPLE_DEV_PM_OPS(hscdtd_pm, hscdtd_suspend, hscdtd_resume);

static struct i2c_driver hscdtd_driver = {
	.probe		= hscdtd_probe,
	.remove		= hscdtd_remove,
	.id_table	= ALPS_id,
	.driver		= {
		.name	= HSCDTD_DRIVER_NAME,
		.of_match_table = hscdtd_of_match,
		.pm	= &hscdtd_pm,
	},
};

static int __init hscdtd_init(void)
{
	pr_debug(HSCDTD_LOG_TAG "%s\n", __func__);
	return i2c_add_driver(&hscdtd_driver);
}

static void __exit hscdtd_exit(void)
{
	pr_debug(HSCDTD_LOG_TAG "%s\n", __func__);
	i2c_del_driver(&hscdtd_driver);
}

module_init(hscdtd_init);
module_exit(hscdtd_exit);

MODULE_DESCRIPTION("Alps HSCDTD Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
