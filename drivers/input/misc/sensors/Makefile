# This software is contributed or developed by KYOCERA Corporation.
# (C) 2015 KYOCERA Corporation
# (C) 2019 KYOCERA Corporation
############################################################################
#
# Makefile
#
# Copyright (C) 2011-2012 ALPS ELECTRIC CO., LTD. All Rights Reserved.
#
# This software is licensed under the terms of the GNU General Public
# License version 2, as published by the Free Software Foundation, and
# may be copied, distributed, and modified under those terms.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
############################################################################

# Debug Option
#CFLAGS_MODULE += -DALPS_IDEV_DEBUG
#CFLAGS_MODULE += -DALPS_MAG_DEBUG
#CFLAGS_MODULE += -DALPS_ACC_DEBUG
#CFLAGS_MODULE += -DCONFIG_LIGHT_DEBUG

#ccflags-y := -mfloat-abi=softfp -mfpu=vfp

#ccflags-y += -Iinclude/linux
ccflags-y := -Iinclude/linux

obj-$(CONFIG_INPUT_ALPS_INPUT) += alps-input.o
obj-$(CONFIG_INPUT_HSCDTD) += hscdtd007a_i2c.o
ifeq ($(CONFIG_INPUT_SENSOR_ACCELEROMETER),y)
obj-$(CONFIG_INPUT_SENSOR_ACC_U2DH)		+= u2dh_i2c.o
obj-$(CONFIG_INPUT_SENSOR_ACC_MXC400XXC)	+= MXC400xXC_i2c.o
endif
obj-$(CONFIG_ROHM_RPR0521) += rohm_rpr0521_i2c.o
obj-$(CONFIG_SENSOR_COMMON) += sensor_api.o
obj-$(CONFIG_SENSOR_COMMON) += sensor_core.o
obj-$(CONFIG_INPUT_SENSOR_LIGHT) += als_sensor.o
obj-$(CONFIG_INPUT_SENSOR_PROXIMITY) += ps_sensor.o
