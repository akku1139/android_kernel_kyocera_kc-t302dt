#
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2019 KYOCERA Corporation
#

ccflags-y += -I$(src)			# needed for trace events

obj-y					+= ion/

obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= lowmemorykiller.o
obj-$(CONFIG_ANDROID_VSOC)		+= vsoc.o
obj-$(CONFIG_ANDROID_CRASH_REASON)		+= crash_reason.o
obj-$(CONFIG_ANDROID_KCJLOG)		+= kcjlog.o

CFLAGS_kcjlog.o += -DBUILD_DISPLAY_ID='"$(RESETLOG_BUILD_DISPLAY_ID)"'
CFLAGS_kcjlog.o += -DPRODUCT_MODEL_NAME='"$(RESETLOG_PRODUCT_MODEL_NAME)"'
