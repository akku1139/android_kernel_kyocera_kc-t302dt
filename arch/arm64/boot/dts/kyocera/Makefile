ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_MSM8917) += apq8017-HINOKI00-rev00-overlay.dtbo \
                               apq8017-HINOKI00-rev51-overlay.dtbo
dtbo-$(CONFIG_ARCH_SDM429)  += sdm429-HINOKI01-rev10-overlay.dtbo

apq8017-HINOKI00-rev00-overlay.dtbo-base := ../qcom/apq8017-pmi8950.dtb \
                                            ../qcom/msm8917-pmi8950.dtb
apq8017-HINOKI00-rev51-overlay.dtbo-base := ../qcom/apq8017-pmi8950.dtb \
                                            ../qcom/msm8917-pmi8950.dtb
sdm429-HINOKI01-rev10-overlay.dtbo-base  := ../qcom/sdm429.dtb

endif

always      := $(dtb-y)
subdir-y    := $(dts-dirs)
clean-files := *.dtb
