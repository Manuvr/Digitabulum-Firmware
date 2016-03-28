
vpath %.c STM32F7xx_HAL_Driver/Src

# Includes
INCLUDES  = -ISTM32F7xx_HAL_Driver/Inc/ -I../Inc/ -ICMSIS/Device/ST/STM32F7xx/Include/ -ICMSIS/Include/
INCLUDES += -IUSB_Device/Core/Inc -IUSB_Device/Class/CDC/Inc

# Sources
SRCS  = stm32f7xx_hal_adc.c
SRCS += stm32f7xx_hal_adc_ex.c
SRCS += stm32f7xx_hal_cec.c
SRCS += stm32f7xx_hal_can.c
SRCS += stm32f7xx_hal.c
SRCS += stm32f7xx_hal_cortex.c
SRCS += stm32f7xx_hal_crc.c
SRCS += stm32f7xx_hal_crc_ex.c
SRCS += stm32f7xx_hal_cryp.c
SRCS += stm32f7xx_hal_cryp_ex.c
SRCS += stm32f7xx_hal_dac.c
SRCS += stm32f7xx_hal_dac_ex.c
SRCS += stm32f7xx_hal_dcmi.c
SRCS += stm32f7xx_hal_dcmi_ex.c
SRCS += stm32f7xx_hal_dma2d.c
SRCS += stm32f7xx_hal_dma.c
SRCS += stm32f7xx_hal_dma_ex.c
SRCS += stm32f7xx_hal_eth.c
SRCS += stm32f7xx_hal_flash.c
SRCS += stm32f7xx_hal_flash_ex.c
SRCS += stm32f7xx_hal_gpio.c
SRCS += stm32f7xx_hal_hash.c
SRCS += stm32f7xx_hal_hash_ex.c
SRCS += stm32f7xx_hal_hcd.c
SRCS += stm32f7xx_hal_i2c.c
SRCS += stm32f7xx_hal_i2c_ex.c
SRCS += stm32f7xx_hal_i2s.c
SRCS += stm32f7xx_hal_irda.c
SRCS += stm32f7xx_hal_iwdg.c
SRCS += stm32f7xx_hal_lptim.c
SRCS += stm32f7xx_hal_ltdc.c
SRCS += stm32f7xx_hal_msp_template.c
SRCS += stm32f7xx_hal_nand.c
SRCS += stm32f7xx_hal_nor.c
SRCS += stm32f7xx_hal_pcd.c
SRCS += stm32f7xx_hal_pcd_ex.c
SRCS += stm32f7xx_hal_pwr.c
SRCS += stm32f7xx_hal_pwr_ex.c
SRCS += stm32f7xx_hal_qspi.c
SRCS += stm32f7xx_hal_rcc.c
SRCS += stm32f7xx_hal_rcc_ex.c
SRCS += stm32f7xx_hal_rng.c
SRCS += stm32f7xx_hal_rtc.c
SRCS += stm32f7xx_hal_rtc_ex.c
SRCS += stm32f7xx_hal_sai.c
SRCS += stm32f7xx_hal_sai_ex.c
SRCS += stm32f7xx_hal_sd.c
SRCS += stm32f7xx_hal_sdram.c
SRCS += stm32f7xx_hal_smartcard.c
SRCS += stm32f7xx_hal_smartcard_ex.c
SRCS += stm32f7xx_hal_spdifrx.c
SRCS += stm32f7xx_hal_spi.c
SRCS += stm32f7xx_hal_sram.c
SRCS += stm32f7xx_hal_tim.c
SRCS += stm32f7xx_hal_tim_ex.c
SRCS += stm32f7xx_hal_uart.c
SRCS += stm32f7xx_hal_usart.c
SRCS += stm32f7xx_hal_wwdg.c
SRCS += stm32f7xx_ll_fmc.c
SRCS += stm32f7xx_ll_sdmmc.c
SRCS += stm32f7xx_ll_usb.c

SRCS += tm_stm32_buffer.c
SRCS += tm_stm32_gpio.c
SRCS += tm_stm32_usb_device.c
SRCS += tm_stm32_usb.c
SRCS += tm_stm32_usb_device_cdc.c

SRCS += USB_Device/Core/Src/usbd_conf.c
SRCS += USB_Device/Core/Src/usbd_core.c
SRCS += USB_Device/Core/Src/usbd_ctlreq.c
SRCS += USB_Device/Core/Src/usbd_ioreq.c
SRCS += USB_Device/Class/CDC/Src/usbd_cdc.c
SRCS += USB_Device/Class/CDC/Src/usbd_cdc_if.c

#LOCAL_FLAGS += -DUSB_USE_FS -DUSB_USE_HS -DUSB_USE_DEVICE

ifeq ($(ENABLE_USB_VCP),)
	LOCAL_FLAGS += -DUSB_USE_FS -DUSB_USE_HS -DUSB_USE_DEVICE
endif

OBJS = $(SRCS:.c=.o)
LIBNAME = libstdperiph.a

all: $(LIBNAME)

%.o : %.c
	$(CC) $(CFLAGS) $(LOCAL_FLAGS) $(INCLUDES) -c -o $@ $^

$(LIBNAME): $(OBJS)
	$(AR) -rvs $(OUTPUT_PATH)/$@ $(OBJS)

clean:
	rm -f $(OBJS) $(LIBNAME)
