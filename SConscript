# RT-Thread building script for bridge

import os
import rtconfig
Import ('RTT_ROOT')
from building import *

cwd = GetCurrentDir()

src = []
cpppath = []
cppdefines = ['CONFIG_PLATFORM_8711B', 'PLATFORM_RTTHREAD']

src += ['rtthread_patch/os/rtthread_service.c']
src += ['rtthread_patch/realtek/8711b/app_start.c']
src += ['rtthread_patch/realtek/8711b/rtl8710b_startup.c']
src += ['rtthread_patch/realtek/common/wifi/wifi_conf.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/api/wifi/rtw_wpa_supplicant/wpa_supplicant/wifi_eap_config.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/api/wifi/wifi_ind.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/api/wifi/wifi_promisc.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/api/wifi/wifi_util.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/analogin_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/dma_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/efuse_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/flash_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/gpio_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/gpio_irq_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/i2c_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/i2s_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/nfc_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/pinmap.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/pinmap_common.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/port_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/pwmout_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/rtc_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/serial_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/sleep.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/spi_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/sys_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/timer_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/us_ticker.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/us_ticker_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/wait_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b/wdt_api.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/os/os_dep/osdep_service.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/app/monitor/ram/monitor.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/app/monitor/ram/rtl_consol.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/cmsis/device/system_8195a.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/fwlib/ram_lib/rtl8710b_intfcfg.c']
src += ['sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/fwlib/ram_lib/rtl8710b_pinmapcfg.c']

cpppath += [cwd + '/rtthread_patch']
cpppath += [cwd + '/rtthread_patch/os']
cpppath += [cwd + '/rtthread_patch/realtek/8711b/include']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/api/platform']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/api']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/api/wifi']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/api/wifi/rtw_wpa_supplicant/src']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/drivers/wlan/realtek/include']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/drivers/wlan/realtek/src/osdep']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/drivers/wlan/realtek/wlan_ram_map/rom']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/network/ssl/polarssl-1.3.8/include']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/network/ssl/ssl_ram_map/rom']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/targets/hal/rtl8711b']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/hal']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/hal_ext']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/common/mbed/api']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/os/os_dep/include']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/os/freertos']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/os/freertos/freertos_v8.1.2/Source/include']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/cmsis']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/misc']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/fwlib/include']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/cmsis/device']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/swlib/rtl_lib']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/app/monitor/include']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/swlib/std_lib/include']
cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/swlib/std_lib/libc/rom/string']

if rtconfig.CROSS_TOOL == 'gcc':
	cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/os/freertos/freertos_v8.1.2/Source/portable/GCC/ARM_CM4F']
elif rtconfig.CROSS_TOOL == 'iar':
	cpppath += [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/os/freertos/freertos_v8.1.2/Source/portable/IAR/ARM_CM4F']

libs = ['lib_platform', 'lib_rtlstd', 'lib_wlan', 'lib_wps']

if rtconfig.CROSS_TOOL == 'gcc':
	libpath = [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/misc/bsp/lib/common/GCC']
	libs += ['lib_dct', '-lnosys']
elif rtconfig.CROSS_TOOL == 'iar':
	libpath = [cwd + '/sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0/component/soc/realtek/8711b/misc/bsp/lib/common/IAR']
	cppdefines += ['__IEEE_LITTLE_ENDIAN']

group = DefineGroup('Libraries', src, depend = [''], CPPPATH = cpppath, CPPDEFINES = cppdefines, LIBS = libs, LIBPATH = libpath)

Return('group')
