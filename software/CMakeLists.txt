CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(${CMAKE_CURRENT_SOURCE_DIR}/cmake_config_custom.txt)
	include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_config_custom.txt)
else()
	include(${CMAKE_CURRENT_SOURCE_DIR}/cmake_config_default.txt)
endif()

SET(PROJECT_NAME brickletboot)
SET(CHIP_FAMILY XMC1)
SET(MCU cortex-m0\ -mthumb)
PROJECT(${PROJECT_NAME})

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
#SET(OPTIMIZATION_LEVEL 0)
SET(DEBUG "-g -ggdb")

SET(OPTIMIZATION_LEVEL s)
#SET(DEBUG "")

SET(CMAKE_BUILD_TYPE None)
ENABLE_LANGUAGE(C ASM)

INCLUDE_DIRECTORIES(
	"${PROJECT_SOURCE_DIR}/src/"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/xmclib/XMCLib/inc/"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/xmclib/CMSIS/Include/"
)

if(CHIP STREQUAL "XMC1100_Q024x0016")
INCLUDE_DIRECTORIES(
	"${PROJECT_SOURCE_DIR}/src/bricklib2/xmclib/CMSIS/Infineon/XMC1100_series/Include/"
)
elseif(CHIP STREQUAL "XMC1403_Q048x0064")
INCLUDE_DIRECTORIES(
	"${PROJECT_SOURCE_DIR}/src/bricklib2/xmclib/CMSIS/Infineon/XMC1400_series/Include/"
)
endif()


# find source files
SET(SOURCES
	"${PROJECT_SOURCE_DIR}/src/main.c"
	"${PROJECT_SOURCE_DIR}/src/boot.c"
	"${PROJECT_SOURCE_DIR}/src/bootloader_spitfp.c"
	"${PROJECT_SOURCE_DIR}/src/tfp_common.c"
	"${PROJECT_SOURCE_DIR}/src/firmware_entry.c"
	
	"${PROJECT_SOURCE_DIR}/src/bricklib2/hal/uartbb/uartbb.c"
#	"${PROJECT_SOURCE_DIR}/src/bricklib2/hal/system_timer/system_timer.c"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/hal/startup/startup_xmc1.c"
	
	"${PROJECT_SOURCE_DIR}/src/bricklib2/bootloader/bootloader_xmc_irq.c"
	
	"${PROJECT_SOURCE_DIR}/src/bricklib2/protocols/tfp/tfp.c"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/utility/ringbuffer.c"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/utility/pearson_hash.c"

	"${PROJECT_SOURCE_DIR}/src/bricklib2/xmclib/XMCLib/src/xmc_gpio.c"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/xmclib/XMCLib/src/xmc1_gpio.c"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/xmclib/XMCLib/src/xmc_spi.c"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/xmclib/XMCLib/src/xmc_usic.c"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/xmclib/XMCLib/src/xmc1_scu.c"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/xmclib/XMCLib/src/xmc1_flash.c"
)

if(CHIP STREQUAL "XMC1100_Q024x0016")
SET(SOURCES "${SOURCES}"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/hal/startup/startup_xmc1100.S"
)
elseif(CHIP STREQUAL "XMC1403_Q048x0064")
SET(SOURCES "${SOURCES}"
	"${PROJECT_SOURCE_DIR}/src/bricklib2/hal/startup/startup_xmc1400.S"
)
endif()

MESSAGE(STATUS "\nFound following source files:\n ${SOURCES}\n")

# define executable
include(${CMAKE_CURRENT_SOURCE_DIR}/src/bricklib2/cmake/configs/config_define_executable.txt)

# define standard compile and linker flags
SET(LINKER_SCRIPT_NAME xmc1100_brickletboot.ld)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/bricklib2/cmake/configs/config_comcu_add_standard_flags.txt)

# add custom build commands
include(${CMAKE_CURRENT_SOURCE_DIR}/src/bricklib2/cmake/configs/config_comcu_add_standalone_custom_commands.txt)
 
# add preprocessor defines
include(${CMAKE_CURRENT_SOURCE_DIR}/src/bricklib2/cmake/configs/config_xmc1_add_preprocessor_defines.txt)
