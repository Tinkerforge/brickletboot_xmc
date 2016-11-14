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
ADD_EXECUTABLE(${PROJECT_NAME}.elf ${SOURCES})
SET_TARGET_PROPERTIES(${PROJECT_NAME}.elf PROPERTIES LINKER_LANGUAGE C)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}.elf -lc_nano)

# define compile flags
SET_TARGET_PROPERTIES(${PROJECT_NAME}.elf PROPERTIES COMPILE_FLAGS
	"${DEBUG} -mcpu=${MCU} -std=gnu99  -Wall --specs=nano.specs -mlong-calls -ffunction-sections -fdata-sections -O${OPTIMIZATION_LEVEL}"
)

#define linker flags
SET_TARGET_PROPERTIES(${PROJECT_NAME}.elf PROPERTIES LINK_FLAGS
	"-nostartfiles -mcpu=${MCU} -Wl,--gc-sections -T\"${PROJECT_SOURCE_DIR}/src/bricklib2/linker_script/xmc1100_brickletboot.ld\" "
)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME}.elf POST_BUILD COMMAND 
                   ${CMAKE_OBJCOPY} -S -O binary 
                   ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME}.elf POST_BUILD COMMAND 
                   ${CMAKE_SIZE} 
                   ${OBJECT} ${PROJECT_NAME}.elf)
                   
ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME}.elf POST_BUILD COMMAND 
                   ${PROJECT_SOURCE_DIR}/src/bricklib2/bootloader/crc32.py
                   ${PROJECT_NAME}.bin > ${PROJECT_NAME}-checksum.bin)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME}.elf POST_BUILD COMMAND 
                   cat
                   ${PROJECT_NAME}.bin ${PROJECT_NAME}-checksum.bin > ${PROJECT_NAME}-with-checksum.bin)
                  
ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME}.elf POST_BUILD COMMAND 
                   ${CMAKE_OBJDUMP} -D
                   ${PROJECT_NAME}.elf > statistics.objdump)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME}.elf POST_BUILD COMMAND 
                   ${CMAKE_NM} --print-size --size-sort --radix=d
                   ${PROJECT_NAME}.elf > statistics.function_sizes)
 
# add preprocessor defines
ADD_DEFINITIONS(-D${CHIP} -D__${CHIP_FAMILY}__ -DNOSTARTFILES -D__SKIP_LIBC_INIT_ARRAY)
