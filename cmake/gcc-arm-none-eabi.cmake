# Specify the target system and processor
set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Define the toolchain prefix
set(TOOLCHAIN_PREFIX "arm-none-eabi-")

# find path to gcc tools folder
# TODO: for some reason cmake was failing to detect arm-none-eabi- from the
# PATH so i patched it to do this lookup instead... 
find_program(ARM_GCC_EXECUTABLE ${TOOLCHAIN_PREFIX}gcc)
if(NOT ARM_GCC_EXECUTABLE)
  message(FATAL_ERROR "Could not find ${TOOLCHAIN_PREFIX}gcc in PATH")
endif()
message(STATUS "Found ARM GCC: ${ARM_GCC_EXECUTABLE}")

# Get the directory where the tools are located
get_filename_component(TOOLCHAIN_BIN_DIR ${ARM_GCC_EXECUTABLE} DIRECTORY)

# Now set the compilers and other tools using the directory we found
set(CMAKE_C_COMPILER       "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}gcc"  CACHE FILEPATH "ARM GCC Compiler" FORCE)
set(CMAKE_ASM_COMPILER     "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}gcc"  CACHE FILEPATH "ARM ASM Compiler" FORCE)
set(CMAKE_CXX_COMPILER     "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}g++"  CACHE FILEPATH "ARM G++ Compiler" FORCE)
set(CMAKE_LINKER           "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}g++"  CACHE FILEPATH "ARM G++ Linker" FORCE)
set(CMAKE_OBJCOPY          "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}objcopy" CACHE FILEPATH "ARM Objcopy" FORCE)
set(CMAKE_SIZE             "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}size"    CACHE FILEPATH "ARM Size" FORCE)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Set executable suffixes for consistency
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C   ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdata-sections -ffunction-sections")

set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32H733VGTX_FLASH.ld\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")
