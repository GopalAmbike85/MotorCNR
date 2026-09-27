cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	ARM_MATH_CM4 
	USE_HAL_DRIVER 
	STM32G431xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/G4xx/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/DSP/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/app_freertos.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/motorcontrol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/speed_torq_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/pwm_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mc_api.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mc_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mc_config_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mcp_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mc_tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mc_tasks_foc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/pwm_curr_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/regular_conversion_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mc_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mc_interface.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32_mc_common_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32g4xx_mc_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mc_parameters.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/hf_registers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/sync_registers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mcp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/usart_aspep_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mc_configuration_registers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/aspep.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/mc_app_hooks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32g4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32g4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32g4xx_hal_timebase_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32g431xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/system_stm32g4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cordic.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
)

# Drivers Midllewares


set(FreeRTOS_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)
set(MotorControl_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/bus_voltage_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/circle_limitation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/digital_output.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/enc_align_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/encoder_speed_pos_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/trajectory_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/ntc_temperature_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/open_loop.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/pid_regulator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/pqd_motor_power_measurement.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/ramp_ext_mngr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/revup_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/speed_pos_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/virtual_bus_voltage_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/virtual_speed_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_1_g4xx_pwm_curr_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/mcpa.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
    FreeRTOS	MotorControl	
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create FreeRTOS static library
add_library(FreeRTOS OBJECT)
target_sources(FreeRTOS PRIVATE ${FreeRTOS_Src})
target_link_libraries(FreeRTOS PUBLIC stm32cubemx)

# Create MotorControl static library
add_library(MotorControl OBJECT)
target_sources(MotorControl PRIVATE ${MotorControl_Src})
target_link_libraries(MotorControl PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
