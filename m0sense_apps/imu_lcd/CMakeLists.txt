set(TARGET_REQUIRED_LIBS usb_stack shell usb_stdio)
set(BSP_COMMON_DIR ${CMAKE_SOURCE_DIR}/bsp/bsp_common)
set(TARGET_REQUIRED_PRIVATE_INCLUDE ${BSP_COMMON_DIR}/mcu_lcd)
set(TARGET_REQUIRED_SRCS ${sources} ${BSP_COMMON_DIR}/mcu_lcd/mcu_lcd.c
                                    ${BSP_COMMON_DIR}/mcu_lcd/st7735s.c 
                                    ${BSP_COMMON_DIR}/mcu_lcd/font.c )
set(mains main.c)
generate_bin()
