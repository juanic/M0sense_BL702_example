set(BSP_COMMON_DIR ${CMAKE_SOURCE_DIR}/bsp/bsp_common)
set(TARGET_REQUIRED_SRCS
	${CMAKE_CURRENT_LIST_DIR}/ble_peripheral_tp_server.c
	${CMAKE_CURRENT_LIST_DIR}/bl_irq.c)
set(TARGET_REQUIRED_LIBS freertos ble mbedtls)
set(mains main.c)
set(LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/bl702_flash_ble.ld)
generate_bin()

