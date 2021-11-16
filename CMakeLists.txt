set(name LAN8720)
cmake_minimum_required(VERSION 3.13)

set(LWIP_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/lwip)

include(pico_sdk_import.cmake)

project(${name} C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#set(PICO_DEOPTIMIZED_DEBUG 1)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

pico_sdk_init()

add_library(pico_lwip INTERFACE)

target_sources(pico_lwip INTERFACE
    ${LWIP_PATH}/src/core/altcp.c
    ${LWIP_PATH}/src/core/altcp_alloc.c
    ${LWIP_PATH}/src/core/altcp_tcp.c
    ${LWIP_PATH}/src/core/def.c
    ${LWIP_PATH}/src/core/dns.c
    ${LWIP_PATH}/src/core/inet_chksum.c
    ${LWIP_PATH}/src/core/init.c
    ${LWIP_PATH}/src/core/ip.c
    ${LWIP_PATH}/src/core/mem.c
    ${LWIP_PATH}/src/core/memp.c
    ${LWIP_PATH}/src/core/netif.c
    ${LWIP_PATH}/src/core/pbuf.c
    ${LWIP_PATH}/src/core/raw.c
    ${LWIP_PATH}/src/core/stats.c
    ${LWIP_PATH}/src/core/sys.c
    ${LWIP_PATH}/src/core/tcp.c
    ${LWIP_PATH}/src/core/tcp_in.c
    ${LWIP_PATH}/src/core/tcp_out.c
    ${LWIP_PATH}/src/core/timeouts.c
    ${LWIP_PATH}/src/core/udp.c
    ${LWIP_PATH}/src/core/ipv4/acd.c
    ${LWIP_PATH}/src/core/ipv4/autoip.c
    ${LWIP_PATH}/src/core/ipv4/dhcp.c
    ${LWIP_PATH}/src/core/ipv4/etharp.c
    ${LWIP_PATH}/src/core/ipv4/icmp.c
    ${LWIP_PATH}/src/core/ipv4/igmp.c
    ${LWIP_PATH}/src/core/ipv4/ip4.c
    ${LWIP_PATH}/src/core/ipv4/ip4_addr.c
    ${LWIP_PATH}/src/core/ipv4/ip4_frag.c
    ${LWIP_PATH}/src/netif/ethernet.c

    ${LWIP_PATH}/src/apps/http/httpd.c
    ${LWIP_PATH}/src/apps/http/fs.c

    ${CMAKE_CURRENT_LIST_DIR}/src/lwip/sys_arch.c
)

target_include_directories(pico_lwip INTERFACE
	${LWIP_PATH}/src/include
	${CMAKE_CURRENT_LIST_DIR}/src/lwip
)

set(srcs
  src/main.c
  src/lan8720.c
  src/lan8720.h
  )

add_executable(${name} ${srcs})
pico_generate_pio_header(${name} ${CMAKE_CURRENT_SOURCE_DIR}/src/lan8720_rx.pio)
pico_generate_pio_header(${name} ${CMAKE_CURRENT_SOURCE_DIR}/src/lan8720_tx.pio)

target_link_libraries(pico_lwip INTERFACE pico_stdlib hardware_sync)
target_link_libraries(${name} pico_stdlib pico_mem_ops pico_unique_id hardware_sync pico_sync_mutex hardware_pio hardware_dma hardware_irq hardware_spi hardware_adc hardware_uart pico_lwip)

# enable usb output, disable uart output
pico_enable_stdio_usb(${name} 1)
pico_enable_stdio_uart(${name} 0)

pico_add_extra_outputs(${name})

