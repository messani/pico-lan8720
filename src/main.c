#include "config.h"
#include "lan8720.h"
#include "lwip/apps/httpd.h"
#include "lwip/timeouts.h"
#include <hardware/clocks.h>
#include <hardware/spi.h>
#include <hardware/uart.h>
#include <hardware/vreg.h>
#include <pico/malloc.h>
#include <pico/stdlib.h>
#include <string.h>

int main()
{
  // it is necessary to overclock RPi to
  // at least 250MHz
  set_sys_clock_khz(CPU_FREQ / KHZ, true);

  stdio_init_all();

  struct lan8720 lan8720;

  lan8720_init(&lan8720, LAN8720_PIO, LAN8720_SM, LAN8720_PIN_MDIO, LAN8720_PIN_RX, LAN8720_PIN_TX);
  httpd_init();
  
  for (;;)
    sys_check_timeouts();
}