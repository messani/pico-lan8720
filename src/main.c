#include "config.h"
#include "eventloop.h"
#include "lan8720.h"
#include "lwip/apps/httpd.h"
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

  struct eventloop eventloop;
  eventloop_init(&eventloop);

  struct lan8720 lan8720;

  lan8720_init(&lan8720, &eventloop, LAN8720_PIO, LAN8720_SM, LAN8720_PIN_MDIO, LAN8720_PIN_RX, LAN8720_PIN_TX);
  httpd_init();
  
  eventloop_run(&eventloop);
}