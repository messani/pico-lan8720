#ifndef LAN8720_H
#define LAN8720_H

#include "lwip/netif.h"
#include <hardware/dma.h>
#include <hardware/pio.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define LAN8720_PREAMBLE_SFD_SIZE 8
#define LAN8720_FRAME_SIZE 1518
#define LAN8720_CRC_SIZE sizeof(uint32_t)

struct lan8720
{
  uint32_t lastupdatetime;

  uint8_t mdio_pin;
  uint8_t mdc_pin;
  uint8_t rx_pin;
  uint8_t tx_pin;

  uint8_t phy_address;
  PIO pio;

  uint rx_sm;
  uint rx_sm_offset;
  uint rx_dma_channel;

  uint tx_sm;
  uint tx_sm_offset;
  uint tx_dma_channel;

  dma_channel_config rx_dma_channel_config;
  dma_channel_config tx_dma_channel_config;

  struct netif netif;
  uint8_t *mac_addr;

  struct lan8720_framedata *curr_packetdata;
  uint8_t txbuffer[LAN8720_PREAMBLE_SFD_SIZE + LAN8720_FRAME_SIZE + LAN8720_CRC_SIZE];

  bool rxfull;

  struct lan8720 *next;
};


void lan8720_init(struct lan8720 *lan8720, PIO pio, int sm, uint8_t mdio_pin, uint8_t rx_pin, uint8_t tx_pin);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // LAN8720_H