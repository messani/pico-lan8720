#include "lan8720.h"
#include "config.h"
#include "lan8720_rx.pio.h"
#include "lan8720_tx.pio.h"
#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/sys.h"
#include <assert.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <pico/critical_section.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>
#include <string.h>

#define _BV(n) (1 << (n))

#define LAN8720_REG0_SPEED100        0x2000
#define LAN8720_REG0_AUTONEG         0x1000
#define LAN8720_REG0_FULLDUPLEX      0x0100

#define LAN8720_REG4_TXABLE          0x0080
#define LAN8720_REG4_10FULLDUPLEX    0x0040
#define LAN8720_REG4_10ABLE          0x0020

#define LAN8720_RX_FRAME_COUNT 2
#define LAN8720_UPDATE_INTERVAL      1000

#define LAN8720_CRC_POLY 0xedb88320

//#define ENABLE_LOG
#ifdef ENABLE_LOG
#define LOG_FMT(fmt, ...) do { printf(fmt "\n", __VA_ARGS__); } while (false)
#define LOG(text) do { printf(text); } while (false)
#else // ENABLE_LOG
#define LOG_FMT(fmt, ...) do { } while (false)
#define LOG(text) do { } while (false)
#endif // ENABLE_LOG

struct critical_section sys_critical_section;

struct lan8720_framedata
{
  struct pbuf_custom p;
  struct lan8720 *lan8720;
  uint16_t size;
  uint8_t data[LAN8720_FRAME_SIZE];
};

LWIP_MEMPOOL_DECLARE(lan8720_rx_pool, LAN8720_RX_FRAME_COUNT, sizeof(struct lan8720_framedata), "rx_pool");

static struct lan8720 *g_lan8720_first;

volatile static int dbg;

static bool lan8720_rx_trystart(struct lan8720 *lan8720);

void lan8720_link_callback(struct netif *netif)
{
    LOG_FMT("netif link status changed %s\n", netif_is_link_up(netif) ? "up" : "down");
}

void lan8720_status_callback(struct netif *netif)
{
    LOG_FMT("netif status changed %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
}

static uint32_t lan8720_frame_crc(const uint8_t *data, int length)
{
  uint32_t crc = 0xffffffff;  /* Initial value. */

  while(--length >= 0) 
  {
    uint8_t current_octet = *data++;

    for (int bit = 8; --bit >= 0; current_octet >>= 1) 
    {
      if ((crc ^ current_octet) & 1) 
      {
        crc >>= 1;
        crc ^= LAN8720_CRC_POLY;
      } 
      else
      {
        crc >>= 1;
      }
    }
  }

  return ~crc;
}

static void lan8720_free_framedata(struct lan8720_framedata *framedata)
{
  struct lan8720 *lan8720 = framedata->lan8720;

  LWIP_MEMPOOL_FREE(lan8720_rx_pool, framedata);
  if (lan8720->rxfull)
  {
    if (lan8720_rx_trystart(lan8720))
    {
      lan8720_rx_init(lan8720->pio, lan8720->rx_sm, lan8720->rx_sm_offset, lan8720->rx_pin);
      pio_sm_set_enabled(lan8720->pio, lan8720->rx_sm, true);
    }
  }
}

static void lan8720_free_pbuf(struct pbuf *p)
{
  struct lan8720_framedata *framedata = (struct lan8720_framedata*)p; // same address
  lan8720_free_framedata(framedata);
}

static void lan8720_rx_process_frame(void *userdata)
{
  struct lan8720_framedata *framedata = (struct lan8720_framedata*)userdata;
  struct lan8720 *lan8720 = framedata->lan8720;

  if (framedata->size > sizeof(uint32_t))
  {
    uint32_t payload_size = framedata->size - sizeof(uint32_t);
    uint32_t crc = lan8720_frame_crc(framedata->data, payload_size);
    uint8_t *frame_crc = framedata->data + payload_size;
    if (memcmp(&crc, frame_crc, sizeof(uint32_t)) == 0)
    {
      framedata->p.custom_free_function = lan8720_free_pbuf;
      struct pbuf *p = pbuf_alloced_custom(
        PBUF_RAW,
        framedata->size,
        PBUF_REF,
        &framedata->p,
        framedata->data,
        sizeof(framedata->data)
      );

      err_t res = lan8720->netif.input(p, &lan8720->netif);
      if (res != ERR_OK)
      {
        pbuf_free(p);
      }
      else
      {
        LOG("valid packet received\n");
      }
    }
    else
    {
      lan8720_free_framedata(framedata);
      LOG("invalid packet received (crc)\n");
    }
  }
  else
  {
    lan8720_free_framedata(framedata);
    LOG("invalid packet received (short)\n");
  }
}

static void lan8720_rx_start(struct lan8720_framedata *framedata)
{
  struct lan8720 *lan8720 = framedata->lan8720;
  
  dma_channel_set_write_addr(lan8720->rx_dma_channel, framedata->data, false);
  dma_channel_set_trans_count(lan8720->rx_dma_channel, sizeof(framedata->data), true);
}

static bool lan8720_rx_trystart(struct lan8720 *lan8720)
{
  struct lan8720_framedata *framedata = (struct lan8720_framedata *)LWIP_MEMPOOL_ALLOC(lan8720_rx_pool);
  
  if (framedata == NULL)
    return false;

  framedata->lan8720 = lan8720;
  lan8720->curr_packetdata = framedata;
  lan8720_rx_start(framedata);
  return true;
}

static void lan8720_rx_finish(struct lan8720 *lan8720)
{
  dma_channel_abort(lan8720->rx_dma_channel);

  struct lan8720_framedata *framedata = lan8720->curr_packetdata;
  assert(framedata != NULL);

  lan8720->curr_packetdata = NULL;

  // get address of write pointer of DMA
  uint8_t *wr = (uint8_t *)dma_channel_hw_addr(lan8720->rx_dma_channel)->write_addr;
  // frame length is calculated by difference of dma_write pointer and frame begin pointer
  framedata->size = (uint16_t)(wr - framedata->data);
  sys_timeout(0, lan8720_rx_process_frame, framedata);
}

static void lan8720_rx_handle(struct lan8720 *lan8720)
{
  if (lan8720->pio->ints0 & (PIO_IRQ0_INTS_SM0_BITS << lan8720->rx_sm))
  {
    lan8720_rx_finish(lan8720);

    if (!lan8720_rx_trystart(lan8720))
    {
      // STOP PIO!
      pio_sm_set_enabled(lan8720->pio, lan8720->rx_sm, false);
      lan8720->rxfull = true;
    }

    pio_interrupt_clear(lan8720->pio, lan8720->rx_sm);
  }
}

static void lan8720_rx_irq_handler() 
{
  struct lan8720 *lan8720 = g_lan8720_first;
  while (lan8720 != NULL)
  {
    lan8720_rx_handle(lan8720);
    lan8720 = lan8720->next;
  }
}

static err_t lan8720_tx_start(struct netif *netif, struct pbuf *p)
{
  struct lan8720 *lan8720 = (struct lan8720 *)netif->state;

  // wait 
  dma_channel_wait_for_finish_blocking(lan8720->tx_dma_channel); // production
  
  uint len = LAN8720_PREAMBLE_SFD_SIZE;
  for (struct pbuf *q = p; q != NULL; q = q->next) 
  {
    memcpy(lan8720->txbuffer + len, q->payload, q->len);

    len += q->len;

    if (q->len == q->tot_len) 
    {
      break;
    }
  }

  if (len < 60 + LAN8720_PREAMBLE_SFD_SIZE) 
  {
    // pad
    len = 60 + LAN8720_PREAMBLE_SFD_SIZE;
  }

  uint32_t crc = lan8720_frame_crc(lan8720->txbuffer + LAN8720_PREAMBLE_SFD_SIZE, len - LAN8720_PREAMBLE_SFD_SIZE);

  memcpy(lan8720->txbuffer + len, &crc, sizeof(crc));
  len += 4;

  dma_channel_set_read_addr(lan8720->tx_dma_channel, lan8720->txbuffer, false);
  dma_channel_set_trans_count(lan8720->tx_dma_channel, len, true);

  LOG("sending packet\n");

  return ERR_OK;
}

static inline void lan8720_rmii_mdio_dir_out(struct lan8720 *lan8720)
{
  gpio_set_dir(lan8720->mdio_pin, GPIO_OUT);
}

static inline void lan8720_rmii_mdio_dir_in(struct lan8720 *lan8720)
{
  gpio_set_dir(lan8720->mdio_pin, GPIO_IN);
}

static inline void lan8720_rmii_mdio_clk_lo(struct lan8720 *lan8720)
{
  gpio_put(lan8720->mdc_pin, 0);
}

static inline void lan8720_rmii_mdio_clk_hi(struct lan8720 *lan8720)
{
  gpio_put(lan8720->mdc_pin, 1);
}

static void lan8720_rmii_mdio_out_bit(struct lan8720 *lan8720, uint8_t bit)
{
    lan8720_rmii_mdio_clk_lo(lan8720);

    sleep_us(1);
    
    gpio_put(lan8720->mdio_pin, bit);

    lan8720_rmii_mdio_clk_hi(lan8720);

    sleep_us(1);
}

static uint8_t lan8720_rmii_mdio_in_bit(struct lan8720 *lan8720)
{
    uint8_t res;
    lan8720_rmii_mdio_clk_lo(lan8720);
    
    sleep_us(1);

    lan8720_rmii_mdio_clk_hi(lan8720);
    res = gpio_get(lan8720->mdio_pin);

    sleep_us(1);

    return res ? 0x01 : 0x00;
}

static uint16_t lan8720_rmii_mdio_read(struct lan8720 *lan8720, uint8_t addr, uint8_t reg)
{
  lan8720_rmii_mdio_dir_out(lan8720);

  // PRE_32
  for (int i = 0; i < 32; i++)
      lan8720_rmii_mdio_out_bit(lan8720, 1);
      
  // ST
  lan8720_rmii_mdio_out_bit(lan8720, 0);
  lan8720_rmii_mdio_out_bit(lan8720, 1);

  // OP
  lan8720_rmii_mdio_out_bit(lan8720, 1);
  lan8720_rmii_mdio_out_bit(lan8720, 0);

  // PA5
  for (int i = 0; i < 5; i++)
  {
    uint bit = (addr >> (4 - i)) & 0x01;
    lan8720_rmii_mdio_out_bit(lan8720, bit);
  }

  // RA5
  for (int i = 0; i < 5; i++)
  {
    uint bit = (reg >> (4 - i)) & 0x01;
    lan8720_rmii_mdio_out_bit(lan8720, bit);
  }

  // TA
  lan8720_rmii_mdio_dir_in(lan8720);
  lan8720_rmii_mdio_out_bit(lan8720, 0);
  lan8720_rmii_mdio_out_bit(lan8720, 0);

  uint16_t data = 0;
  for (int i = 0; i < 16; i++)
  {
      data <<= 1;

      data |= lan8720_rmii_mdio_in_bit(lan8720);
  }

  return data;
}

static void lan8720_rmii_mdio_write(struct lan8720 *lan8720, uint8_t addr, uint8_t reg, uint16_t value)
{
  lan8720_rmii_mdio_dir_out(lan8720);

  // PRE_32
  for (int i = 0; i < 32; i++)
      lan8720_rmii_mdio_out_bit(lan8720, 1);
      
  // ST
  lan8720_rmii_mdio_out_bit(lan8720, 0);
  lan8720_rmii_mdio_out_bit(lan8720, 1);

  // OP
  lan8720_rmii_mdio_out_bit(lan8720, 0);
  lan8720_rmii_mdio_out_bit(lan8720, 1);

  // PA5
  for (int i = 0; i < 5; i++)
  {
    uint bit = (addr >> (4 - i)) & 0x01;
    lan8720_rmii_mdio_out_bit(lan8720, bit);
  }

  // RA5
  for (int i = 0; i < 5; i++)
  {
    uint bit = (reg >> (4 - i)) & 0x01;
    lan8720_rmii_mdio_out_bit(lan8720, bit);
  }

  // TA
  lan8720_rmii_mdio_out_bit(lan8720, 1);
  lan8720_rmii_mdio_out_bit(lan8720, 0);

  uint16_t data = 0;
  for (int i = 0; i < 16; i++)
  {
      uint bit = (value >> (15 - i)) & 0x01;

      lan8720_rmii_mdio_out_bit(lan8720, bit);
  }
}

void lan8720_updatestate(struct lan8720 *lan8720)
{
  volatile uint16_t link_status = (lan8720_rmii_mdio_read(lan8720, lan8720->phy_address, 1) & 0x04) >> 2;
  if (netif_is_link_up(&lan8720->netif) ^ link_status) 
  {
    if (link_status) 
    {
      // LOG("netif_set_link_up\n");
      netif_set_link_up(&lan8720->netif);
    }
    else 
    {
      // LOG("netif_set_link_down\n");
      netif_set_link_down(&lan8720->netif);
    }
  }
}

void lan8720_idlehandler(void *userdata)
{
  struct lan8720 *lan8720 = (struct lan8720 *)userdata;

  sys_check_timeouts();

  u32_t now = sys_now();
  if (now - lan8720->lastupdatetime >= LAN8720_UPDATE_INTERVAL)
  {
    lan8720->lastupdatetime = now;
    lan8720_updatestate(lan8720);
  }

  sys_timeout(0, lan8720_idlehandler, lan8720);
}

err_t lan8720_netif_init(struct netif *netif)
{
  struct lan8720 *lan8720 = (struct lan8720 *)netif->state;

  netif->linkoutput = lan8720_tx_start;
  netif->output     = etharp_output;
  netif->mtu        = 1500; 
  netif->flags      = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP | NETIF_FLAG_MLD6;

  // generate one for unique board id
  pico_unique_board_id_t board_id;

  pico_get_unique_board_id(&board_id);

  netif->hwaddr[0] = 0xb8;
  netif->hwaddr[1] = 0x27;
  netif->hwaddr[2] = 0xeb;
  memcpy(&netif->hwaddr[3], &board_id.id[5], 3);

  netif->hwaddr_len = ETH_HWADDR_LEN;

  uint8_t iodir;
  
  gpio_init(lan8720->mdio_pin);
  gpio_init(lan8720->mdc_pin);
  gpio_set_dir(lan8720->mdc_pin, GPIO_OUT);

  lan8720_rmii_mdio_clk_lo(lan8720);

  for (int i = 0; i < 32; i++) 
  {
    uint16_t aux = lan8720_rmii_mdio_read(lan8720, i, 0);
    if (aux != 0xffff) 
    {
      lan8720->phy_address = i;
      LOG_FMT("detected LAN8720 with address %d\n", i);

      break;
    }
  }

  // 100MBit
  lan8720_rmii_mdio_write(lan8720, lan8720->phy_address, 0, LAN8720_REG0_SPEED100 | LAN8720_REG0_AUTONEG | LAN8720_REG0_FULLDUPLEX); // 100MBit + AutoNegotiation
  lan8720_rmii_mdio_write(lan8720, lan8720->phy_address, 4, 0x00c1); // SPEED 100Mbps, 100 Full + TX ability

  lan8720->rx_sm_offset = pio_add_program(lan8720->pio, &lan8720_rx_program);
  lan8720->rx_dma_channel = dma_claim_unused_channel(true);
  lan8720->rx_dma_channel_config = dma_channel_get_default_config(lan8720->rx_dma_channel);

  channel_config_set_read_increment(&lan8720->rx_dma_channel_config, false);
  channel_config_set_write_increment(&lan8720->rx_dma_channel_config, true);
  channel_config_set_dreq(&lan8720->rx_dma_channel_config, pio_get_dreq(lan8720->pio, lan8720->rx_sm, false));
  channel_config_set_transfer_data_size(&lan8720->rx_dma_channel_config, DMA_SIZE_8);
  dma_channel_set_config(lan8720->rx_dma_channel, &lan8720->rx_dma_channel_config, false);
  dma_channel_set_read_addr(lan8720->rx_dma_channel, ((uint8_t*)&lan8720->pio->rxf[lan8720->rx_sm]) + 3, false); 

  lan8720->tx_sm_offset = pio_add_program(lan8720->pio, &lan8720_tx_program);
  lan8720->tx_dma_channel = dma_claim_unused_channel(true);
  lan8720->tx_dma_channel_config = dma_channel_get_default_config(lan8720->tx_dma_channel);

  channel_config_set_read_increment(&lan8720->tx_dma_channel_config, true);
  channel_config_set_write_increment(&lan8720->tx_dma_channel_config, false);
  channel_config_set_dreq(&lan8720->tx_dma_channel_config, pio_get_dreq(lan8720->pio, lan8720->tx_sm, true));
  channel_config_set_transfer_data_size(&lan8720->tx_dma_channel_config, DMA_SIZE_8);
  dma_channel_set_config(lan8720->tx_dma_channel, &lan8720->tx_dma_channel_config, false);
  dma_channel_set_write_addr(lan8720->tx_dma_channel, ((uint8_t*)&lan8720->pio->txf[lan8720->tx_sm]) + 0, false);

  for (int i = 0; i < 7; i++)
    lan8720->txbuffer[i] = 0x55;
  lan8720->txbuffer[7] = 0xd5;

  lan8720->rxfull = false;

  // netif_set_link_up will tx packet
  lan8720_tx_init(lan8720->pio, lan8720->tx_sm, lan8720->tx_sm_offset, lan8720->tx_pin);
  
  irq_set_exclusive_handler(lan8720->pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_0, lan8720_rx_irq_handler);
  pio_set_irq0_source_enabled(lan8720->pio, pis_interrupt0, true);
  irq_set_enabled(lan8720->pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_0, true);

  if (lan8720_rx_trystart(lan8720))
  {
    lan8720_rx_init(lan8720->pio, lan8720->rx_sm, lan8720->rx_sm_offset, lan8720->rx_pin);
    pio_sm_set_enabled(lan8720->pio, lan8720->rx_sm, true);
  }
  
  sys_timeout(0, lan8720_idlehandler, lan8720);

  return ERR_OK;
}

void lan8720_init(struct lan8720 *lan8720, PIO pio, int sm, uint8_t mdio_pin, uint8_t rx_pin, uint8_t tx_pin)
{
  lan8720->mdio_pin = mdio_pin;
  lan8720->mdc_pin = mdio_pin + 1;
  lan8720->pio = pio;
  lan8720->rx_sm = sm;
  lan8720->tx_sm = sm + 1;
  lan8720->rx_pin = rx_pin;
  lan8720->tx_pin = tx_pin;
  lan8720->next = g_lan8720_first;
  g_lan8720_first = lan8720;

  static bool sys_critical_section_inited = false;
  if (!sys_critical_section_inited)
  {
    critical_section_init(&sys_critical_section);
    sys_critical_section_inited = true;
  }
  
  static bool pool_initialized = false;
  if (!pool_initialized)
  {
    LWIP_MEMPOOL_INIT(lan8720_rx_pool);
    pool_initialized = true;
  }

  lwip_init();
  netif_add(&lan8720->netif, IP4_ADDR_ANY4, IP4_ADDR_ANY, IP4_ADDR_ANY, lan8720, lan8720_netif_init, netif_input);

  netif_set_link_callback(&lan8720->netif, lan8720_link_callback);
  netif_set_status_callback(&lan8720->netif, lan8720_status_callback);

  netif_set_default(&lan8720->netif);
  netif_set_up(&lan8720->netif);
  dhcp_start(&lan8720->netif);
}
