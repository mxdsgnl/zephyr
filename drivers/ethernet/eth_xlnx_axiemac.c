/* zynq parrot axiemac Ethernet Controller
 *
 * Copyright (c) 2021, Maxentric Tehcnologies LLC
 *
 * Known current limitations / TODOs:
 * - Speed limited to 10/100.
 * - No support for jumbo frame.
 * - No support for Hardware timestamps.
 * - VLAN tags not considered.
 * - Wake-on-LAN interrupt not supported.
 * - Send function is not SMP-capable (due to single TX done semaphore).
 * - No detailed error handling when evaluating the Interrupt Status,
 *   RX Status and TX Status registers.
 */
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <sys/__assert.h>
#include <soc.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <net/ethernet.h>
#include <ethernet/eth_stats.h>
#include "eth_xlnx_axiemac_priv.h"
#define LOG_MODULE_NAME eth_bpaxiemac
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static int eth_axiemac_dev_init(const struct device *dev);
static void eth_axiemac_xStmFifo_init(const struct device *dev);
static void eth_axiemac_iface_init(struct net_if *iface);
static void eth_xStmFifo_isr(const struct device *dev);
static void fifo_error_handler(const struct device *dev, uint32_t pending_intr);
static void xStrmFifo_recv_handler(const struct device *dev);
void eth_axiemac_err_isr(const struct device *dev);
static int eth_axiemac_send(const struct device *dev, struct net_pkt *pkt);
static int eth_axiemac_start_device(const struct device *dev);
static int eth_axiemac_stop_device(const struct device *dev);
static enum ethernet_hw_caps eth_axiemac_get_capabilities(
	const struct device *dev);
static void eth_axiemac_assign_mac(const struct device *dev);
static int eth_axiemac_set_speed(const struct device *dev);
static void eth_axiemac_config_options(const struct device *dev,
		uint32_t option, uint8_t set1clr0);
static uint32_t eth_axiemac_check_options(const struct device *dev);
static int xStrmFifo_Write(XStrmFifo *fifoPtr, struct net_pkt *pkt, uint32_t Bytes);
static void xStrmFifo_Read(XStrmFifo *fifoPtr, uint8_t *buf, uint32_t Bytes);
static int xStrmFifo_getTxVacancy(XStrmFifo *fifoPtr);
static void xStmFifo_iWrite_Aligned(XStrmFifo *fifoPtr, uint32_t *BufPtr,
		uint32_t WordCount);
static void xStmFifo_iRead_Aligned(XStrmFifo *fifoPtr, uint32_t *BufPtr,
		uint32_t WordCount);  
static void xStrmFifo_flush(XStrmFifo *fifoPtr, uint32_t Bytes);
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
struct net_stats_eth *eth_axiemac_stats(const struct device *dev);
#endif
    
static const struct ethernet_api eth_axiemac_apis = {
	.iface_api.init	= eth_axiemac_iface_init,
  .get_capabilities = eth_axiemac_get_capabilities,
	.send =  eth_axiemac_send,
  .start = eth_axiemac_start_device,
  .stop = eth_axiemac_stop_device,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = eth_axiemac_stats,
#endif
};

DT_INST_FOREACH_STATUS_OKAY(ETH_XLNX_AXIEMAC_INITIALIZE)

/**
 * @brief xilinx axi ethernet device initialization function
 * Initialize the axi stream fifo attached to the ethernet mac.
 * Config axiemac, the obtain auto negotiation status, duplex and 
 * link speed.  Sets link speed on ethernet device and enable 
 * interrupt.
 * @param dev Pointer to the device data
 * @retval 0 if the device initialization completed successfully
 */
static int eth_axiemac_dev_init(const struct device *dev)
{

	uint16_t tmpU16;
	uint32_t tmpU32;
	uint32_t options;
	int status;

	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);


	eth_axiemac_xStmFifo_init(dev);

	//wait for pll lock and pull mgtrdy bit
	int timeout = 5;
  do {
    if (timeout == 0){
      LOG_ERR("emac mgtrdy not ready timeout\r\n");
      return -EIO;
    }
    my_msleep(100);
    tmpU32 = sys_read32(dev_conf->base_addr+XAE_IS_OFFSET);
    timeout--;
	} while ((tmpU32 & XAE_INT_MGTRDY_MASK) == 0);

	//stop emac
	/* Disable interrupts */
	sys_write32(0x0, dev_conf->base_addr+XAE_IE_OFFSET);

	/* Disable the receiver */
	tmpU32 = sys_read32(dev_conf->base_addr+XAE_RCW1_OFFSET);
	tmpU32 &= ~XAE_RCW1_RX_MASK;
	sys_write32(tmpU32, dev_conf->base_addr+XAE_RCW1_OFFSET);

	/* clear any pending register */
	tmpU32 = sys_read32(dev_conf->base_addr+XAE_IP_OFFSET);
	sys_write32(tmpU32, dev_conf->base_addr+XAE_IS_OFFSET);

	/*convert dts option to option word disable transmit and recev initially*/
	options = eth_axiemac_check_options(dev);
	options &= ~(XAE_RECEIVER_ENABLE_MASK | XAE_TRANSMITTER_ENABLE_MASK);

	eth_axiemac_config_options(dev, options, SET_OPTIONS);
	eth_axiemac_config_options(dev, ~options, CLR_OPTIONS);

	/* Set default MDIO divisor */
	sys_write32(((uint32_t) XAE_MDIO_DIV_DFT | XAE_MDIO_MC_MDIOEN_MASK),
			dev_conf->base_addr+XAE_MDIO_MC_OFFSET);

	/* Assign MAC address to Hardware */
	eth_axiemac_assign_mac(dev);

	/* detect phy , update phy addr and api based on vendor type */
	int phy_found = axiemac_detect_phy(dev);
	int Success = 0;
	if (phy_found == 0&& dev_data->phy_id != 0x00000000 &&
	dev_data->phy_id != 0xFFFFFFFF &&
	dev_data->phy_access_api != NULL) {
		/* A compatible PHY was detected -> reset & configure it */
		dev_data->phy_access_api->phy_reset_func(dev);
		dev_data->phy_access_api->phy_configure_func(dev);

		/*required wait after reconfig*/

		/* poll status change autoneg complete bit for now*/
		tmpU32 = 2000;
		do {
			tmpU16 = dev_data->phy_access_api->phy_poll_status_change_func(dev);
			if ((tmpU16 & PHY_XLNX_AXIEMAC_EVENT_AUTONEG_COMPLETE) != 0) {
				dev_data->eff_link_speed =
						dev_data->phy_access_api->phy_poll_link_speed_func(dev);
				Success = 1;
				LOG_DBG("%s auto neg complete.\r\n", dev->name);
				break;
			}
			my_msleep(1);
			tmpU32--;
		} while (tmpU32 > 0);
		if (!Success){
      dev_data->eff_link_speed = LINK_10MBIT;
			LOG_WRN("%s auto neg timed out, link speed set to 10Mb\r\n", dev->name);
    }
	} else {
    dev_data->eff_link_speed = LINK_10MBIT;
		LOG_WRN("%s no compatible PHY detected, link speed set to 10Mb\r\n", dev->name);
	}

	status = eth_axiemac_set_speed(dev);
	if (status == -EIO) {
		return -EIO;
	}

	/* Setting the operating speed of the MAC needs a delay. */
	{
		my_msleep(100);
	}

	/* enable MAC interrupts */
	tmpU32 = sys_read32(dev_conf->base_addr+XAE_IE_OFFSET);
	tmpU32 |= XAE_INT_ALL_MASK;
	sys_write32(tmpU32, dev_conf->base_addr+XAE_IE_OFFSET);

	return 0;
}

/**
 * @brief Initializes the interface associated with 
 * a xilinx axi ethernet device.
 *
 * @param iface Pointer to the associated interface data struct
 */
static void eth_axiemac_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);

	dev_data->iface = iface;

	/* Assign link local address. */
	net_if_set_link_addr(iface,
			dev_data->mac_addr, 6, NET_LINK_ETHERNET);

	ethernet_init(iface);
	net_if_flag_set(iface, NET_IF_NO_AUTO_START);

	/* Initialize semaphore. */
	k_sem_init(&dev_data->tx_sem, 0, 1);

	/* Initialize Interrupts. */
	dev_conf->config_func(dev);
}

/**
 * @brief axi stream fifo interrupt service routine
 * Checks for indications of errors
 * handles receive pending and tx complete notifications
 * 
 *
 * @param dev Pointer to the device data
 */
static void eth_xStmFifo_isr(const struct device *dev) {
	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	uint32_t tmpU32;
	uint32_t maskU32;
	uint32_t pendingIntr;

	uint32_t lock;
	lock = irq_lock();

	/* read interrupt pending */
	maskU32 = sys_read32(dev_conf->fifo_addr+XLLF_IER_OFFSET);
	tmpU32 = sys_read32(dev_conf->fifo_addr+XLLF_ISR_OFFSET);
	pendingIntr = tmpU32 & maskU32;

	while (pendingIntr) {
		if (pendingIntr & XLLF_INT_RC_MASK) {
			/* receive interrupt */
			sys_write32(XLLF_INT_RC_MASK, dev_conf->fifo_addr+XLLF_ISR_OFFSET);
			xStrmFifo_recv_handler(dev);
		} else if (pendingIntr & XLLF_INT_TC_MASK) {
			sys_write32(XLLF_INT_TC_MASK, dev_conf->fifo_addr+XLLF_ISR_OFFSET);
			dev_data->tx_err = false;
			k_sem_give(&dev_data->tx_sem);
		} else {
			sys_write32(
					(XLLF_INT_ALL_MASK & ~(XLLF_INT_RC_MASK|XLLF_INT_TC_MASK)),
					dev_conf->fifo_addr+XLLF_ISR_OFFSET);
			fifo_error_handler(dev, pendingIntr);
		}
		pendingIntr = sys_read32(dev_conf->fifo_addr+XLLF_ISR_OFFSET) & maskU32;
	}
	irq_unlock(lock);
}

/**
 * @brief axi ethernet device send function. 
 * Blocks until a TX complete notification has been
 * received & processed.
 *
 * @param dev pointer to the device
 * @param pkt Pointer to the data packet to be sent
 * @retval -EINVAL in case of invalid parameters, e.g. zero data length
 * or larger than XAE_MAX_FRAME_SIZE.
 * @retval -EIO in case of:
 *         (1) the attempt to TX data while the device is stopped,
 *             the interface is down or the link is down,
 *         (2) the attempt to TX data while no free buffers are available
 *             and timeing out.
 *         (3) the transmission completion notification timing out
 * @retval 0 if the packet was transmitted successfully
 */
static int eth_axiemac_send(const struct device *dev, struct net_pkt *pkt)
{

	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	uint16_t bytesToSend;
	int sem_status;

	if (!dev_data->started || dev_data->eff_link_speed == LINK_DOWN
			||(!net_if_flag_is_set(dev_data->iface, NET_IF_UP))
			) {
#ifdef CONFIG_NET_STATISTICS_ETHERNET
		dev_data->stats.tx_dropped++;
#endif
		return -EIO;
	}

	bytesToSend = net_pkt_get_len(pkt);

	if ((bytesToSend == 0)||
  (bytesToSend > XAE_MAX_FRAME_SIZE)) {
		LOG_ERR("%s cannot TX, invalid packet length %d ", 
    dev->name, bytesToSend);
#ifdef CONFIG_NET_STATISTICS_ETHERNET
		dev_data->stats.errors.tx++;
#endif
		return -EINVAL;
	}
  
  int timeout = 100;
  while((xStrmFifo_getTxVacancy(&dev_data->axififo)*4)
    <XAE_MAX_FRAME_SIZE){
    if (timeout < 0){
      LOG_ERR("%s cannot TX, tx fifo no space", dev->name);
      dev_data->stats.errors.tx++;
      return -EIO;
    }
    my_msleep(1);
    timeout--;
  }

	/* write frame data to FIFO and start transfer*/
	if (xStrmFifo_Write(&dev_data->axififo, pkt, bytesToSend) == 0){
		/* Transmit successfully issued to hardware
		 Wait and check if transmit successful or not. */
		sem_status = k_sem_take(&dev_data->tx_sem, K_MSEC(200));
	} else {
#ifdef CONFIG_NET_STATISTICS_ETHERNET
		dev_data->stats.tx_dropped++;
#endif
		return -EIO;
	}

	if (dev_data->tx_err) {
		dev_data->tx_err = false;
		return -EIO;
	}

	LOG_DBG("pkt sent %p len %d", pkt, bytesToSend);
#ifdef CONFIG_NET_STATISTICS_ETHERNET
	dev_data->stats.bytes.sent += bytesToSend;
	dev_data->stats.pkts.tx++;
#endif

	return 0;

}

/**
 * @brief axi ethernet device start function
 * Clears all status registers and any
 * pending interrupts, enables RX and TX, enables interrupts. 
 * declares the physical link up at the configured nominal
 * link speed. Phy is not managed by driver. 
 * @param dev Pointer to the device data
 * @retval    0 upon successful completion
 */
static int eth_axiemac_start_device(const struct device *dev)
{
	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	uint32_t tmpU32;
	if (dev_data->started) {
		return 0;
	}
	/* Disable & clear all the MAC interrupts */
	sys_write32(0x0, dev_conf->base_addr+XAE_IE_OFFSET);
	/* clear any pending register */
	tmpU32 = sys_read32(dev_conf->base_addr+XAE_IP_OFFSET);
	sys_write32(tmpU32, dev_conf->base_addr+XAE_IS_OFFSET);

	/* enable MAC interrupts */
	tmpU32 = sys_read32(dev_conf->base_addr+XAE_IE_OFFSET);
	tmpU32 |= XAE_INT_ALL_MASK;
	sys_write32(tmpU32, dev_conf->base_addr+XAE_IE_OFFSET);

	/* Enable/ receiver */
	tmpU32 = sys_read32(dev_conf->base_addr + XAE_RCW1_OFFSET);
	tmpU32 |= XAE_RCW1_RX_MASK; //recv enable mask
	sys_write32(tmpU32, dev_conf->base_addr + XAE_RCW1_OFFSET);

	/* Enable/ transmitter */
	tmpU32 = sys_read32(dev_conf->base_addr + XAE_TC_OFFSET);
	tmpU32 |= XAE_TC_TX_MASK; //transmit enable mask
	sys_write32(tmpU32, dev_conf->base_addr + XAE_TC_OFFSET);

	dev_data->started = true;

	LOG_DBG("%s started", dev->name);
	return 0;

}

/**
 * @brief xilinx axi ethernet device stop function
 * Disables all interrupts, disables RX and TX, 
 * clears all status registers. No PHY is managed
 * by the current driver instance, 
 * this function also declares
 * the physical link down.
 *
 * @param dev Pointer to the device data
 * @retval    0 upon successful completion
 */
static int eth_axiemac_stop_device(const struct device *dev)
{
	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	uint32_t tmpU32;

	if (!dev_data->started) {
		return 0;
	}

	/* Enable/ receiver */
	tmpU32 = sys_read32(dev_conf->base_addr + XAE_RCW1_OFFSET);
	tmpU32 &= ~XAE_RCW1_RX_MASK; //recv enable mask
	sys_write32(tmpU32, dev_conf->base_addr + XAE_RCW1_OFFSET);

	/* Enable/ transmitter */
	tmpU32 = sys_read32(dev_conf->base_addr + XAE_TC_OFFSET);
	tmpU32 &= ~XAE_TC_TX_MASK; //transmit enable mask
	sys_write32(tmpU32, dev_conf->base_addr + XAE_TC_OFFSET);

	/* Disable & clear all the MAC interrupts */
	sys_write32(0x0, dev_conf->base_addr+XAE_IE_OFFSET);
	/* clear any pending register */
	tmpU32 = sys_read32(dev_conf->base_addr+XAE_IP_OFFSET);
	sys_write32(tmpU32, dev_conf->base_addr+XAE_IS_OFFSET);

	dev_data->started = false;
	LOG_DBG("%s stopped", dev->name);
	return 0;
}


static enum ethernet_hw_caps eth_axiemac_get_capabilities(
	const struct device *dev)
{
	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	enum ethernet_hw_caps caps = (enum ethernet_hw_caps)0;

	if (dev_conf->max_link_speed == LINK_1GBIT) {
			caps |= (ETHERNET_LINK_1000BASE_T |
				ETHERNET_LINK_100BASE_T |
				ETHERNET_LINK_10BASE_T);
	} else if (dev_conf->max_link_speed == LINK_100MBIT) {
			caps |= (ETHERNET_LINK_100BASE_T |
				ETHERNET_LINK_10BASE_T);
	} else {
		caps |= ETHERNET_LINK_10BASE_T;
	}

	if (dev_conf->xae_rx_chksum_offload) {
		caps |= ETHERNET_HW_RX_CHKSUM_OFFLOAD;
	}

	if (dev_conf->xae_tx_chksum_offload) {
		caps |= ETHERNET_HW_TX_CHKSUM_OFFLOAD;
	}

	if (dev_conf->enable_fdx) {
		caps |= ETHERNET_DUPLEX_SET;
	}

	if (dev_conf->xae_promisc_option) {
		caps |= ETHERNET_PROMISC_MODE;
	}

	return caps;
}


static void eth_axiemac_assign_mac(const struct device *dev) {
	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	uint8_t mac_addr[6] = DT_INST_PROP(0, local_mac_address);

	uint32_t tmpU32 = 0;
	uint32_t extMode;

	for (int i = 0; i < 6; i++) {
		dev_data->mac_addr[i] = mac_addr[i];
	}

	tmpU32 |= mac_addr[0];
	tmpU32 |= mac_addr[1] << 8;
	tmpU32 |= mac_addr[2] << 16;
	tmpU32 |= mac_addr[3] << 24;

	extMode = (sys_read32(dev_conf->base_addr+XAE_RAF_OFFSET)
			& XAE_RAF_NEWFNCENBL_MASK);

	if (extMode == 0) {
		sys_write32(tmpU32, dev_conf->base_addr+XAE_UAW0_OFFSET);

		tmpU32 = sys_read32(dev_conf->base_addr+XAE_UAW1_OFFSET);
		tmpU32 &= ~XAE_UAW1_UNICASTADDR_MASK;
		tmpU32 |= mac_addr[4];
		tmpU32 |= mac_addr[5] << 8;
		sys_write32(tmpU32, dev_conf->base_addr+XAE_UAW1_OFFSET);
	} else {
		/* Extended mode */
		sys_write32(tmpU32, dev_conf->base_addr+XAE_UAWL_OFFSET);

		tmpU32 = 0;
		tmpU32 |= mac_addr[4];
		tmpU32 |= mac_addr[5] << 8;
		sys_write32(tmpU32, dev_conf->base_addr+XAE_UAWU_OFFSET);
	}
}

/**
 * @brief 
 * Convert option from device config to option word
 * Checks option dependency and apply dependent options
 * returns the option word to be used for eth_axiemac_config_options
 *
 * @param dev Pointer to the device data
 * @return option word
 */
static uint32_t eth_axiemac_check_options(const struct device *dev) {

	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	uint32_t optionU32 = 0;

	/* set options based on dev_conf */
	if (dev_conf->xae_promisc_option == 1) {
		optionU32 |= XAE_PROMISC_MASK;
	}
	if (dev_conf->xae_jumbo_option == 1) {
		optionU32 |= XAE_JUMBO_MASK;
	}
	if (dev_conf->xae_vlan_option == 1) {
		optionU32 |= XAE_VLAN_MASK;
	}
	if (dev_conf->xae_flow_control_option == 1) {
		optionU32 |= XAE_FLOW_CONTROL_MASK;
	}
	if (dev_conf->xae_fcs_strip_option == 1) {
		optionU32 |= XAE_FCS_STRIP_MASK;
	}
	if (dev_conf->xae_fcs_insert_option == 1) {
		optionU32 |= XAE_FCS_INSERT_MASK;
	}
	if (dev_conf->xae_lentype_err_option == 1) {
		optionU32 |= XAE_LENTYPE_ERR_MASK;
	}
	if (dev_conf->xae_transmitter_enable_option == 1) {
		optionU32 |= XAE_TRANSMITTER_ENABLE_MASK;
	}
	if (dev_conf->xae_receiver_enable_option == 1) {
		optionU32 |= XAE_RECEIVER_ENABLE_MASK;
	}
	if (dev_conf->xae_broadcast_option == 1) {
		optionU32 |= XAE_BROADCAST_MASK;
	}
	if (dev_conf->xae_multicast_option == 1) {
		optionU32 |= XAE_MULTICAST_MASK;
	}
	if (dev_conf->xae_ext_multicast_option == 1) {
		optionU32 |= XAE_EXT_MULTICAST_MASK;
		optionU32 |= XAE_PROMISC_MASK;
	}
	if (dev_conf->xae_ext_txvlan_tran_option == 1) {
		optionU32 |= XAE_EXT_TXVLAN_TRAN_MASK;
		optionU32 |= XAE_FCS_INSERT_MASK;
		optionU32 &= ~XAE_VLAN_MASK;
	}
	if (dev_conf->xae_ext_rxvlan_tran_option == 1) {
		optionU32 |= XAE_EXT_RXVLAN_TRAN_MASK;
		optionU32 |= XAE_FCS_STRIP_MASK;
		optionU32 &= ~XAE_VLAN_MASK;
	}
	if (dev_conf->xae_ext_txvlan_tag_option == 1) {
		optionU32 |= XAE_EXT_TXVLAN_TAG_MASK;
		optionU32 |= XAE_FCS_INSERT_MASK;
		optionU32 &= ~XAE_VLAN_MASK;
		optionU32 |= XAE_JUMBO_MASK;
	}
	if (dev_conf->xae_ext_rxvlan_tag_option == 1) {
		optionU32 |= XAE_EXT_RXVLAN_TAG_MASK;
		optionU32 |= XAE_FCS_STRIP_MASK;
		optionU32 &= ~XAE_VLAN_MASK;
		optionU32 |= XAE_JUMBO_MASK;
	}
	if (dev_conf->xae_ext_txvlan_strp_option == 1) {
		optionU32 |= XAE_EXT_TXVLAN_STRP_MASK;
		optionU32 |= XAE_FCS_INSERT_MASK;
		optionU32 &= ~XAE_VLAN_MASK;
		optionU32 |= XAE_JUMBO_MASK;
	}
	if (dev_conf->xae_ext_rxvlan_strp_option == 1) {
		optionU32 |= XAE_EXT_RXVLAN_STRP_MASK;
		optionU32 |= XAE_FCS_STRIP_MASK;
		optionU32 &= ~XAE_VLAN_MASK;
		optionU32 |= XAE_JUMBO_MASK;
	}

	return optionU32;
}

/**
 * @brief 
 * Set or clear the options selected by the option mask
 * 
 * @param 
 * dev : Pointer to the device data
 * option : option config mask.  
 * set1clr0 : 1 = set options selected by the mask, 
 * 0= clear the options selected by the mask.
 * @return void
 */
static void eth_axiemac_config_options(const struct device *dev,
		uint32_t option, uint8_t set1clr0) {

	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	uint32_t optionU32;
	uint32_t tmpU32;
	uint32_t newRcwU32, newTcU32;

	optionU32 = option;
	/*
	 * New/extended function bit should be on if any new/extended features
	 * are on and hardware is built with them. 
	 */
	tmpU32 = sys_read32(dev_conf->base_addr+XAE_RAF_OFFSET);
	if (optionU32 & (XAE_EXT_MULTICAST_MASK |
	XAE_EXT_TXVLAN_TRAN_MASK |
	XAE_EXT_RXVLAN_TRAN_MASK |
	XAE_EXT_TXVLAN_TAG_MASK |
	XAE_EXT_RXVLAN_TAG_MASK |
	XAE_EXT_TXVLAN_STRP_MASK |
	XAE_EXT_RXVLAN_STRP_MASK)) {
		sys_write32((tmpU32|XAE_RAF_NEWFNCENBL_MASK),
				dev_conf->base_addr+XAE_RAF_OFFSET);
	} else {
		sys_write32((tmpU32&~XAE_RAF_NEWFNCENBL_MASK),
				dev_conf->base_addr+XAE_RAF_OFFSET);
	}

	newRcwU32 = sys_read32(dev_conf->base_addr+XAE_RCW1_OFFSET);
	newTcU32 = sys_read32(dev_conf->base_addr+XAE_TC_OFFSET);

	/* Turn on/off jumbo packet support for both Rx and Tx */
	if (optionU32 & XAE_JUMBO_MASK) {
		if (set1clr0 == 1) {
			newTcU32 |= XAE_TC_JUM_MASK;
			newRcwU32 |= XAE_RCW1_JUM_MASK;
		} else if (set1clr0 == 0) {
			newTcU32 &= ~XAE_TC_JUM_MASK;
			newRcwU32 &= ~XAE_RCW1_JUM_MASK;
		}
	}

	/* Turn on/off VLAN packet support for both Rx and Tx */
	if (optionU32 & XAE_VLAN_MASK) {
		if (set1clr0 == 1) {
			newTcU32 |= XAE_TC_VLAN_MASK;
			newRcwU32 |= XAE_RCW1_VLAN_MASK;
		} else if (set1clr0 == 0) {
			newTcU32 &= ~XAE_TC_VLAN_MASK;
			newRcwU32 &= ~XAE_RCW1_VLAN_MASK;
		}
	}

	/* Turn on/off FCS stripping on receive packets */
	if (optionU32 & XAE_FCS_STRIP_MASK) {
		if (set1clr0 == 1) {
			newRcwU32 &= ~XAE_RCW1_FCS_MASK;
		} else if (set1clr0 == 0) {
			newRcwU32 |= XAE_RCW1_FCS_MASK;
		}
	}

	/* Turn on/off FCS insertion on transmit packets */
	if (optionU32 & XAE_FCS_INSERT_MASK) {
		if (set1clr0 == 1) {
			newTcU32 &= ~XAE_TC_FCS_MASK;
		} else if (set1clr0 == 0) {
			newTcU32 |= XAE_TC_FCS_MASK;
		}
	}

	/* Turn on/off length/type field checking on receive packets */
	if (optionU32 & XAE_LENTYPE_ERR_MASK) {
		if (set1clr0 == 1) {
			newRcwU32 &= ~XAE_RCW1_LT_DIS_MASK;
		} else if (set1clr0 == 0) {
			newRcwU32 |= XAE_RCW1_LT_DIS_MASK;
		}
	}

	/* Enable/disable transmitter */
	if (optionU32 & XAE_TRANSMITTER_ENABLE_MASK) {
		if (set1clr0 == 1) {
			newTcU32 |= XAE_TC_TX_MASK;
		} else if (set1clr0 == 0) {
			newTcU32 &= ~XAE_TC_TX_MASK;
		}
	}

	/* Enable/disable receiver */
	if (optionU32 & XAE_RECEIVER_ENABLE_MASK) {
		if (set1clr0 == 1) {
			newRcwU32 |= XAE_RCW1_RX_MASK;
		} else if (set1clr0 == 0) {
			newRcwU32 &= ~XAE_RCW1_RX_MASK;
		}
	}

	sys_write32(newRcwU32, dev_conf->base_addr+XAE_RCW1_OFFSET);
	sys_write32(newTcU32, dev_conf->base_addr+XAE_TC_OFFSET);

	/* Turn on/off flow control */
	if (optionU32 & XAE_FLOW_CONTROL_MASK) {
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_FCC_OFFSET);
		if (set1clr0 == 1) {
			tmpU32 |= XAE_FCC_FCRX_MASK;
		} else if (set1clr0 == 0) {
			tmpU32 &= ~XAE_FCC_FCRX_MASK;
		}
		sys_write32(tmpU32, dev_conf->base_addr+XAE_FCC_OFFSET);
	}

	/* Turn on/off promiscuous frame filtering (all frames are received ) */
	if (optionU32 & XAE_PROMISC_MASK) {
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_FMI_OFFSET);
		if (set1clr0 == 1) {
			tmpU32 |= XAE_FMI_PM_MASK;
		} else if (set1clr0 == 0) {
			tmpU32 &= ~XAE_FMI_PM_MASK;
		}
		sys_write32(tmpU32, dev_conf->base_addr+XAE_FMI_OFFSET);
	}

	/* Allow/disable broadcast address filtering */
	if (optionU32 & XAE_BROADCAST_MASK) {
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_RAF_OFFSET);
		if (set1clr0 == 1) {
			tmpU32 &= ~XAE_RAF_BCSTREJ_MASK;
		} else if (set1clr0 == 0) {
			tmpU32 |= XAE_RAF_BCSTREJ_MASK;
		}
		sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
	}

	/* Allow/disable multicast address filtering */
	tmpU32 = sys_read32(dev_conf->base_addr+XAE_RAF_OFFSET);
	if (set1clr0 == 1) {
		if (optionU32 & (XAE_MULTICAST_MASK | XAE_EXT_MULTICAST_MASK)) {
			tmpU32 &= ~XAE_RAF_MCSTREJ_MASK;
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		}
	} else if (set1clr0 == 0) {
		if ((optionU32 & XAE_MULTICAST_MASK)
				&& (optionU32 & XAE_EXT_MULTICAST_MASK)) {
			tmpU32 |= XAE_RAF_MCSTREJ_MASK;
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		}
	}

	/* Enable/disable extended multicast option */
	if (optionU32 & XAE_EXT_MULTICAST_MASK) {
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_RAF_OFFSET);
		if (set1clr0 == 1) {
			tmpU32 |= XAE_RAF_EMULTIFLTRENBL_MASK;
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		} else if (set1clr0 == 0) {
			tmpU32 &= ~XAE_RAF_EMULTIFLTRENBL_MASK;
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		}
	}

	/* Enable/disable extended transmit VLAN tag option */
	if (optionU32 & XAE_EXT_TXVLAN_TAG_MASK) {
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_RAF_OFFSET);
		if (set1clr0 == 1) {
			tmpU32 = (tmpU32 & ~XAE_RAF_TXVTAGMODE_MASK)
					| (XAE_DEFAULT_TXVTAG_MODE <<
					XAE_RAF_TXVTAGMODE_SHIFT);
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		} else if (set1clr0 == 0) {
			tmpU32 = (tmpU32 & ~XAE_RAF_TXVTAGMODE_MASK);
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		}
	}

	/* Enable/disable extended receive VLAN tag option */
	if (optionU32 & XAE_EXT_RXVLAN_TAG_MASK) {
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_RAF_OFFSET);
		if (set1clr0 == 1) {
			tmpU32 = (tmpU32 & ~XAE_RAF_RXVTAGMODE_MASK)
					| (XAE_DEFAULT_RXVTAG_MODE <<
					XAE_RAF_RXVTAGMODE_SHIFT);
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		} else if (set1clr0 == 0) {
			tmpU32 = (tmpU32 & ~XAE_RAF_RXVTAGMODE_MASK);
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		}
	}

	/* Enable/disable extended transmit VLAN strip option */
	if (optionU32 & XAE_EXT_TXVLAN_STRP_MASK) {
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_RAF_OFFSET);
		if (set1clr0 == 1) {
			tmpU32 = (tmpU32 & ~XAE_RAF_TXVSTRPMODE_MASK)
					| (XAE_DEFAULT_TXVSTRP_MODE <<
					XAE_RAF_TXVSTRPMODE_SHIFT);
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		} else if (set1clr0 == 0) {
			tmpU32 = (tmpU32 & ~XAE_RAF_TXVSTRPMODE_MASK);
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		}
	}

	/* Enable/disable extended receive VLAN strip option */
	if (optionU32 & XAE_EXT_RXVLAN_STRP_MASK) {
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_RAF_OFFSET);
		if (set1clr0 == 1) {
			tmpU32 = (tmpU32 & ~XAE_RAF_RXVSTRPMODE_MASK)
					| (XAE_DEFAULT_RXVSTRP_MODE <<
					XAE_RAF_RXVSTRPMODE_SHIFT);
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		} else if (set1clr0 == 0) {
			tmpU32 = (tmpU32 & ~XAE_RAF_RXVSTRPMODE_MASK);
			sys_write32(tmpU32, dev_conf->base_addr+XAE_RAF_OFFSET);
		}
	}
}

/**
 * @brief 
 * Updates only the link speed bits [31:30] of temmc mode config
 * register offset 0x410. Use after obtain link speed from phy
 *
 * @param dev Pointer to the device data
 */
static int eth_axiemac_set_speed(const struct device *dev) {

	uint32_t EmmcReg;
	uint8_t PhyType;
	enum eth_xlnx_link_speed link_speed;
	uint8_t SetSpeed = 1;

	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);

	PhyType = dev_conf->phy_if_type;
	link_speed = dev_data->eff_link_speed;

	/*
	 * The following code checks for all allowable speed conditions before
	 * writing to the register. Please refer to the hardware specs for
	 * more information on it.
	 * For PHY type 1000Base-x, 10 and 100 Mbps are not supported.
	 * For soft/hard Axi Ethernet core, 1000 Mbps is supported in all PHY
	 * types except MII.
	 */
	if ((link_speed == LINK_10MBIT) || (link_speed == LINK_100MBIT)) {
		if (PhyType == XAE_PHY_TYPE_1000BASE_X) {
			SetSpeed = 0;
		}
	} else {
		if ((link_speed == LINK_1GBIT) && (PhyType == XAE_PHY_TYPE_MII)) {
			SetSpeed = 0;
		}
	}

	if (SetSpeed == 1) {
		/*
		 * Get the current contents of the EMAC config register and
		 * zero out speed bits
		 */
		EmmcReg = sys_read32(dev_conf->base_addr+XAE_EMMC_OFFSET);
		EmmcReg &= ~XAE_EMMC_LINKSPEED_MASK;

		switch (link_speed) {
		case LINK_10MBIT:
			/*for link_10Mbit , already cleared emmreg[31:30] = 00*/
			break;

		case LINK_100MBIT:
			EmmcReg |= XAE_EMMC_LINKSPD_100;
			break;

		case LINK_1GBIT:
			EmmcReg |= XAE_EMMC_LINKSPD_1000;
			break;

		default:
			return -EIO;
		}

		/* Set register and return */
		sys_write32(EmmcReg, dev_conf->base_addr+XAE_EMMC_OFFSET);
		return 0;
	} else {
		LOG_WRN("Speed not compatible with the Axi Ethernet Phy type\n");
		return -EIO;
	}
}

/**
 * @brief axi stream FIFO initialize function
 * Initializes the tx and rx fifo buffer
 *
 * @param dev Pointer to the device data
 * @retval 
 */
static void eth_axiemac_xStmFifo_init(const struct device *dev) {
	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	uint32_t tmpU32;

	XStrmFifo *fifoPtr = &(DEV_DATA(dev)->axififo);

	/* Clear instance memory */
	memset(fifoPtr, 0, sizeof(XStrmFifo));

	fifoPtr->base_addr = dev_conf->fifo_addr;

	/* reset tx fifo  */
	sys_write32(XLLF_TDFR_RESET_MASK, dev_conf->fifo_addr+XLLF_TDFR_OFFSET);
	/* reset rx fifo */
	sys_write32(XLLF_RDFR_RESET_MASK, dev_conf->fifo_addr+XLLF_RDFR_OFFSET);
	/*
	 * Reset the core and generate the external reset by writing to
	 * the Local Link/AXI Streaming Reset Register.
	 */
	sys_write32(XLLF_RDFR_RESET_MASK, dev_conf->fifo_addr+XLLF_LLR_OFFSET);

	fifoPtr->xStrm_rx.HeadIndex = FIFO_WIDTH_BYTES;
	fifoPtr->xStrm_rx.FifoWidth = FIFO_WIDTH_BYTES;
	fifoPtr->xStrm_tx.TailIndex = 0;
	fifoPtr->xStrm_tx.FifoWidth = FIFO_WIDTH_BYTES;

	/* Clear any pending FIFO interrupts */
	sys_write32(XLLF_INT_ALL_MASK, dev_conf->fifo_addr+XLLF_ISR_OFFSET);

	/* enable fifo interrupts */
	tmpU32 = sys_read32(dev_conf->fifo_addr+XLLF_IER_OFFSET);
	tmpU32 |= XLLF_INT_ALL_MASK;
	sys_write32(tmpU32, dev_conf->fifo_addr+XLLF_IER_OFFSET);

}

/**
 * @brief axi ethernet device err interrupt 
 * 
 *
 * @param dev Pointer to the device data
 * @retval 
 */
void eth_axiemac_err_isr(const struct device *dev) {
	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	uint32_t Pending;
  
	Pending = sys_read32(dev_conf->base_addr+XAE_IP_OFFSET);
	sys_write32(((Pending) & XAE_INT_ALL_MASK),
			dev_conf->base_addr+XAE_IS_OFFSET);
}

/**
 * @brief read data from pkt , 
 * asume bytes is less than XAE_MAX_FRAME_SIZE
 *
 * @param *fifoPtr Pointer to the xStmFifo
 * @param *pkt Pointer to the net_pkt
 * @param Bytes number of bytes to write or flush
 * @retval -EIO : 
 *    when net_pkt pointer is null or net_pkt_read fail
 * @retval 0 : successfully initiates hardware write
      
 */
static int xStrmFifo_Write(XStrmFifo *fifoPtr, struct net_pkt *pkt,
		uint32_t Bytes)
{

  static uint8_t tx_buf[XAE_MAX_FRAME_SIZE]__aligned(ETH_BUF_ALIGNMENT);
	uint32_t BytesRemaining = Bytes;
	uint32_t BytesWritten = 0;
	uint32_t FifoWordsToXfer;
	uint32_t PartialBytes;
	uint32_t i;
  uint8_t *SrcPtr = NULL;

	txStrm_t *tStrm = &(fifoPtr->xStrm_tx);

  if(pkt == NULL){
    return -EIO;
  }
  
	net_pkt_cursor_init(pkt);

	if (net_pkt_read(pkt, tx_buf, Bytes)) {
		LOG_ERR("net_pkt_read failed");
		return -EINVAL;
	}else{
    SrcPtr = &tx_buf[0];
  }

	while (BytesRemaining) {
		/* Case 1: The holding buffer is full
		 *
		 *   1) Write it to the fifo.
		 *   2) Fall through to transfer more bytes in this iteration.
		 */
		if (tStrm->TailIndex == tStrm->FifoWidth) {
			xStmFifo_iWrite_Aligned(fifoPtr,
					(uint32_t*) &(tStrm->AlignedBuffer.bytes[0]), 1);
			tStrm->TailIndex = 0;
		}
		/* Case 2: There are no bytes in the holding buffer and
		 *         the target buffer is 32 bit aligned and
		 *         the number of bytes remaining to transfer is greater
		 *         than or equal to the fifo width.
		 *
		 *   1) We can go fast by writing a long string of fifo words right out
		 *      of the source buffer into the fifo.
		 *   2) Loop back around to transfer the last few bytes.
		 */
		if ((tStrm->TailIndex == 0) && (BytesRemaining >= tStrm->FifoWidth)
				&& (((uintptr_t) SrcPtr & 0x3) == 0)) {

			FifoWordsToXfer = BytesRemaining / tStrm->FifoWidth;

			xStmFifo_iWrite_Aligned(fifoPtr, (uint32_t*) SrcPtr,
					FifoWordsToXfer);

			BytesWritten = FifoWordsToXfer * tStrm->FifoWidth;
			BytesRemaining -= BytesWritten;

			SrcPtr += BytesWritten;

		}
		/* Case 3: The alignment of the "galaxies" didn't occur in
		 *         Case 2 above, so we must pump the bytes through the
		 *         holding buffer.
		 *
		 *   1) Write bytes from the source buffer to the holding buffer
		 *   2) Loop back around and handle the rest of the transfer.
		 */
		else {
			i = tStrm->TailIndex;

			PartialBytes =
					(BytesRemaining <= (tStrm->FifoWidth - tStrm->TailIndex)) ?
							BytesRemaining :
							(tStrm->FifoWidth - tStrm->TailIndex);

			BytesRemaining -= PartialBytes;
			tStrm->TailIndex += PartialBytes;

			while (PartialBytes--) {
				tStrm->AlignedBuffer.bytes[i] = *SrcPtr;
				i++;
				SrcPtr++;
			}
		}
	}

	/* write the partial bytes */
	if (tStrm->TailIndex != 0) {
		xStmFifo_iWrite_Aligned(fifoPtr,
				(uint32_t*) (&(tStrm->AlignedBuffer.bytes[0])), 1);
		tStrm->TailIndex = 0;
	}
	/* Kick off the hw write */
	sys_write32(Bytes, fifoPtr->base_addr+XLLF_TLF_OFFSET);

	return 0;

}

static uint32_t xStmFifo_RxOccupancy(XStrmFifo *fifoPtr) {
	uint32_t occupancy;

	occupancy = sys_read32(fifoPtr->base_addr + XLLF_RDFO_OFFSET);
	if (fifoPtr->xStrm_rx.FrmByteCnt) {
		occupancy +=
				(fifoPtr->xStrm_rx.FifoWidth - fifoPtr->xStrm_rx.HeadIndex);
	}
	return occupancy;
}

static uint32_t xStmFifo_RxGetLen(XStrmFifo *fifoPtr) {
	uint32_t len;

	fifoPtr->xStrm_rx.HeadIndex = fifoPtr->xStrm_rx.FifoWidth;
	len = sys_read32(fifoPtr->base_addr + XLLF_RLF_OFFSET);
	fifoPtr->xStrm_rx.FrmByteCnt = len;
	return len;

}

static void xStmFifo_iRead_Aligned(XStrmFifo *fifoPtr, uint32_t *BufPtr,
		uint32_t WordCount) 
{
	uint32_t WordsRemaining = WordCount;
	uint32_t *BufPtrIdx = (uint32_t *) BufPtr;

	while (WordsRemaining) {
		*BufPtrIdx = sys_read32(fifoPtr->base_addr+XLLF_RDFD_OFFSET);
		BufPtrIdx++;
		WordsRemaining--;
	}
}

static void xStmFifo_iWrite_Aligned(XStrmFifo *fifoPtr, uint32_t *BufPtr,
		uint32_t WordCount) 
{
	uint32_t WordsRemaining = WordCount;
	uint32_t *BufPtrIdx = (uint32_t *) BufPtr;

	while (WordsRemaining) {
		sys_write32(*BufPtrIdx, fifoPtr->base_addr+XLLF_TDFD_OFFSET);
		BufPtrIdx++;
		WordsRemaining--;
	}
}



static int xStrmFifo_getTxVacancy(XStrmFifo *fifoPtr)
{
	int tmpInt;
	tmpInt = sys_read32(fifoPtr->base_addr+XLLF_TDFV_OFFSET);
	return (tmpInt - ((fifoPtr->xStrm_tx.TailIndex + 3) / 4));
}

/**
 * @brief write data from fifo's receive buffer to pkt
 *
 * @param *fifoPtr Pointer to the xStmFifo
 * @param *pkt Pointer to the net_pkt
 * @param Bytes number of bytes to write or flush
 * @retval 
 */
static void xStrmFifo_Read(XStrmFifo *fifoPtr, uint8_t *buf, uint32_t Bytes) {

	uint32_t BytesRemaining = Bytes;
	uint32_t BytesWritten = 0;
	uint32_t FifoWordsToXfer;
	uint32_t PartialBytes;
	uint32_t i;

	uint8_t *tmpU8Ptr = buf;

	rxStrm_t *rStrm = &(fifoPtr->xStrm_rx);

	while (BytesRemaining) {
		/* Case 1: There are bytes in the holding buffer
		 *
		 *   1) Read the bytes from the holding buffer to the target buffer.
		 *   2) Loop back around and handle the rest of the transfer.
		 */
		if (rStrm->HeadIndex != rStrm->FifoWidth) {
			i = rStrm->HeadIndex;
			PartialBytes =
					(BytesRemaining <= (rStrm->FifoWidth - rStrm->HeadIndex)) ?
							BytesRemaining :
							(rStrm->FifoWidth - rStrm->HeadIndex);
			rStrm->HeadIndex += PartialBytes;
			BytesRemaining -= PartialBytes;
			rStrm->FrmByteCnt -= PartialBytes;
			while (PartialBytes--) {
				*tmpU8Ptr = rStrm->AlignedBuffer.bytes[i];
				tmpU8Ptr++;
				i++;
			}
		}
		/* Case 2: There are no more bytes in the holding buffer and
		 *         the target buffer is 32 bit aligned and
		 *         the number of bytes remaining to transfer is greater
		 *         than or equal to the fifo width.
		 *
		 *   1) We can go fast by reading a long string of fifo words right out
		 *      of the fifo into the target buffer.
		 *   2) Loop back around to transfer the last few bytes.
		 */
		else if (BytesRemaining >= rStrm->FifoWidth) {
			FifoWordsToXfer = BytesRemaining / rStrm->FifoWidth;

			/* fifo read function required 32bits word aligned address.
			 * take data to a aligned buffer first, not ideal need to find
			 * better way.
			 */
			xStmFifo_iRead_Aligned(fifoPtr, (uint32_t*) tmpU8Ptr,
					FifoWordsToXfer);
			BytesWritten = FifoWordsToXfer * rStrm->FifoWidth;
			BytesRemaining -= BytesWritten;
			rStrm->FrmByteCnt -= BytesWritten;
			tmpU8Ptr += BytesWritten;
		}
		/* Case 3: There are no more bytes in the holding buffer and
		 *         the number of bytes remaining to transfer is less than
		 *         the fifo width or
		 *         things just don't line up.
		 *
		 *   1) Fill the holding buffer.
		 *   2) Loop back around and handle the rest of the transfer.
		 */
		else {
			/*
			 * At the tail end, read one fifo word into the local holding
			 * buffer and loop back around to take care of the transfer.
			 */
			xStmFifo_iRead_Aligned(fifoPtr, (uint32_t*)&(rStrm->AlignedBuffer.bytes[0]),
					1);
			rStrm->HeadIndex = 0;
		}
	}
	if ((rStrm->FrmByteCnt) == 0) {
		rStrm->HeadIndex = 0;
	}
}


/**
 * @brief flush data from fifo's receive buffer 
 *
 * @param *fifoPtr Pointer to the xStmFifo
 * @param Bytes number of bytes to write or flush
 * @retval 
 */
static void xStrmFifo_flush(XStrmFifo *fifoPtr, uint32_t Bytes) {

	uint32_t BytesRemaining = Bytes;
	uint32_t BytesWritten = 0;
	uint32_t FifoWordsToXfer;
	uint32_t PartialBytes;
	uint32_t i;
  uint8_t tmpU8;

	rxStrm_t *rStrm = &(fifoPtr->xStrm_rx);

	while (BytesRemaining) {
		/* Case 1: There are bytes in the holding buffer
		 *
		 *   1) Read the bytes from the holding buffer to the target buffer.
		 *   2) Loop back around and handle the rest of the transfer.
		 */
		if (rStrm->HeadIndex != rStrm->FifoWidth) {
			i = rStrm->HeadIndex;
			PartialBytes =
					(BytesRemaining <= (rStrm->FifoWidth - rStrm->HeadIndex)) ?
							BytesRemaining :
							(rStrm->FifoWidth - rStrm->HeadIndex);
			rStrm->HeadIndex += PartialBytes;
			BytesRemaining -= PartialBytes;
			rStrm->FrmByteCnt -= PartialBytes;
			while (PartialBytes--) {
				tmpU8 = rStrm->AlignedBuffer.bytes[i];
				i++;
			}
		}
		/* Case 2: There are no more bytes in the holding buffer and
		 *         the target buffer is 32 bit aligned and
		 *         the number of bytes remaining to transfer is greater
		 *         than or equal to the fifo width.
		 *
		 *   1) We can go fast by reading a long string of fifo words right out
		 *      of the fifo into the target buffer.
		 *   2) Loop back around to transfer the last few bytes.
		 */
		else if (BytesRemaining >= rStrm->FifoWidth) {
			FifoWordsToXfer = BytesRemaining / rStrm->FifoWidth;
      uint32_t WordsRemaining = FifoWordsToXfer;
			/* fifo read function required 32bits word aligned address.
			 * take data to a aligned buffer first, not ideal need to find
			 * better way.
			 */
      while(WordsRemaining){
        tmpU8 = sys_read32(fifoPtr->base_addr+XLLF_RDFD_OFFSET);    
        WordsRemaining--;
      }
			BytesWritten = FifoWordsToXfer * rStrm->FifoWidth;
			BytesRemaining -= BytesWritten;
			rStrm->FrmByteCnt -= BytesWritten;
		}
		/* Case 3: There are no more bytes in the holding buffer and
		 *         the number of bytes remaining to transfer is less than
		 *         the fifo width or
		 *         things just don't line up.
		 *
		 *   1) Fill the holding buffer.
		 *   2) Loop back around and handle the rest of the transfer.
		 */
		else {
			/*
			 * At the tail end, read one fifo word into the local holding
			 * buffer and loop back around to take care of the transfer.
			 */
			xStmFifo_iRead_Aligned(fifoPtr, (uint32_t*)&(rStrm->AlignedBuffer.bytes[0]),
					1);
			rStrm->HeadIndex = 0;
		}
	}
	if ((rStrm->FrmByteCnt) == 0) {
		rStrm->HeadIndex = 0;
	}
}


/**
 * @brief axi stream fifo receive isr  
 * check receive fifo for occupancy ,
 * unload data from fifo and propagate the 
 * received packet to the network stack.
 * if length is invalid, the fifo will be flushed.
 *
 * @param dev Pointer to the device data
 * interrupt pending register
 */
static void xStrmFifo_recv_handler(const struct device *dev) {

	uint32_t frame_length;
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	XStrmFifo *fifoPtr = &(DEV_DATA(dev)->axififo);
	static uint8_t rx_buf[XAE_MAX_FRAME_SIZE]__aligned(ETH_BUF_ALIGNMENT);
  struct net_pkt *pkt;
  
	/* While there is data in the fifo ... */
	while (xStmFifo_RxOccupancy(fifoPtr)) {
		/* find packet length */
		frame_length = xStmFifo_RxGetLen(fifoPtr);
		if (frame_length > XAE_MAX_FRAME_SIZE) {
			LOG_ERR("Received frame size %d exceeds max frame size %d ",
					frame_length, XAE_MAX_FRAME_SIZE);
#ifdef CONFIG_NET_STATISTICS_ETHERNET
			dev_data->stats.errors.rx++;
#endif
      /* flush receive fifo */
      xStrmFifo_flush(fifoPtr, frame_length);
		}else{

      xStrmFifo_Read(fifoPtr, rx_buf, frame_length);
      /*
       * Allocate a destination packet from the network stack
       * now that the total frame length is known.
       */
      pkt = net_pkt_rx_alloc_with_buffer(dev_data->iface, frame_length,
          AF_UNSPEC, 0, K_NO_WAIT);

      if (pkt == NULL) {
        LOG_ERR("RX packet buffer alloc failed: %u bytes", frame_length);
#ifdef CONFIG_NET_STATISTICS_ETHERNET
        dev_data->stats.errors.rx++;
        dev_data->stats.error_details.rx_no_buffer_count++;
#endif
      }

      if (pkt != NULL) {
        net_pkt_write(pkt, (uint8_t*)rx_buf,
            frame_length);
        
        /* Propagate the received packet to the network stack */
        if (net_recv_data(dev_data->iface, pkt) < 0) {
          LOG_ERR("%s RX packet hand-over to IP stack failed", dev->name);
          net_pkt_unref(pkt);
          pkt = NULL;
        }
  #ifdef CONFIG_NET_STATISTICS_ETHERNET
        else {
          dev_data->stats.bytes.received += frame_length;
          dev_data->stats.pkts.rx++;
        }
  #endif
      }
    }
	}
}


/**
 * @brief axi stream fifo error isr  
 * Checks for indications of errors
 * resets fifo for hard error 
 * 
 * @param dev Pointer to the device data
 * @param pending_intr value read from 
 * interrupt pending register
 */
static void fifo_error_handler(const struct device *dev, uint32_t pending_intr) {
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	XStrmFifo *fifoPtr = &(DEV_DATA(dev)->axififo);

	if (pending_intr & XLLF_INT_RPURE_MASK) {
		LOG_ERR("llfifo: Rx under-read error");
	}
	if (pending_intr & XLLF_INT_RPORE_MASK) {
		LOG_ERR("llfifo: Rx over-read error");
	}
	if (pending_intr & XLLF_INT_RPUE_MASK) {
		LOG_ERR("llfifo: Rx fifo empty");
	}
	if (pending_intr & XLLF_INT_TPOE_MASK) {
		LOG_ERR("llfifo: Tx fifo overrun");
	}
	if (pending_intr & XLLF_INT_TSE_MASK) {
		LOG_ERR("llfifo: Tx length mismatch");
	}

	/* Reset the tx or rx side of the fifo as needed */
	if (pending_intr & XLLF_INT_RXERROR_MASK) {
		sys_write32(XLLF_INT_RRC_MASK, fifoPtr->base_addr+XLLF_ISR_OFFSET);
		sys_write32(XLLF_RDFR_RESET_MASK, fifoPtr->base_addr+XLLF_RDFR_OFFSET);
	}

	if (pending_intr & XLLF_INT_TXERROR_MASK) {
		sys_write32(XLLF_INT_TRC_MASK, fifoPtr->base_addr+XLLF_ISR_OFFSET);
		sys_write32(XLLF_TDFR_RESET_MASK, fifoPtr->base_addr+XLLF_TDFR_OFFSET);
		dev_data->tx_err = true;
		k_sem_give(&dev_data->tx_sem);
	}
}

#ifdef CONFIG_NET_STATISTICS_ETHERNET
struct net_stats_eth *eth_axiemac_stats(const struct device *dev) {
	return &(DEV_DATA(dev)->stats);
}
#endif

