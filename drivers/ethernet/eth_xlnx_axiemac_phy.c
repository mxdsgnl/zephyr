/* zynq parrot axiemac Ethernet Controller
 *
 * 
 *
 * 
 */
#include <zephyr.h>
#include <device.h>
#include <sys/__assert.h>
#include "eth_xlnx_axiemac_priv.h"
#define LOG_MODULE_NAME eth_bpaxiemac_phy
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* milli second sleep routineloop with 20MHz input clock
 loop parameter is simulated in modelsim with 20MHz cpu clock.
 replace this with zsleep once zsleep is working with zynq parrot */
void my_msleep(uint32_t ms) {
	volatile int count;
	uint32_t tmpU32;
	tmpU32 = ms * 4000;
	for (int i = 0; i < tmpU32; i++) {  //roughly 2 ms with 20MHz clk to bp core
		count++;
	}
}

/*****************************************************************************/
/* axiemac_PhyWrite port from XAxiEthernet_PhyWrite
 * XAxiEthernet_PhyWrite writes <i>PhyData</i> to the specified PHY register,
 * <i>RegiseterNum</i> on the PHY specified by <i>PhyAddress</i>. This Ethernet
 * driver does not require the device to be stopped before writing to the PHY.
 * It is the responsibility of the calling code to stop the device if it is
 * deemed necessary.
 *
 * Note that the Axi Ethernet hardware provides the ability to talk to a PHY
 * that adheres to the Media Independent Interface (MII) as defined in the
 * IEEE 802.3 standard.
 *
 * <b>It is important that calling code set up the MDIO clock with
 * XAxiEthernet_PhySetMdioDivisor() prior to accessing the PHY with this
 * function.</b>
 *
 * @param	InstancePtr is a pointer to the Axi Ethernet instance to be
 *		worked on.
 * @param	PhyAddress is the address of the PHY to be written (multiple
 *		PHYs supported).
 * @param	RegisterNum is the register number, 0-31, of the specific PHY
 *		register to write.
 * @param	PhyData is the 16-bit value that will be written to the
 *		register.
 *
 * @return	None.
 *
 * @note
 *
 * There is the possibility that this function will not return if the hardware
 * is broken (i.e., it never sets the status bit indicating that the write is
 * done). If this is of concern, the calling code should provide a mechanism
 * suitable for recovery.
 *
 ******************************************************************************/
static void axiemac_phy_write(const struct device *dev, uint32_t PhyAddress,
		uint32_t RegisterNum, uint16_t PhyData) {

	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);

	uint32_t tmpU32;
	uint32_t MdioCtrlReg = 0;



	/*
	 * Verify that each of the inputs are valid.
	 */
	__ASSERT(PhyAddress <= XAE_PHY_ADDR_LIMIT,
			"%s invalid PHY address %u, must be in range "
			"1 to 32, or 0 for auto-detection", dev->name, PhyAddress);

	__ASSERT(RegisterNum <= XAE_PHY_REG_NUM_LIMIT,
			"%s invalid PHY register address %u, must be in range "
			"0 to 31", dev->name);

	/*
	 * Wait till the MDIO interface is ready to accept a new transaction.
	 */
	uint32_t iter_max = 0;
	do {
		my_msleep(1);
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_MDIO_MCR_OFFSET);
		if ((tmpU32 & XAE_MDIO_MCR_READY_MASK) != 0) {
			break;
		}
		iter_max++;
	} while (iter_max < XAE_RST_DEFAULT_TIMEOUT_MVAL);

	__ASSERT(((tmpU32 & XAE_MDIO_MCR_READY_MASK) != 0),
			"%s MDIO interface is not ready after timeout = 1 sec", dev->name);


	MdioCtrlReg = ((PhyAddress << XAE_MDIO_MCR_PHYAD_SHIFT) &
	XAE_MDIO_MCR_PHYAD_MASK) | ((RegisterNum << XAE_MDIO_MCR_REGAD_SHIFT) &
	XAE_MDIO_MCR_REGAD_MASK) |
	XAE_MDIO_MCR_INITIATE_MASK |
	XAE_MDIO_MCR_OP_WRITE_MASK;

	sys_write32((uint32_t )PhyData, dev_conf->base_addr+XAE_MDIO_MWD_OFFSET);
	sys_write32(MdioCtrlReg, dev_conf->base_addr+XAE_MDIO_MCR_OFFSET);

	/*
	 * Wait till the MDIO interface is ready to accept a new transaction.
	 */
	iter_max = 0;
	do {
		my_msleep(1);
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_MDIO_MCR_OFFSET);
		if ((tmpU32 & XAE_MDIO_MCR_READY_MASK) != 0) {
			break;
		}
		iter_max++;
	} while (iter_max < XAE_RST_DEFAULT_TIMEOUT_MVAL);

	__ASSERT(((tmpU32 & XAE_MDIO_MCR_READY_MASK) != 0),
			"%s MDIO interface is not ready after timeout = 1 sec", dev->name);

}

/*****************************************************************************/
/*
 * axiemac_phy_read port from XAxiEthernet_PhyRead
 * XAxiEthernet_PhyRead reads the specified PHY register, <i>RegiseterNum</i>
 * on the PHY specified by <i>PhyAddress</i> into <i>PhyDataPtr</i>.
 * This Ethernet driver does not require the device to be stopped before reading
 * from the PHY. It is the responsibility of the calling code to stop the
 * device if it is deemed necessary.
 *
 * Note that the Axi Ethernet hardware provides the ability to talk to a PHY
 * that adheres to the Media Independent Interface (MII) as defined in the
 * IEEE 802.3 standard.
 *
 * <b>It is important that calling code set up the MDIO clock with
 * XAxiEthernet_PhySetMdioDivisor() prior to accessing the PHY with this
 * function.
 * </b>
 *
 * @param	InstancePtr is a pointer to the Axi Ethernet instance to be
 *		worked on.
 * @param	PhyAddress is the address of the PHY to be written (multiple
 *		PHYs supported).
 * @param	RegisterNum is the register number, 0-31, of the specific PHY
 *		register to write.
 * @param	PhyDataPtr is a reference to the location where the 16-bit
 *		result value is stored.
 *
 * @return	None.
 *
 *
 * @note
 *
 * There is the possibility that this function will not return if the hardware
 * is broken (i.e., it never sets the status bit indicating that the write is
 * done). If this is of concern, the calling code should provide a mechanism
 * suitable for recovery.
 *
 ******************************************************************************/
void axiemac_phy_read(const struct device *dev, uint32_t PhyAddress,
		uint32_t RegisterNum, uint16_t *PhyDataPtr) {
	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);

	uint32_t tmpU32;
	uint32_t MdioCtrlReg = 0;


	/*
	 * Verify that each of the inputs are valid.
	 */

	__ASSERT(PhyAddress <= XAE_PHY_ADDR_LIMIT,
			"%s invalid PHY address %u, must be in range "
			"1 to 32, or 0 for auto-detection", dev->name, PhyAddress);

	__ASSERT(RegisterNum <= XAE_PHY_REG_NUM_LIMIT,
			"%s invalid PHY register address %u, must be in range "
			"0 to 31", dev->name);


	/*
	 * Wait till the MDIO interface is ready to accept a new transaction.
	 */
	uint32_t iter_max = 0;
	do {
		my_msleep(1);
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_MDIO_MCR_OFFSET);
		if ((tmpU32 & XAE_MDIO_MCR_READY_MASK) != 0) {
			break;
		}
		iter_max++;
	} while (iter_max < XAE_RST_DEFAULT_TIMEOUT_MVAL);

	__ASSERT(((tmpU32 & XAE_MDIO_MCR_READY_MASK) != 0),
			"%s MDIO interface is not ready after timeout = 1 sec", dev->name);


	MdioCtrlReg = ((PhyAddress << XAE_MDIO_MCR_PHYAD_SHIFT) &
	XAE_MDIO_MCR_PHYAD_MASK)
			| ((RegisterNum << XAE_MDIO_MCR_REGAD_SHIFT)
					& XAE_MDIO_MCR_REGAD_MASK) |
			XAE_MDIO_MCR_INITIATE_MASK |
			XAE_MDIO_MCR_OP_READ_MASK;

	sys_write32(MdioCtrlReg, dev_conf->base_addr+XAE_MDIO_MCR_OFFSET);

	/*
	 * Wait till the MDIO interface is ready to accept a new transaction.
	 */
	iter_max = 0;
	do {
		my_msleep(1);
		tmpU32 = sys_read32(dev_conf->base_addr+XAE_MDIO_MCR_OFFSET);
		if ((tmpU32 & XAE_MDIO_MCR_READY_MASK) != 0) {
			break;
		}
		iter_max++;
	} while (iter_max < XAE_RST_DEFAULT_TIMEOUT_MVAL);

	__ASSERT(((tmpU32 & XAE_MDIO_MCR_READY_MASK) != 0),
			"%s MDIO interface is not ready after timeout = 1 sec", dev->name);


	/* Read data */
	tmpU32 = sys_read32(dev_conf->base_addr+XAE_MDIO_MRD_OFFSET);
	*PhyDataPtr = (uint16_t)(tmpU32 & 0x0000ffff);
}

/**
 * @brief Microchip ksz9031 PHY reset function
 * Reset function for the Microchip ksz9031 PHY
 *
 * @param dev Pointer to the device data
 */
static void phy_axiemac_microchip_ksz9031_reset(const struct device *dev) {

	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	uint16_t phy_data;
	uint32_t retries = 0;

	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_MODE_REG, &phy_data);

	phy_data |= PHY_KSZ9031_BASIC_MODE_RESET_MASK;

	axiemac_phy_write(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_MODE_REG, phy_data);

	do {
		axiemac_phy_read(dev, dev_data->phy_addr,
		PHY_KSZ9031_BASIC_MODE_REG, &phy_data);
	} while (((phy_data & PHY_KSZ9031_BASIC_MODE_RESET_MASK) != 0)
			&& (retries++ < 10));

	if (retries == 10) {
//		LOG_ERR("%s reset PHY address %hhu (MICROCHIP KSZ9031) timed out",
//				dev->name, dev_data->phy_addr);
	}
	my_msleep(1);
}

/**
 * @brief Microchip KSZ9031 configuration function
 * 
 * @param dev Pointer to the device data
 */
static void phy_axiemac_microchip_ksz9031_cfg(const struct device *dev) {
	const struct eth_axiemac_dev_config *dev_conf = DEV_CFG(dev);
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	
	uint16_t tmpPhyReg0, tmpPhyReg4, tmpPhyReg9;

	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_MODE_REG, &tmpPhyReg0);
	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_1000BASET_CTRL_REG, &tmpPhyReg9);
	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_AUTONEG_ADV_REG, &tmpPhyReg4);

	/* config gmii/rgmii timing for enclustra board */
	//Ctrl Delay
	uint16_t RxCtrlDelay = 7; //0..15
	uint16_t TxCtrlDelay = 7; //0..15
	//procedure to write mmd register
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_CTRL_REG, 0x02);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_DATA_REG, 0x04);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_CTRL_REG,
			0x4002);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_DATA_REG,
			(TxCtrlDelay + (RxCtrlDelay << 4)));

	//Data Delay
	uint16_t RxDataDelay = 7; //0..15
	uint16_t TxDataDelay = 7; //0..15
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_CTRL_REG, 0x02);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_DATA_REG, 0x05);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_CTRL_REG,
			0x4002);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_DATA_REG,
			(RxDataDelay + (RxDataDelay << 4) + (RxDataDelay << 8)
					+ (RxDataDelay << 12)));

	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_CTRL_REG, 0x02);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_DATA_REG, 0x06);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_CTRL_REG,
			0x4002);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_DATA_REG,
			(TxDataDelay + (TxDataDelay << 4) + (TxDataDelay << 8)
					+ (TxDataDelay << 12)));

	//Clock Delay
	uint16_t RxClockDelay = 31; //0..31
	uint16_t TxClockDelay = 31; //0..31
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_CTRL_REG, 0x02);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_DATA_REG, 0x08);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_CTRL_REG,
			0x4002);
	axiemac_phy_write(dev, dev_data->phy_addr, PHY_KSZ9031_MMD_DATA_REG,
			(RxClockDelay + (TxClockDelay << 5)));
	/* end config gmii/rgmii timing for enclustra board  */

	/* Configure link advertisement */
	if (dev_conf->max_link_speed == LINK_100MBIT) {
		tmpPhyReg0 &= ~PHY_KSZ9031_BASIC_MODE_SPEED_MSB_MASK;
		tmpPhyReg0 |= PHY_KSZ9031_BASIC_MODE_SPEED_LSB_MASK;
		tmpPhyReg9 &= ~(PHY_KSZ9031_1000BASET_CTRL_FDUPLEX_MASK |
		PHY_KSZ9031_1000BASET_CTRL_HDUPLEX_MASK);

		if (dev_conf->enable_fdx) {
			tmpPhyReg0 |= PHY_KSZ9031_BASIC_MODE_DUPLEX_MASK;
			tmpPhyReg4 |= PHY_KSZ9031_AUTONEG_ADV_100FD_MASK;
			tmpPhyReg4 &= ~PHY_KSZ9031_AUTONEG_ADV_100HD_MASK;
		} else {
			tmpPhyReg0 &= ~PHY_KSZ9031_BASIC_MODE_DUPLEX_MASK;
			tmpPhyReg4 &= ~PHY_KSZ9031_AUTONEG_ADV_100FD_MASK;
			tmpPhyReg4 |= PHY_KSZ9031_AUTONEG_ADV_100HD_MASK;
		}

	} else if (dev_conf->max_link_speed == LINK_10MBIT) {
		tmpPhyReg0 &= ~PHY_KSZ9031_BASIC_MODE_SPEED_MSB_MASK;
		tmpPhyReg0 &= ~PHY_KSZ9031_BASIC_MODE_SPEED_LSB_MASK;
		tmpPhyReg9 &= ~(PHY_KSZ9031_1000BASET_CTRL_FDUPLEX_MASK |
		PHY_KSZ9031_1000BASET_CTRL_HDUPLEX_MASK);
		tmpPhyReg4 &= ~PHY_KSZ9031_AUTONEG_ADV_100FD_MASK;
		tmpPhyReg4 &= ~PHY_KSZ9031_AUTONEG_ADV_100HD_MASK;
		if (dev_conf->enable_fdx) {
			tmpPhyReg0 |= PHY_KSZ9031_BASIC_MODE_DUPLEX_MASK;
			tmpPhyReg4 |= PHY_KSZ9031_AUTONEG_ADV_10FD_MASK;
			tmpPhyReg4 &= ~PHY_KSZ9031_AUTONEG_ADV_10HD_MASK;
		} else {
			tmpPhyReg0 &= ~PHY_KSZ9031_BASIC_MODE_DUPLEX_MASK;
			tmpPhyReg4 |= PHY_KSZ9031_AUTONEG_ADV_10HD_MASK;
			tmpPhyReg4 &= ~PHY_KSZ9031_AUTONEG_ADV_10FD_MASK;
		}
	}
	/* Enable auto-negotiation */
	tmpPhyReg0 |= PHY_KSZ9031_BASIC_MODE_AUTONEGEN_MASK;
	/* Write config */
	axiemac_phy_write(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_MODE_REG, tmpPhyReg0);
	axiemac_phy_write(dev, dev_data->phy_addr,
	PHY_KSZ9031_AUTONEG_ADV_REG, tmpPhyReg4);
	axiemac_phy_write(dev, dev_data->phy_addr,
	PHY_KSZ9031_1000BASET_CTRL_REG, tmpPhyReg9);
	/* restart autoneg bit is self clear*/
	tmpPhyReg0 |= PHY_KSZ9031_BASIC_MODE_AUTONEG_RSTRT_MASK;
	axiemac_phy_write(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_MODE_REG, tmpPhyReg0);

	dev_data->eff_link_speed = LINK_DOWN;
	my_msleep(10);
}

/**
 * @brief Microchip KSZ9031 PHY status change polling function
 *
 * @param dev Pointer to the device data
 * @return A set of bits indicating whether one or more of the following
 *         events has occurred: auto-negotiation completed, link state
 *         changed, link speed changed.
 */
static uint16_t phy_axiemac_microchip_ksz9031_sc(const struct device *dev) {
	
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	uint16_t phy_data;
	uint16_t phy_status = 0;
	/*
	 * The relevant status bits are obtained from the Interrupt
	 * CtrlStatus Register 0x1B. The lower byte of the register's data word
	 * contains the status bits which are set regardless of whether
	 * the corresponding interrupt enable bits are set in the upper
	 * byte or not , the lower byte is read clear
	 */
	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_INTRPT_CTRL_STAT_REG, &phy_data);

	if ((phy_data & (PHY_KSZ9031_INTRPT_CTRL_STAT_LINK_DN_MASK |
	PHY_KSZ9031_INTRPT_CTRL_STAT_LINK_UP_MASK)) != 0) {
		phy_status |= PHY_XLNX_AXIEMAC_EVENT_LINK_STATE_CHANGED;
	}

	/* Any link failure condition is latched in the status register. Reading
	 the register twice will always return the actual link status */
	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_STATUS_REG, &phy_data);
	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_STATUS_REG, &phy_data);

	if ((phy_data & PHY_KSZ9031_BASIC_STATUS_AUTONEG_COMPLT_MASK) != 0) {
		phy_status |= PHY_XLNX_AXIEMAC_EVENT_AUTONEG_COMPLETE;
	}

	return phy_status;
}

/**
 * @brief Microchip KSZ9031 PHY link status polling function
 *
 * @param dev Pointer to the device data
 * @return 1 if the PHY indicates link up, 0 if the link is down
 */
static uint8_t phy_axiemac_microchip_ksz9031_poll_lsts(const struct device *dev) {
	
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	uint16_t phy_data;

	/* When basic mode register bit 11 power down is set to 1,
	 the link-down status might not get updated in the PHY register.
	 Software should note link is down and should not rely on the
	 PHY register link status. */
	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_MODE_REG, &phy_data);
	if (phy_data & PHY_KSZ9031_BASIC_MODE_PD_MASK) {
		return 0;
	}
	/* Any link failure condition is latched in the status register. Reading
	 the register twice will always return the actual link status */
	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_STATUS_REG, &phy_data);
	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_STATUS_REG, &phy_data);

	if ((phy_data & PHY_KSZ9031_BASIC_STATUS_LINKSTAT_MASK) != 0) {
		return 1;
	}
	return 0;
}

/**
 * @brief Microchip KSZ9031 PHY link speed polling function
 *
 * @param dev Pointer to the device data
 * @return    Enum containing the current link speed reported by the PHY
 */
static enum eth_xlnx_link_speed phy_axiemac_microchip_ksz9031_poll_lspd(
		const struct device *dev) {
	
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);
	enum eth_xlnx_link_speed link_speed;
	uint16_t phy_data;

	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_MODE_REG, &phy_data);

	if ((phy_data & PHY_KSZ9031_BASIC_MODE_PD_MASK) != 0) {
		return LINK_DOWN;
	}

	/* Any link failure condition is latched in the status register. Reading
	 the register twice will always return the actual link status */
	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_STATUS_REG, &phy_data);
	axiemac_phy_read(dev, dev_data->phy_addr,
	PHY_KSZ9031_BASIC_STATUS_REG, &phy_data);

	if ((phy_data & PHY_KSZ9031_BASIC_STATUS_LINKSTAT_MASK) != 0) {

		axiemac_phy_read(dev, dev_data->phy_addr,
		PHY_KSZ9031_VENDOR_PHY_CTRL_REG, &phy_data);

		/* check current speed */
		if ((phy_data & PHY_KSZ9031_VENDOR_PHY_CTRL_SPD1000_MASK) != 0) {
			link_speed = LINK_1GBIT;
		} else if ((phy_data & PHY_KSZ9031_VENDOR_PHY_CTRL_SPD100_MASK) != 0) {
			link_speed = LINK_100MBIT;
		} else if ((phy_data & PHY_KSZ9031_VENDOR_PHY_CTRL_SPD10_MASK) != 0) {
			link_speed = LINK_10MBIT;
		} else {
			link_speed = LINK_DOWN;
		}
	}
  return LINK_DOWN;
}

static struct phy_xlnx_axiemac_api phy_axiemac_microchip_ksz9031_api = {
		.phy_reset_func = phy_axiemac_microchip_ksz9031_reset,
		.phy_configure_func = phy_axiemac_microchip_ksz9031_cfg,
		.phy_poll_status_change_func = phy_axiemac_microchip_ksz9031_sc,
		.phy_poll_link_status_func = phy_axiemac_microchip_ksz9031_poll_lsts,
		.phy_poll_link_speed_func = phy_axiemac_microchip_ksz9031_poll_lspd };

/**
 * @brief Top-level table of supported PHYs
 * Top-level table of PHYs supported by the GEM driver. Contains 1..n
 * supported PHY specifications, consisting of the PHY ID plus a mask
 * for masking out variable parts of the PHY ID such as hardware revisions,
 * as well as a textual description of the PHY model and a pointer to
 * the corresponding PHY management function pointer table.
 */
static struct phy_xlnx_axiemac_dev phy_xlnx_axiemac_supported_devs[] = { {
		.phy_id = PHY_MICROLCHIP_PHY_ID_MODEL_KSZ9031, .phy_id_mask =
		PHY_MICROLCHIP_PHY_ID_MODEL_KSZ9031_MASK, .api =
				&phy_axiemac_microchip_ksz9031_api, .identifier =
				"Microchip ksz9031" } };

/**
 * @brief Top-level PHY detection function
 * Top-level PHY detection function called by the axiemac driver 
 * for the current axiemac device instance. 
 *
 * @param dev Pointer to the device data
 * @retval    -ENOTSUP if PHY is detected but not in the supported phy list       
 * @retval    -EIO if no (supported) PHY was detected
 * @retval    0 if a supported PHY has been detected
 */
int axiemac_detect_phy(const struct device *dev) {

	
	struct eth_axiemac_dev_data *dev_data = DEV_DATA(dev);

	uint16_t phy_reg;
	uint32_t phy_id;
	uint32_t phy_addr;

	dev_data->phy_addr = 0;
	dev_data->phy_id = 0;
	dev_data->phy_access_api = NULL;

	for (phy_addr = XAE_PHY_ADDR_LIMIT; phy_addr > 0; phy_addr--) {
		axiemac_phy_read(dev, phy_addr, PHY_DETECT_REG, &phy_reg);

		/* phy detect used in xilinx xaxiemacif_physpeed by reading phy reg 1  */
		if ((phy_reg != 0xFFFF)
				&& ((phy_reg & PHY_DETECT_MASK) == PHY_DETECT_MASK)) {
			/* Found a valid PHY address */

			axiemac_phy_read(dev, phy_addr, PHY_IDENTIFIER_1_REG, &phy_reg);
			phy_id = (((uint32_t) phy_reg << 16) & 0xFFFF0000);

			axiemac_phy_read(dev, phy_addr, PHY_IDENTIFIER_2_REG, &phy_reg);
			phy_id |= ((uint32_t) phy_reg & 0x0000FFFF);

			for (int i = 0; i < ARRAY_SIZE(phy_xlnx_axiemac_supported_devs);
					i++) {
				if ((phy_id & phy_xlnx_axiemac_supported_devs[i].phy_id_mask)
						== phy_xlnx_axiemac_supported_devs[i].phy_id) {
					dev_data->phy_addr = phy_addr;
					dev_data->phy_id = phy_id;
					dev_data->phy_access_api =
							phy_xlnx_axiemac_supported_devs[i].api;

					return 0;
				}
			}
			dev_data->phy_addr = phy_addr;
			dev_data->phy_id = phy_id;
//      LOG_DBG("WARNING: Not a supported Ethernet PHY.\r\n");
			return -ENOTSUP;
		}
	}

	LOG_DBG(
			"XAxiEthernet detect_phy: No PHY detected.  Assuming a PHY at address 0\r\n");
	/* default to zero */
	return -EIO;
}
