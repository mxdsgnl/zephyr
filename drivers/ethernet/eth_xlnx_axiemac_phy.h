/* zynq parrot axiemac Ethernet Controller
 *
 * 
 *
 * 
 */
#ifndef ETH_XLNX_AXIEMAC_PHY_H_
#define ETH_XLNX_AXIEMAC_PHY_H_

#include <kernel.h>
#include <zephyr/types.h>

/*defines from axiethernet_v5_8/src/xaxiemacif_physpeed.c start*/
#define PHY_R0_ISOLATE  						0x0400
#define PHY_DETECT_REG  						1
#define PHY_IDENTIFIER_1_REG					2
#define PHY_IDENTIFIER_2_REG					3
#define PHY_DETECT_MASK 						0x1808

/* Loop counters to check for reset done
 */
#define RESET_TIMEOUT							0xFFFF
#define AUTO_NEG_TIMEOUT 						0x00FFFFFF

#define PHY_XILINX_PCS_PMA_ID1			0x0174
#define PHY_XILINX_PCS_PMA_ID2			0x0C00

/*defines from axiethernet_v5_8/src/xaxiemacif_physpeed.c end*/

/* Event codes used to indicate a particular state change to the driver */
#define PHY_XLNX_AXIEMAC_EVENT_LINK_SPEED_CHANGED		(1 << 0)
#define PHY_XLNX_AXIEMAC_EVENT_LINK_STATE_CHANGED		(1 << 1)
#define PHY_XLNX_AXIEMAC_EVENT_AUTONEG_COMPLETE		(1 << 2)

#define PHY_KSZ9031_BASIC_MODE_REG                      0x0000
#define PHY_KSZ9031_BASIC_MODE_RESET_MASK               (1<<15)
#define PHY_KSZ9031_BASIC_MODE_SPEED_MSB_MASK           (1<<6)
#define PHY_KSZ9031_BASIC_MODE_SPEED_LSB_MASK           (1<<13)
#define PHY_KSZ9031_BASIC_MODE_AUTONEGEN_MASK           (1<<12)
#define PHY_KSZ9031_BASIC_MODE_PD_MASK                  (1<<11)
#define PHY_KSZ9031_BASIC_MODE_AUTONEG_RSTRT_MASK       (1<<9)
#define PHY_KSZ9031_BASIC_MODE_DUPLEX_MASK              (1<<8)

#define PHY_KSZ9031_BASIC_STATUS_REG                    0x0001
#define PHY_KSZ9031_BASIC_STATUS_AUTONEG_COMPLT_MASK    (1<<5)
#define PHY_KSZ9031_BASIC_STATUS_LINKSTAT_MASK          (1<<2)

#define PHY_KSZ9031_PHY_ID1_REG                         0x0002
#define PHY_KSZ9031_PHY_ID2_REG                         0x0003
#define PHY_KSZ9031_AUTONEG_ADV_REG                     0x0004
#define PHY_KSZ9031_AUTONEG_ADV_100FD_MASK              (1<<8)
#define PHY_KSZ9031_AUTONEG_ADV_100HD_MASK              (1<<7)
#define PHY_KSZ9031_AUTONEG_ADV_10FD_MASK               (1<<6)
#define PHY_KSZ9031_AUTONEG_ADV_10HD_MASK               (1<<5)

#define PHY_KSZ9031_LINK_PARTNER_REG                    0x0005
#define PHY_KSZ9031_AUTONEG_EXP_REG                     0x0006
#define PHY_KSZ9031_AUTONEG_NEXT_PG_REG                 0x0007
#define PHY_KSZ9031_LINK_PARTNER_NEXT_PG_REG            0x0008 
#define PHY_KSZ9031_1000BASET_CTRL_REG                  0x0009 
#define PHY_KSZ9031_1000BASET_CTRL_FDUPLEX_MASK         (1<<9)
#define PHY_KSZ9031_1000BASET_CTRL_HDUPLEX_MASK         (1<<8)

#define PHY_KSZ9031_1000BASET_STAT_REG                  0x000A 
#define PHY_KSZ9031_INTRPT_CTRL_STAT_REG                0x001B 
#define PHY_KSZ9031_INTRPT_CTRL_STAT_JABBER_MASK              (1<<7)
#define PHY_KSZ9031_INTRPT_CTRL_STAT_RXERR_MASK               (1<<6)
#define PHY_KSZ9031_INTRPT_CTRL_STAT_PGRECV_MASK              (1<<5)
#define PHY_KSZ9031_INTRPT_CTRL_STAT_PARALLEL_DET_MASK        (1<<4)
#define PHY_KSZ9031_INTRPT_CTRL_STAT_LINK_PARTNER_ACK_MASK    (1<<3)
#define PHY_KSZ9031_INTRPT_CTRL_STAT_LINK_DN_MASK             (1<<2)
#define PHY_KSZ9031_INTRPT_CTRL_STAT_REMOTE_FAULT_MASK             (1<<1)
#define PHY_KSZ9031_INTRPT_CTRL_STAT_LINK_UP_MASK             (1<<0)

#define PHY_KSZ9031_MMD_CTRL_REG                     0x000D 
#define PHY_KSZ9031_MMD_DATA_REG                     0x000E

#define PHY_KSZ9031_VENDOR_PHY_CTRL_REG                0x001F 
#define PHY_KSZ9031_VENDOR_PHY_CTRL_SPD1000_MASK            (1<<6)
#define PHY_KSZ9031_VENDOR_PHY_CTRL_SPD100_MASK             (1<<5)
#define PHY_KSZ9031_VENDOR_PHY_CTRL_SPD10_MASK              (1<<4)

#define PHY_MICROLCHIP_PHY_ID_MODEL_KSZ9031_MASK			0xFFFFFC00
#define PHY_MICROLCHIP_PHY_ID_MODEL_KSZ9031			      0x00221400 

enum eth_xlnx_link_speed {
	/* The values of this enum are consecutively numbered */
	LINK_DOWN = 0, LINK_10MBIT, LINK_100MBIT, LINK_1GBIT
};

/**
 * @brief Vendor-specific PHY management function pointer table struct
 *
 * Contains the PHY management function pointers for a specific PHY
 * make or model.
 */
struct phy_xlnx_axiemac_api {
	void (*phy_reset_func)(const struct device *dev);
	void (*phy_configure_func)(const struct device *dev);
	uint16_t (*phy_poll_status_change_func)(const struct device *dev);
	uint8_t (*phy_poll_link_status_func)(const struct device *dev);
	enum eth_xlnx_link_speed (*phy_poll_link_speed_func)(
			const struct device *dev);
};

/**
 * @brief Supported PHY list entry struct
 *
 * Contains the PHY management function pointers for a specific PHY
 * make or model.
 */
struct phy_xlnx_axiemac_dev {
	uint32_t phy_id;
	uint32_t phy_id_mask;
	struct phy_xlnx_axiemac_api *api;
	const char *identifier;
};

/* PHY identification function -> generic, not vendor-specific */
int axiemac_detect_phy(const struct device *dev);

#endif /* ETH_XLNX_AXIEMAC_PHY_H_ */
