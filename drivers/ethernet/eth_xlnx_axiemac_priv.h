/* zynq parrot axiemac Ethernet Controller
 *
 * 
 *
 * 
 */
#ifndef ETH_XLNX_AXIEMAC_PRIV_H_
#define ETH_XLNX_AXIEMAC_PRIV_H_

#define DT_DRV_COMPAT xlnx_axiemac

#include <kernel.h>
#include <zephyr/types.h>
#include <net/net_pkt.h>
#include "eth_xlnx_axiemac_phy.h"

#define DEV_DATA(dev) \
	((struct eth_axiemac_dev_data *)((struct device*)dev)->data)
#define DEV_CFG(dev) \
	((const struct eth_axiemac_dev_config *const)((struct device*)dev)->config)

/* defines from from \libsrc\llfifo_v5_5\src\xllfifo_hw.h */
#define XLLF_ISR_OFFSET           0x00000000  /**< Interrupt Status */
#define XLLF_IER_OFFSET           0x00000004  /**< Interrupt Enable */

#define XLLF_TDFR_OFFSET          0x00000008  /**< Transmit Reset */
#define XLLF_TDFV_OFFSET          0x0000000c  /**< Transmit Vacancy */
#define XLLF_TDFD_OFFSET          0x00000010  /**< Transmit Data */
#define XLLF_AXI4_TDFD_OFFSET     0x00000000  /**< Axi4 Transmit Data */
#define XLLF_TLF_OFFSET           0x00000014  /**< Transmit Length */

#define XLLF_RDFR_OFFSET          0x00000018  /**< Receive Reset */
#define XLLF_RDFO_OFFSET          0x0000001c  /**< Receive Occupancy */
#define XLLF_RDFD_OFFSET          0x00000020  /**< Receive Data */
#define XLLF_AXI4_RDFD_OFFSET 	   0x00001000  /**< Axi4 Receive Data */
#define XLLF_RLF_OFFSET           0x00000024  /**< Receive Length */
#define XLLF_LLR_OFFSET           0x00000028  /**< Local Link Reset */
#define XLLF_TDR_OFFSET           0x0000002C  /**< Transmit Destination  */
#define XLLF_RDR_OFFSET           0x00000030  /**< Receive Destination  */

/*@}*/

/* Register masks. The following constants define bit locations of various
 * control bits in the registers. Constants are not defined for those registers
 * that have a single bit field representing all 32 bits. For further
 * information on the meaning of the various bit masks, refer to the HW spec.
 */

/** @name Interrupt bits
 *  These bits are associated with the XLLF_IER_OFFSET and XLLF_ISR_OFFSET
 *  registers.
 * @{
 */
#define XLLF_INT_RPURE_MASK       0x80000000 /**< Receive under-read */
#define XLLF_INT_RPORE_MASK       0x40000000 /**< Receive over-read */
#define XLLF_INT_RPUE_MASK        0x20000000 /**< Receive underrun (empty) */
#define XLLF_INT_TPOE_MASK        0x10000000 /**< Transmit overrun */
#define XLLF_INT_TC_MASK          0x08000000 /**< Transmit complete */
#define XLLF_INT_RC_MASK          0x04000000 /**< Receive complete */
#define XLLF_INT_TSE_MASK         0x02000000 /**< Transmit length mismatch */
#define XLLF_INT_TRC_MASK         0x01000000 /**< Transmit reset complete */
#define XLLF_INT_RRC_MASK         0x00800000 /**< Receive reset complete */
#define XLLF_INT_TFPF_MASK        0x00400000 /**< Tx FIFO Programmable Full,
						* AXI FIFO MM2S Only */
#define XLLF_INT_TFPE_MASK        0x00200000 /**< Tx FIFO Programmable Empty
						* AXI FIFO MM2S Only */
#define XLLF_INT_RFPF_MASK        0x00100000 /**< Rx FIFO Programmable Full
						* AXI FIFO MM2S Only */
#define XLLF_INT_RFPE_MASK        0x00080000 /**< Rx FIFO Programmable Empty
						* AXI FIFO MM2S Only */
#define XLLF_INT_ALL_MASK         0xfff80000 /**< All the ints */
#define XLLF_INT_ERROR_MASK       0xf2000000 /**< Error status ints */
#define XLLF_INT_RXERROR_MASK     0xe0000000 /**< Receive Error status ints */
#define XLLF_INT_TXERROR_MASK     0x12000000 /**< Transmit Error status ints */
/*@}*/

/** @name Reset register values
 *  These bits are associated with the XLLF_TDFR_OFFSET and XLLF_RDFR_OFFSET
 *  reset registers.
 * @{
 */
#define XLLF_RDFR_RESET_MASK        0x000000a5 /**< receive reset value */
#define XLLF_TDFR_RESET_MASK        0x000000a5 /**< Transmit reset value */
#define XLLF_LLR_RESET_MASK         0x000000a5 /**< Local Link reset value */
/* end defines from from \libsrc\llfifo_v5_5\src\xllfifo_hw.h */

/* defines from from libsrc/axiethernet_v5_8/src/axiethernet_hw.h start 
 * Registers for ethernet system, base_addr + offset
 * Register offset definitions. Unless otherwise noted, register access is
 * 32 bit. 
 */
#define XAE_RAF_OFFSET		  0x00000000 /**< Reset and Address filter */
#define XAE_TPF_OFFSET		  0x00000004 /**< Tx Pause Frame */
#define XAE_IFGP_OFFSET		0x00000008 /**< Tx Inter-frame gap adjustment*/
#define XAE_IS_OFFSET		  0x0000000C /**< Interrupt status */
#define XAE_IP_OFFSET		  0x00000010 /**< Interrupt pending */
#define XAE_IE_OFFSET		  0x00000014 /**< Interrupt enable */
#define XAE_TTAG_OFFSET		0x00000018 /**< Tx VLAN TAG */
#define XAE_RTAG_OFFSET		0x0000001C /**< Rx VLAN TAG */
#define XAE_UAWL_OFFSET		0x00000020 /**< Unicast address word lower */
#define XAE_UAWU_OFFSET		0x00000024 /**< Unicast address word upper */
#define XAE_TPID0_OFFSET	  0x00000028 /**< VLAN TPID0 register */
#define XAE_TPID1_OFFSET	  0x0000002C /**< VLAN TPID1 register */

/*
 * Statistics Counter Registers are from offset 0x200 to 0x3FF
 * They are defined from offset 0x200 to 0x34C in this device.
 * The offsets from 0x350 to 0x3FF are reserved.
 * The counters are 64 bit.
 * The Least Significant Word (LSW) are stored in one 32 bit register and
 * the Most Significant Word (MSW) are stored in one 32 bit register
 */
/* Start of Statistics Counter Registers Definitions */
#define XAE_RXBL_OFFSET		0x00000200 /**< Received Bytes, LSW */
#define XAE_RXBU_OFFSET		0x00000204 /**< Received Bytes, MSW */
#define XAE_TXBL_OFFSET		0x00000208 /**< Transmitted Bytes, LSW */
#define XAE_TXBU_OFFSET		0x0000020C /**< Transmitted Bytes, MSW */
#define XAE_RXUNDRL_OFFSET	0x00000210 /**< Count of undersize(less than
					     *  64 bytes) frames received,
					     *  LSW
					     */
#define XAE_RXUNDRU_OFFSET	0x00000214 /**< Count of undersize(less than
					     *  64 bytes) frames received,
					     *  MSW
					     */
#define XAE_RXFRAGL_OFFSET	0x00000218 /**< Count of undersized(less
					     *  than 64 bytes) and bad FCS
					     *  frames received, LSW
					     */
#define XAE_RXFRAGU_OFFSET	0x0000021C /**< Count of undersized(less
					     *  than 64 bytes) and bad FCS
					     *  frames received, MSW
					     */
#define XAE_RX64BL_OFFSET	0x00000220 /**< Count of 64 bytes frames
					     *  received, LSW
					     */
#define XAE_RX64BU_OFFSET	0x00000224 /**< Count of 64 bytes frames
					     *  received, MSW
					     */
#define XAE_RX65B127L_OFFSET	0x00000228 /**< Count of 65-127 bytes
					     *  Frames received, LSW
					     */
#define XAE_RX65B127U_OFFSET	0x0000022C /**< Count of 65-127 bytes
					     *  Frames received, MSW
					     */
#define XAE_RX128B255L_OFFSET	0x00000230 /**< Count of 128-255 bytes
					     *  Frames received, LSW
					     */
#define XAE_RX128B255U_OFFSET	0x00000234 /**< Count of 128-255 bytes
					     *  frames received, MSW
					     */
#define XAE_RX256B511L_OFFSET	0x00000238 /**< Count of 256-511 bytes
					     *  Frames received, LSW
					     */
#define XAE_RX256B511U_OFFSET	0x0000023C /**< Count of 256-511 bytes
					     *  frames received, MSW
					     */
#define XAE_RX512B1023L_OFFSET	0x00000240 /**< Count of 512-1023 bytes
					     *  frames received, LSW
					     */
#define XAE_RX512B1023U_OFFSET	0x00000244 /**< Count of 512-1023 bytes
					     *  frames received, MSW
					     */
#define XAE_RX1024BL_OFFSET	0x00000248 /**< Count of 1024-MAX bytes
					     *  frames received, LSW
					     */
#define XAE_RX1024BU_OFFSET	0x0000024C /**< Count of 1024-MAX bytes
					     *  frames received, MSW
					     */
#define XAE_RXOVRL_OFFSET	0x00000250 /**< Count of oversize frames
					     *  received, LSW
					     */
#define XAE_RXOVRU_OFFSET	0x00000254 /**< Count of oversize frames
					     *  received, MSW
					     */
#define XAE_TX64BL_OFFSET	0x00000258 /**< Count of 64 bytes frames
					     *  transmitted, LSW
					     */
#define XAE_TX64BU_OFFSET	0x0000025C /**< Count of 64 bytes frames
					     *  transmitted, MSW
					     */
#define XAE_TX65B127L_OFFSET	0x00000260 /**< Count of 65-127 bytes
					     *  frames transmitted, LSW
					     */
#define XAE_TX65B127U_OFFSET	0x00000264 /**< Count of 65-127 bytes
					     *  frames transmitted, MSW
					     */
#define XAE_TX128B255L_OFFSET	0x00000268 /**< Count of 128-255 bytes
					     *  frames transmitted, LSW
					     */
#define XAE_TX128B255U_OFFSET	0x0000026C /**< Count of 128-255 bytes
					     *  frames transmitted, MSW
					     */
#define XAE_TX256B511L_OFFSET	0x00000270 /**< Count of 256-511 bytes
					     *  frames transmitted, LSW
					     */
#define XAE_TX256B511U_OFFSET	0x00000274 /**< Count of 256-511 bytes
					     *  frames transmitted, MSW
					     */
#define XAE_TX512B1023L_OFFSET	0x00000278 /**< Count of 512-1023 bytes
					     *  frames transmitted, LSW
					     */
#define XAE_TX512B1023U_OFFSET	0x0000027C /**< Count of 512-1023 bytes
					     *  frames transmitted, MSW
					     */
#define XAE_TX1024L_OFFSET	0x00000280 /**< Count of 1024-MAX bytes
					     *  frames transmitted, LSW
					     */
#define XAE_TX1024U_OFFSET	0x00000284 /**< Count of 1024-MAX bytes
					     *  frames transmitted, MSW
					     */
#define XAE_TXOVRL_OFFSET	0x00000288 /**< Count of oversize frames
					     *  transmitted, LSW
					     */
#define XAE_TXOVRU_OFFSET	0x0000028C /**< Count of oversize frames
					     *  transmitted, MSW
					     */
#define XAE_RXFL_OFFSET		0x00000290 /**< Count of frames received OK,
					     *  LSW
					     */
#define XAE_RXFU_OFFSET		0x00000294 /**< Count of frames received OK,
					     *  MSW
					     */
#define XAE_RXFCSERL_OFFSET	0x00000298 /**< Count of frames received with
					     *  FCS error and at least 64
					     *  bytes, LSW
					     */
#define XAE_RXFCSERU_OFFSET	0x0000029C /**< Count of frames received with
					     *  FCS error and at least 64
					     *  bytes,MSW
					     */
#define XAE_RXBCSTFL_OFFSET	0x000002A0 /**< Count of broadcast frames
					     *  received, LSW
					     */
#define XAE_RXBCSTFU_OFFSET	0x000002A4 /**< Count of broadcast frames
					     *  received, MSW
					     */
#define XAE_RXMCSTFL_OFFSET	0x000002A8 /**< Count of multicast frames
					     *  received, LSW
					     */
#define XAE_RXMCSTFU_OFFSET	0x000002AC /**< Count of multicast frames
					     *  received, MSW
					     */
#define XAE_RXCTRFL_OFFSET	0x000002B0 /**< Count of control frames
					     *  received, LSW
					     */
#define XAE_RXCTRFU_OFFSET	0x000002B4 /**< Count of control frames
					     *  received, MSW
					     */
#define XAE_RXLTERL_OFFSET	0x000002B8 /**< Count of frames received
					     *  with length error, LSW
					     */
#define XAE_RXLTERU_OFFSET	0x000002BC /**< Count of frames received
					     *  with length error, MSW
					     */
#define XAE_RXVLANFL_OFFSET	0x000002C0 /**< Count of VLAN tagged
					     *  frames received, LSW
					     */
#define XAE_RXVLANFU_OFFSET	0x000002C4 /**< Count of VLAN tagged frames
					     *  received, MSW
					     */
#define XAE_RXPFL_OFFSET	0x000002C8 /**< Count of pause frames received,
					     *  LSW
					     */
#define XAE_RXPFU_OFFSET	0x000002CC /**< Count of pause frames received,
					     *  MSW
					     */
#define XAE_RXUOPFL_OFFSET	0x000002D0 /**< Count of control frames
					     *  received with unsupported
					     *  opcode, LSW
					     */
#define XAE_RXUOPFU_OFFSET	0x000002D4 /**< Count of control frames
					     *  received with unsupported
					     *  opcode, MSW
					     */
#define XAE_TXFL_OFFSET		0x000002D8 /**< Count of frames transmitted OK,
					     *  LSW
					     */
#define XAE_TXFU_OFFSET		0x000002DC /**< Count of frames transmitted OK,
					     *  MSW
					     */
#define XAE_TXBCSTFL_OFFSET	0x000002E0 /**< Count of broadcast frames
					     *  transmitted OK, LSW
					     */
#define XAE_TXBCSTFU_OFFSET	0x000002E4 /**< Count of broadcast frames
					     *  transmitted, MSW
					     */
#define XAE_TXMCSTFL_OFFSET	0x000002E8 /**< Count of multicast frames
					     *  transmitted, LSW
					     */
#define XAE_TXMCSTFU_OFFSET	0x000002EC /**< Count of multicast frames
					     *  transmitted, MSW
					     */
#define XAE_TXUNDRERL_OFFSET	0x000002F0 /**< Count of frames transmitted
					     *  underrun error, LSW
					     */
#define XAE_TXUNDRERU_OFFSET	0x000002F4 /**< Count of frames transmitted
					     *  underrun error, MSW
					     */
#define XAE_TXCTRFL_OFFSET	0x000002F8 /**< Count of control frames
					     *  transmitted, LSW
					     */
#define XAE_TXCTRFU_OFFSET	0x000002FC /**< Count of control frames,
					     *  transmitted, MSW
					     */
#define XAE_TXVLANFL_OFFSET	0x00000300 /**< Count of VLAN tagged frames
					     *  transmitted, LSW
					     */
#define XAE_TXVLANFU_OFFSET	0x00000304 /**< Count of VLAN tagged
					     *  frames transmitted, MSW
					     */
#define XAE_TXPFL_OFFSET	0x00000308 /**< Count of pause frames
					     *  transmitted, LSW
					     */
#define XAE_TXPFU_OFFSET	0x0000030C /**< Count of pause frames
					     *  transmitted, MSW
					     */
#define XAE_TXSCL_OFFSET	0x00000310 /**< Single Collision Frames
					     *  Transmitted OK, LSW
					     */
#define XAE_TXSCU_OFFSET	0x00000314 /**< Single Collision Frames
					     *  Transmitted OK, MSW
					     */
#define XAE_TXMCL_OFFSET	0x00000318 /**< Multiple Collision Frames
					     *  Transmitted OK, LSW
					     */
#define XAE_TXMCU_OFFSET	0x0000031C /**< Multiple Collision Frames
					     *  Transmitted OK, MSW
					     */
#define XAE_TXDEFL_OFFSET	0x00000320 /**< Deferred Tx Frames, LSW */
#define XAE_TXDEFU_OFFSET	0x00000324 /**< Deferred Tx Frames, MSW */
#define XAE_TXLTCL_OFFSET	0x00000328 /**< Frames transmitted with late
					     *  Collisions, LSW
					     */
#define XAE_TXLTCU_OFFSET	0x0000032C /**< Frames transmitted with late
					     *  Collisions, MSW
					     */
#define XAE_TXAECL_OFFSET	0x00000330 /**< Frames aborted with excessive
					     *  Collisions, LSW
					     */
#define XAE_TXAECU_OFFSET	0x00000334 /**< Frames aborted with excessive
					     *  Collisions, MSW
					     */
#define XAE_TXEDEFL_OFFSET	0x00000338 /**< Transmit Frames with excessive
					     *  Defferal, LSW
					     */
#define XAE_TXEDEFU_OFFSET	0x0000033C /**< Transmit Frames with excessive
					     *  Defferal, MSW
					     */
#define XAE_RXAERL_OFFSET	0x00000340 /**< Frames received with alignment
					     *  errors, LSW
					     */
#define XAE_RXAERU_OFFSET	0x0000034C /**< Frames received with alignment
					     *  errors, MSW
					     */
/* End of Statistics Counter Registers Offset definitions */

#define XAE_RCW0_OFFSET			0x00000400 /**< Rx Configuration Word 0 */
#define XAE_RCW1_OFFSET			0x00000404 /**< Rx Configuration Word 1 */
#define XAE_TC_OFFSET			  0x00000408 /**< Tx Configuration */
#define XAE_FCC_OFFSET			0x0000040C /**< Flow Control Configuration */
#define XAE_EMMC_OFFSET			0x00000410 /**< EMAC mode configuration */
#define XAE_RXFC_OFFSET			0x00000414 /**< Rx Max Frm Config Register */
#define XAE_TXFC_OFFSET			0x00000418 /**< Tx Max Frm Config Register */
#define XAE_TX_TIMESTAMP_ADJ_OFFSET		0x0000041C /**< Transmitter time stamp
										* adjust control Register
										*/
#define XAE_PHYC_OFFSET		0x00000420 /**< RGMII/SGMII configuration */

/* 0x00000424 to 0x000004F4 are reserved */

#define XAE_IDREG_OFFSET	0x000004F8 /**< Identification Register */
#define XAE_ARREG_OFFSET	0x000004FC /**< Ability Register */
#define XAE_MDIO_MC_OFFSET	0x00000500 /**< MII Management Config */
#define XAE_MDIO_MCR_OFFSET	0x00000504 /**< MII Management Control */
#define XAE_MDIO_MWD_OFFSET	0x00000508 /**< MII Management Write Data */
#define XAE_MDIO_MRD_OFFSET	0x0000050C /**< MII Management Read Data */

/* 0x00000510 to 0x000005FC are reserved */

#define XAE_MDIO_MIS_OFFSET	0x00000600 /**< MII Management Interrupt
					    *   Status
					    */
/* 0x00000604-0x0000061C are reserved */

#define XAE_MDIO_MIP_OFFSET	0x00000620 /**< MII Management Interrupt
					    *   Pending register offse
					    */
/* 0x00000624-0x0000063C are reserved */

#define XAE_MDIO_MIE_OFFSET	0x00000640 /**< MII Management Interrupt
					    *   Enable register offset
					    */
/* 0x00000644-0x0000065C are reserved */

#define XAE_MDIO_MIC_OFFSET	0x00000660 /**< MII Management Interrupt
					    *   Clear register offset.
					    */

/* 0x00000664-0x000006FC are reserved */

#define XAE_UAW0_OFFSET		0x00000700  /**< Unicast address word 0 */
#define XAE_UAW1_OFFSET		0x00000704  /**< Unicast address word 1 */
#define XAE_FMI_OFFSET		0x00000708  /**< Filter Mask Index */
/* 0x0000070C is reserved */
#define XAE_AF0_OFFSET		0x00000710  /**< Address Filter 0 */
#define XAE_AF1_OFFSET		0x00000714  /**< Address Filter 1 */

/* 0x00000718-0x00003FFC are reserved */

/*
 * Transmit VLAN Table is from 0x00004000 to 0x00007FFC
 * This offset defines an offset to table that has provisioned transmit
 * VLAN data. The VLAN table will be used by hardware to provide
 * transmit VLAN tagging, stripping, and translation.
 */
#define XAE_TX_VLAN_DATA_OFFSET 0x00004000 /**< TX VLAN data table address */

/*
 * Receive VLAN Data Table is from 0x00008000 to 0x0000BFFC
 * This offset defines an offset to table that has provisioned receive
 * VLAN data. The VLAN table will be used by hardware to provide
 * receive VLAN tagging, stripping, and translation.
 */
#define XAE_RX_VLAN_DATA_OFFSET 0x00008000 /**< RX VLAN data table address */

/* 0x0000C000-0x0000FFFC are reserved */

/* 0x00010000-0x00013FFC are Ethenet AVB address offset */

/* 0x00014000-0x0001FFFC are reserved */

/*
 * Extended Multicast Address Table is from 0x0020000 to 0x0003FFFC.
 * This offset defines an offset to table that has provisioned multicast
 * addresses. It is stored in BRAM and will be used by hardware to provide
 * first line of address matching when a multicast frame is reveived. It
 * can minimize the use of CPU/software hence minimize performance impact.
 */
#define XAE_MCAST_TABLE_OFFSET   0x00020000 /**< Multicast table address */
/*@}*/

/* Register masks. The following constants define bit locations of various
 * bits in the registers. Constants are not defined for those registers
 * that have a single bit field representing all 32 bits. For further
 * information on the meaning of the various bit masks, refer to the HW spec.
 */

/** @name Reset and Address Filter (RAF) Register bit definitions.
 *  These bits are associated with the XAE_RAF_OFFSET register.
 * @{
 */
#define XAE_RAF_MCSTREJ_MASK	     	0x00000002 /**< Reject receive
						    *   multicast destination
						    *   address
						    */
#define XAE_RAF_BCSTREJ_MASK	     	0x00000004 /**< Reject receive
						    *   broadcast destination
						    *   address
						    */
#define XAE_RAF_TXVTAGMODE_MASK  	0x00000018 /**< Tx VLAN TAG mode */
#define XAE_RAF_RXVTAGMODE_MASK  	0x00000060 /**< Rx VLAN TAG mode */
#define XAE_RAF_TXVSTRPMODE_MASK 	0x00000180 /**< Tx VLAN STRIP mode */
#define XAE_RAF_RXVSTRPMODE_MASK 	0x00000600 /**< Rx VLAN STRIP mode */
#define XAE_RAF_NEWFNCENBL_MASK  	0x00000800 /**< New function mode */
#define XAE_RAF_EMULTIFLTRENBL_MASK 	0x00001000 /**< Exteneded Multicast
						     *  Filtering mode
						     */
#define XAE_RAF_STATSRST_MASK  	0x00002000 	   /**< Statistics Counter
						    *   Reset
						    */
#define XAE_RAF_RXBADFRMEN_MASK     	0x00004000 /**< Receive Bad Frame
						    *   Enable
						    */
#define XAE_RAF_TXVTAGMODE_SHIFT 	3	/**< Tx Tag mode shift bits */
#define XAE_RAF_RXVTAGMODE_SHIFT 	5	/**< Rx Tag mode shift bits */
#define XAE_RAF_TXVSTRPMODE_SHIFT	7	/**< Tx strip mode shift bits*/
#define XAE_RAF_RXVSTRPMODE_SHIFT	9	/**< Rx Strip mode shift bits*/
/*@}*/

/** @name Transmit Pause Frame Register (TPF) bit definitions
 *  @{
 */
#define XAE_TPF_TPFV_MASK		0x0000FFFF /**< Tx pause frame value */
/*@}*/

/** @name Transmit Inter-Frame Gap Adjustement Register (TFGP) bit definitions
 *  @{
 */
#define XAE_TFGP_IFGP_MASK		0x0000007F /**< Transmit inter-frame
					            *   gap adjustment value
					            */
/*@}*/

/** @name Interrupt Status/Enable/Mask Registers bit definitions
 *  The bit definition of these three interrupt registers are the same.
 *  These bits are associated with the XAE_IS_OFFSET, XAE_IP_OFFSET, and
 *  XAE_IE_OFFSET registers.
 * @{
 */
#define XAE_INT_HARDACSCMPLT_MASK	0x00000001 /**< Hard register
						     *	access complete
						     */
#define XAE_INT_AUTONEG_MASK		0x00000002 /**< Auto negotiation
						     *  complete
						     */
#define XAE_INT_RXCMPIT_MASK		0x00000004 /**< Rx complete */
#define XAE_INT_RXRJECT_MASK		0x00000008 /**< Rx frame rejected */
#define XAE_INT_RXFIFOOVR_MASK		0x00000010 /**< Rx fifo overrun */
#define XAE_INT_TXCMPIT_MASK		0x00000020 /**< Tx complete */
#define XAE_INT_RXDCMLOCK_MASK		0x00000040 /**< Rx Dcm Lock */
#define XAE_INT_MGTRDY_MASK		0x00000080 /**< MGT clock Lock */
#define XAE_INT_PHYRSTCMPLT_MASK	0x00000100 /**< Phy Reset complete */

#define XAE_INT_ALL_MASK		0x0000003F /**< All the ints */

#define XAE_INT_RECV_ERROR_MASK			\
	(XAE_INT_RXRJECT_MASK | XAE_INT_RXFIFOOVR_MASK) /**< INT bits that
							 *   indicate receive
							 *   errors
							 */
/*@}*/

/** @name TPID Register (TPID) bit definitions
 *  @{
 */
#define XAE_TPID_0_MASK			0x0000FFFF   /**< TPID 0 */
#define XAE_TPID_1_MASK			0xFFFF0000   /**< TPID 1 */
/*@}*/

/** @name Receive Configuration Word 1 (RCW1) Register bit definitions
 *  @{
 */
#define XAE_RCW1_RST_MASK	0x80000000 /**< Reset */
#define XAE_RCW1_JUM_MASK	0x40000000 /**< Jumbo frame enable */
#define XAE_RCW1_FCS_MASK	0x20000000 /**< In-Band FCS enable
					     *  (FCS not stripped) */
#define XAE_RCW1_RX_MASK	0x10000000 /**< Receiver enable */
#define XAE_RCW1_VLAN_MASK	0x08000000 /**< VLAN frame enable */
#define XAE_RCW1_LT_DIS_MASK	0x02000000 /**< Length/type field valid check
					     *  disable
					     */
#define XAE_RCW1_CL_DIS_MASK	0x01000000 /**< Control frame Length check
					     *  disable
					     */
#define XAE_RCW1_1588_TIMESTAMP_EN_MASK		0x00400000 /**< Inband 1588 time
											* stamp enable
											*/
#define XAE_RCW1_PAUSEADDR_MASK 0x0000FFFF /**< Pause frame source
					     *  address bits [47:32].Bits
					     *	[31:0] are stored in register
					     *  RCW0
					     */
/*@}*/

/** @name Transmitter Configuration (TC) Register bit definitions
 *  @{
 */
#define XAE_TC_RST_MASK		0x80000000 /**< Reset */
#define XAE_TC_JUM_MASK		0x40000000 /**< Jumbo frame enable */
#define XAE_TC_FCS_MASK		0x20000000 /**< In-Band FCS enable
					     *  (FCS not generated)
					     */
#define XAE_TC_TX_MASK		0x10000000 /**< Transmitter enable */
#define XAE_TC_VLAN_MASK	0x08000000 /**< VLAN frame enable */
#define XAE_TC_IFG_MASK		0x02000000 /**< Inter-frame gap adjustment
					      * enable
					      */
#define XAE_TC_1588_CMD_EN_MASK		0x00400000 /**< 1588 Cmd field enable */
/*@}*/

/** @name Flow Control Configuration (FCC) Register Bit definitions
 *  @{
 */
#define XAE_FCC_FCRX_MASK	0x20000000   /**< Rx flow control enable */
#define XAE_FCC_FCTX_MASK	0x40000000   /**< Tx flow control enable */
/*@}*/

/** @name Ethernet MAC Mode Configuration (EMMC) Register bit definitions
 * @{
 */
#define XAE_EMMC_LINKSPEED_MASK 	0xC0000000 /**< Link speed */
#define XAE_EMMC_RGMII_MASK	 	0x20000000 /**< RGMII mode enable */
#define XAE_EMMC_SGMII_MASK	 	0x10000000 /**< SGMII mode enable */
#define XAE_EMMC_GPCS_MASK	 	0x08000000 /**< 1000BaseX mode enable*/
#define XAE_EMMC_HOST_MASK	 	0x04000000 /**< Host interface enable*/
#define XAE_EMMC_TX16BIT	 	0x02000000 /**< 16 bit Tx client
						    *   enable
						    */
#define XAE_EMMC_RX16BIT	 	0x01000000 /**< 16 bit Rx client
						    *   enable
						    */
#define XAE_EMMC_LINKSPD_10		0x00000000 /**< Link Speed mask for
						    *   10 Mbit
						    */
#define XAE_EMMC_LINKSPD_100		0x40000000 /**< Link Speed mask for 100
						    *   Mbit
						    */
#define XAE_EMMC_LINKSPD_1000		0x80000000 /**< Link Speed mask for
						    *   1000 Mbit
						    */
/*@}*/

/** @name RGMII/SGMII Configuration (PHYC) Register bit definitions
 * @{
 */
#define XAE_PHYC_SGMIILINKSPEED_MASK 	0xC0000000 /**< SGMII link speed mask*/
#define XAE_PHYC_RGMIILINKSPEED_MASK 	0x0000000C /**< RGMII link speed */
#define XAE_PHYC_RGMIIHD_MASK	 	0x00000002 /**< RGMII Half-duplex */
#define XAE_PHYC_RGMIILINK_MASK 	0x00000001 /**< RGMII link status */
#define XAE_PHYC_RGLINKSPD_10		0x00000000 /**< RGMII link 10 Mbit */
#define XAE_PHYC_RGLINKSPD_100		0x00000004 /**< RGMII link 100 Mbit */
#define XAE_PHYC_RGLINKSPD_1000 	0x00000008 /**< RGMII link 1000 Mbit */
#define XAE_PHYC_SGLINKSPD_10		0x00000000 /**< SGMII link 10 Mbit */
#define XAE_PHYC_SGLINKSPD_100		0x40000000 /**< SGMII link 100 Mbit */
#define XAE_PHYC_SGLINKSPD_1000 	0x80000000 /**< SGMII link 1000 Mbit */
/*@}*/

/** @name MDIO Management Configuration (MC) Register bit definitions
 * @{
 */
#define XAE_MDIO_MC_MDIOEN_MASK		0x00000040 /**< MII management enable*/
#define XAE_MDIO_MC_CLOCK_DIVIDE_MAX	0x3F	   /**< Maximum MDIO divisor */
/*@}*/

/** @name MDIO Management Control Register (MCR) Register bit definitions
 * @{
 */
#define XAE_MDIO_MCR_PHYAD_MASK		0x1F000000 /**< Phy Address Mask */
#define XAE_MDIO_MCR_PHYAD_SHIFT	24	   /**< Phy Address Shift */
#define XAE_MDIO_MCR_REGAD_MASK		0x001F0000 /**< Reg Address Mask */
#define XAE_MDIO_MCR_REGAD_SHIFT	16	   /**< Reg Address Shift */
#define XAE_MDIO_MCR_OP_MASK		0x0000C000 /**< Operation Code Mask */
#define XAE_MDIO_MCR_OP_SHIFT		13	   /**< Operation Code Shift */
#define XAE_MDIO_MCR_OP_READ_MASK	0x00008000 /**< Op Code Read Mask */
#define XAE_MDIO_MCR_OP_WRITE_MASK	0x00004000 /**< Op Code Write Mask */
#define XAE_MDIO_MCR_INITIATE_MASK	0x00000800 /**< Ready Mask */
#define XAE_MDIO_MCR_READY_MASK		0x00000080 /**< Ready Mask */

/*@}*/

/** @name MDIO Interrupt Enable/Mask/Status Registers bit definitions
 *  The bit definition of these three interrupt registers are the same.
 *  These bits are associated with the XAE_IS_OFFSET, XAE_IP_OFFSET, and
 *  XAE_IE_OFFSET registers.
 * @{
 */
#define XAE_MDIO_INT_MIIM_RDY_MASK	0x00000001 /**< MIIM Interrupt */
/*@}*/

/** @name Axi Ethernet Unicast Address Register Word 1 (UAW1) Register Bit
 *  definitions
 * @{
 */
#define XAE_UAW1_UNICASTADDR_MASK 	0x0000FFFF  /**< Station address bits
						     *  [47:32]
						     *  Station address bits [31:0]
						     *  are stored in register
						     *  UAW0 */
/*@}*/
/** @name Filter Mask Index (FMI) Register bit definitions
 * @{
 */
#define XAE_FMI_PM_MASK			0x80000000   /**< Promiscuous mode
						      *   enable
						      */
#define XAE_FMI_IND_MASK		0x00000003   /**< Index Mask */

/*@}*/
/** @name Extended multicast buffer descriptor bit mask
 * @{
 */
#define XAE_BD_RX_USR2_BCAST_MASK	0x00000004
#define XAE_BD_RX_USR2_IP_MCAST_MASK	0x00000002
#define XAE_BD_RX_USR2_MCAST_MASK	0x00000001
/*@}*/

/** @name Axi Ethernet Multicast Address Register Word 1 (MAW1)
 * @{
 */
#define XAE_MAW1_RNW_MASK         	0x00800000   /**< Multicast address
						      * table register read
						      * enable
						      */
#define XAE_MAW1_ADDR_MASK        	0x00030000   /**< Multicast address
						      *  table register address
						      */
#define XAE_MAW1_MULTICADDR_MASK  	0x0000FFFF   /**< Multicast address
						      *  bits [47:32]
						      *  Multicast address
						      *  bits [31:0] are stored
						      *  in register MAW0
						      */
#define XAE_MAW1_MATADDR_SHIFT_MASK 	16	 /**< Number of bits to shift
						  *  right to align with
						  *  XAE_MAW1_CAMADDR_MASK
						  */
/*@}*/

/** @name Other Constant definitions used in the driver
 * @{
 */

#define XAE_SPEED_10_MBPS		10	/**< Speed of 10 Mbps */
#define XAE_SPEED_100_MBPS		100	/**< Speed of 100 Mbps */
#define XAE_SPEED_1000_MBPS		1000	/**< Speed of 1000 Mbps */
#define XAE_SPEED_2500_MBPS		2500	/**< Speed of 2500 Mbps */

#define XAE_SOFT_TEMAC_LOW_SPEED	0	/**< For soft cores with 10/100
						 *   Mbps speed.
						 */
#define XAE_SOFT_TEMAC_HIGH_SPEED	1	/**< For soft cores with
						 *   10/100/1000 Mbps speed.
						 */
#define XAE_HARD_TEMAC_TYPE		2	/**< For hard TEMAC cores used
						 *   virtex-6.
						 */
#define XAE_PHY_ADDR_LIMIT		31	/**< Max limit while accessing
						  *  and searching for available
						  * PHYs.
						  */
#define XAE_PHY_REG_NUM_LIMIT		31	/**< Max register limit in PHY
						  * as mandated by the spec.
						  */
#define XAE_RST_DEFAULT_TIMEOUT_VAL 1000000 /**< Timeout in us used
						  *  while checking if the core
						  *  had come out of reset or for the driver
						  *   API to wait for before
						  *   returning a failure case.
						  */
#define XAE_RST_DEFAULT_TIMEOUT_MVAL 1000          

#define XAE_VLAN_TABL_STRP_FLD_LEN	1	/**< Strip field length in vlan
						 *   table used for extended
						 *   vlan features.
						 */
#define XAE_VLAN_TABL_TAG_FLD_LEN	1	/**< Tag field length in vlan
						 *   table used for extended
						 *   vlan features.
						 */
#define XAE_MAX_VLAN_TABL_ENTRY		0xFFF	/**< Max possible number of
						 *   entries in vlan table used
						 *   for extended vlan
						 *   features.
						 */
#define XAE_VLAN_TABL_VID_START_OFFSET	2	/**< VID field start offset in
						 *   each entry in the VLAN
						 *   table.
						 */
#define XAE_VLAN_TABL_STRP_STRT_OFFSET	1	/**< Strip field start offset
						 *   in each entry in the VLAN
						 *  table.
						 */
#define XAE_VLAN_TABL_STRP_ENTRY_MASK	0x01	/**< Mask used to extract the
						 *   the strip field from an
						 *   entry in VLAN table.
						 */
#define XAE_VLAN_TABL_TAG_ENTRY_MASK	0x01	/**< Mask used to extract the
						 *   the tag field from an
						 *   entry in VLAN table.
						 */
/*defines from libsrc/axiethernet_v5_8/src/axiethernet_hw.h end*/

/*defines from libsrc/axiethernet_v5_8/src/axiethernet.h start*/
#define XAE_PROMISC_MASK		0x00000001
#define XAE_JUMBO_MASK		0x00000002
#define XAE_VLAN_MASK			0x00000004
#define XAE_FLOW_CONTROL_MASK		0x00000008
#define XAE_FCS_STRIP_MASK		0x00000010
#define XAE_FCS_INSERT_MASK		0x00000020
#define XAE_LENTYPE_ERR_MASK		0x00000040
#define XAE_TRANSMITTER_ENABLE_MASK	0x00000080
#define XAE_RECEIVER_ENABLE_MASK	0x00000100
#define XAE_BROADCAST_MASK		0x00000200
#define XAE_MULTICAST_MASK		0x00000400
#define XAE_EXT_MULTICAST_MASK	0x00000800
#define XAE_EXT_TXVLAN_TRAN_MASK	0x00001000
#define XAE_EXT_RXVLAN_TRAN_MASK	0x00002000
#define XAE_EXT_TXVLAN_TAG_MASK	0x00004000
#define XAE_EXT_RXVLAN_TAG_MASK	0x00008000
#define XAE_EXT_TXVLAN_STRP_MASK	0x00010000
#define XAE_EXT_RXVLAN_STRP_MASK	0x00020000      

#define XAE_MDIO_DIV_DFT	29	/**< Default MDIO clock divisor */

/*
 * The next few constants help upper layers determine the size of memory
 * pools used for Ethernet buffers and descriptor lists.
 */
#define XAE_MAC_ADDR_SIZE		6	/* MAC addresses are 6 bytes */
#define XAE_MTU				1500	/* Max MTU size of an Ethernet
						 * frame
						 */
#define XAE_JUMBO_MTU			8982	/* Max MTU size of a jumbo
						 * Ethernet frame
						 */
#define XAE_HDR_SIZE			14	/* Size of an Ethernet header*/
#define XAE_HDR_VLAN_SIZE		18	/* Size of an Ethernet header
						 * with VLAN
						 */
#define XAE_TRL_SIZE			4	/* Size of an Ethernet trailer
						 * (FCS)
						 */
#define XAE_MAX_FRAME_SIZE	 (XAE_MTU + XAE_HDR_SIZE + XAE_TRL_SIZE)
#define XAE_MAX_VLAN_FRAME_SIZE  (XAE_MTU + XAE_HDR_VLAN_SIZE + XAE_TRL_SIZE)
#define XAE_MAX_JUMBO_FRAME_SIZE (XAE_JUMBO_MTU + XAE_HDR_SIZE + XAE_TRL_SIZE)
#define XAE_MAX_FRAME_WORD_SIZE  XAE_MAX_FRAME_SIZE/4
/*
 * Constant values returned by XAxiEthernet_GetPhysicalInterface(). Note that
 * these values match design parameters from the Axi Ethernet spec.
 */

#define XAE_TPID_MAX_ENTRIES		4 /* Number of storable TPIDs in
					   * Table
					   */

/*
 * Constant values pass into XAxiEthernet_SetV[tag|Strp]Mode() and returned by
 * XAxiEthernet_GetV[tag|Strp]Mode().
 */
#define XAE_VTAG_NONE			0	/* No tagging */
#define XAE_VTAG_ALL			1	/* Tag all frames */
#define XAE_VTAG_EXISTED		2	/* Tag already tagged frames */
#define XAE_VTAG_SELECT		  	3	/*
						 * Tag selected already tagged
						 * frames
						 */
#define XAE_DEFAULT_TXVTAG_MODE  	XAE_VTAG_ALL
#define XAE_DEFAULT_RXVTAG_MODE  	XAE_VTAG_ALL

#define XAE_VSTRP_NONE			0	/* No stripping */
#define XAE_VSTRP_ALL			1	/* Strip one tag from all
						 * frames
						 */
#define XAE_VSTRP_SELECT		3	/* Strip one tag from selected
						 * frames
						 */
#define XAE_DEFAULT_TXVSTRP_MODE	XAE_VSTRP_ALL
#define XAE_DEFAULT_RXVSTRP_MODE	XAE_VSTRP_ALL

#define XAE_RX				1 /* Receive direction  */
#define XAE_TX				2 /* Transmit direction */

#define XAE_SOFT_TEMAC_10_100_MBPS	0
#define XAE_SOFT_TEMAC_10_100_1000_MBPS	1
#define XAE_HARD_TEMC			2

/*defines from axiethernet_v5_8/src/axiethernet.h end*/

#define SET_OPTIONS 1
#define CLR_OPTIONS 0

#define XAE_RST_DEFAULT_TIMEOUT_VAL 1000000 /**< Timeout in us used
						  *  while checking if the core
						  *  had come out of reset or for the driver
						  *   API to wait for before
						  *   returning a failure case.
						  */

enum phy_Interface_type {
	/* The values of this enum are consecutively numbered */
	XAE_PHY_TYPE_MII = 0,
	XAE_PHY_TYPE_GMII = 1,
	XAE_PHY_TYPE_RGMII_1_3 = 2,
	XAE_PHY_TYPE_RGMII_2_0 = 3,
	XAE_PHY_TYPE_SGMII = 4,
	XAE_PHY_TYPE_1000BASE_X = 5
};

typedef void (*eth_axiemac_config_irq_t)(const struct device *dev);

/*
 * key hole size in 32 bit words
 */
#define LARGEST_FIFO_KEYHOLE_SIZE_WORDS 4
#define FIFO_WIDTH_BYTES                4  
#define ETH_BUF_ALIGNMENT               4  

 
/*
 * This union is used simply to force a 32bit alignment on the
 * buffer. Only the 'bytes' member is really used.
 */
union XStrm_AlignBuf_t {
	uint32_t _words[LARGEST_FIFO_KEYHOLE_SIZE_WORDS];
	uint8_t bytes[LARGEST_FIFO_KEYHOLE_SIZE_WORDS * 4];
};

typedef struct rxStrm_t {
	union XStrm_AlignBuf_t AlignedBuffer;
	unsigned HeadIndex; /* HeadIndex is the index to the AlignedBuffer as bytes. */
	unsigned FifoWidth;  /* FifoWidth is the FIFO key hole width in bytes. */
	unsigned FrmByteCnt; /* FrmByteCnt is the number of bytes in the next */
} rxStrm_t;

typedef struct txStrm_t {
	union XStrm_AlignBuf_t AlignedBuffer;
	unsigned TailIndex; /* TailIndex is the index to the AlignedBuffer */
	unsigned FifoWidth; /* FifoWidth is the FIFO key hole width in bytes.*/
} txStrm_t;

/**
 * This typedef defines a run-time instance of an XLlFifo device.
 */
typedef struct XStrmFifo {
	uint32_t base_addr; /**< BaseAddress is the physical base address of the*/
	rxStrm_t xStrm_rx; /**< RxStreamer is the byte streamer*/
	txStrm_t xStrm_tx; /**< TxStreamer is the byte streamer*/
} XStrmFifo;

struct eth_axiemac_dev_config {
	uint32_t base_addr; /* base address of the mac */
	uint32_t fifo_addr; /* base address of the fifo buffer */
	eth_axiemac_config_irq_t config_func;

	enum phy_Interface_type phy_if_type;
	enum eth_xlnx_link_speed max_link_speed;
	bool enable_fdx :1;

	/* run time configurable options */
	bool xae_promisc_option :1;
	bool xae_jumbo_option :1;
	bool xae_vlan_option :1;
	bool xae_flow_control_option :1;
	bool xae_fcs_strip_option :1;
	bool xae_fcs_insert_option :1;
	bool xae_lentype_err_option :1;
	bool xae_transmitter_enable_option :1;
	bool xae_receiver_enable_option :1;
	bool xae_broadcast_option :1;
	bool xae_multicast_option :1;
	/* hardware capability during ip instantiation in fpga */
	bool xae_ext_multicast_option :1;
	bool xae_ext_txvlan_tran_option :1;
	bool xae_ext_rxvlan_tran_option :1;
	bool xae_ext_txvlan_tag_option :1;
	bool xae_ext_rxvlan_tag_option :1;
	bool xae_ext_txvlan_strp_option :1;
	bool xae_ext_rxvlan_strp_option :1;
  bool xae_rx_chksum_offload :1;
  bool xae_tx_chksum_offload :1;
};

/**
 * @brief Run-time device configuration data structure.
 *
 * This struct contains all device configuration data for 
 * controller instance which is modifyable at run-time, such as
 * data relating to the attached PHY or the auxiliary thread.
 */
struct eth_axiemac_dev_data {
	struct net_if *iface;
	uint8_t mac_addr[6];
	enum eth_xlnx_link_speed eff_link_speed;
	struct k_sem tx_sem;
	uint8_t phy_addr;
	uint32_t phy_id;
	struct phy_xlnx_axiemac_api *phy_access_api;
	XStrmFifo axififo;
#ifdef CONFIG_NET_STATISTICS_ETHERNET
	struct net_stats_eth stats;
#endif
	bool started :1;
	bool tx_err :1;
// #ifdef ZYNQ_HOST_TEST
	// struct xemac_s* xemac;
// #endif
};

/* Top-level device initialization macro - bundles all of the above */
#define ETH_XLNX_AXIEMAC_INITIALIZE(port) \
ETH_XLNX_AXIEMAC_CONFIG_IRQ_FUNC(port);\
ETH_XLNX_AXIEMAC_DEV_CONFIG(port);\
ETH_XLNX_AXIEMAC_DEV_DATA(port);\
ETH_XLNX_AXIEMAC_NET_DEV_INIT(port);\

/* Device configuration data declaration macro */
#define ETH_XLNX_AXIEMAC_DEV_CONFIG(port) \
static const struct eth_axiemac_dev_config eth_xlnx_axiemac##port##_dev_cfg = {\
	.base_addr			                  = DT_REG_ADDR_BY_IDX(DT_INST(port, xlnx_axiemac), 0),\
  .fifo_addr			                  = DT_REG_ADDR_BY_IDX(DT_INST(port, xlnx_axiemac), 1),\
	.config_func		                  = eth_xlnx_axiemac##port##_irq_config,\
  .phy_if_type		                  = XAE_PHY_TYPE_GMII,\
	.max_link_speed			              = (enum eth_xlnx_link_speed)\
		(DT_INST_PROP(port, link_speed)),\
  .enable_fdx			                  = DT_INST_PROP(port, full_duplex),\
	.xae_promisc_option               = DT_INST_PROP(port, promiscuous_mode),\
  .xae_jumbo_option                 = false,\
  .xae_vlan_option                  = false,\
  .xae_flow_control_option          = DT_INST_PROP(port, flow_control),\
  .xae_fcs_strip_option             = true,\
  .xae_fcs_insert_option            = true,\
  .xae_lentype_err_option           = true,\
  .xae_transmitter_enable_option    = true,\
  .xae_receiver_enable_option       = true,\
  .xae_broadcast_option             = true,\
  .xae_multicast_option             = false,\
  .xae_ext_multicast_option         = false,\
  .xae_ext_txvlan_tran_option       = false,\
  .xae_ext_rxvlan_tran_option       = false,\
  .xae_ext_txvlan_tag_option        = false,\
  .xae_ext_rxvlan_tag_option        = false,\
  .xae_ext_txvlan_strp_option       = false,\
  .xae_ext_rxvlan_strp_option       = false,\
  .xae_rx_chksum_offload            = false,\
  .xae_tx_chksum_offload            = false\
};

/* Device run-time data declaration macro */
#define ETH_XLNX_AXIEMAC_DEV_DATA(port) \
static struct eth_axiemac_dev_data eth_xlnx_axiemac##port##_dev_data = {\
	.mac_addr        = DT_INST_PROP(port, local_mac_address),\
	.started         = 0,\
	.eff_link_speed  = LINK_DOWN,\
	.phy_addr        = 0,\
	.phy_id          = 0,\
	.phy_access_api  = NULL,\
  .tx_err           =0\
};

/* Device initialization macro */
#define ETH_XLNX_AXIEMAC_NET_DEV_INIT(port) \
ETH_NET_DEVICE_DT_INST_DEFINE(port,\
	eth_axiemac_dev_init,\
	NULL,\
	&eth_xlnx_axiemac##port##_dev_data,\
	&eth_xlnx_axiemac##port##_dev_cfg,\
	CONFIG_ETH_INIT_PRIORITY,\
	&eth_axiemac_apis,\
	NET_ETH_MTU);

/* Interrupt configuration function macro */
#define ETH_XLNX_AXIEMAC_CONFIG_IRQ_FUNC(port) \
static void eth_xlnx_axiemac##port##_irq_config(const struct device *dev)\
{\
	ARG_UNUSED(dev);\
	IRQ_CONNECT(DT_INST_IRQN(port), DT_INST_IRQ(port, priority),\
	eth_xStmFifo_isr, DEVICE_DT_INST_GET(port), 0);\
	irq_enable(DT_INST_IRQN(port));\
}

void my_msleep(uint32_t ms);


#endif /* ETH_XLNX_AXIEMAC_PRIV_H_ */
