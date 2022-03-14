/*
 * This file is part of the XILINX ZynqMP PS-PCIe End Point DMA driver
 *
 * Copyright (c) 2022-2023,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */
 
#ifndef PS_PCIE_DMA_H_
#define PS_PCIE_DMA_H_

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/msi.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>

#include "../common/common_include.h"

#define PCI_VENDOR_XILINX (0x10EE)
#define ZYNQMP_DMA_DEVID0 (0xA024)
#define ZYNQMP_DMA_DEVID1 (0xA808)

#define MAX_BARS 6

#define DMA_BAR_NUMBER 0

#define PIO_MEMORY_BAR_NUMBER            2

#define WORKQ_NAME_SIZE       100
#define INTR_HANDLR_NAME_SIZE 100

#define EXP_DMA_IRQ_NOSHARE    0

#define MAX_COAELSE_COUNT     255

/*
 * DMA Channel status bits
 */
#define DMA_INTSTATUS_DMAERR_BIT  BIT(1)
#define DMA_INTSTATUS_SGLINTR_BIT BIT(2)
#define DMA_INTSTATUS_SWINTR_BIT  BIT(3)

#define DMA_SRC_Q_LOW_BIT_SHIFT   6

#define MAX_TRANSFER_LENGTH       0x1000000

#define AXI_ATTRIBUTE       0x3
#define PCIe_ATTRIBUTE      0x2

/*
 * User Id programmed into Source Q will be copied into Status Q of Destination
 */
#define DEFAULT_UID 1

/*
 * DMA channel registers
 */
typedef struct _DMA_ENGINE_REGISTERS {
	union _SRC_Q_LOW {
		uint32_t UINT;
		struct {
			uint32_t QUEUE_LOCATION :1;         // Bit 0
			uint32_t QUEUE_ENABLE :1;           // Bit 1
			uint32_t READ_ATTR :4;              // Bit 2 - 5
			uint32_t START_ADDR_LOW :26;        // Bit 6-31
		} BIT;
	} SRC_Q_LOW;                     //0x00
	uint32_t SRC_Q_HI;               //0x04
	uint32_t SRC_Q_SIZE;             //0x08
	uint32_t SRC_Q_LIMIT;            //0x0C
	union _DST_Q_LOW {
		uint32_t UINT;
		struct {
			uint32_t QUEUE_LOCATION :1;         // Bit 0
			uint32_t QUEUE_ENABLE :1;           // Bit 1
			uint32_t READ_ATTR :4;              // Bit 2 - 5
			uint32_t START_ADDR_LOW :26;        // Bit 6-31
		} BIT;
	} DST_Q_LOW;                       //0x10
	uint32_t DST_Q_HI;                 //0x14
	uint32_t DST_Q_SIZE;               //0x18
	uint32_t DST_Q_LIMIT;              //0x1c
	union _STAS_Q_LOW {
		uint32_t UINT;
		struct {
			uint32_t QUEUE_LOCATION :1;         // Bit 0
			uint32_t QUEUE_ENABLE :1;           // Bit 1
			uint32_t READ_ATTR :4;              // Bit 2 - 5
			uint32_t START_ADDR_LOW :26;        // Bit 6-31
		} BIT;
	} STAS_Q_LOW;                       //0x20
	uint32_t STAS_Q_HI;                 //0x24
	uint32_t STAS_Q_SIZE;               //0x28
	uint32_t STAS_Q_LIMIT;              //0x2C
	union _STAD_Q_LOW {
		uint32_t UINT;
		struct {
			uint32_t QUEUE_LOCATION :1;         // Bit 0
			uint32_t QUEUE_ENABLE :1;           // Bit 1
			uint32_t READ_ATTR :4;              // Bit 2 - 5
			uint32_t START_ADDR_LOW :26;        // Bit 6 - 31
		} BIT;
	} STAD_Q_LOW;                      //0x30
	uint32_t STAD_Q_HI;                //0x34
	uint32_t STAD_Q_SIZE;              //0x38
	uint32_t STAD_Q_LIMIT;             //0x3C
	uint32_t SRC_Q_NEXT;               //0x40
	uint32_t DST_Q_NEXT;               //0x44
	uint32_t STAS_Q_NEXT;              //0x48
	uint32_t STAD_Q_NEXT;              //0x4C
	uint32_t SCRATHC0;                 //0x50
	uint32_t SCRATHC1;                 //0x54
	uint32_t SCRATHC2;                 //0x58
	uint32_t SCRATHC3;                 //0x5C
	union _PCIE_INTR_CNTRL {
		uint32_t UINT;
		struct {
			uint32_t INTR_MASK :1;          // Bit 0
			uint32_t DMA_ERR_INT_ENABLE :1; // Bit 1
			uint32_t SGL_INT_ENABLE :1;     // Bit 2
			uint32_t :13;                   // Bit 3-15
			uint32_t COALESCE_COUNT :8;     // Bit 16-23
			uint32_t :8;                    // Bit 24-31
		} BIT;
	} PCIE_INTR_CNTRL;                       //0x60
	union _PCIE_INTR_STATUS {
		uint32_t UINT;
		struct {
			uint32_t :1;                   // Bit 0
			uint32_t DMA_ERR_INT :1;       // Bit 1
			uint32_t DMA_SGL_INT :1;       // Bit 2
			uint32_t DMA_SW_INT :1;        // Bit 3
			uint32_t :28;                  // Bit 4-31
		} BIT;
	} PCIE_INTR_STATUS;                      //0x64
	union _AXI_INTR_CNTRL {
		uint32_t UINT;
		struct {
			uint32_t INTR_ENABLE :1;          // Bit 0
			uint32_t DMA_ERR_INT_ENABLE :1;   // Bit 1
			uint32_t SGL_INT_ENABLE :1;       // Bit 2
			uint32_t :13;                     // Bit 3-15
			uint32_t COAlLESCE_COUNT :8;      // Bit 16-23
			uint32_t :8;                      // Bit 24-31
		} BIT;
	} AXI_INTR_CNTRL;                       //0x68
	union _AXI_INTR_STATUS {
		uint32_t UINT;
		struct {
			uint32_t :1;                   // Bit 0
			uint32_t DMA_ERR_INT :1;       // Bit 1
			uint32_t DMA_SGL_INT :1;       // Bit 2
			uint32_t DMA_SW_INT :1;        // Bit 3
			uint32_t :28;                  // Bit 4-31
		} BIT;
	} AXI_INTR_STATUS;                      //0x6C
	union _PCIE_INTR_ASSERT {
		uint32_t UINT;
		struct {
			uint32_t :3;                      // Bit 0 - 2
			uint32_t SOFTWARE_INTRPT :1;      // Bit 3
			uint32_t :28;                     // Bit 4-31
		} BIT;
	} PCIE_INTR_ASSERT;                     //0x70
	union _AXI_INTR_ASSERT {
		uint32_t UINT;
		struct {
			uint32_t :3;                   // Bit 0 - 2
			uint32_t SOFTWARE_INTRPT :1;   // Bit 3
			uint32_t :28;                  // Bit 4-31
		} BIT;
	} AXI_INTR_ASSERT;                      //0x74
	union _DMA_CHANNEL_CTRL {
		uint32_t UINT;
		struct {
			uint32_t DMA_ENABLE :1;            // Bit 0
			uint32_t DMA_RESET :1;             // Bit 1
			uint32_t STA_Q_ELE_SIZE :1;        // Bit 2
			uint32_t :29;                      // Bit 3-31
		} BIT;
	} DMA_CHANNEL_CTRL;                     //0x78
	union _DMA_CHANNEL_STATUS {
		uint32_t UINT;
		struct {
			uint32_t DMA_RUNNING :1;           // Bit 0
			uint32_t :3;                       // Bit 1-3
			uint32_t CHANNEL_NUMBER :10;       // Bit 4-13
			uint32_t :1;                       // Bit 14
			uint32_t CHANNEL_PRESENT :1;       // Bit 15
			uint32_t :16;                      // Bit 16-31
		} BIT;
	} DMA_CHANNEL_STATUS;                   //0x7C
}__attribute__((__packed__)) DMA_ENGINE_REGISTERS, *PDMA_ENGINE_REGISTERS;

/**
 * struct _SOURCE_DMA_DESCRIPTOR - Source Hardware Descriptor
 * @SYSTEM_ADDRESS_PHYSICAL: 64 bit buffer physical address
 * @CONTROL_BYTE_COUNT: Byte count/buffer length and control flags
 * @USER: User handle and id
 */
typedef struct _SOURCE_DMA_DESCRIPTOR {
	union _SYSTEM_ADDRESS_PHYSICAL {
		uint64_t UINT64;
		struct _UINT64_REG {
			uint32_t LOW;
			uint32_t HIGH;
		} UINT64_REG;
	} SYSTEM_ADDRESS_PHYSICAL;
	union _CONTROL_BYTE_COUNT {
		uint32_t UINT32_REG;
		struct {
			uint32_t BYTE_COUNT :24;
			uint32_t LOCATION :1;
			uint32_t EOP :1;
			uint32_t INTERRUPT :1;
			uint32_t :1;
			uint32_t DMA_DATA_READ_ATTRIBUTES :4;
		} BIT;
	} CONTROL_BYTE_COUNT;
	union _USER {
		uint32_t UINT32_REG;
		struct { /*  Bit  Access */
			uint32_t USER_HANDLE :16;
			uint32_t USER_ID :16;
		} BIT;
	} USER;
}__attribute__((__packed__)) SOURCE_SGL_DMA_DESCRIPTOR,
		*PSOURCE_SGL_DMA_DESCRIPTOR __aligned(64);

/**
 * struct _DEST_DMA_DESCRIPTOR - Destination Hardware Descriptor
 * @SYSTEM_ADDRESS_PHYSICAL: 64 bit buffer physical address
 * @CONTROL_BYTE_COUNT: Byte count/buffer length and control flags
 * @USER: User handle and id
 */
typedef struct _DEST_DMA_DESCRIPTOR {
	union {
		uint64_t UINT64;
		struct {
			uint32_t LOW;
			uint32_t HIGH;
		} UINT64_REG;
	} SYSTEM_ADDRESS_PHYSICAL;
	union {
		uint32_t UINT32_REG;
		struct {
			uint32_t BYTE_COUNT :24;
			uint32_t LOCATION :1;
			uint32_t BACK_TO_BACK_PACKING :1;
			uint32_t :2;
			uint32_t DMA_DATA_WRITE_ATTRIBUTES :4;
		} BIT;
	} CONTROL_BYTE_COUNT;
	union {
		uint32_t UINT32_REG;
		struct { /*  Bit  Access */
			uint32_t USER_HANDLE :16;
			uint32_t :16;
		} BIT;
	} USER;
}__attribute__((__packed__)) DEST_SGL_DMA_DESCRIPTOR,
			*PDEST_SGL_DMA_DESCRIPTOR __aligned(64);

/**
 * struct _STATUS_DMA_DESCRIPTOR - Status Hardware Descriptor
 * @STATUS_FLAG_BYTE_COUNT: Byte count/buffer length and status flags
 * @USER: User handle and id
 */
typedef struct _STATUS_DMA_DESCRIPTOR {
	union _STATUS_FLAG_BYTE_COUNT {
		uint32_t UINT32_REG;
		struct { /*  Bit  Access */
			uint32_t COMPLETED :1;
			uint32_t SOURCE_ERROR :1;
			uint32_t DESTINATION_ERROR :1;
			uint32_t INTERNAL_ERROR :1;
			uint32_t BYTE_COUNT :27;
			uint32_t UPPER_STATUS_IS_NONZERO :1;
		} BIT;
	} STATUS_FLAG_BYTE_COUNT;
	union {
		uint32_t UINT32_REG;
		struct { /*  Bit  Access */
			uint32_t USER_HANDLE :16;
			uint32_t USER_ID :16;
		} BIT;
	} USER;
}__attribute__((__packed__)) STATUS_DMA_DESCRIPTOR,
		*PSTATUS_DMA_DESCRIPTOR __aligned(64);

typedef enum _PACKET_CONTEXT_AVAILABILITY {
	FREE = 0,    //Packet transfer Parameter context is free.
	IN_USE       //Packet transfer Parameter context is in use.
} PACKET_CONTEXT_AVAILABILITY;

/**
 * Callback function pointer which will be invoked upon packet completion
 * @data: Transfer specific user supplied data
 * @requestedBytes: Number of bytes requested for transfer
 * @completedBytes: Number of bytes actually transferred for the transaction
 * @uid: User id programmed into Src Q of channel.
 */
typedef void (*funcPtrDmaCbk)(void *data, uint32_t requestedBytes,
		uint32_t completedBytes, uint16_t uid);

/**
 * The context structure stored for each DMA transaction
 * This structure is maintained separately for Src Q and Destination Q
 * @availabilityStatus: Indicates whether packet context is available for storing data
 * @idxSOP: Indicates starting index of buffer descriptor corresponding to the transfer
 * @idxEOP: Indicates ending index of buffer descriptor corresponding to the transfer
 * @requestedBytes: Requested bytes for transaction.
 * @sgl: Scatter Gather list of buffer to be processed by DMA
 * @nents: Number of entries in Scatter Gather List.
 * @cbk: Callback function pointer which needs to be invoked upon transaction completion
 * @transferSpecificData: User supplied data for the particular transaction
 */
typedef struct _PACKET_TRANSFER_PARAMS {
	PACKET_CONTEXT_AVAILABILITY availabilityStatus;
	uint32_t idxSOP;
	uint32_t idxEOP;
	uint32_t requestedBytes;
	struct scatterlist *sgl;
	unsigned int nents;
	funcPtrDmaCbk cbk;
	void *transferSpecificData;
} PACKET_TRANSFER_PARAMS, *PPACKET_TRANSFER_PARAMS;

typedef enum _CHANNEL_STATE {
	CHANNEL_RESOURCE_UNALLOCATED = 0, // DMA channel resources are not allocated
	CHANNEL_INITIALIZING,             // DMA channel is getting initialized.
	CHANNEL_FAILED,                   // DMA channel initialization failed.
	CHANNEL_AVAILABLE,                // DMA channel is available for transfers
	CHANNEL_RESET,                    // DMA channel is in reset
	CHANNEL_CONFIGURING,              // DMA channel is getting configured
	CHANNEL_SHUTDOWN                  // DMA channel is getting shutdown.
} CHANNEL_STATE;

typedef enum _INTR_TYPE {
	INTR_LEGACY = 0,       //PS PCIe  DMA device uses Legacy interrupt
	INTR_MSI,              //PS PCIe  DMA device uses MSI interrupt
	INTR_MSIX              //PS PCIe  DMA device uses multiple MSI X interrupts
} INTR_TYPE;

typedef enum _QUEUE_TYPE {
         SRC_Q, //SRC SGL Queue.
         DST_Q  //DST SGL Queue.
}SGL_QUEUE_TYPE;

typedef enum _TRANSFER_TYPE {
         TRANS_SYNCHRONOUS,  //Synchronous Transfer.
         TRANS_ASYNCHRONOUS  //Asynchronous Transfer.
}TRANSFER_TYPE;

/**
 * struct _expresso_dma_chan - Driver specific DMA channel structure
 * @xdev: Driver specific device structure
 * @dev: The dma device
 * @direction_flag: Flag to indicate whether direction is set for channel
 * @direction: Transfer direction
 * @state: Indicates channel state
 * @channelLock: Spin lock to be used before changing channel state
 * @pDMAEngRegs: Pointer to Channel registers
 * @coaelseCount: Indicates number of packet transfer after which interrupt will be raised
 * @pollTimer: Timer to poll dma buffer descriptors if coaelse count is greater than zero
 * @deferpollTimerWorkQ: WorkQ to defer poll timer when interrupt is raised.
 * @deferPollTimerWork: Work which defers poll timer when interrupt is raised.
 * @sglSourceTotalNumberDescriptors: Total sgl source descriptors of channel
 * @sglSourceAvailNumberDescriptors: Available sgl source descriptors for submission
 * @sglSourceDescLock: Lock to be used before modifying sglSourceAvailNumberDescriptors
 * @sglDestinationTotalNumberDescriptors: Total sgl destination descriptors of channel
 * @sglDestinationAvailNumberDescriptors: Available sgl destination descriptors for submission
 * @sglDestinationDescLock: Lock to be used before modifying sglDestinationAvailNumberDescriptors
 * @staSourceTotalNumberDescriptors: Total status Descriptors corresponding to source Queue
 * @staDestinationTotalNumberDescriptors: Total status Descriptors corresponding to Destination Queue
 * @sglSourceQLock: Lock to serialize Source Q updates
 * @srcSGLPhysicalAddressBase: Physical address of Source SGL buffer Descriptors
 * @pSrcSGLDMADescriptorBase: Virtual address of Source SGL buffer Descriptors
 * @srcSGLFreeidx: Holds index of Source SGL buffer descriptor to be filled
 * @sglDestinationQLock:Lock to serialize Destination Q updates
 * @dstSGLPhysicalAddressBase: Physical address of Destination SGL buffer Descriptors
 * @pDstSGLDMADescriptorBase: Virtual address of Destination SGL buffer Descriptors
 * @dstSGLFreeidx: Holds index of Destination SGL
 * @srcSTAPhysicalAddressBase: Physical address of Status buffer Descriptors related to Source Q
 * @pSrcSTADMADescriptorBase: Virtual address of Status buffer Descriptors related to Source Q
 * @srcStaProbeidx: Holds index of Status Q to be examined for updates related to Source Q
 * @srcStaHWProbeidx: Holds index of maximum limit of Status Q for hardware related to Source Q
 * @dstSTAPhysicalAddressBase: Physical address of Status buffer Descriptors related to Dest Q
 * @pDstSTADMADescriptorBase: Virtual address of Status buffer Descriptors related to Dest Q
 * @dstStaProbeidx: Holds index of Status Q to be examined for updates related to Dest Q
 * @dstStaHWProbeidx: Holds index of maximum limit of Status Q for hardware related to Dest Q
 * @pPktCtxSrcQ: Virtual address of packet context which stores information related to Src Q updates
 * @idxCtxSrcQ: Holds index of packet context to be filled for Source Q updation
 * @pPktCtxDstQ: Virtual address of packet context which stores information related to Dst Q updates
 * @idxCtxDstQ: Holds index of packet context to be filled for Destination Q updation
 * @primaryQ: Work Q which performs work related to sgl handling
 * @handlePrimaryWork: Work containing reference to function while getting executed in primary queue.
 * @srcQCompletion: Work Q which performs work related to source Q sgl completion handling
 * @handleSrcQCompletion: Work containing reference to function while handling Src Q completions
 * @dstQCompletion: Work Q which performs work related to Destination Q sgl completion handling
 * @handleDstQCompletion: Work containing reference to function while handling Dst Q completions
 * @srcQWorkCompletion: Completion variable used to notify primary work that src Q handling is done.
 * @dstQWorkCompletion: Completion variable used to notify primary work that dst Q handling is done.
 */
typedef struct _expresso_dma_chan {

	struct expresso_dma_device *xdev;
	struct device *dev;

	unsigned char direction_flag;
	enum dma_data_direction direction;

	CHANNEL_STATE state;
	spinlock_t channelLock;

	volatile PDMA_ENGINE_REGISTERS pDMAEngRegs;

	uint32_t coaelseCount;
	struct timer_list pollTimer;
	struct workqueue_struct *deferpollTimerWorkQ;
	struct work_struct deferPollTimerWork;

	uint32_t sglSourceTotalNumberDescriptors;
	uint32_t sglSourceAvailNumberDescriptors;
	spinlock_t sglSourceDescLock;

	uint32_t sglDestinationTotalNumberDescriptors;
	uint32_t sglDestinationAvailNumberDescriptors;
	spinlock_t sglDestinationDescLock;

	uint32_t staSourceTotalNumberDescriptors;
	uint32_t staDestinationTotalNumberDescriptors;

	spinlock_t sglSourceQLock;
	dma_addr_t srcSGLPhysicalAddressBase;
	PSOURCE_SGL_DMA_DESCRIPTOR pSrcSGLDMADescriptorBase;
	uint32_t srcSGLFreeidx;

	spinlock_t sglDestinationQLock;
	dma_addr_t dstSGLPhysicalAddressBase;
	PDEST_SGL_DMA_DESCRIPTOR pDstSGLDMADescriptorBase;
	uint32_t dstSGLFreeidx;

	dma_addr_t srcSTAPhysicalAddressBase;
	PSTATUS_DMA_DESCRIPTOR pSrcSTADMADescriptorBase;
	uint32_t srcStaProbeidx;
	uint32_t srcStaHWProbeidx;

	dma_addr_t dstSTAPhysicalAddressBase;
	PSTATUS_DMA_DESCRIPTOR pDstSTADMADescriptorBase;
	uint32_t dstStaProbeidx;
	uint32_t dstStaHWProbeidx;

	PPACKET_TRANSFER_PARAMS pPktCtxSrcQ;
	uint16_t idxCtxSrcQ;

	PPACKET_TRANSFER_PARAMS pPktCtxDstQ;
	uint16_t idxCtxDstQ;

	struct workqueue_struct *primaryQ;
	struct work_struct handlePrimaryWork;

	struct workqueue_struct *srcQCompletion;
	struct work_struct handleSrcQCompletion;

	struct workqueue_struct *dstQCompletion;
	struct work_struct handleDstQCompletion;

	struct completion srcQWorkCompletion;
	struct completion dstQWorkCompletion;
} expresso_dma_chan_t;

typedef struct _xlnx_exp_dma_synchronous_transaction {
	SGL_QUEUE_TYPE q_type;
	struct page** cachePages;
	int numPages;
	struct sg_table *sg;
	expresso_dma_chan_t *chan;
	enum dma_data_direction direction;
	struct completion *cmpl_ptr;
	size_t completedBytes;
}exp_dma_sync_trans_t;

typedef enum _buffer_location {
	HOST_MEMORY = 0,    //Buffer location of referenced transaction is HOST memory.
	EP_MEMORY       //Buffer location of referenced transaction is End Point memory
}buffer_location_t;

/**
 * Parameters structure that need to be filled before requesting DMA transaction
 * @t_type: Indicates transfer type, can be either synchronous or asynchronous
 * @q_type: Indicates whether Src Q or Dst Q has to be updated with the specified parameters
 * @chan: Pointer to channel which needs to be updated
 * @sg: Scatter Gather table with sg list for programming
 * @length: Requested byte count for transaction
 * @loc: Location of buffer in memory. Can be either Pci or Axi
 * @uid: User id field. Valid for only Source Q updates
 * @cbk: Function pointer to be invoked upon completion of DMA transaction.
 * @ptr: Transaction specific user supplied data
 */
typedef struct _transfer_initiation_params {
	TRANSFER_TYPE  t_type;
	SGL_QUEUE_TYPE q_type;
	expresso_dma_chan_t *chan;
	struct sg_table *sg;
	size_t length;
	buffer_location_t loc;
	uint16_t uid;
	funcPtrDmaCbk cbk;
	void     *ptr;
}transfer_initiation_params_t;

typedef struct _BAR_PARAMS {
	dma_addr_t BAR_PHYS_ADDR; /**< Base physical address of BAR memory window */
	unsigned long BAR_LENGTH; /**< Length of BAR memory window */
	void *BAR_VIRT_ADDR;      /**< Virtual Address of mapped BAR memory window */
} BAR_PARAMS, *PBAR_PARAMS;

typedef enum _DMA_DEV_CONTEXT {
	DEV_CTXT_FREE,    //DMA dev context free.
	DEV_CTXT_IN_USE   //DMA dev context is in use.
} DMA_DEV_CONTEXT;

/**
 * Global devices list indicating number of similar devices with the same device id
 * that can be connected simultaneously
 * @availability: Indicates whether a specified device context is available or not for a new pci device
 * @pdev: Pci device id of connected device
 * @id: Identification number of pci connected device
 */
typedef struct _EXP_DMA_DEVICES {
	DMA_DEV_CONTEXT availability;
	struct pci_dev *pdev;
	uint32_t  id;
}EXP_DMA_DEVICES;

/**
 * Driver private structure for pci device
 * @channels: Indicates all chanels supported by device
 * @numberBARs: Indicates number of bars which are io remapped
 * @BARInfo: Indicates bar virtual address and length
 * @barmap: Indicates which bars are mapped into virtual space
 * @dev: The dma device
 * @pdev: Pci device
 * @deviceInstance: indicates id in global EXP_DMA_DEVICES array
 * @intrType: Indicates interrupt mechanism currently being used
 * @entry: Indicates all msi x entries for the device
 * @pDMAEngRegs: Channel registers for the very first channel of DMA
 * @charDevice: Character Device for DMA transfers
 * @expressoDMACharDev: Character device id to support DMA transfers
 * @chardev: Array of device nodes supported by character driver for channel specific DMA transfers
 * @pioBarMapAccessCharDevice: Character Device for Bar mapped memory access
 * @pioBarMapAccessCharDev: Character Device id for BAR mapped memory access
 * @pioBarMapAccessCharDevice: Device node supported by Bar mapped memory access
 */
struct expresso_dma_device {

	expresso_dma_chan_t channels[MAX_NUMBER_OF_CHANNELS];

	int numberBARs;
	BAR_PARAMS BARInfo[MAX_BARS];
	int barmap;

	struct device *dev;
	struct pci_dev *pdev;
	uint32_t deviceInstance;

	INTR_TYPE intrType;

	struct msix_entry entry[MAX_NUMBER_OF_CHANNELS];

	volatile PDMA_ENGINE_REGISTERS pDMAEngRegs;

	dev_t charDevice;
	struct cdev expressoDMACharDev;
	struct device *chardev[MAX_NUMBER_OF_CHAR_DEVICES];

	dev_t pioBarMapAccessCharDevice;
	struct cdev pioBarMapAccessCharDev;
	struct device *barMapAccessCharDevice;
	
	struct   mutex    pioCharDevMutex;
	struct   completion translationCmpltn;
	uint32_t pioMappedTranslationSize;
};

static irqreturn_t exp_dma_dev_intr_handler(int irq, void *data);
static irqreturn_t exp_dma_chan_intr_handler(int irq, void *data);
static int expDmaCheckInterruptStatus(expresso_dma_chan_t *chan);

static int msix_free(struct expresso_dma_device *xdev);
static int msi_free(struct expresso_dma_device *xdev);
static int msi_free(struct expresso_dma_device *xdev);

static int msix_setup(struct expresso_dma_device *xdev);
static int msi_setup(struct expresso_dma_device *xdev);
static int legacy_intr_setup(struct expresso_dma_device *xdev);

static int msi_msix_capable(struct pci_dev *dev, int type);
static int irq_probe(struct expresso_dma_device *xdev);
static int irq_setup(struct expresso_dma_device *xdev);

static int irq_free(struct expresso_dma_device *xdev);

static int initializeDMADescriptors(expresso_dma_chan_t *chan);
static void expDMAChannelreset(expresso_dma_chan_t *chan);
static int resetChannel(expresso_dma_chan_t *chan);

static void primaryWorkforChannel(struct work_struct *work);
static void srcQCompletionWork(struct work_struct *work);
static void dstQCompletionWork(struct work_struct *work);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
void pollCompletedTransactions(unsigned long arg);
#else
void pollCompletedTransactions(struct timer_list *t);
#endif
static void deferPollTimer(struct work_struct *work);

int createDeferPollWorkQ(expresso_dma_chan_t *chan);
void destroyDeferPollWorkQ(expresso_dma_chan_t *chan);

void enablePollTimer(expresso_dma_chan_t *chan);
void deletePollTimer(expresso_dma_chan_t *chan);

int allocate_channel_resources(expresso_dma_chan_t *chan);
void deallocate_channel_resources(expresso_dma_chan_t *chan);

static int expressoDMASetMask(struct expresso_dma_device *xdev);
static int exp_dma_open(struct inode * in, struct file * file);
static ssize_t exp_dma_read (struct file *file,
		char __user * buffer, size_t length, loff_t * f_offset);
static ssize_t
exp_dma_write (struct file *file,
		const char __user * buffer, size_t length, loff_t * f_offset);
static int exp_dma_release(struct inode * in, struct file * filp);

static ssize_t
exp_dma_initiate_synchronous_transfer(expresso_dma_chan_t *chan,
		const char __user *buffer, size_t length,loff_t * f_offset,enum dma_data_direction direction);

static int exp_dma_setup_transfer(transfer_initiation_params_t params);
static int exp_dma_update_dstQ(transfer_initiation_params_t params);
static int exp_dma_update_srcQ(transfer_initiation_params_t params);

static int createCharacterInterfaceforDmaDevice(struct expresso_dma_device *xdev,uint32_t devCount);
static void destroyCharacterInterfaceforDmaDevice(
		struct expresso_dma_device *xdev);
static int createSysfsInterfaceforDmaDevice(struct expresso_dma_device *xdev);
static void destroySysfsInterfaceforDmaDevice(struct expresso_dma_device *xdev);
ssize_t sysfs_read_coaelseCount(struct device *dev, struct device_attribute *attr,
			char *buf);
ssize_t sysfs_write_coaelseCount(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);
ssize_t sysfs_channel_reset(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

#endif /* PS_PCIE_DMA_H_ */
