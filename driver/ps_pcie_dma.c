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

#include "ps_pcie_dma.h"

#define DRV_MODULE_NAME		"ps_pcie_dma"
#define EXP_DMA_MAJ_NUM			1
#define EXP_DMA_MIN_NUM			0
#define DRV_MODULE_VERSION	\
	__stringify(EXP_DMA_MAJ_NUM) "." __stringify(EXP_DMA_MIN_NUM)

MODULE_AUTHOR("Xilinx Inc");
MODULE_DESCRIPTION("Xilinx PS PCIe Endpoint DMA Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_MODULE_VERSION);

static struct class *g_expresso_dma_class;	/* global device class */
EXP_DMA_DEVICES devlist[MAX_EXP_DMA_DEVICES]; /* global device list */

/**
 * Callback handler for Synchronous transfers
 * Handles both S2C and C2S transfer call backs.
 * Indicates to blocked applications that DMA transfers are complete
 */
void exp_dma_sync_transfer_cbk(void *data, uint32_t requestedBytes,
		uint32_t completedBytes, uint16_t uid) {

	exp_dma_sync_trans_t *trans = (exp_dma_sync_trans_t *) data;
	int i;

	BUG_ON(!trans);
	BUG_ON(!trans->chan);

	dev_dbg(trans->chan->dev, "Received callback for chan %d Q %d\n",
			trans->chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER,
			trans->q_type);

	trans->completedBytes = completedBytes;

	if (trans->sg != NULL) {
		if (trans->cachePages != NULL) {
			dma_unmap_sg(trans->chan->dev, trans->sg->sgl, trans->sg->nents,
					trans->direction);
		}
		sg_free_table(trans->sg);
		devm_kfree(trans->chan->dev, trans->sg);
	}

	if (trans->cachePages != NULL) {
		for (i = 0; i < trans->numPages; i++) {
			put_page(trans->cachePages[i]);
		}
		devm_kfree(trans->chan->dev, trans->cachePages);
	}

	if (trans->cmpl_ptr != NULL) {
		complete(trans->cmpl_ptr);
	}

	return;
}

/**
 * Function to initiate C2S transfer on a DMA channel
 */
static ssize_t
exp_dma_read (struct file *file,
		char __user * buffer, size_t length, loff_t * f_offset) {

	expresso_dma_chan_t *chan;
	ssize_t ret;

	chan =   file->private_data;

	if(chan->direction_flag == 0) {
		chan->direction = DMA_FROM_DEVICE;
		chan->direction_flag ++;
	}

	ret = exp_dma_initiate_synchronous_transfer(chan,buffer,length,f_offset,DMA_FROM_DEVICE);

	if(ret != length){
		dev_dbg(chan->dev, "Initiate synchronous transfer unsuccessful\n");
	}

	return ret;
}

/**
 * Function to initiate S2C transfer on a DMA channel
 */
static ssize_t
exp_dma_write (struct file *file,
		const char __user * buffer, size_t length, loff_t * f_offset) {

	expresso_dma_chan_t *chan;
	ssize_t ret;

	chan =   file->private_data;

	if(chan->direction_flag == 0) {
		chan->direction = DMA_TO_DEVICE;
		chan->direction_flag ++;
	}

	ret = exp_dma_initiate_synchronous_transfer(chan,buffer,length,f_offset,DMA_TO_DEVICE);

	if(ret != length){
		dev_dbg(chan->dev, "Initiate synchronous transfer unsuccessful\n");
	}

	return ret;
}

/**
 * Function to obtain PS PCIe DMA character device file descriptor
 */
static int exp_dma_open(struct inode * in, struct file * file) {

	struct expresso_dma_device *xdev;
	int minor_num = iminor(in);

	xdev = container_of(in->i_cdev,
			struct expresso_dma_device, expressoDMACharDev);

	file->private_data = &xdev->channels[minor_num];

	dev_dbg(xdev->dev, "PS PCIe DMA character device minor number %d is opened\n",
					minor_num);

	return 0;
}

/**
 * Function to release PS PCIe DMA character device file descriptor
 */
static int exp_dma_release(struct inode * in, struct file * filp) {

	struct expresso_dma_device *xdev;
	int minor_num = iminor(in);

	xdev = container_of(in->i_cdev,
				struct expresso_dma_device, expressoDMACharDev);

	dev_dbg(xdev->dev, "PS PCIe DMA character device minor number %d is released\n",
						minor_num);

	return 0;
}

/**
 * File operation supported by PS PCIe DMA character interface
 */
static const struct file_operations exp_dma_comm_fops = {
	.owner		= THIS_MODULE,
	.read		= exp_dma_read,
	.write      = exp_dma_write,
	.open		= exp_dma_open,
	.release	= exp_dma_release,
};

/**
 * Function to execute io control codes on pio character device
 */
static long pio_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg) {

	volatile uint32_t translationSize = 0;
	long err = 0;
	struct expresso_dma_device *xdev;
	phys_addr_t 	EGRESS_PHYS;
	u32         	val;
	void *kbuf;

	xdev = filp->private_data;

	switch (cmd) {

	case IOCTL_INGRESS_EP_CHECK_TRANSLATION:

		mutex_lock(&xdev->pioCharDevMutex);
		reinit_completion(&xdev->translationCmpltn);
		xdev->pDMAEngRegs->SCRATHC0 =
				EP_TRANSLATION_CHECK;
		xdev->pDMAEngRegs->AXI_INTR_ASSERT.BIT.SOFTWARE_INTRPT = 1;
		wait_for_completion_interruptible(&xdev->translationCmpltn);
		translationSize = xdev->pDMAEngRegs->SCRATHC1;
		if(translationSize > 0) {
		    xdev->pioMappedTranslationSize =
		    		translationSize;
		}else {
			err = -EAGAIN;
		}
		xdev->pDMAEngRegs->SCRATHC1 = 0;
		mutex_unlock(&xdev->pioCharDevMutex);
		break;
	
	case IOCTL_EGRESS_EP_CHECK_TRANSLATION :
		pr_info ("Inside EGRESS IOCTL \n");
		mutex_lock(&xdev->pioCharDevMutex);
		reinit_completion(&xdev->translationCmpltn);
		kbuf = kmalloc(EGRESS_TEST_BUF_SIZE, GFP_KERNEL);
		if (!kbuf) {
			dev_err(xdev->dev,
			"Failed to allocate memory for Egress testing\n");
			goto errout;
		}
		EGRESS_PHYS = (unsigned long)virt_to_phys(kbuf);
		dev_info(xdev->dev, "Egress buffer allocated => 0x%lx\n",
			 (unsigned long)virt_to_phys(kbuf));
		val = lower_32_bits(EGRESS_PHYS);
		xdev->channels[1].pDMAEngRegs->SCRATHC0 = val;
		val = upper_32_bits(EGRESS_PHYS);
		xdev->channels[1].pDMAEngRegs->SCRATHC1 = val;
		xdev->channels[1].pDMAEngRegs->SCRATHC2 =
				EGRESS_TEST_BUF_SIZE;
		xdev->channels[1].pDMAEngRegs->SCRATHC3 =
				EP_TRANSLATION_CHECK;
		xdev->channels[1].pDMAEngRegs->AXI_INTR_ASSERT.BIT.SOFTWARE_INTRPT = 1;
		dev_info(xdev->dev, "Egress buffer received => 0x%llx ",(int *)kbuf);
		translationSize = xdev->channels[1].pDMAEngRegs->SCRATHC2;
		if(translationSize > 0) {
		    xdev->pioMappedTranslationSize =
		    		translationSize;
		}else {
			err = -EAGAIN;
		}
		xdev->channels[1].pDMAEngRegs->SCRATHC2 = 0;
		mutex_unlock(&xdev->pioCharDevMutex);

		mdelay(2000);
		printk("Post egress transfer buf val[0] = %lx\n", readl(kbuf+0));
		printk("Post egress transfer buf val[1] = %lx\n", readl(kbuf+4));
		break;
	errout:
		err = -EINVAL;
	default:
		err = -EINVAL;

	}
	return err;
}

/**
 * Function to initiate read on memory mapped to PCIe BAR
 */
static ssize_t
pio_read (struct file *file,
		char __user * buffer, size_t length, loff_t * f_offset) {

	char *barMemory = NULL;
	struct expresso_dma_device *xdev;
	ssize_t numBytes = 0;

	xdev = file->private_data;
	barMemory = (char *) xdev->BARInfo[PIO_MEMORY_BAR_NUMBER].BAR_VIRT_ADDR;

	if (length > xdev->pioMappedTranslationSize) {
		dev_err(xdev->dev,
				"Error! Invalid buffer length supplied at PIO read\n");
		numBytes = -1;
		goto err_out_too_long_length;
	}

	if ((length + *f_offset)
			> xdev->pioMappedTranslationSize) {
		dev_err(xdev->dev,
				"Error! Invalid buffer offset supplied at PIO read\n");
		numBytes = -1;
		goto err_out_too_long_length_offset;
	}

	barMemory += *f_offset;

	numBytes = copy_to_user(buffer, barMemory, length);

	if (numBytes != 0) {
		dev_err(xdev->dev, "Error! copy_to_user failed at PIO read\n");
		numBytes = length - numBytes;
	}else {
		numBytes = length;
	}

	err_out_too_long_length_offset:
	err_out_too_long_length:
	return numBytes;
}

/**
 * Function to initiate write on memory mapped to PCIe BAR
 */
static ssize_t
pio_write (struct file *file,
		const char __user * buffer, size_t length, loff_t * f_offset) {

	char *barMemory = NULL;
	struct expresso_dma_device *xdev;
	ssize_t numBytes = 0;

	xdev = file->private_data;
	barMemory = (char *) xdev->BARInfo[PIO_MEMORY_BAR_NUMBER].BAR_VIRT_ADDR;

	if (length > xdev->pioMappedTranslationSize) {
		dev_err(xdev->dev,
				"Error! Invalid buffer length supplied at PIO write\n");
		numBytes = -1;
		goto err_out_too_long_length;
	}

	if ((length + *f_offset)
			> xdev->pioMappedTranslationSize) {
		dev_err(xdev->dev,
				"Error! Invalid buffer offset supplied at PIO write\n");
		numBytes = -1;
		goto err_out_too_long_length_offset;
	}

	barMemory += *f_offset;

	numBytes = copy_from_user(barMemory, buffer, length);

	if (numBytes != 0) {
		dev_err(xdev->dev, "Error! copy_from_user failed at PIO write\n");
		numBytes = length - numBytes;
	}else {
		numBytes = length;
	}

	err_out_too_long_length_offset:
	err_out_too_long_length:

	return numBytes;
}

/**
 * Function to obtain PS PCIe PIO character device file descriptor
 */
static int pio_open(struct inode * in, struct file * file) {

	struct expresso_dma_device *xdev;

	xdev = container_of(in->i_cdev,
			struct expresso_dma_device, pioBarMapAccessCharDev);

	file->private_data = xdev;

	dev_dbg(xdev->dev, "PS PCIe PIO character device is opened\n");

	return 0;
}

/**
 * Function to release PS PCIe PIO character device file descriptor
 */
static int pio_release(struct inode * in, struct file * filp) {

	struct expresso_dma_device *xdev;

	xdev = container_of(in->i_cdev,
				struct expresso_dma_device, pioBarMapAccessCharDev);

	dev_dbg(xdev->dev, "PS PCIe PIO character device released\n");

	return 0;
}

/**
 * File operation supported by PS PCIe PIO character interface
 */
static const struct file_operations ps_pcie_pio_fops = {
	.owner		= THIS_MODULE,
	.read		= pio_read,
	.write      = pio_write,
	.unlocked_ioctl = pio_ioctl,
	.open		= pio_open,
	.release	= pio_release,
};

/**
 * exp_dma_update_dstQ
 * Programs Destination Q based on parameters supplied
 *
 * @params: Transfer initiation parameters containing all relevant data to program
 * 			buffer descriptors and initiate call back on completion.
 *
 * Return: 0 on success and non zero value when buffer descriptors are unavailable for programming
 */
static int exp_dma_update_dstQ(transfer_initiation_params_t params) {
	expresso_dma_chan_t *chan = params.chan;
	PDEST_SGL_DMA_DESCRIPTOR pDesc;
	PPACKET_TRANSFER_PARAMS pktContext = NULL;
	struct scatterlist *sgl_ptr;
	unsigned int i;
	int err = 0;

	/* Acquire the lock for the head of the queue, we need to serialize the
	 * DMA Channel queue update
	 */

	spin_lock(&chan->sglDestinationQLock);

	if (chan->sglDestinationAvailNumberDescriptors > params.sg->nents) {
		pktContext = chan->pPktCtxDstQ + chan->idxCtxDstQ;
		if (pktContext->availabilityStatus == IN_USE) {
			spin_unlock(&chan->sglDestinationQLock);
			err = PTR_ERR(chan->pPktCtxDstQ) + 1;
			goto err_out_dst_invalid_pkt_ctxt;
		} else {
			pktContext->availabilityStatus = IN_USE;
			pktContext->requestedBytes = params.length;
			pktContext->sgl = params.sg->sgl;
			pktContext->nents = params.sg->nents;
			pktContext->cbk = params.cbk;
			pktContext->transferSpecificData = params.ptr;
		}
		pDesc = chan->pDstSGLDMADescriptorBase + chan->dstSGLFreeidx;
		pktContext->idxSOP = chan->dstSGLFreeidx;

		/* Build transactions using information in the scatter gather list */
		for_each_sg(params.sg->sgl, sgl_ptr, params.sg->nents, i) {
			pDesc->SYSTEM_ADDRESS_PHYSICAL.UINT64 = sg_dma_address(sgl_ptr);
			pDesc->CONTROL_BYTE_COUNT.BIT.BYTE_COUNT = sg_dma_len(sgl_ptr);
			if(params.loc == HOST_MEMORY) {
			pDesc->CONTROL_BYTE_COUNT.BIT.DMA_DATA_WRITE_ATTRIBUTES =
					PCIe_ATTRIBUTE;
			}else{
				pDesc->CONTROL_BYTE_COUNT.BIT.DMA_DATA_WRITE_ATTRIBUTES =
					AXI_ATTRIBUTE;
			}
			pDesc->CONTROL_BYTE_COUNT.BIT.LOCATION = params.loc;
			// Not Enabling Back to Back Packing. Next Dst SGL element is used for next packet.
			pDesc->CONTROL_BYTE_COUNT.BIT.BACK_TO_BACK_PACKING = 1;
			pDesc->USER.BIT.USER_HANDLE = chan->idxCtxDstQ;
			/* Check if this is last descriptor */
			if (i == (params.sg->nents - 1)) {
				pktContext->idxEOP = chan->dstSGLFreeidx;
			}
			chan->dstSGLFreeidx++;
			if (chan->dstSGLFreeidx
					== chan->sglDestinationTotalNumberDescriptors) {
				chan->dstSGLFreeidx = 0;
			}
			pDesc = chan->pDstSGLDMADescriptorBase + chan->dstSGLFreeidx;
			spin_lock(&chan->sglDestinationDescLock);
			chan->sglDestinationAvailNumberDescriptors--;
			spin_unlock(&chan->sglDestinationDescLock);
		}

		chan->pDMAEngRegs->DST_Q_LIMIT = chan->dstSGLFreeidx;
		chan->idxCtxDstQ++;
		if (chan->idxCtxDstQ == chan->staDestinationTotalNumberDescriptors) {
			chan->idxCtxDstQ = 0;
		}
	}else {
		spin_unlock(&chan->sglDestinationQLock);
		err = chan->sglDestinationAvailNumberDescriptors + 1;
		goto err_out_dst_insufficient_descriptors;
	}

	spin_unlock(&chan->sglDestinationQLock);

	return 0;

	err_out_dst_insufficient_descriptors:
	err_out_dst_invalid_pkt_ctxt:
	return err;
}

/**
 * exp_dma_update_srcQ
 * Programs source Q based on parameters supplied
 *
 * @params: Transfer initiation parameters containing all relevant data to program
 * 			buffer descriptors and initiate call back on completion.
 *
 * Return: 0 on success and non zero value when buffer descriptors are unavailable for programming
 */
static int exp_dma_update_srcQ(transfer_initiation_params_t params) {

	expresso_dma_chan_t *chan = params.chan;
	PSOURCE_SGL_DMA_DESCRIPTOR pDesc;
	PPACKET_TRANSFER_PARAMS pktContext = NULL;
	struct scatterlist *sgl_ptr;
	unsigned int i;
	int err = 0;

	/* Acquire the lock for the head of the queue, we need to serialize the
	 * DMA Channel queue update
	 */

	spin_lock(&chan->sglSourceQLock);

	if (chan->sglSourceAvailNumberDescriptors > params.sg->nents) {
		pktContext = chan->pPktCtxSrcQ + chan->idxCtxSrcQ;
		if (pktContext->availabilityStatus == IN_USE) {
			spin_unlock(&chan->sglSourceQLock);
			err = PTR_ERR(chan->pPktCtxSrcQ) + 1;
			goto err_out_invalid_pkt_ctxt;
		} else {
			pktContext->availabilityStatus = IN_USE;
			pktContext->requestedBytes = params.length;
			pktContext->sgl = params.sg->sgl;
			pktContext->nents = params.sg->nents;
			pktContext->cbk = params.cbk;
			pktContext->transferSpecificData = params.ptr;
		}
		// Get the address of the next available DMA Descriptor
		pDesc = chan->pSrcSGLDMADescriptorBase + chan->srcSGLFreeidx;
		pktContext->idxSOP = chan->srcSGLFreeidx;

		/* Build transactions using information in the scatter gather list */
		for_each_sg(params.sg->sgl, sgl_ptr, params.sg->nents, i) {
			pDesc->SYSTEM_ADDRESS_PHYSICAL.UINT64 = sg_dma_address(sgl_ptr);
			pDesc->CONTROL_BYTE_COUNT.BIT.BYTE_COUNT = sg_dma_len(sgl_ptr);
			if(params.loc == HOST_MEMORY) {
			pDesc->CONTROL_BYTE_COUNT.BIT.DMA_DATA_READ_ATTRIBUTES =
					PCIe_ATTRIBUTE;
			}else{
				pDesc->CONTROL_BYTE_COUNT.BIT.DMA_DATA_READ_ATTRIBUTES =
					AXI_ATTRIBUTE;
			}
			pDesc->CONTROL_BYTE_COUNT.BIT.LOCATION = params.loc;
			pDesc->USER.BIT.USER_HANDLE = chan->idxCtxSrcQ;
			pDesc->USER.BIT.USER_ID = params.uid;
			/* Check if this is last descriptor */
			if (i == (params.sg->nents - 1)) {
				pktContext->idxEOP = chan->srcSGLFreeidx;
				pDesc->CONTROL_BYTE_COUNT.BIT.EOP = 1;
				pDesc->CONTROL_BYTE_COUNT.BIT.INTERRUPT = 1;
			}
			chan->srcSGLFreeidx++;
			if (chan->srcSGLFreeidx == chan->sglSourceTotalNumberDescriptors) {
				chan->srcSGLFreeidx = 0;
			}
			pDesc = chan->pSrcSGLDMADescriptorBase + chan->srcSGLFreeidx;
			spin_lock(&chan->sglSourceDescLock);
			chan->sglSourceAvailNumberDescriptors--;
			spin_unlock(&chan->sglSourceDescLock);
		}

		chan->pDMAEngRegs->SRC_Q_LIMIT = chan->srcSGLFreeidx;
		chan->idxCtxSrcQ++;
		if (chan->idxCtxSrcQ == chan->staSourceTotalNumberDescriptors) {
			chan->idxCtxSrcQ = 0;
		}
	} else {
		spin_unlock(&chan->sglSourceQLock);
		err = chan->sglSourceAvailNumberDescriptors + 1;
		goto err_out_insufficient_descriptors;
	}

	spin_unlock(&chan->sglSourceQLock);

	return 0;

	err_out_insufficient_descriptors:
	err_out_invalid_pkt_ctxt:
	return err;
}

/**
 * exp_dma_setup_transfer
 * Programs either source Q or Destination Q based on parameters supplied
 *
 * @params: Transfer initiation parameters containing all relevant data to program
 * 			buffer descriptors and initiate call back on completion.
 *
 * Return: 0 on success and non zero value for failure
 */
static int exp_dma_setup_transfer(transfer_initiation_params_t params) {

	int err = 0;

	BUG_ON(!params.chan);

	if(params.q_type == SRC_Q) {
		err = exp_dma_update_srcQ(params);
	}else if(params.q_type == DST_Q) {
		err = exp_dma_update_dstQ(params);
	}

	return err;
}

/**
 * exp_dma_initiate_synchronous_transfer
 * Programs both Source Q and Destination Q of channel
 * after setting up sg lists and transaction specific data
 * This functions waits until transaction completion is notified
 *
 * @chan: Pointer to the PS PCIe DMA channel structure
 * @buffer: User land virtual address containing data to be sent or received
 * @length: Length of user land buffer
 * @f_offset: AXI domain address to which data pointed by user buffer has to be sent/received from
 * @direction: Transfer of data direction
 *
 * Return: 0 on success and non zero value for failure
 */
static ssize_t exp_dma_initiate_synchronous_transfer(expresso_dma_chan_t *chan,
		const char __user *buffer, size_t length, loff_t * f_offset,enum dma_data_direction direction) {

	int offset;
	unsigned int allocPages;
	unsigned long first, last;
	struct page** cachePages;
	int err;
	struct sg_table *sg;
	int i;
	exp_dma_sync_trans_t *trans;
	struct completion *cmpl_ptr;
	struct sg_table *ep_mem_desc_table;
	exp_dma_sync_trans_t *ep_mem_trans;
	struct completion *ep_mem_cmpl_ptr;
	SGL_QUEUE_TYPE q_type,dstq_type;
	TRANSFER_TYPE  t_type = TRANS_SYNCHRONOUS;
	transfer_initiation_params_t params;
	size_t completedBytes;

	BUG_ON(!chan);
	BUG_ON(!buffer);

	if(chan->state != CHANNEL_AVAILABLE) {
		err = PTR_ERR(chan);
		goto err_out_chan_unavailable;
	}

	if(chan->direction != direction) {
		err = PTR_ERR(chan);
		dev_err(chan->dev, "Error! Channel used for one data direction is being used for other direction!\n");
		goto err_out_invalid_direction;
	}

	if(direction == DMA_TO_DEVICE) {
		q_type    = SRC_Q;
		dstq_type = DST_Q;
	} else if(direction == DMA_FROM_DEVICE){
		q_type = DST_Q;
		dstq_type = SRC_Q;
	}else {
		dev_err(chan->dev,"Invalid channel direction specified\n");
		err = PTR_ERR(chan);
		goto err_out_invalid_direction;
	}

	offset = offset_in_page(buffer);
	first = ((unsigned long) buffer & PAGE_MASK) >> PAGE_SHIFT;
	last = (((unsigned long) buffer + length - 1) & PAGE_MASK) >> PAGE_SHIFT;
	allocPages = (last - first) + 1;

	cachePages = devm_kzalloc(chan->dev,(allocPages * (sizeof(struct page*))), GFP_ATOMIC);
	if (cachePages == NULL) {
		dev_err(chan->dev, "Unable to allocate memory for page table holder\n");
		err = PTR_ERR(cachePages);
		goto err_out_cachepages_alloc;
	}

	err = get_user_pages_fast((unsigned long) buffer, allocPages,
			!(direction), cachePages);
	if (err <= 0) {
		dev_err(chan->dev, "Unable to pin user pages\n");
		err = PTR_ERR(cachePages);
		goto err_out_pin_pages;
	} else if (err < allocPages) {
		dev_err(chan->dev, "Only pinned few user pages %d\n", err);
		err = PTR_ERR(cachePages);
		for (i = 0; i < err; i++) {
			put_page(cachePages[i]);
		}
		goto err_out_pin_pages;
	}

	sg = devm_kzalloc(chan->dev,sizeof(struct sg_table), GFP_ATOMIC);
	if(sg == NULL) {
		dev_err(chan->dev, "Unable to allocate scatter gather table\n");
		err = PTR_ERR(sg);
		goto err_out_alloc_sg_table;
	}

	err = sg_alloc_table_from_pages(sg, cachePages, allocPages,
				offset, length, GFP_ATOMIC);
	if(err < 0) {
		dev_err(chan->dev,"Unable to create sg table\n");
		goto err_out_sg_to_sgl;
	}

	err = dma_map_sg(chan->dev,sg->sgl,sg->nents,direction);
	if(err == 0) {
		dev_err(chan->dev,"Unable to map user buffer to sg table\n");
		err = PTR_ERR(sg);
		goto err_out_dma_map_sg;
	}

	trans = devm_kzalloc(chan->dev,sizeof(exp_dma_sync_trans_t),GFP_ATOMIC);
	if(trans == NULL) {
		dev_err(chan->dev,"Unable to allocate dma transaction memory\n");
		err = PTR_ERR(trans);
		goto err_out_dma_transaction;
	}

	cmpl_ptr = devm_kzalloc(chan->dev,sizeof(struct completion),GFP_ATOMIC);
	if(cmpl_ptr == NULL) {
		dev_err(chan->dev,"Unable to allocate dma completion pointer\n");
		err = PTR_ERR(cmpl_ptr);
		goto err_out_cmpl_ptr;
	}

	init_completion(cmpl_ptr);

	ep_mem_desc_table = devm_kzalloc(chan->dev,sizeof(struct sg_table), GFP_ATOMIC);
	if(ep_mem_desc_table == NULL) {
		dev_err(chan->dev,"Unable to allocate end point memory sg table\n");
		err = PTR_ERR(ep_mem_desc_table);
		goto err_out_ep_mem_desc_table;
	}

	// EP side memory is assumed to spawn only 1 descriptor always.
	err = sg_alloc_table(ep_mem_desc_table,1,GFP_ATOMIC);
	if(err < 0) {
		dev_err(chan->dev,"Unable to create sg table for ep memory description\n");
		goto err_out_sg_to_sgl_ep_mem;
	}

	sg_dma_address(ep_mem_desc_table->sgl) = (dma_addr_t)(*f_offset);
	sg_dma_len(ep_mem_desc_table->sgl) = length;

	ep_mem_trans = devm_kzalloc(chan->dev,sizeof(exp_dma_sync_trans_t),GFP_ATOMIC);
	if(ep_mem_trans == NULL) {
		dev_err(chan->dev,"Unable to allocate dma ep memory transaction\n");
		err = PTR_ERR(ep_mem_trans);
		goto err_out_ep_dma_transaction;
	}

	ep_mem_cmpl_ptr = devm_kzalloc(chan->dev,sizeof(struct completion),GFP_ATOMIC);
	if(ep_mem_cmpl_ptr == NULL) {
		dev_err(chan->dev,"Unable to allocate dma ep memory transaction completion\n");
		err = PTR_ERR(ep_mem_cmpl_ptr);
		goto err_out_ep_dma_completion;
	}
	init_completion(ep_mem_cmpl_ptr);

	/* Filling out host memory dma transaction.
	 * Will receive this in callback on transaction completion.
	 */
	trans->q_type     = q_type;
	trans->cachePages = cachePages;
	trans->numPages   = allocPages;
	trans->sg         = sg;
	trans->chan       = chan;
	trans->direction  = direction;
	trans->cmpl_ptr   = cmpl_ptr;

	params.t_type = t_type;
	params.q_type = q_type;
	params.chan   = chan;
	params.loc    = HOST_MEMORY;
	params.sg     = sg;
	params.length = length;
	params.uid    = DEFAULT_UID;
	params.cbk    = exp_dma_sync_transfer_cbk;
	params.ptr    = (void *)trans;

	//Retrying till descriptors are available to initiate transfer.
	while(exp_dma_setup_transfer(params));

	/* Filling out ep memory dma transaction.
	 * Will receive this in callback on transaction completion.
	 */

	ep_mem_trans->q_type     = dstq_type;;
	ep_mem_trans->cachePages = NULL;
	ep_mem_trans->numPages   = 1;
	ep_mem_trans->sg         = ep_mem_desc_table;
	ep_mem_trans->chan       = chan;
	ep_mem_trans->direction  = direction;
	ep_mem_trans->cmpl_ptr   = ep_mem_cmpl_ptr;

	params.t_type = t_type;
	params.q_type = dstq_type;
	params.chan   = chan;
	params.loc    = EP_MEMORY;
	params.sg     = ep_mem_desc_table;
	params.length = length;
	params.uid    = DEFAULT_UID;
	params.ptr    = (void *)ep_mem_trans;

	//Retrying till descriptors are available to initiate transfer.
	while(exp_dma_setup_transfer(params));

	wait_for_completion_io(cmpl_ptr);

	wait_for_completion_io(ep_mem_cmpl_ptr);

	completedBytes = trans->completedBytes;
	devm_kfree(chan->dev,cmpl_ptr);
	devm_kfree(chan->dev,ep_mem_cmpl_ptr);
	devm_kfree(chan->dev,trans);
	devm_kfree(chan->dev,ep_mem_trans);

	return completedBytes;

	err_out_ep_dma_completion:
	devm_kfree(chan->dev,ep_mem_trans);
	err_out_ep_dma_transaction:
	sg_free_table(ep_mem_desc_table);
	err_out_sg_to_sgl_ep_mem:
	devm_kfree(chan->dev,ep_mem_desc_table);
	err_out_ep_mem_desc_table:
	devm_kfree(chan->dev,cmpl_ptr);
	err_out_cmpl_ptr:
	devm_kfree(chan->dev,trans);
	err_out_dma_transaction:
	dma_unmap_sg(chan->dev,sg->sgl,sg->nents,direction);
	err_out_dma_map_sg:
	sg_free_table(sg);
	err_out_sg_to_sgl:
	devm_kfree(chan->dev,sg);
	err_out_alloc_sg_table:
	for(i=0;i<allocPages;i++) {
			put_page(cachePages[i]);
	}
	err_out_pin_pages:
	devm_kfree(chan->dev,cachePages);
	err_out_chan_unavailable:
	err_out_cachepages_alloc:
	err_out_invalid_direction:
	return (ssize_t) err;
}

/**
 * msix_free
 * Releases MSI-X interrupt resources
 *
 * @xdev: Driver specific data for device
 *
 * Return: Always 0
 */
static int msix_free(struct expresso_dma_device *xdev) {
	expresso_dma_chan_t *chan;
	int i;

	BUG_ON(!xdev);

	for (i = 0; i < MAX_NUMBER_OF_CHANNELS; i++) {
		chan = &xdev->channels[i];
		free_irq(xdev->entry[i].vector, chan);
		dev_info(xdev->dev, "MSIX irq %d for channel %d freed\n",
				xdev->entry[i].vector,
				chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
	}
	pci_disable_msix(xdev->pdev);

	return 0;
}

/**
 * msi_free
 * Releases MSI interrupt resources
 *
 * @xdev: Driver specific data for device
 *
 * Return: Always 0
 */
static int msi_free(struct expresso_dma_device *xdev) {

	BUG_ON(!xdev);

	free_irq(xdev->pdev->irq, xdev);
	pci_disable_msi(xdev->pdev);

	dev_info(xdev->dev, "MSI irq %d freed\n", xdev->pdev->irq);

	return 0;
}

/**
 * legacy_intr_free
 * Releases legacy interrupt resources
 *
 * @xdev: Driver specific data for device
 *
 * Return: Always 0
 */
static int legacy_intr_free(struct expresso_dma_device *xdev) {

	BUG_ON(!xdev);

	free_irq(xdev->pdev->irq, xdev);

	dev_info(xdev->dev, "Legacy Interrupt irq %d freed\n", xdev->pdev->irq);

	return 0;
}

/**
 * msix_setup
 * Requests MSI X interrupt and registers handlers
 *
 * @xdev: Driver specific data for device
 *
 * Return: 0 on success and non zero value on failure.
 */
static int msix_setup(struct expresso_dma_device *xdev) {
	expresso_dma_chan_t *chan;
	int i;
	int err = 0;

	BUG_ON(!xdev);

	for (i = 0; i < MAX_NUMBER_OF_CHANNELS; i++) {
		chan = &xdev->channels[i];
		err = request_irq(xdev->entry[i].vector, exp_dma_chan_intr_handler,
		EXP_DMA_IRQ_NOSHARE,"PS PCIe DMA MSI-X handler", chan);

		if (err) {
			dev_err(xdev->dev,
					"Couldn't request MSIX irq %d for channel %d error %d\n",
					xdev->entry[i].vector,
					chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER,
					err);
			break;
		}
	}

	if (err) {
		while (--i >= 0) {
			chan = &xdev->channels[i];
			free_irq(xdev->entry[i].vector, chan);
		}
	}

	return err;
}

/**
 * msi_setup
 * Requests MSI interrupt and registers handler
 *
 * @xdev: Driver specific data for device
 *
 * Return: 0 on success and non zero value on failure.
 */
static int msi_setup(struct expresso_dma_device *xdev) {

	int err = 0;

	BUG_ON(!xdev);

	err = request_irq(xdev->pdev->irq, exp_dma_dev_intr_handler,
	EXP_DMA_IRQ_NOSHARE, "PS PCIe DMA MSI Handler", xdev);

	if (err) {
		dev_err(xdev->dev, "Couldn't request MSI irq %d\n", xdev->pdev->irq);
	}

	return err;
}

/**
 * legacy_intr_setup
 * Requests Legacy interrupt and registers handler
 *
 * @xdev: Driver specific data for device
 *
 * Return: 0 on success and non zero value on failure.
 */
static int legacy_intr_setup(struct expresso_dma_device *xdev) {

	int err = 0;

	BUG_ON(!xdev);

	err = request_irq(xdev->pdev->irq, exp_dma_dev_intr_handler,
	IRQF_SHARED, "PS PCIe DMA Legacy Handler", xdev);

	if (err) {
		dev_err(xdev->dev, "Couldn't request Legacy irq %d\n", xdev->pdev->irq);
	}

	return err;
}

/**
 * irq_setup
 * Requests interrupts based on the interrupt type detected from irq_probe
 *
 * @xdev: Driver specific data for device
 *
 * Return: 0 on success and non zero value on failure.
 */
static int irq_setup(struct expresso_dma_device *xdev) {
	int err = 0;

	BUG_ON(!xdev);

	switch (xdev->intrType) {
	case INTR_MSIX:
		err = msix_setup(xdev);
		if (err) {
			dev_err(xdev->dev, "Couldn't setup MSI-X mode: err = %d\n", err);
		}
		break;
	case INTR_MSI:
		err = msi_setup(xdev);
		if (err) {
			dev_err(xdev->dev, "Couldn't setup MSI-X mode: err = %d\n", err);
		}
		break;
	case INTR_LEGACY:
		err = legacy_intr_setup(xdev);
		if (err) {
			dev_err(xdev->dev, "Couldn't setup Legacy interrupt: err = %d\n",
					err);
		}
		break;
	default:
		dev_err(xdev->dev, "Invalid interrupt type!\n");
		err = PTR_ERR(xdev);
	}

	return err;
}

/**
 * irq_free
 * Release interrupt resources
 *
 * @xdev: Driver specific data for device
 *
 * Return: 0 on success and non zero value on failure.
 */
static int irq_free(struct expresso_dma_device *xdev) {
	int err = 0;

	BUG_ON(!xdev);

	switch (xdev->intrType) {
	case INTR_MSIX:
		err = msix_free(xdev);
		if (err) {
			dev_err(xdev->dev, "Couldn't free MSI-X mode: err = %d\n", err);
		}
		break;
	case INTR_MSI:
		err = msi_free(xdev);
		if (err) {
			dev_err(xdev->dev, "Couldn't free MSI-X mode: err = %d\n", err);
		}
		break;
	case INTR_LEGACY:
		err = legacy_intr_free(xdev);
		if (err) {
			dev_err(xdev->dev, "Couldn't free Legacy interrupt: err = %d\n",
					err);
		}
		break;
	default:
		dev_err(xdev->dev, "Invalid interrupt type!\n");
		err = PTR_ERR(xdev);
	}

	return err;
}

/**
 * msi_msix_capable
 * Checks MSI/MSIX capability of pci dev
 *
 * @pci_dev: Pci device
 * @type: PCI_CAP_ID_MSI or PCI_CAP_ID_MSIX
 * Return: 1 on success and 0 on failure
 */
static int msi_msix_capable(struct pci_dev *dev, int type) {

	struct pci_bus *bus;

	if (!dev || dev->no_msi)
		return 0;

	for (bus = dev->bus; bus; bus = bus->parent)
		if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI)
			return 0;

	if (!pci_find_capability(dev, type))
		return 0;

	return 1;
}

/**
 * irq_probe
 * Checks which interrupt types can be services by hardware
 *
 * @xdev: Driver specific data for device
 *
 * Return: Always 0
 */
static int irq_probe(struct expresso_dma_device *xdev) {

	int i;
	int err = 0;
	struct pci_dev *pdev;

	BUG_ON(!xdev);

	pdev = xdev->pdev;

	if (msi_msix_capable(pdev, PCI_CAP_ID_MSIX)) {
		dev_info(&pdev->dev, "Enabling MSI-X\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,12,0) 
		for (i = 0; i < MAX_NUMBER_OF_CHANNELS; i++)
			xdev->entry[i].entry = i;

		err = pci_enable_msix(pdev, xdev->entry, MAX_NUMBER_OF_CHANNELS);
		if (err < 0) {
			dev_err(&pdev->dev, "Couldn't enable MSI-X mode: err = %d\n", err);
		} else {
			xdev->intrType = INTR_MSIX;
			return 0;
		}

#else
		err = pci_alloc_irq_vectors(pdev, 1, 4, PCI_IRQ_ALL_TYPES);
		if (err < 0) {
			dev_err(&pdev->dev, "Couldn't enable MSI-X mode: err = %d\n", err);
		} else {
			for (i = 0; i < MAX_NUMBER_OF_CHANNELS; i++){
				xdev->entry[i].entry = i;
				xdev->entry[i].vector= pci_irq_vector(pdev,i);
			}
			xdev->intrType = INTR_MSIX;
			return 0;
		}
#endif		
		
	}

	if (msi_msix_capable(pdev, PCI_CAP_ID_MSI)) {
		/* enable message signaled interrupts */
		dev_info(&pdev->dev, "Enabling MSI\n");
		err = pci_enable_msi(pdev);
		if (err < 0) {
			dev_err(&pdev->dev, "Couldn't enable MSI mode: err = %d\n", err);
		} else {
			xdev->intrType = INTR_MSI;
			return 0;
		}
	}

	dev_info(&pdev->dev, "MSI/MSI-X not detected - using legacy interrupts\n");
	xdev->intrType = INTR_LEGACY;

	return 0;
}

/**
 * expDmaCheckInterruptStatus
 * Checks channel interrupt status
 *
 * @chan: Pointer to the PS PCIe DMA channel structure
 *
 * Return: 0 if interrupt is pending on channel
 * 		   -1 if no interrupt is pending on channel
 */
static int expDmaCheckInterruptStatus(expresso_dma_chan_t *chan) {

	int err = -1;
	uint32_t status;

	BUG_ON(!chan);

	if (chan->state != CHANNEL_AVAILABLE) {
		return err;
	}

	status = chan->pDMAEngRegs->PCIE_INTR_STATUS.UINT;

	if (status & DMA_INTSTATUS_SGLINTR_BIT) {
		queue_work(chan->primaryQ, &chan->handlePrimaryWork);
		if (chan->deferpollTimerWorkQ != NULL && (chan->coaelseCount >= 1)) {
			queue_work(chan->deferpollTimerWorkQ, &chan->deferPollTimerWork);
		}
		/* Clearing Persistent bit */
		chan->pDMAEngRegs->PCIE_INTR_STATUS.BIT.DMA_SGL_INT = 1;
		err = 0;
	}

	if (status & DMA_INTSTATUS_SWINTR_BIT) {
		if(chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER == 0) {
			complete(&chan->xdev->translationCmpltn);
		}
		/* Clearing Persistent bit */
		chan->pDMAEngRegs->PCIE_INTR_STATUS.BIT.DMA_SW_INT = 1;
		err = 0;
	}

	if (status & DMA_INTSTATUS_DMAERR_BIT) {
		dev_err(chan->dev,
				"DMA Channel %d reported error, ControlStatus Reg: 0x%x",
				chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER, status);
		dev_err(chan->dev,
				"DMA Channel %d Src Q Limit = %d Src Q Size = %d Src Q Next = %d",
				chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER,
				chan->pDMAEngRegs->SRC_Q_LIMIT,
				chan->pDMAEngRegs->SRC_Q_SIZE,
				chan->pDMAEngRegs->SRC_Q_NEXT);
		dev_err(chan->dev,
				"DMA Channel %d Src Sta Q Limit = %d Src Sta Q Size = %d Src Sta Q Next = %d",
				chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER,
				chan->pDMAEngRegs->STAS_Q_LIMIT,
				chan->pDMAEngRegs->STAS_Q_SIZE,
				chan->pDMAEngRegs->STAS_Q_NEXT);
		dev_err(chan->dev,
				"DMA Channel %d Dst Q Limit = %d Dst Q Size = %d Dst Q Next = %d",
				chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER,
				chan->pDMAEngRegs->DST_Q_LIMIT,
				chan->pDMAEngRegs->DST_Q_SIZE,
				chan->pDMAEngRegs->DST_Q_NEXT);
		dev_err(chan->dev,
				"DMA Channel %d Dst Sta Q Limit = %d Dst Sta Q Size = %d Dst Sta Q Next = %d",
				chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER,
				chan->pDMAEngRegs->STAD_Q_LIMIT,
				chan->pDMAEngRegs->STAD_Q_SIZE,
				chan->pDMAEngRegs->STAD_Q_NEXT);
		/* Clearing Persistent bit */
		chan->pDMAEngRegs->PCIE_INTR_STATUS.BIT.DMA_ERR_INT = 1;
		resetChannel(chan);
		err = 0;
	}

	return err;
}

/**
 * exp_dma_dev_intr_handler
 * This will be invoked when MSI or Legacy interrupts are enabled.
 *
 * @irq: IRQ number
 * @data: Pointer to the PS PCIe DMA channel structure
 *
 * Return: IRQ_HANDLED/IRQ_NONE
 */
static irqreturn_t exp_dma_dev_intr_handler(int irq, void *data) {

	struct expresso_dma_device *xdev =
			(struct expresso_dma_device *)data;
	expresso_dma_chan_t *chan = NULL;
	int i;
	int err = -1;
	int ret = -1;

	BUG_ON(!xdev);

	for(i=0;i<MAX_NUMBER_OF_CHANNELS; i++) {
		chan = &xdev->channels[i];
		err = expDmaCheckInterruptStatus(chan);
		if(err == 0) {
			ret = 0;
		}
	}

	return (ret == 0)? IRQ_HANDLED : IRQ_NONE;
}

/**
 * exp_dma_chan_intr_handler
 * This will be invoked only when MSI X interrupts are enabled.
 *
 * @irq: IRQ number
 * @data: Pointer to the PS PCIe DMA channel structure
 *
 * Return: IRQ_HANDLED
 */
static irqreturn_t exp_dma_chan_intr_handler(int irq, void *data) {

	expresso_dma_chan_t *chan = (expresso_dma_chan_t *)data;

	BUG_ON(!chan);

	expDmaCheckInterruptStatus(chan);

	return IRQ_HANDLED;
}

/**
 * initializeDMADescriptors
 * Initializes channel private variables and programs buffer
 * descriptor addresses into DMA channel registers
 *
 * @chan: PS PCIe DMA channel information holder
 * Return: 0 on success and non zero on failure.
 */
static int initializeDMADescriptors(expresso_dma_chan_t *chan) {

	int err;

	BUG_ON(!chan);

	if (chan->pPktCtxSrcQ != NULL && chan->pPktCtxDstQ != NULL) {
		// Zeroing out SourceQ and StatusQ packet contexts
		memset(chan->pPktCtxSrcQ, 0,
				sizeof(PACKET_TRANSFER_PARAMS)
						* chan->staSourceTotalNumberDescriptors);
		memset(chan->pPktCtxDstQ, 0,
				sizeof(PACKET_TRANSFER_PARAMS)
						* chan->staDestinationTotalNumberDescriptors);
	} else {
		dev_err(chan->dev, "PktContext is NULL");
		err = PTR_ERR(chan->pPktCtxSrcQ);
		goto init_desc_err;
	}

	chan->direction_flag = 0;

	chan->sglSourceAvailNumberDescriptors =
			chan->sglSourceTotalNumberDescriptors;
	chan->sglDestinationAvailNumberDescriptors =
			chan->sglDestinationTotalNumberDescriptors;

	chan->srcSGLFreeidx = 0;
	chan->srcStaProbeidx = 0;
	chan->srcStaHWProbeidx = chan->staSourceTotalNumberDescriptors - 1;
	chan->idxCtxSrcQ = 0;

	chan->dstSGLFreeidx = 0;
	chan->dstStaProbeidx = 0;
	chan->dstStaHWProbeidx = chan->staDestinationTotalNumberDescriptors - 1;
	chan->idxCtxDstQ = 0;

	if ((chan->pSrcSGLDMADescriptorBase != NULL)
			&& (chan->pSrcSTADMADescriptorBase != NULL)
			&& (chan->pDstSGLDMADescriptorBase != NULL)
			&& (chan->pDstSTADMADescriptorBase != NULL)) {

		memset(chan->pSrcSGLDMADescriptorBase, 0,
				chan->sglSourceTotalNumberDescriptors
						* sizeof(SOURCE_SGL_DMA_DESCRIPTOR));

		memset(chan->pSrcSTADMADescriptorBase, 0,
				chan->staSourceTotalNumberDescriptors
						* sizeof(STATUS_DMA_DESCRIPTOR));

		memset(chan->pDstSGLDMADescriptorBase, 0,
				chan->sglDestinationTotalNumberDescriptors
						* sizeof(DEST_SGL_DMA_DESCRIPTOR));

		memset(chan->pDstSTADMADescriptorBase, 0,
				chan->staDestinationTotalNumberDescriptors
						* sizeof(STATUS_DMA_DESCRIPTOR));

		// Programming SourceQ and StatusQ buffer descriptor addresses
		chan->pDMAEngRegs->SRC_Q_NEXT = 0;
		chan->pDMAEngRegs->SRC_Q_LOW.BIT.START_ADDR_LOW =
		lower_32_bits(
				(chan->srcSGLPhysicalAddressBase)) >> DMA_SRC_Q_LOW_BIT_SHIFT;
		chan->pDMAEngRegs->SRC_Q_HI = upper_32_bits(
				chan->srcSGLPhysicalAddressBase);
		chan->pDMAEngRegs->SRC_Q_SIZE = chan->sglSourceTotalNumberDescriptors;
		chan->pDMAEngRegs->SRC_Q_LIMIT = 0;
		chan->pDMAEngRegs->SRC_Q_LOW.BIT.QUEUE_LOCATION = 0; //Queue is in PCI memory
		chan->pDMAEngRegs->SRC_Q_LOW.BIT.QUEUE_ENABLE = 1;

		chan->pDMAEngRegs->STAS_Q_NEXT = 0;
		chan->pDMAEngRegs->STAS_Q_LOW.BIT.START_ADDR_LOW =
		lower_32_bits(
				chan->srcSTAPhysicalAddressBase) >> DMA_SRC_Q_LOW_BIT_SHIFT;
		chan->pDMAEngRegs->STAS_Q_HI = upper_32_bits(
				chan->srcSTAPhysicalAddressBase);
		chan->pDMAEngRegs->STAS_Q_SIZE = chan->staSourceTotalNumberDescriptors;
		chan->pDMAEngRegs->STAS_Q_LIMIT = chan->staSourceTotalNumberDescriptors
				- 1;
		chan->pDMAEngRegs->STAS_Q_LOW.BIT.QUEUE_LOCATION = 0; //Queue is in PCI memory
		chan->pDMAEngRegs->STAS_Q_LOW.BIT.QUEUE_ENABLE = 1;

		// Programming DestinationQ and StatusQ buffer descriptor addresses
		chan->pDMAEngRegs->DST_Q_NEXT = 0;
		chan->pDMAEngRegs->DST_Q_LOW.BIT.START_ADDR_LOW =
		lower_32_bits(
				(chan->dstSGLPhysicalAddressBase)) >> DMA_SRC_Q_LOW_BIT_SHIFT;
		chan->pDMAEngRegs->DST_Q_HI = upper_32_bits(
				chan->dstSGLPhysicalAddressBase);
		chan->pDMAEngRegs->DST_Q_SIZE =
				chan->sglDestinationTotalNumberDescriptors;
		chan->pDMAEngRegs->DST_Q_LIMIT = 0;
		chan->pDMAEngRegs->DST_Q_LOW.BIT.QUEUE_LOCATION = 0; //Queue is in PCI memory
		chan->pDMAEngRegs->DST_Q_LOW.BIT.QUEUE_ENABLE = 1;

		chan->pDMAEngRegs->STAD_Q_NEXT = 0;
		chan->pDMAEngRegs->STAD_Q_LOW.BIT.START_ADDR_LOW =
		lower_32_bits(
				chan->dstSTAPhysicalAddressBase) >> DMA_SRC_Q_LOW_BIT_SHIFT;
		chan->pDMAEngRegs->STAD_Q_HI = upper_32_bits(
				chan->dstSTAPhysicalAddressBase);
		chan->pDMAEngRegs->STAD_Q_SIZE =
				chan->staDestinationTotalNumberDescriptors;
		chan->pDMAEngRegs->STAD_Q_LIMIT =
				chan->staDestinationTotalNumberDescriptors - 1;
		chan->pDMAEngRegs->STAD_Q_LOW.BIT.QUEUE_LOCATION = 0; //Queue is in PCI memory
		chan->pDMAEngRegs->STAD_Q_LOW.BIT.QUEUE_ENABLE = 1;
	} else {
		dev_err(chan->dev, "DMA buffer descriptors are unallocated\n");
		err = PTR_ERR(chan->pSrcSGLDMADescriptorBase);
		goto init_desc_err;
	}

	return 0;

	init_desc_err:
	return err;
}

/**
 * expDMAChannelreset
 * Resets channel, by programming relevant registers
 *
 * @chan: PS PCIe DMA channel information holder
 * Return: void
 */
static void expDMAChannelreset(expresso_dma_chan_t *chan) {

	BUG_ON(!chan);

	//Enable channel reset
	chan->pDMAEngRegs->DMA_CHANNEL_CTRL.BIT.DMA_RESET = 1;

	mdelay(10);

	//Disable channel reset
	chan->pDMAEngRegs->DMA_CHANNEL_CTRL.BIT.DMA_RESET = 0;
}

/**
 * resetChannel
 * Resets channel, initializes channel private structures
 * and programs DMA channel registers with buffer descriptor addresses
 *
 * @chan: PS PCIe DMA channel information holder
 * Return: 0 on success and non zero value on failure
 */
static int resetChannel(expresso_dma_chan_t *chan) {

	uint32_t Status = 0;
	int err;
	CHANNEL_STATE prev_state;

	BUG_ON(!chan);

	//Disable interrupts for Channel
	chan->pDMAEngRegs->PCIE_INTR_CNTRL.BIT.INTR_MASK = 0;

	if (chan->deferpollTimerWorkQ != NULL) {
		destroyDeferPollWorkQ(chan);
	}
	if (chan->pollTimer.function != NULL) {
		deletePollTimer(chan);
	}

	flush_workqueue(chan->primaryQ);

	spin_lock(&chan->channelLock);
	prev_state = chan->state;
	chan->state = CHANNEL_RESET;
	spin_unlock(&chan->channelLock);

	//Disable the DMA engine before we get started
	chan->pDMAEngRegs->DMA_CHANNEL_CTRL.BIT.DMA_ENABLE = 0;

	mdelay(10);

	// Clear the persistent bits
	chan->pDMAEngRegs->PCIE_INTR_STATUS.UINT = Status
			| (DMA_INTSTATUS_SWINTR_BIT)
			| ((DMA_INTSTATUS_SGLINTR_BIT) | (DMA_INTSTATUS_DMAERR_BIT));

	expDMAChannelreset(chan);

	err = initializeDMADescriptors(chan);
	if (err) {
		dev_err(chan->dev, "Unable to initialize DMA descriptors");
		goto err_reset_channel;
	}

	chan->pDMAEngRegs->PCIE_INTR_CNTRL.BIT.COALESCE_COUNT = chan->coaelseCount;

	//Enable interrupts for Channel
	chan->pDMAEngRegs->PCIE_INTR_CNTRL.BIT.DMA_ERR_INT_ENABLE = 1;
	chan->pDMAEngRegs->PCIE_INTR_CNTRL.BIT.SGL_INT_ENABLE = 1;
	chan->pDMAEngRegs->PCIE_INTR_CNTRL.BIT.INTR_MASK = 1;

	//64 bit Status Element size
	chan->pDMAEngRegs->DMA_CHANNEL_CTRL.BIT.STA_Q_ELE_SIZE = 1;

	spin_lock(&chan->channelLock);
	chan->state = prev_state;
	spin_unlock(&chan->channelLock);

	//Enable DMA
	chan->pDMAEngRegs->DMA_CHANNEL_CTRL.BIT.DMA_ENABLE = 1;

	if (chan->coaelseCount >= 1) {
		err = createDeferPollWorkQ(chan);
		if (err) {
			dev_err(chan->dev, "Unable to create Defer Poll WorkQ at %s()\n",
					__FUNCTION__);
			goto err_reset_channel;
		}
		enablePollTimer(chan);
	}

	return 0;

	err_reset_channel: return err;
}

/**
 * pollCompletedTransactions
 * Function invoked by poll timer which scedules primary work
 * for channel.
 *
 * @arg: Pointer to PS PCIe DMA channel information
 * Return: void
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
void pollCompletedTransactions(unsigned long arg) {
	expresso_dma_chan_t *chan = (expresso_dma_chan_t *) arg;
#else
void pollCompletedTransactions(struct timer_list *t){
	expresso_dma_chan_t *chan = from_timer(chan, t, pollTimer);
#endif
	BUG_ON(!chan);

	if(chan->state == CHANNEL_AVAILABLE) {
		queue_work(chan->primaryQ, &chan->handlePrimaryWork);
	}

	mod_timer(&chan->pollTimer,jiffies + CHANNEL_POLL_TIMER_FREQUENCY);

	return;
}

/**
 * destroyDeferPollWorkQ
 * Destroys work queue to run defer poll timer work
 *
 * @chan: PS PCIe DMA channel information holder
 *
 * Return: void
 */
void destroyDeferPollWorkQ(expresso_dma_chan_t *chan) {

	BUG_ON(!chan);

	destroy_workqueue(chan->deferpollTimerWorkQ);
	chan->deferpollTimerWorkQ = NULL;

	return;
}

/**
 * createDeferPollWorkQ
 * Creates work queue to run defer poll timer work
 *
 * @chan: PS PCIe DMA channel information holder
 *
 * Return: void
 */
int createDeferPollWorkQ(expresso_dma_chan_t *chan) {

	int err = 0;
	char wq_name[WORKQ_NAME_SIZE];

	BUG_ON(!chan);

	sprintf(wq_name, "PS PCIe DMA Channel %d defer PollTimer WorkQ",
			chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
	chan->deferpollTimerWorkQ = create_singlethread_workqueue(
			(const char* )wq_name);
	if (chan->deferpollTimerWorkQ == NULL) {
		dev_err(chan->dev,
				"Unable to create WorkQ for deferring timer channel %d",
				chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
		err = PTR_ERR(chan->deferpollTimerWorkQ);
	} else {
		INIT_WORK(&(chan->deferPollTimerWork), deferPollTimer);
	}

	return err;
}

/**
 * enablePollTimer
 * Adds timer for polling status Queues periodically
 * when coaelse count is greater than zero
 *
 * @chan: PS PCIe DMA channel information holder
 *
 * Return: void
 */
void enablePollTimer(expresso_dma_chan_t *chan) {

	BUG_ON(!chan);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
	init_timer (&chan->pollTimer);
	chan->pollTimer.function = pollCompletedTransactions;
	chan->pollTimer.expires = jiffies + CHANNEL_POLL_TIMER_FREQUENCY;
	chan->pollTimer.data = (unsigned long)chan;

	add_timer(&chan->pollTimer);
#else
	 timer_setup(&chan->pollTimer,pollCompletedTransactions, 0);
#endif	
	return;
}

/**
 * deletePollTimer
 * Deletes poll timer
 *
 * @chan: PS PCIe DMA channel information holder
 *
 * Return: void
 */
void deletePollTimer(expresso_dma_chan_t *chan) {

	BUG_ON(!chan);

	del_timer_sync(&chan->pollTimer);
	chan->pollTimer.function = NULL;

	return;
}

/**
 * deferPollTimer
 * Defers invoking of poll timer by CHANNEL_POLL_TIMER_FREQUENCY
 * when interrupt for channel is received
 *
 * @work: Work associated with the task
 *
 * Return: void
 */
static void deferPollTimer(struct work_struct *work) {
	expresso_dma_chan_t *chan = (expresso_dma_chan_t *) container_of(work,
			expresso_dma_chan_t, deferPollTimerWork);

	BUG_ON(!work);
	BUG_ON(!chan);

	if(chan->state == CHANNEL_AVAILABLE) {
	    mod_timer(&chan->pollTimer,jiffies + CHANNEL_POLL_TIMER_FREQUENCY);
	}

	return;
}

/**
 * dstQCompletionWork
 * Goes through all completed elements in status Q and invokes callbacks
 * if they are registered for the concerned DMA transaction.
 * Intimates primary work of it's completion
 *
 * @work: Work associated with the task
 *
 * Return: void
 */
static void dstQCompletionWork(struct work_struct *work) {
	expresso_dma_chan_t *chan = (expresso_dma_chan_t *) container_of(work,
			expresso_dma_chan_t, handleDstQCompletion);

	PSTATUS_DMA_DESCRIPTOR pStaBD;
	PDEST_SGL_DMA_DESCRIPTOR pDestBD;
	PPACKET_TRANSFER_PARAMS pPktCxt;
	uint32_t completedBytes = 0;
	uint16_t uid;
	uint32_t dstQdescIdx;

	BUG_ON(!work);
	BUG_ON(!chan);

	pStaBD = chan->pDstSTADMADescriptorBase + chan->dstStaProbeidx;

	while (pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.COMPLETED) {
		if (pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.DESTINATION_ERROR) {
			dev_err(chan->dev,
					"Destination Status Element %d for channel %d reported Destination Error",
					chan->dstStaProbeidx + 1,
					chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
			break;
		}
		if (pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.SOURCE_ERROR) {
			dev_err(chan->dev,
					"Destination Status Element %d for channel %d reported Source Error",
					chan->dstStaProbeidx + 1,
					chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
			break;
		}
		if (pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.INTERNAL_ERROR) {
			dev_err(chan->dev,
					"Destination Status Element %d for channel %d reported Internal Error",
					chan->dstStaProbeidx + 1,
					chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
			break;
		}
		//we are using 64 bit USER field.
		if (pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.UPPER_STATUS_IS_NONZERO == 0) {
			dev_err(chan->dev,
					"Destination Status Element %d for channel %d reported Upper Field is not NonZero Error",
					chan->dstStaProbeidx + 1,
					chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
			break;
		}

		pPktCxt = chan->pPktCtxDstQ + pStaBD->USER.BIT.USER_HANDLE;
		completedBytes =
				(uint32_t) pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.BYTE_COUNT;
		uid = (uint16_t) pStaBD->USER.BIT.USER_ID;

		memset(pStaBD, 0, sizeof(STATUS_DMA_DESCRIPTOR));

		chan->dstStaProbeidx++;

		if (chan->dstStaProbeidx == chan->staDestinationTotalNumberDescriptors)
			chan->dstStaProbeidx = 0;

		chan->dstStaHWProbeidx++;

		if (chan->dstStaHWProbeidx
				== chan->staDestinationTotalNumberDescriptors)
			chan->dstStaHWProbeidx = 0;

		chan->pDMAEngRegs->STAD_Q_LIMIT = chan->dstStaHWProbeidx;

		pStaBD = chan->pDstSTADMADescriptorBase + chan->dstStaProbeidx;

		dstQdescIdx = pPktCxt->idxSOP;

		do {
			pDestBD = chan->pDstSGLDMADescriptorBase + dstQdescIdx;
			memset(pDestBD, 0, sizeof(DEST_SGL_DMA_DESCRIPTOR));

			spin_lock(&chan->sglDestinationDescLock);
			chan->sglDestinationAvailNumberDescriptors++;
			spin_unlock(&chan->sglDestinationDescLock);

			if (dstQdescIdx == pPktCxt->idxEOP) {
				break;
			}
			dstQdescIdx++;

			if (dstQdescIdx == chan->sglDestinationTotalNumberDescriptors) {
				dstQdescIdx = 0;
			}

		} while (1);

		//Invoking callback
		if (pPktCxt->cbk != NULL) {
			pPktCxt->cbk(pPktCxt->transferSpecificData, pPktCxt->requestedBytes,
					completedBytes, uid);
		}
		pPktCxt->availabilityStatus = FREE;
	}

	complete(&chan->dstQWorkCompletion);

	return;
}

/**
 * srcQCompletionWork
 * Goes through all completed elements in status Q and invokes callbacks
 * if they are registered for the concerned DMA transaction.
 * Intimates primary work of it's completion
 *
 * @work: Work associated with the task
 *
 * Return: void
 */
static void srcQCompletionWork(struct work_struct *work) {
	expresso_dma_chan_t *chan = (expresso_dma_chan_t *) container_of(work,
			expresso_dma_chan_t, handleSrcQCompletion);

	PSTATUS_DMA_DESCRIPTOR pStaBD;
	PSOURCE_SGL_DMA_DESCRIPTOR pSrcBD;
	PPACKET_TRANSFER_PARAMS pPktCxt;
	uint32_t completedBytes = 0;
	uint16_t uid;
	uint32_t srcQdescIdx;

	BUG_ON(!work);
	BUG_ON(!chan);

	pStaBD = chan->pSrcSTADMADescriptorBase + chan->srcStaProbeidx;

	while (pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.COMPLETED) {
		if (pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.DESTINATION_ERROR) {
			dev_err(chan->dev,
					"Source Status Element %d for channel %d reported Destination Error",
					chan->srcStaProbeidx + 1,
					chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
			break;
		}
		if (pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.SOURCE_ERROR) {
			dev_err(chan->dev,
					"Source Status Element %d for channel %d reported Source Error",
					chan->srcStaProbeidx + 1,
					chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
			break;
		}
		if (pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.INTERNAL_ERROR) {
			dev_err(chan->dev,
					"Source Status Element %d for channel %d reported Internal Error",
					chan->srcStaProbeidx + 1,
					chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
			break;
		}
		if (pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.UPPER_STATUS_IS_NONZERO == 0) {
			dev_err(chan->dev,
					"Source Status Element %d for channel %d reported Upper Field is not NonZero Error",
					chan->srcStaProbeidx + 1,
					chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
			break;
		}

		pPktCxt = chan->pPktCtxSrcQ + pStaBD->USER.BIT.USER_HANDLE;
		completedBytes = pStaBD->STATUS_FLAG_BYTE_COUNT.BIT.BYTE_COUNT;
		uid = (uint16_t) pStaBD->USER.BIT.USER_ID;

		memset(pStaBD, 0, sizeof(STATUS_DMA_DESCRIPTOR));

		chan->srcStaProbeidx++;

		if (chan->srcStaProbeidx == chan->staSourceTotalNumberDescriptors)
			chan->srcStaProbeidx = 0;

		chan->srcStaHWProbeidx++;

		if (chan->srcStaHWProbeidx == chan->staSourceTotalNumberDescriptors)
			chan->srcStaHWProbeidx = 0;

		chan->pDMAEngRegs->STAS_Q_LIMIT = chan->srcStaHWProbeidx;

		pStaBD = chan->pSrcSTADMADescriptorBase + chan->srcStaProbeidx;

		srcQdescIdx = pPktCxt->idxSOP;

		do {
			pSrcBD = chan->pSrcSGLDMADescriptorBase + srcQdescIdx;
			memset(pSrcBD, 0, sizeof(SOURCE_SGL_DMA_DESCRIPTOR));

			spin_lock(&chan->sglSourceDescLock);
			chan->sglSourceAvailNumberDescriptors++;
			spin_unlock(&chan->sglSourceDescLock);

			if (srcQdescIdx == pPktCxt->idxEOP) {
				break;
			}
			srcQdescIdx++;

			if (srcQdescIdx == chan->sglSourceTotalNumberDescriptors) {
				srcQdescIdx = 0;
			}

		} while (1);

		//Invoking callback
		if (pPktCxt->cbk != NULL) {
			pPktCxt->cbk(pPktCxt->transferSpecificData, pPktCxt->requestedBytes,
					completedBytes, uid);
		}

		pPktCxt->availabilityStatus = FREE;
	}

	complete(&chan->srcQWorkCompletion);
	return;
}

/**
 * primaryWorkforChannel
 * Masks out interrupts, invokes source Q and destination Q processing.
 * Waits for source Q and destination Q processing and re enables interrupts
 *
 * Same work is invoked by timer if coaelse count is greater than zero
 * and interrupts are not invoked before the timeout period
 *
 * @work: Work associated with the task
 *
 * Return: void
 */
static void primaryWorkforChannel(struct work_struct *work) {
	expresso_dma_chan_t *chan = (expresso_dma_chan_t *) container_of(work,
			expresso_dma_chan_t, handlePrimaryWork);

	BUG_ON(!work);
	BUG_ON(!chan);

	//Disabling Interrupts for the channel
	chan->pDMAEngRegs->PCIE_INTR_CNTRL.BIT.INTR_MASK = 0;

	if (chan->srcQCompletion != NULL) {
		init_completion(&chan->srcQWorkCompletion);
		queue_work(chan->srcQCompletion, &chan->handleSrcQCompletion);
	}
	if (chan->dstQCompletion != NULL) {
		init_completion(&chan->dstQWorkCompletion);
		queue_work(chan->dstQCompletion, &chan->handleDstQCompletion);
	}

	if (chan->srcQCompletion != NULL) {
		wait_for_completion_interruptible(&chan->srcQWorkCompletion);
	}
	if (chan->dstQCompletion != NULL) {
		wait_for_completion_interruptible(&chan->dstQWorkCompletion);
	}

	//Enabling Interrupts for the channel
	chan->pDMAEngRegs->PCIE_INTR_CNTRL.BIT.INTR_MASK = 1;

	return;
}

/**
 * deallocate_channel_resources
 * Frees buffer descriptors, work queues and other private structures
 * specific to DMA channel
 *
 * @chan: PS PCIe DMA channel information holder
 *
 * Return: void
 */
void deallocate_channel_resources(expresso_dma_chan_t *chan) {

	size_t size;

	BUG_ON(!chan);

	if (chan->dstQCompletion != NULL) {
		destroy_workqueue(chan->dstQCompletion);
	}
	if (chan->srcQCompletion != NULL) {
		destroy_workqueue(chan->srcQCompletion);
	}
	if (chan->primaryQ != NULL) {
		destroy_workqueue(chan->primaryQ);
	}
	if (chan->pPktCtxDstQ != NULL) {
		devm_kfree(chan->dev, chan->pPktCtxDstQ);
	}
	if (chan->pPktCtxSrcQ != NULL) {
		devm_kfree(chan->dev, chan->pPktCtxSrcQ);
	}
	if (chan->pDstSTADMADescriptorBase != NULL) {
		size = chan->staDestinationTotalNumberDescriptors
				* sizeof(STATUS_DMA_DESCRIPTOR);
		dmam_free_coherent(chan->dev, size, chan->pDstSTADMADescriptorBase,
				chan->dstSTAPhysicalAddressBase);
	}
	if (chan->pDstSGLDMADescriptorBase != NULL) {
		size = chan->sglDestinationTotalNumberDescriptors
				* sizeof(DEST_SGL_DMA_DESCRIPTOR);
		dmam_free_coherent(chan->dev, size, chan->pDstSGLDMADescriptorBase,
				chan->dstSGLPhysicalAddressBase);
	}
	if (chan->pSrcSTADMADescriptorBase != NULL) {
		size = chan->staSourceTotalNumberDescriptors
				* sizeof(STATUS_DMA_DESCRIPTOR);
		dmam_free_coherent(chan->dev, size, chan->pSrcSTADMADescriptorBase,
				chan->srcSTAPhysicalAddressBase);
	}
	if (chan->pSrcSGLDMADescriptorBase != NULL) {
		size = chan->sglSourceTotalNumberDescriptors
				* sizeof(SOURCE_SGL_DMA_DESCRIPTOR);
		dmam_free_coherent(chan->dev, size, chan->pSrcSGLDMADescriptorBase,
				chan->srcSGLPhysicalAddressBase);
	}

	return;
}

/**
 * allocate_channel_resources
 * Allocates buffer descriptors, work queues and other private structures
 * specific to DMA channel
 *
 * @chan: PS PCIe DMA channel information holder
 *
 * Return: '0' on success and failure value on error
 */
int allocate_channel_resources(expresso_dma_chan_t *chan) {

	int err;
	size_t size;
	char wq_name[WORKQ_NAME_SIZE];

	BUG_ON(!chan);

	chan->dev = chan->xdev->dev;

	chan->sglSourceTotalNumberDescriptors =
			chan->sglDestinationTotalNumberDescriptors =
					chan->staSourceTotalNumberDescriptors =
							chan->staDestinationTotalNumberDescriptors =
									NUMBER_OF_BUFFER_DESCRIPTORS;

	size = chan->sglSourceTotalNumberDescriptors
			* sizeof(SOURCE_SGL_DMA_DESCRIPTOR);
	chan->pSrcSGLDMADescriptorBase = dmam_alloc_coherent(chan->dev, size,
			&chan->srcSGLPhysicalAddressBase, GFP_KERNEL);
	if (chan->pSrcSGLDMADescriptorBase == NULL) {
		err = PTR_ERR(chan->pSrcSGLDMADescriptorBase);
		dev_err(chan->dev, "Unable to allocate Src SGL buffer descriptors");
		goto err_out_alloc_bds;
	}

	size = chan->staSourceTotalNumberDescriptors
			* sizeof(STATUS_DMA_DESCRIPTOR);
	chan->pSrcSTADMADescriptorBase = dmam_alloc_coherent(chan->dev, size,
			&chan->srcSTAPhysicalAddressBase, GFP_KERNEL);
	if (chan->pSrcSTADMADescriptorBase == NULL) {
		err = PTR_ERR(chan->pSrcSTADMADescriptorBase);
		dev_err(chan->dev, "Unable to allocate Src Sta buffer descriptors");
		goto err_out_alloc_src_sta_bds;
	}

	size = chan->sglDestinationTotalNumberDescriptors
			* sizeof(DEST_SGL_DMA_DESCRIPTOR);
	chan->pDstSGLDMADescriptorBase = dmam_alloc_coherent(chan->dev, size,
			&chan->dstSGLPhysicalAddressBase, GFP_KERNEL);
	if (chan->pDstSGLDMADescriptorBase == NULL) {
		err = PTR_ERR(chan->pDstSGLDMADescriptorBase);
		dev_err(chan->dev, "Unable to allocate Dst SGL buffer descriptors");
		goto err_out_alloc_dst_sgl_bds;
	}

	size = chan->staDestinationTotalNumberDescriptors
			* sizeof(STATUS_DMA_DESCRIPTOR);
	chan->pDstSTADMADescriptorBase = dmam_alloc_coherent(chan->dev, size,
			&chan->dstSTAPhysicalAddressBase, GFP_KERNEL);
	if (chan->pDstSTADMADescriptorBase == NULL) {
		err = PTR_ERR(chan->pDstSTADMADescriptorBase);
		dev_err(chan->dev, "Unable to allocate Dst Sta buffer descriptors");
		goto err_out_alloc_dst_sta_bds;
	}

	chan->pPktCtxSrcQ = devm_kmalloc(chan->dev,
			sizeof(PACKET_TRANSFER_PARAMS)
					* chan->staSourceTotalNumberDescriptors, GFP_KERNEL);
	if (chan->pPktCtxSrcQ == NULL) {
		err = PTR_ERR(chan->pPktCtxSrcQ);
		dev_err(chan->dev, "Unable to allocate SrcQ Context");
		goto err_out_alloc_srcq_ctxt;
	}

	chan->pPktCtxDstQ = devm_kmalloc(chan->dev,
			sizeof(PACKET_TRANSFER_PARAMS)
					* chan->staDestinationTotalNumberDescriptors, GFP_KERNEL);
	if (chan->pPktCtxDstQ == NULL) {
		err = PTR_ERR(chan->pPktCtxDstQ);
		dev_err(chan->dev, "Unable to allocate DstQ Context");
		goto err_out_alloc_dsq_ctxt;
	}

	sprintf(wq_name, "PS PCIe DMA Channel %d Primary WQ",
			chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
	chan->primaryQ = create_singlethread_workqueue((const char* )wq_name);
	if (chan->primaryQ == NULL) {
		dev_err(chan->dev, "Unable to create primary work Queue for channel %d",
				chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
		err = PTR_ERR(chan->primaryQ);
		goto err_out_primary_wq;
	} else {
		INIT_WORK(&(chan->handlePrimaryWork), primaryWorkforChannel);
	}
	memset(wq_name, 0, WORKQ_NAME_SIZE);

	sprintf(wq_name, "PS PCIe DMA Channel %d SrcQ WorkQ",
			chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
	chan->srcQCompletion = create_singlethread_workqueue((const char* )wq_name);
	if (chan->srcQCompletion == NULL) {
		dev_err(chan->dev,
				"Unable to create SrcQ Completion Queue for channel %d",
				chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
		err = PTR_ERR(chan->srcQCompletion);
		goto err_out_src_wq;
	} else {
		INIT_WORK(&(chan->handleSrcQCompletion), srcQCompletionWork);
	}
	memset(wq_name, 0, WORKQ_NAME_SIZE);

	sprintf(wq_name, "PS PCIe DMA Channel %d DstQ WorkQ",
			chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
	chan->dstQCompletion = create_singlethread_workqueue((const char* )wq_name);
	if (chan->dstQCompletion == NULL) {
		dev_err(chan->dev,
				"Unable to create DstQ Completion Queue for channel %d",
				chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER);
		err = PTR_ERR(chan->dstQCompletion);
		goto err_out_dst_wq;
	} else {
		INIT_WORK(&(chan->handleDstQCompletion), dstQCompletionWork);
	}
	memset(wq_name, 0, WORKQ_NAME_SIZE);

	spin_lock_init(&chan->channelLock);
	spin_lock_init(&chan->sglSourceQLock);
	spin_lock_init(&chan->sglSourceDescLock);
	spin_lock_init(&chan->sglDestinationQLock);
	spin_lock_init(&chan->sglDestinationDescLock);

	chan->coaelseCount = CHANNEL_COAELSE_COUNT;

	return 0;

	err_out_dst_wq:
	destroy_workqueue(chan->srcQCompletion);
	err_out_src_wq:
	destroy_workqueue(chan->primaryQ);
	err_out_primary_wq:
	devm_kfree(chan->dev, chan->pPktCtxDstQ);
	err_out_alloc_dsq_ctxt:
	devm_kfree(chan->dev, chan->pPktCtxSrcQ);
	err_out_alloc_srcq_ctxt:
	size = chan->staDestinationTotalNumberDescriptors
			* sizeof(STATUS_DMA_DESCRIPTOR);
	dmam_free_coherent(chan->dev, size, chan->pDstSTADMADescriptorBase,
			chan->dstSTAPhysicalAddressBase);
	err_out_alloc_dst_sta_bds:
	size = chan->sglDestinationTotalNumberDescriptors
			* sizeof(DEST_SGL_DMA_DESCRIPTOR);
	dmam_free_coherent(chan->dev, size, chan->pDstSGLDMADescriptorBase,
			chan->dstSGLPhysicalAddressBase);
	err_out_alloc_dst_sgl_bds:
	size = chan->staSourceTotalNumberDescriptors
			* sizeof(STATUS_DMA_DESCRIPTOR);
	dmam_free_coherent(chan->dev, size, chan->pSrcSTADMADescriptorBase,
			chan->srcSTAPhysicalAddressBase);
	err_out_alloc_src_sta_bds:
	size = chan->sglSourceTotalNumberDescriptors
			* sizeof(SOURCE_SGL_DMA_DESCRIPTOR);
	dmam_free_coherent(chan->dev, size, chan->pSrcSGLDMADescriptorBase,
			chan->srcSGLPhysicalAddressBase);
	err_out_alloc_bds:
	return err;
}

/* Declare the sysfs entries. The macros create instances of dev_attr_coaelseCount */
static DEVICE_ATTR(coaelseCount, S_IWUSR | S_IRUSR, sysfs_read_coaelseCount, sysfs_write_coaelseCount);


ssize_t sysfs_read_coaelseCount(struct device *dev, struct device_attribute *attr,
			char *buf) {
	expresso_dma_chan_t *chan;

	chan = dev_get_drvdata(dev);

	return sprintf(buf,"%d\n",chan->coaelseCount);
}

ssize_t sysfs_write_coaelseCount(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	expresso_dma_chan_t *chan;
	uint32_t coaelseCount = 0;
	int err = 0;

	chan = dev_get_drvdata(dev);

	sscanf(buf, "%d", &coaelseCount);

	if ((coaelseCount < 0) || (coaelseCount > MAX_COAELSE_COUNT)) {
		return 0;
	}

	if (coaelseCount == 0) {
		if (chan->deferpollTimerWorkQ != NULL) {
			destroyDeferPollWorkQ(chan);
		}
		if (chan->pollTimer.function != NULL) {
			deletePollTimer(chan);
		}
	} else {
		if ((chan->deferpollTimerWorkQ == NULL)
				&& (chan->pollTimer.function == NULL)) {
			err = createDeferPollWorkQ(chan);
			if (err) {
				dev_err(chan->dev,
						"Unable to create Defer Poll WorkQ at %s()\n",
						__FUNCTION__);
				goto err_sysfs_create_defer_workQ;
			}
			enablePollTimer(chan);
		}
	}

	chan->coaelseCount = coaelseCount;
	chan->pDMAEngRegs->PCIE_INTR_CNTRL.BIT.COALESCE_COUNT = chan->coaelseCount;

	dev_info(chan->dev,"Updated channel %d coaelse count as %d\n",
			chan->pDMAEngRegs->DMA_CHANNEL_STATUS.BIT.CHANNEL_NUMBER,
			coaelseCount);

	return count;

	err_sysfs_create_defer_workQ: return -1;
}

/**
 * destroySysfsInterfaceforDmaDevice
 * Removes sysfs file created for each channel in device
 *
 * @xdev: Driver specific data for device
 *
 * Return: void
 */
static void destroySysfsInterfaceforDmaDevice(struct expresso_dma_device *xdev) {
	int i;

	BUG_ON(!xdev);

	for (i = 0; i < MAX_NUMBER_OF_CHAR_DEVICES; i++) {
		device_remove_file(xdev->chardev[i], &dev_attr_coaelseCount);
		}

	return;
}
/**
 * createSysfsInterfaceforDmaDevice
 * Creates sysfs entries to modify coaelseCount and initiate reset
 * for each channel in device
 *
 * @xdev: Driver specific data for device
 *
 * Return: '0' on success and failure value on error
 */
static int createSysfsInterfaceforDmaDevice(struct expresso_dma_device *xdev) {
	int err = 0;
	int i;

	BUG_ON(!xdev);

	for (i = 0; i < MAX_NUMBER_OF_CHAR_DEVICES; i++) {
		if (device_create_file(xdev->chardev[i], &dev_attr_coaelseCount) != 0) {
			err = PTR_ERR(xdev->chardev[i]);
			dev_err(xdev->dev,
					"PS PCIe DMA Unable to create sysfs coaelseCount file %d\n",
					i);
			goto err_out_sysfs_create;
		}
	}

	return 0;

err_out_sysfs_create:
	while (--i >= 0) {
		device_remove_file(xdev->chardev[i], &dev_attr_coaelseCount);
	}
	return err;
}

/**
 * destroyCharacterInterfaceforBarMappedAccess
 * Unregisters character dev region and dev node for DMA device
 * which was used to do Bar Mapped memory access from user space
 *
 * @xdev: Driver specific data for device
 *
 * Return: void
 */
static void destroyCharacterInterfaceforBarMappedAccess(
		struct expresso_dma_device *xdev) {

	BUG_ON(!xdev);

	device_destroy(g_expresso_dma_class,
			MKDEV(MAJOR(xdev->pioBarMapAccessCharDevice), 0));
	cdev_del(&xdev->pioBarMapAccessCharDev);
	unregister_chrdev_region(xdev->pioBarMapAccessCharDevice,1);

	return;
}

/**
 * createCharacterInterfaceforBarMappedAccess
 * Allocates character dev region and dev node for DMA device
 * to do Bar Mapped memory access from user space
 *
 * @xdev: Driver specific data for device
 * @deviceCount: Board number of current device
 *
 * Return: '0' on success and failure value on error
 */
static int createCharacterInterfaceforBarMappedAccess(
		struct expresso_dma_device *xdev, uint32_t deviceCount) {

	int err = 0;

	BUG_ON(!xdev);

	err = alloc_chrdev_region(&xdev->pioBarMapAccessCharDevice, 0, 1,
			PIO_CHAR_DRIVER_NAME);

	if (err < 0) {
		dev_err(xdev->dev,
				"PS PCIe DMA Unable to allocate pio character device region\n");
		goto err_out_pio_chrdev_region;
	}

	xdev->pioBarMapAccessCharDev.owner = THIS_MODULE;
	cdev_init(&xdev->pioBarMapAccessCharDev, &ps_pcie_pio_fops);
	xdev->pioBarMapAccessCharDev.dev = xdev->pioBarMapAccessCharDevice;

	err = cdev_add(&xdev->pioBarMapAccessCharDev,
			xdev->pioBarMapAccessCharDevice, 1);
	if (err < 0) {
		dev_err(xdev->dev, "PS PCIe DMA unable to add cdev for pio\n");
		goto err_out_pio_cdev_add;
	}

	xdev->barMapAccessCharDevice = device_create(g_expresso_dma_class,
			xdev->dev, MKDEV(MAJOR(xdev->pioBarMapAccessCharDevice), 0), xdev,
			"%s_%d", PIO_CHAR_DRIVER_NAME, deviceCount);

	if (xdev->barMapAccessCharDevice == NULL) {
		err = PTR_ERR(xdev->barMapAccessCharDevice);
		dev_err(xdev->dev, "PS PCIe DMA Unable to create pio device\n");
		goto err_out_pio_dev_create;
	}

	return 0;

	err_out_pio_dev_create:
	cdev_del(&xdev->pioBarMapAccessCharDev);
	err_out_pio_cdev_add:
	unregister_chrdev_region(
			xdev->pioBarMapAccessCharDevice, 1);
	err_out_pio_chrdev_region:
	return err;
}

/**
 * destroyCharacterInterfaceforDmaDevice
 * Unregisters character dev region and dev nodes for each channel in device
 *
 * @xdev: Driver specific data for device
 *
 * Return: void
 */
static void destroyCharacterInterfaceforDmaDevice(
		struct expresso_dma_device *xdev) {
	int i;

	BUG_ON(!xdev);

	for (i = 0; i < MAX_NUMBER_OF_CHAR_DEVICES; i++) {
		device_destroy(g_expresso_dma_class, MKDEV(MAJOR(xdev->charDevice), i));
	}
	cdev_del(&xdev->expressoDMACharDev);
	unregister_chrdev_region(xdev->charDevice, MAX_NUMBER_OF_CHAR_DEVICES);

	return;
}

/**
 * createCharacterInterfaceforDmaDevice
 * Allocates character dev region and dev nodes for each channel in device
 *
 * @xdev: Driver specific data for device
 * @deviceCount: Board number of current device
 *
 * Return: '0' on success and failure value on error
 */
static int createCharacterInterfaceforDmaDevice(struct expresso_dma_device *xdev,
		uint32_t deviceCount) {

	int err = 0;
	int i;

	BUG_ON(!xdev);

	err = alloc_chrdev_region(&xdev->charDevice, 0,
			MAX_NUMBER_OF_CHAR_DEVICES, CHAR_DRIVER_NAME);

	if(err < 0) {
		dev_err(xdev->dev, "PS PCIe DMA Unable to allocate character device region\n");
		goto err_out_chrdev_region;
	}

	xdev->expressoDMACharDev.owner = THIS_MODULE;
	cdev_init(&xdev->expressoDMACharDev, &exp_dma_comm_fops);
	xdev->expressoDMACharDev.dev = xdev->charDevice;

	err = cdev_add(&xdev->expressoDMACharDev,
				xdev->charDevice, MAX_NUMBER_OF_CHAR_DEVICES);
		if(err < 0) {
			dev_err(xdev->dev,"PS PCIe DMA unable to add cdev\n");
			goto err_out_cdev_add;
		}

	for(i=0;i<MAX_NUMBER_OF_CHAR_DEVICES;i++) {

		xdev->chardev[i] =
				device_create(g_expresso_dma_class,xdev->dev,
						MKDEV(MAJOR(xdev->charDevice), i),&xdev->channels[i],
						"%s%d_%d",CHAR_DRIVER_NAME,i,deviceCount);

		if(xdev->chardev[i] == NULL) {
			err = PTR_ERR(xdev->chardev[i]);
			dev_err(xdev->dev, "PS PCIe DMA Unable to create device %d\n",i);
			goto err_out_dev_create;
		}
	}

	return 0;

	err_out_dev_create:
	while (--i >= 0) {
		device_destroy(g_expresso_dma_class,
				MKDEV(MAJOR(xdev->charDevice), i));
	}
	cdev_del(&xdev->expressoDMACharDev);
	err_out_cdev_add:
	unregister_chrdev_region(xdev->charDevice,MAX_NUMBER_OF_CHAR_DEVICES);
	err_out_chrdev_region:
	return err;
}

/**
 * expressoDMASetMask - Sets DMA mask
 * @xdev: Driver specific data for device
 *
 * Return: '0' on success and failure value on error
 */

static int expressoDMASetMask(struct expresso_dma_device *xdev) {

	int err = 0;

	BUG_ON(!xdev);

	if (!dma_set_mask(xdev->dev, DMA_BIT_MASK(64))
			&& !dma_set_coherent_mask(xdev->dev, DMA_BIT_MASK(64))) {
		dev_info(xdev->dev, "PS PCIe DMA PCIe 64bit access capable\n");
	} else {
		err = dma_set_mask(xdev->dev, DMA_BIT_MASK(32));
		if (err) {
			err = dma_set_coherent_mask(xdev->dev, DMA_BIT_MASK(32));
			dev_info(xdev->dev, "PS PCIe DMA PCIe 32bit access capable\n");
			if (err) {
				dev_err(xdev->dev, "No usable DMA configuration, aborting\n");
			}
		}
	}

	return err;

}

/**
 * exp_pci_probe - Driver probe function
 * @pdev: Pointer to the pci_dev structure
 * @ent: pci device id
 *
 * Return: '0' on success and failure value on error
 */

static int exp_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {

	int err, i;
	int numChannels;
	unsigned long pciBarLength;

	uint32_t deviceCount;

	struct expresso_dma_device *xdev = NULL;

	dev_info(&pdev->dev, "PS PCIe DMA PCIe Driver probe");

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "Cannot enable PCI device, aborting\n");
		return err;
	}

	err = pci_request_regions(pdev, DRV_MODULE_NAME);
	if (err) {
		dev_err(&pdev->dev, "Cannot obtain PCI resources, aborting\n");
		goto err_out_disable_pdev;
	}

	pci_set_master(pdev);

	xdev = devm_kzalloc(&pdev->dev, sizeof(struct expresso_dma_device),
	GFP_KERNEL);

	if (xdev == NULL) {
		dev_err(&pdev->dev,
				"Cannot Allocate Device private structure, aborting");
		err = PTR_ERR(xdev);
		goto err_out_free_res;
	}

	xdev->dev = &pdev->dev;
	xdev->pdev = pdev;

	err = expressoDMASetMask(xdev);
	if (err) {
		dev_err(&pdev->dev, "Unable to set DMA Mask\n");
		goto err_out_free_dev;
	}

	for (i = 0; i < MAX_BARS; i++) {
		pciBarLength = pci_resource_len(pdev, i);
		if (pciBarLength > 0) {
			xdev->numberBARs++;
			xdev->BARInfo[i].BAR_LENGTH = pciBarLength;
			xdev->BARInfo[i].BAR_PHYS_ADDR = pci_resource_start(pdev, i);
			xdev->BARInfo[i].BAR_VIRT_ADDR = pci_ioremap_bar(pdev, i);
			xdev->barmap |= 1 << i;
		} else {
			xdev->BARInfo[i].BAR_LENGTH = 0;
			xdev->BARInfo[i].BAR_PHYS_ADDR = 0;
			xdev->BARInfo[i].BAR_VIRT_ADDR = NULL;
		}
	}

	xdev->pDMAEngRegs =
			((PDMA_ENGINE_REGISTERS) xdev->BARInfo[DMA_BAR_NUMBER].BAR_VIRT_ADDR);

	for (i = 0, numChannels = 0; i < MAX_NUMBER_OF_CHANNELS; i++) {

		xdev->channels[i].xdev = xdev;
		xdev->channels[i].pDMAEngRegs =
				xdev->pDMAEngRegs + numChannels;
		err = allocate_channel_resources(
				&xdev->channels[i]);
		if (err) {
			dev_err(&pdev->dev, "Unable to allocate channel %d resources",
					numChannels);
			goto err_out_iounmap;
		} else {
			xdev->channels[i].state =
					CHANNEL_INITIALIZING;
		}
		numChannels++;
	}

	for (i = 0; i < MAX_NUMBER_OF_CHANNELS; i++) {
		err = resetChannel(&xdev->channels[i]);
		if (err) {
			dev_err(&pdev->dev, "Unable to reset channel %d", i);
			xdev->channels[i].state = CHANNEL_FAILED;
			goto err_out_iounmap;
		}
	}

	err = irq_probe(xdev);
	if (err) {
		dev_err(&pdev->dev, "Unable to query interrupts information\n");
		goto err_out_interrupts_probe;
	}

	err = irq_setup(xdev);
	if (err) {
		dev_err(&pdev->dev, "Unable to setup interrupts\n");
		goto err_out_interrupts_setup;
	}

	for(i=0;i<MAX_EXP_DMA_DEVICES;i++) {
		if(devlist[i].availability == DEV_CTXT_FREE) {
			devlist[i].availability = DEV_CTXT_IN_USE;
			devlist[i].pdev = pdev;
			deviceCount = devlist[i].id;
			xdev->deviceInstance = deviceCount;
			break;
		}
	}

	if(i == MAX_EXP_DMA_DEVICES) {
		dev_err(&pdev->dev, "Driver reached maximum capacity in terms of number of parallel PCIe boards\n");
		goto err_out_dma_dev_list;
	}

	pci_set_drvdata(pdev, xdev);

	err = createCharacterInterfaceforDmaDevice(xdev,deviceCount);
	if(err) {
		dev_err(&pdev->dev, "Unable to create Character Device Interface\n");
		goto err_out_create_char_iface;
	}

	err = createSysfsInterfaceforDmaDevice(xdev);
	if(err) {
		dev_err(&pdev->dev, "Unable to create sysfs Interface\n");
		goto err_out_create_sysfs_iface;
	}

	err = createCharacterInterfaceforBarMappedAccess(xdev,deviceCount);
	if(err) {
		dev_err(&pdev->dev, "Unable to create Character Driver Interface for pio\n");
		goto err_out_create_char_iface_pio;
	}

	mutex_init(&xdev->pioCharDevMutex);
	xdev->pioMappedTranslationSize = 0;
	init_completion(&xdev->translationCmpltn);

	for (i = 0; i < MAX_NUMBER_OF_CHANNELS; i++) {
		xdev->channels[i].state = CHANNEL_AVAILABLE;
	}

	dev_info(&pdev->dev, "PS PCIe DMA driver successfully probed\n");

	return 0;

	err_out_create_char_iface_pio:
	destroySysfsInterfaceforDmaDevice(xdev);
	err_out_create_sysfs_iface:
	destroyCharacterInterfaceforDmaDevice(xdev);
	err_out_create_char_iface:
	err_out_dma_dev_list:
	irq_free(xdev);
	err_out_interrupts_setup:
	err_out_interrupts_probe:
	err_out_iounmap:
	for (i = 0, numChannels = 0; i < MAX_NUMBER_OF_CHANNELS; i++) {
		xdev->channels[i].state = CHANNEL_FAILED;
		if (xdev->channels[i].deferpollTimerWorkQ != NULL) {
			destroyDeferPollWorkQ(&xdev->channels[i]);
		}
		if (xdev->channels[i].pollTimer.function != NULL) {
			deletePollTimer(&xdev->channels[i]);
		}
		deallocate_channel_resources(&xdev->channels[i]);
	}
	for (i = 0; i < MAX_BARS; i++) {
		if (xdev->BARInfo[i].BAR_VIRT_ADDR != NULL) {
			iounmap(xdev->BARInfo[i].BAR_VIRT_ADDR);
		}
	}
	err_out_free_dev:
	devm_kfree(&pdev->dev, xdev);
	err_out_free_res:
	pci_release_regions(pdev);
	err_out_disable_pdev:
	if (pci_is_enabled(pdev))
		pci_disable_device(pdev);

	return err;
}

/**
 * exp_pci_remove - Driver remove function
 * @pdev: Pointer to the pci_dev structure
 *
 * Return: void
 */

static void exp_pci_remove(struct pci_dev *pdev) {
	struct expresso_dma_device *xdev = NULL;
	int i;

	xdev = (struct expresso_dma_device *) pci_get_drvdata(pdev);

	BUG_ON(!xdev);

	irq_free(xdev);

	destroyCharacterInterfaceforBarMappedAccess(xdev);
	destroySysfsInterfaceforDmaDevice(xdev);
	destroyCharacterInterfaceforDmaDevice(xdev);

	for (i = 0; i < MAX_NUMBER_OF_CHANNELS; i++) {
		spin_lock(&xdev->channels[i].channelLock);
		xdev->channels[i].state = CHANNEL_SHUTDOWN;
		spin_unlock(&xdev->channels[i].channelLock);
		if (xdev->channels[i].deferpollTimerWorkQ != NULL) {
			destroyDeferPollWorkQ(&xdev->channels[i]);
		}
		if (xdev->channels[i].pollTimer.function != NULL) {
			deletePollTimer(&xdev->channels[i]);
		}
		deallocate_channel_resources(&xdev->channels[i]);
	}

	for (i = 0; i < MAX_BARS; i++) {
		if (xdev->BARInfo[i].BAR_VIRT_ADDR != NULL) {
			iounmap(xdev->BARInfo[i].BAR_VIRT_ADDR);
		}
	}

	if (devlist[xdev->deviceInstance].pdev == pdev) {
		devlist[xdev->deviceInstance].availability = DEV_CTXT_FREE;
		devlist[xdev->deviceInstance].pdev = NULL;
	}

	devm_kfree(&pdev->dev, xdev);

	pci_release_regions(pdev);

	if (pci_is_enabled(pdev)) {
		pci_disable_device(pdev);
	}

	return;
}

static struct pci_device_id exp_pci_tbl[] = { {
		PCI_DEVICE(PCI_VENDOR_XILINX, ZYNQMP_DMA_DEVID0) },{
		PCI_DEVICE(PCI_VENDOR_XILINX, ZYNQMP_DMA_DEVID1) }, { } };

static struct pci_driver expresso_dma_driver = { .name = DRV_MODULE_NAME,
		.id_table = exp_pci_tbl, .probe = exp_pci_probe, .remove =
				exp_pci_remove, };
/**
 * exp_pci_init - Driver init function
 *
 * Return: 0 on success. Non zero on failue
 */
static int __init exp_pci_init(void)
{
	int err = 0;
	int i;

	printk(KERN_ERR"%s init()\n",DRV_MODULE_NAME);

	g_expresso_dma_class = class_create(THIS_MODULE, DRV_MODULE_NAME);

	if (IS_ERR(g_expresso_dma_class)) {
		printk(KERN_ERR"%s failed to create class\n",DRV_MODULE_NAME);
		err = -1;
		goto err_class;
	}

	for (i=0; i < MAX_EXP_DMA_DEVICES; i++) {
		devlist[i].availability = DEV_CTXT_FREE;
		devlist[i].pdev = NULL;
		devlist[i].id = i;
	}

	err = pci_register_driver(&expresso_dma_driver);

err_class:
	return err;
}

/**
 * exp_pci_exit - Driver exit function
 *
 * Return: void
 */

static void __exit exp_pci_exit(void) {
	printk(KERN_ERR"%s exit()\n",DRV_MODULE_NAME);

	pci_unregister_driver(&expresso_dma_driver);

	if (g_expresso_dma_class)
		class_destroy(g_expresso_dma_class);
}

module_init(exp_pci_init);
module_exit(exp_pci_exit);
