/*
 * This file is part of the XILINX ZynqMP PS-PCIe End Point userspace application
 * and driver.
 *
 * Copyright (c) 2022-2023,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree) and GNU General Public License(found
 * in the COPYING file in the root directory of this source tree).
 * You may select, at your option, one of the above-listed licenses.
 */

#ifndef _XLNX_PCIE_DMA_COMMON_H_
#define _XLNX_PCIE_DMA_COMMON_H_


#define MAX_NUMBER_OF_CHANNELS   4

#define MAX_ALLOWED_CHANNELS_IN_HW   4

#if MAX_NUMBER_OF_CHANNELS > MAX_ALLOWED_CHANNELS_IN_HW
#error "Please reduce number of DMA engines defined in MAX_NUMBER_OF_CHANNELS macro"
#endif

/* Each char device is associated to 1 channel.
 * write call of char device services Host to Card transfers for the channel
 * read call of char device services Card to Host transfer for the channel
 * */
#define MAX_NUMBER_OF_CHAR_DEVICES     MAX_NUMBER_OF_CHANNELS

/* This macro defines the number of Pcie devices that can be
 * supported by this driver when they are inserted parallelly
 * in different slots of the host machine
 */
#define MAX_EXP_DMA_DEVICES            3

#define CHAR_DRIVER_NAME               "ps_pcie_dmachan"

#define PIO_CHAR_DRIVER_NAME           "ps_pcie_pio"

#define EP_TRANSLATION_CHECK            0xCCCCCCCC
#define IOCTL_EP_CHECK_TRANSLATION     _IO('z', 0x01)

#define NUMBER_OF_BUFFER_DESCRIPTORS   1999

#define CHANNEL_COAELSE_COUNT          0

#define CHANNEL_POLL_TIMER_FREQUENCY   (HZ) //Lower value indicates frequent checks and greater processing.

#endif
