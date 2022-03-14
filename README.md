###############################################################################

                 Xilinx ZynqMP PS-PCIe End Point DMA Software README

###############################################################################

The files in this directory provide Xilinx ZynqMP PS-PCIe End Point DMA drivers,and test application for testing  DMA Transfers and Programmable Input Output functionality .

Operating System Support:
=========================
Operating System Architecture: x86_64
Ubuntu:		Ubuntu 20.04.3 LTS

HW Requirement:
===============
ZynqMP PS-PCIe End Point devices.


Directory and file description:
===============================
 - driver/: This directory contains the ZynqMP PS-PCIe End Point DMA kernel module
       driver files.Source code in this directory is licensed under GNU General Public License.

 - common/: This directory contains  include file that is needed for
	compiling driver.Source code in this directory is licensed under BSD-style license and the GPL license (found in the COPYING file in the root directory of this source tree).
        You may select, at your option, one of the above-listed licenses.

 - apps/: This directory contains example application software  files to test the
	provided kernel module driver for DMA Transfers and Programmable Input Output functionality .
         Source code in this directory is licensed under BSD-style license.
	

	 - pci_pio_test.c :
		 This application enable the user to test the Programmable Input Output functionality .
	
	 - sync_test.c :
	 This application enable the user to test Synchronous DMA Transfer functionality. 
	 
 - LICENSE : This file contains BSD license.																	

 - COPYING : This file contains GNU GPL license.			 

Usage:																				

		
 - Clone the directory on ubuntu machine to which ZynqMP PS-PCIe End Point card is connected.												      
			git clone git@github.com:Xilinx/zynqmp-pspcie-epdma.git
	
 - Change directory to the cloned directory.																	
 	
	cd zynqmp_ep_ps_pcie_dma
 - Compile  kernel module driver and application.																

	make
 - install the kernel module driver.																		

	make install
 - Test App - DMA Transfers Application 

        ./simple_test -c 0 -a 0x100000 -l 1024 -d s2c
		./simple_test -c 1 -a 0x100000 -l 1024 -d c2s

		-c option specifies channel number.
		-a option specifies end point address.
		-l option specifies packet length.
		-d option specifies transfer direction. It can be either s2c or c2s.

- Test App - Programmable Input Output
		./pio_test -o 0x0 -l 64	

		-o option specifies offset at PCI BAR 2.
		-l option specifies length of data to be written and read back.		

Frequently Asked Questions                                                                                                                                                             

	Q. How do we unistall kernel module driver ? 																
	A: Use the following commands to uninstall the driver.																
		- Uninstall the kernel module.																		
			make remove  																			
	Q: How do I modify the PCIe Device IDs recognized by the kernel module driver?												
	A: The driver/ps_pcie_dma.c file constains the pci_device_id struct that identifies												
	   the PCIe Device IDs that are recognized by the driver in the following format:												
	   { PCI_DEVICE(<VENDOR ID> , <DEVICE ID>), },																	
	   Add, remove, or modify the PCIe Device IDs in this struct as desired. 													
	   Then uninstall the existing kernel module, compile the driver again, and re-install the driver.
