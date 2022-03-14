export KDIR= /lib/modules/$(shell uname -r)/build
export CC = $(CROSS_COMPILE)gcc 
export PS_PCIE_DMA_PATH=$(ROOTDIR)/driver
export APP_PATH=$(ROOTDIR)/apps
export INSMOD=/sbin/insmod
export RMMOD=/sbin/rmmod
export DMA_DRIVER_NAME=ps_pcie_dma.ko
export SLEEP_TIME=1

define compile_dma_driver
	echo Compiling PS PCIe DMA Driver
	$(MAKE) -C $(PS_PCIE_DMA_PATH)
endef

define compile_test_application
	echo Compiling Test Applications
	$(MAKE) -C $(APP_PATH)
endef

define insert_dma_driver
	echo Inserting PS PCIe  DMA Driver
	$(INSMOD) $(PS_PCIE_DMA_PATH)/$(DMA_DRIVER_NAME); sleep $(SLEEP_TIME)
endef

define clean_dma_driver
	echo Cleaning PS PCIe  DMA Driver
	$(MAKE) -C $(PS_PCIE_DMA_PATH) clean
endef

define clean_test_application
	echo Cleaning Test Applications
	$(MAKE) -C $(APP_PATH) clean
endef

define remove_dma_driver
	echo Removing PS PCIe DMA Driver
	$(RMMOD) $(DMA_DRIVER_NAME); sleep $(SLEEP_TIME)
endef
