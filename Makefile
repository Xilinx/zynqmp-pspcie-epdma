#
#	Makefile for Xilinx PS PCIe Endpoint DMA Driver 
#

export ROOTDIR=$(shell pwd)
include $(ROOTDIR)/include.mk

all::
	@$(call compile_dma_driver)
		@echo "***** Driver Compiled *****"
	@$(call compile_test_application)
		@echo "***** Applications Compiled *****"

driver::
	@$(call compile_dma_driver)
		@echo "***** Only Driver is Compiled *****"

app::
	@$(call compile_test_application)
		@echo "***** Only Applications are Compiled *****"

clean::
	@$(call clean_dma_driver)
		@echo "***** Driver Cleaned *****"
	@$(call clean_test_application)
		@echo "***** Applications Cleaned *****"

clean_driver::
	@$(call clean_dma_driver)
		@echo "***** Only Driver is Cleaned *****"

clean_apps::
	@$(call clean_test_application)
		@echo "***** Only Applications are Cleaned *****"

insert:: 
	@$(call insert_dma_driver)
		@echo "***** Driver Loaded *****"

remove::
	@$(call remove_dma_driver)
		@echo "***** Driver Unloaded *****"
