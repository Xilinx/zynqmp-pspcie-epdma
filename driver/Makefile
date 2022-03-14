KBUILD_EXTRA_SYMBOLS += $(KDIR)/Module.symvers
EXTRA_CFLAGS += -O3 -Os

TARGET = ps_pcie_dma
obj-m   += $(TARGET).o 

all:
	$(MAKE) $(CFLAGS) -C $(KDIR) M=$(PWD)/driver  SUBDIRS=$(PWD/driver/sources) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD)/driver clean
