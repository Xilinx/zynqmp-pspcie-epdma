CFLAGS += -O3 -Os -I $(PWD)
LFLAGS += -lpthread

SYNC_TEST_OBJS = sync_test.o
PIO_TEST_OBJS = pci_pio_test.o

all: SYNC_TEST PIO_TEST

SYNC_TEST:	$(SYNC_TEST_OBJS) 
		$(CC) $(CFLAGS) $(SYNC_TEST_OBJS) $(LFLAGS) -o simple_test

PIO_TEST:	$(PIO_TEST_OBJS) 
		$(CC) $(CFLAGS) $(PIO_TEST_OBJS) $(LFLAGS) -o pio_test

sync_test.o:	sync_test.c
		$(CC) -c $(CFLAGS) -I ../common/ sync_test.c -o sync_test.o

pci_pio_test.o:	pci_pio_test.c
		$(CC) -c $(CFLAGS) -I ../common/ pci_pio_test.c -o pci_pio_test.o


clean:
	rm -f $(SYNC_TEST_OBJS) simple_test $(PIO_TEST_OBJS) pio_test
