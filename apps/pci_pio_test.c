/*
 * This file is part of the XILINX ZynqMP PS-PCIe End Point userspace application
 * to enable the user to execute the Programmable Input Output  functionality.
 *
 * Copyright (c) 2022-2023,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include "common_include.h"

enum errors {
	BUSY = 0,
	OPEN,
	MALLOC_RD_WR,
	MALLOC_VERIF,
	READ,
	IOCTL,
	WRITE,
};

static void error(int err);

static void usage(void)
{
        printf("Offset, packet length\n");
        printf( "Offset      :Address on EP to do DMA\n"
                "length      :Number of bytes to be transferred\n"
                "             This number has to be choosen according to translation size set\n");

        printf ("Example Usage :./pio_test -o 0x0 -l 64\n");
        exit(EXIT_SUCCESS);
}

static void error(int err)
{
	switch(err) {
	case OPEN:
		printf("Failed to open zynqmp_pci dev file\n");
		break;

	case MALLOC_RD_WR:
		printf("Allocating memory for read/write buffer failed\n");
		break;

	case MALLOC_VERIF:
		printf("Allocating memory for verif buffer failed\n");
		break;

	case READ:
		printf("Failed to read from the EP\n");
		break;

	case IOCTL:
		printf("Ingress translation no set on EP\n");
		break;
	case WRITE:
		printf("Failed to write to the EP\n");
		break;
	}

	exit(1);
}

int main (int argc, char *argv[])
{
	int            err, i, pci_fd;
	off_t          offset;
	ssize_t        rd_size, wr_size;
	u_int32_t      data_size;
	u_int32_t     *rd_wr_buf, *verif_buf;
	int opt;
	unsigned char pio_dev_path[50];
	
	while ((opt = getopt(argc, argv, "l:o:h")) != -1) {
                switch(opt){
                case 'h' :
                        usage();
                        break;
                case 'o' :
                        offset = strtoul(optarg, NULL, 16);;
                        if ((offset < 0) || (offset > 0x80000000)) {
                                printf("Enter valid address greater than zero and lesser than 2GB\n");
                                return -1;
                        }
                        break;
                case 'l' :
                        data_size = atoi(optarg);
                        if (data_size < 1) {
                                printf("Packet length cannot be negative or zero\n");
                                return -1;
                        }
                        break;
                default :
                        printf("Enter valid option\n");
                        usage();
                }
        }

	if (argc < 5) {
                printf("Error: Give exactly 9 arguments\n");
                usage();
                return -1;
        }

	memset (pio_dev_path, '\0', 50);
	sprintf (pio_dev_path,"/dev/%s_0",PIO_CHAR_DRIVER_NAME);

	pci_fd = open(pio_dev_path, O_RDWR);

	if (pci_fd == -1) {
		error(OPEN);
	}
        err = ioctl(pci_fd, IOCTL_EP_CHECK_TRANSLATION);
        if (err != 0) {
                error(IOCTL);
	}

	rd_wr_buf = malloc(data_size);

	if (!rd_wr_buf) {
		error(MALLOC_RD_WR);
	}

	verif_buf = malloc(data_size);

	if (!verif_buf) {
		error(MALLOC_VERIF);
	}

	/* Fill the rd/wr buffer with random data */
	srand(time(NULL));
	for (i = 0; i < (data_size / 4); i++) {
		rd_wr_buf[i] = rand();
		verif_buf[i] = rd_wr_buf[i];
	}

	/* Write the data into the EP */
	wr_size  = pwrite(pci_fd, rd_wr_buf, data_size, offset);

	if (wr_size == -1) {
		error(WRITE);
	}

	if (wr_size != data_size) {
		printf("Unable to Write the entire test payload data to the PCIe EP\n");
		printf("payload size => 0x%x, payload written => 0x%x\n",
		       (uint32_t)data_size, (uint32_t)wr_size);
		free(rd_wr_buf);
		free(verif_buf);
		close(pci_fd);
		exit(1);
	}


	/* Clear out the buffer */
	memset(rd_wr_buf, 0, data_size);

	/* Read back the data written earlier from the EP */
	rd_size = pread(pci_fd, rd_wr_buf, data_size, offset);

	if (rd_size == -1) {
		error(READ);
	}

	if (rd_size != data_size) {
		printf("Unable to Read the entire test payload data from the PCIe EP\n");
		printf("payload size => 0x%x, payload read => 0x%x\n",
		       (uint32_t)data_size, (uint32_t)rd_size);
		free(rd_wr_buf);
		free(verif_buf);
		close(pci_fd);
		exit(1);
	}

	for (i = 0; i < (data_size / 4); i++) {
		if (rd_wr_buf[i] != verif_buf[i]) {
			printf("Data Mismatch!\n");
			printf("Data Read => 0x%x \t Data Expected => 0x%x at location %d\n",
			       rd_wr_buf[i], verif_buf[i], i);
		break;
		}
	}

	printf("PIO test done!\n");

	free(rd_wr_buf);
	free(verif_buf);
	close(pci_fd);
	return 0;
}
