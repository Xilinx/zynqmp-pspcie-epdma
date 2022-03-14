/*
 * This file is part of the XILINX ZynqMP PS-PCIe End Point userspace application
 * to enable the user to execute the Synchronous DMA Transfer functionality.
 *
 * Copyright (c) 2022-2023,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <pthread.h>
#include <sys/time.h>

#include "common_include.h"

static void usage(void)
{
        printf("Channel, EP Address, packet length, Direction\n");
        printf("Channel          :Channel to use\n"
                "                 Allowed value = 0, 1, 2, 3\n"
                "EP Address      :Address on EP to do DMA\n"
                "Packet length   :Number of bytes to be transferred\n"
		"		  Allowed range 64 to 4 Mega bytes"
                "Direction       :Direction of DMA transfer\n"
                "                 Allowed value: s2c, c2s\n");

        printf ("Example Usage for Transmitting:./simple_test -c 0 -a 0x100000 -l 1024 -d s2c\n");
        printf ("Example Usage for Receiving:./simple_test -c 1 -a 0x200000 -l 1024 -d c2s\n");
        exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
	unsigned char *txbuffer = NULL;
	unsigned char *rxbuffer = NULL;
	unsigned char dma_path[50];
	unsigned char dir[5] = {'\0'};
	int opt;
	long offset;	
	int chan_no;
	int packet_len;	
	int datafd;
	int ret, i, j, k;
	char c;
	struct timeval before, after;
	size_t compute_time_in_usecs;

	while ((opt = getopt(argc, argv, "c:l:a:d:h")) != -1) {
                switch(opt){
                case 'h' :
                        usage();
                        break;
		case 'c' :
			chan_no = atoi(optarg);
			if ((chan_no < 0) || (chan_no > (MAX_NUMBER_OF_CHANNELS - 1)))
			{
				printf("Channel number should be within 0 to %d range\n",MAX_NUMBER_OF_CHANNELS - 1);
				return -1;
			}
			memset (dma_path, '\0', 50);
			sprintf (dma_path, "/dev/%s%d_0",CHAR_DRIVER_NAME,chan_no);
			break;
		case 'a' :
			offset = strtoul(optarg, NULL, 16);;
			if ((offset < 0) || (offset > 0x80000000)) {
				printf("Enter valid address greater than zero and lesser than 2GB\n");
				return -1;
			}
			break; 
		case 'l' :
			packet_len = atoi(optarg);
			if (packet_len < 1) {
				printf("Packet length cannot be negative or zero\n");
				return -1;
			}
			else if ((packet_len < 64) || (packet_len >4194304)) {
				printf("Packet length cannot be less than 64 bytes or greater than 4 Mega Bytes\n");
				return -1;
			}
			break;
		case 'd' :
			if ((strncasecmp(optarg, "s2c", 3) != 0) && (strncasecmp(optarg, "c2s", 3) != 0)) {
				printf("Please enter correct direction of transfer\n");
				return -1;
			}		
			strncpy (dir, optarg, 3);
			break;
                default :
			printf("Enter valid option\n");
                        usage();
                }
	}
       	if (argc < 9) {
		printf("Error: Give exactly 9 arguments\n");
		usage();	
		return -1;
	}

	datafd = open(dma_path, O_RDWR);
	if (datafd < 0)
	{
		perror("failed to open data file\n");
                exit(-1);
	}

	if (strncasecmp(dir, "s2c", 3) == 0)
	{
		txbuffer =(unsigned char *) valloc(packet_len);
		if(!txbuffer)
		{
			printf("Unable to allocate tx buffer\n");
			close(datafd);
			exit(-1);
		}
		memset((unsigned char *)txbuffer, 'Z', packet_len);
		gettimeofday(&before,NULL);
		ret = pwrite (datafd, txbuffer, packet_len, offset);
		gettimeofday(&after,NULL);
		printf("write return value is %d\n", ret);
		if(ret != packet_len) {
			printf("pwrite unsuccessful\n");
		}
	} 
	else if (strncasecmp(dir, "c2s", 3) == 0) 
	{	
		rxbuffer = (unsigned char *) valloc(packet_len);
		if(!rxbuffer)
		{
			printf("Unable to allocate rx buffer\n");
			free(txbuffer);
			close(datafd);
		}
		memset((unsigned char*)rxbuffer, '\0', packet_len);
		gettimeofday(&before,NULL);
		ret = pread (datafd, rxbuffer, packet_len, offset);	
		gettimeofday(&after,NULL);
		printf("read return value is %d\n", ret);
		if(ret == packet_len) {
		printf("Received data is :: ");	
		printf(" Data :: First Byte %x \t Last Byte %x\n",rxbuffer[0], rxbuffer[packet_len-1]);
		#ifdef PRINT_FULL_RECEIVED_PACKET
		for(i = 0; i < packet_len; i++ )
		{
			printf(" Data[%d] :: %x ",i, rxbuffer[i]);
		}
		printf("\n");
		#endif
		} else
			printf("pread unsuccessful\n");
	}

	if(after.tv_sec == before.tv_sec)
		compute_time_in_usecs = after.tv_usec - before.tv_usec;
	else
		compute_time_in_usecs	= after.tv_usec - before.tv_usec + 1000000;

	printf("Total time taken for transferring %d bytes of data is %ld micro seconds\n",packet_len,compute_time_in_usecs);


	if (txbuffer != NULL) 	 
		free(txbuffer);
	if (rxbuffer != NULL) 	 
		free(rxbuffer);
	close(datafd);
	return 0;
}
