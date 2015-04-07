/*
 * Hidraw Userspace Example
 *
 * Copyright (c) 2010 Alan Ott <alan@signal11.us>
 * Copyright (c) 2010 Signal 11 Software
 *
 * The code may be used by anyone for any purpose,
 * and can serve as a starting point for developing
 * applications using hidraw.
 */

/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 *
 * If you need this, please have your distro update the kernel headers.
 */
#ifndef HIDIOCSFEATURE
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <fcntl.h>
#include <unistd.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <pthread.h>
#include <stdint.h>
#include <errno.h>

const char *bus_str(int bus);

struct rtk_update_hid {
	int write_fd;
	int read_fd;
	int keep_polling;
	int rtkbt_thread_id;
	int write_idx;
	int is_updating;
};
struct rtk_update_hid update_info;
char write_buf[128];

void write_start() {
	perror("***start to update hid device ***");
	if(update_info.is_updating == 1) {
		perror("update_info.is_updating == true!");
		return;
	}
		
	update_info.write_idx = 0;
	printf("***start to write %d ***", update_info.write_idx);
	memset(write_buf, update_info.write_idx , sizeof(write_buf));
	write(update_info.write_fd, write_buf, sizeof(write_buf));
}

void write_ack() {
	if(update_info.is_updating == 1) {
		printf(" %d is ack!", update_info.write_idx);
		update_info.write_idx++;
		printf("***start to write %d ***", update_info.write_idx);
		memset(write_buf, update_info.write_idx , sizeof(write_buf));
		write(update_info.write_fd, write_buf, sizeof(write_buf));
		return;
	} else
		perror("update_info.is_updating != true!");
}

void write_end() {
	if(update_info.is_updating == 1) {
		perror("***update hid device success!***");
		update_info.is_updating = 0;
	}
}

static int read_event() {
	char buf[18];
	int read_result = 0;
	perror("start to read data from /dev/hidraw0");
	read_result = read(update_info.read_fd, buf, sizeof(buf));
	if(read_result > 0) {
		perror("read ok!");
		if(buf[0] == 0x07){
			if(buf[1] == 0x15) 
				write_start();
			else if(buf[1] == 0x06)
				write_ack();
			else if(buf == 0x18)
				write_end();
		}
	} else {
		perror("read fail!");
		update_info.keep_polling = 0;
	}

	return read_result;
}

static void *poll_event_thread(void *arg)
{

    struct pollfd pfds[1];
    int ret;
    pfds[0].fd = update_info.read_fd;
    pfds[0].events = POLLIN;

    while(update_info.keep_polling == 1){
        ret = poll(pfds, 1, 500);
        if (ret < 0) {
            printf("%s: Cannot poll for fds: %s\n", __FUNCTION__, strerror(errno));
            break;
        }
        if (pfds[0].revents & POLLIN) {
            printf("btif_hh_poll_event_thread: POLLIN");
            ret = read_event();
            if (ret){
                break;
            }
        }
    }

    update_info.rtkbt_thread_id = -1;
    return 0;
}

static inline pthread_t rtkbt_create_thread(void *(*start_routine)(void *), void * arg){
    printf("create_thread: entered");
    pthread_attr_t thread_attr;

    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_JOINABLE);
    pthread_t thread_id = -1;
    if ( pthread_create(&thread_id, &thread_attr, start_routine, arg)!=0 )
    {
        printf("pthread_create : %s", strerror(errno));
        return -1;
    }
    printf("create_thread: thread created successfully");
    return thread_id;
}

int main(int argc, char **argv)
{
	int fd;
	int i, res, desc_size = 0, count = 1;
	char buf[256];
	struct hidraw_report_descriptor rpt_desc;
	struct hidraw_devinfo info;
	memset(&rpt_desc, 0x0, sizeof(rpt_desc));
	memset(&info, 0x0, sizeof(info));
	memset(buf, 0x0, sizeof(buf));
	
	memset(&update_info, 0x0, sizeof(update_info));
	/* Open the Device with non-blocking reads. In real life,
	   don't use a hard coded path; use libudev instead. */
	fd = open("/dev/hidraw0", O_RDWR);

	if (fd < 0) {
		perror("Unable to open /dev/hidraw0");
		return 1;
	} else {
		perror("open /dev/hidraw0 success!");
		/* Get Raw Info */
		res = ioctl(fd, HIDIOCGRAWINFO, &info);
		if (res < 0) {
			perror("HIDIOCGRAWINFO");
		} else {
			printf("Raw Info:\n");
			printf("\tbustype: %d (%s)\n",
				info.bustype, bus_str(info.bustype));
			printf("\tvendor: 0x%04hx\n", info.vendor);
			printf("\tproduct: 0x%04hx\n", info.product);
		}
		update_info.read_fd = fd;
	}

	fd = open("/dev/hidraw1", O_RDWR);

	if (fd < 0) {
		perror("Unable to open /dev/hidraw1");
		return 1;
	} else {
		perror("open /dev/hidraw1 success!");
		/* Get Raw Info */
		res = ioctl(fd, HIDIOCGRAWINFO, &info);
		if (res < 0) {
			perror("HIDIOCGRAWINFO");
		} else {
			printf("Raw Info:\n");
			printf("\tbustype: %d (%s)\n",
				info.bustype, bus_str(info.bustype));
			printf("\tvendor: 0x%04hx\n", info.vendor);
			printf("\tproduct: 0x%04hx\n", info.product);
		}
		update_info.write_fd = fd;
	}

	update_info.rtkbt_thread_id = rtkbt_create_thread(poll_event_thread, NULL);
	return 0;
}

const char *
bus_str(int bus)
{
	switch (bus) {
	case BUS_USB:
		return "USB";
		break;
	case BUS_HIL:
		return "HIL";
		break;
	case BUS_BLUETOOTH:
		return "Bluetooth";
		break;
	case BUS_VIRTUAL:
		return "Virtual";
		break;
	default:
		return "Other";
		break;
	}
}


