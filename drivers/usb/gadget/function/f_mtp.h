/*
 * Copyright (C) 2014 Google, Inc.
 * Author: Badhri Jagan Sridharan <badhri@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */

#include <linux/switch.h>

#define RX_REQ_MAX 2
#define MAX_ITERATION		100

struct mtp_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;
	struct usb_ep *ep_intr;

	int state;

	/* synchronize access to our device file */
	atomic_t open_excl;
	/* to enforce only one ioctl at a time */
	atomic_t ioctl_excl;

	struct list_head tx_idle;
	struct list_head intr_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	wait_queue_head_t intr_wq;
	struct usb_request *rx_req[RX_REQ_MAX];
	int rx_done;

	/* for processing MTP_SEND_FILE, MTP_RECEIVE_FILE and
	 * MTP_SEND_FILE_WITH_HEADER ioctls on a work queue
	 */
	struct workqueue_struct *wq;
	struct work_struct send_file_work;
	struct work_struct receive_file_work;
	struct work_struct vendor_req_work;
	struct file *xfer_file;
	loff_t xfer_file_offset;
	int64_t xfer_file_length;
	unsigned xfer_send_header;
	uint16_t xfer_command;
	uint32_t xfer_transaction_id;
	int xfer_result;
	struct {
		unsigned long vfs_rbytes;
		unsigned long vfs_wbytes;
		unsigned int vfs_rtime;
		unsigned int vfs_wtime;
	} perf[MAX_ITERATION];
	unsigned int dbg_read_index;
	unsigned int dbg_write_index;
	unsigned int mtp_rx_req_len;
	unsigned int mtp_tx_req_len;
	unsigned int mtp_tx_reqs;
	struct mutex  read_mutex;

	/* vendor_req (1 char), vendor_data (20 chars) */
	unsigned char vendor_req[1 + 20];
	struct switch_dev *sdev;
	uint32_t vendor_req_no;
};

struct mtp_instance {
	struct usb_function_instance func_inst;
	const char *name;
	struct mtp_dev *dev;
	char mtp_ext_compat_id[16];
	struct usb_os_desc mtp_os_desc;
};

extern struct usb_function_instance *alloc_inst_mtp_ptp(bool mtp_config);
extern struct usb_function *function_alloc_mtp_ptp(
			struct usb_function_instance *fi, bool mtp_config);

int mtp_set_vendor_req(const char *buf);
