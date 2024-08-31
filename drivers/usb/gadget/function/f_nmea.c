/*
 * f_nmea.c - USB nmea function driver
 *
 * This code also borrows from drivers/usb/gadget/u_serial.c, which is
 * Copyright (C) 2000 - 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 * Copyright (C) 1999 - 2002 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2000 Peter Berger (pberger@brimson.com)
 */

/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2018 KYOCERA Corporation
 */


#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/usb/gadget.h>
#include <linux/usb/cdc.h>
#include <linux/usb/composite.h>
#include <linux/module.h>
#include <asm/ioctls.h>
#include <asm-generic/termios.h>

#define DEVICE_NAME "at_nmea_usb"
#define MODULE_NAME "msm_nmea_usb_bridge"
#define NUM_INSTANCE 3

#define MAX_CDEV_INST_NAME	15
#define MAX_CDEV_FUNC_NAME	5

#define BRIDGE_RX_QUEUE_SIZE	8
#define BRIDGE_RX_BUF_SIZE	2048
#define BRIDGE_TX_QUEUE_SIZE	8
#define BRIDGE_TX_BUF_SIZE	2048

#define GS_LOG2_NOTIFY_INTERVAL		5  /* 1 << 5 == 32 msec */
#define GS_NOTIFY_MAXPACKET		10 /* notification + 2 bytes */

struct cserial {
	struct usb_function		func;
	struct usb_ep			*in;
	struct usb_ep			*out;
	struct usb_ep			*notify;
	struct usb_request		*notify_req;
	struct usb_cdc_line_coding	port_line_coding;
	u8				pending;
	u8				q_again;
	u8				data_id;
	u16				serial_state;
	u16				port_handshake_bits;
	/* control signal callbacks*/
	unsigned int (*get_dtr)(struct cserial *p);
	unsigned int (*get_rts)(struct cserial *p);

	/* notification callbacks */
	void (*connect)(struct cserial *p);
	void (*disconnect)(struct cserial *p);
	int (*send_break)(struct cserial *p, int duration);
	unsigned int (*send_carrier_detect)(struct cserial *p, unsigned int);
	unsigned int (*send_ring_indicator)(struct cserial *p, unsigned int);
	int (*send_modem_ctrl_bits)(struct cserial *p, int ctrl_bits);

	/* notification changes to modem */
	void (*notify_modem)(void *port, int ctrl_bits);
};

struct f_nmea {
	struct cdev		fnmea_cdev;
	struct device		*dev;
	unsigned		port_num;
	char			name[sizeof(DEVICE_NAME) + 2];
	int			minor;

	spinlock_t		port_lock;

	wait_queue_head_t	open_wq;
	wait_queue_head_t	read_wq;

	struct list_head	read_pool;
	struct list_head	read_queued;
	struct list_head	write_pool;

	/* current active USB RX request */
	struct usb_request	*current_rx_req;
	/* number of pending bytes */
	size_t			pending_rx_bytes;
	/* current USB RX buffer */
	u8			*current_rx_buf;

	struct cserial		port_usb;

#define ACM_CTRL_DTR		0x01
#define ACM_CTRL_RTS		0x02
#define ACM_CTRL_DCD		0x01
#define ACM_CTRL_DSR		0x02
#define ACM_CTRL_BRK		0x04
#define ACM_CTRL_RI		0x08

	unsigned		cbits_to_modem;
	bool			cbits_updated;

	struct workqueue_struct *fnmea_wq;
	bool			is_connected;
	bool			port_open;

	unsigned long           nbytes_from_host;
	unsigned long		nbytes_to_host;
	unsigned long           nbytes_to_port_bridge;
	unsigned long		nbytes_from_port_bridge;
};

struct f_nmea_opts {
	struct usb_function_instance func_inst;
	struct f_nmea *port;
	char *func_name;
	u8 port_num;
};

static int major, minors;
struct class *fnmea_classp;
static DEFINE_IDA(chardev_ida);
static DEFINE_MUTEX(chardev_ida_lock);

static int usb_nmea_alloc_chardev_region(void);
static void usb_nmea_chardev_deinit(void);
static void usb_nmea_read_complete(struct usb_ep *ep, struct usb_request *req);
static int usb_nmea_connect(struct f_nmea *port);
static void usb_nmea_disconnect(struct f_nmea *port);
static struct f_nmea *f_nmea_alloc(char *func_name, int portno);
static void usb_nmea_free_req(struct usb_ep *ep, struct usb_request *req);

static struct usb_interface_descriptor nmea_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
#ifdef CONFIG_KC_USB_US
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceProtocol =	USB_CLASS_VENDOR_SPEC,
#else
	.bInterfaceClass =	0x02,
	.bInterfaceSubClass =	0x0A,
	.bInterfaceProtocol =	0x01,
#endif
	/* .iInterface = DYNAMIC */
};

#ifndef CONFIG_KC_USB_US
static struct usb_cdc_header_desc nmea_header_desc  = {
	.bLength =		sizeof(nmea_header_desc),
	.bDescriptorType =	0x24,
	.bDescriptorSubType =	0x00,
	.bcdCDC =		__constant_cpu_to_le16(0x0110),
};

static struct usb_cdc_mdlm_desc nmea_mdlm_desc = {
	.bLength = sizeof(struct usb_cdc_mdlm_desc),
	.bDescriptorType = 0x24,
	.bDescriptorSubType = 0x12,
	.bcdVersion = __constant_cpu_to_le16(0x0100),
	.bGUID[0] = 0xC2,
	.bGUID[1] = 0x29,
	.bGUID[2] = 0x9F,
	.bGUID[3] = 0xCC,
	.bGUID[4] = 0xD4,
	.bGUID[5] = 0x89,
	.bGUID[6] = 0x40,
	.bGUID[7] = 0x66,
	.bGUID[8] = 0x89,
	.bGUID[9] = 0x2B,
	.bGUID[10] = 0x10,
	.bGUID[11] = 0xC3,
	.bGUID[12] = 0x41,
	.bGUID[13] = 0xDD,
	.bGUID[14] = 0x98,
	.bGUID[15] = 0xA9,
};
#endif /* CONFIG_KC_USB_US */

/* full speed support: */
static struct usb_endpoint_descriptor nmea_fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
#ifndef CONFIG_KC_USB_US
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	.bInterval =		0,
#endif /* CONFIG_KC_USB_US */
};

static struct usb_endpoint_descriptor nmea_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
#ifndef CONFIG_KC_USB_US
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	.bInterval =		0,
#endif /* CONFIG_KC_USB_US */
};

static struct usb_descriptor_header *nmea_fs_function[] = {
	(struct usb_descriptor_header *) &nmea_interface_desc,
#ifndef CONFIG_KC_USB_US
	(struct usb_descriptor_header *) &nmea_header_desc,
	(struct usb_descriptor_header *) &nmea_mdlm_desc,
#endif /* CONFIG_KC_USB_US */
	(struct usb_descriptor_header *) &nmea_fs_in_desc,
	(struct usb_descriptor_header *) &nmea_fs_out_desc,
	NULL,
};

/* high speed support: */
static struct usb_endpoint_descriptor nmea_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
#ifndef CONFIG_KC_USB_US
	.bEndpointAddress =	USB_DIR_IN,
#endif /* CONFIG_KC_USB_US */
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
#ifndef CONFIG_KC_USB_US
	.bInterval =		0,
#endif /* CONFIG_KC_USB_US */
};

static struct usb_endpoint_descriptor nmea_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
#ifndef CONFIG_KC_USB_US
	.bEndpointAddress =	USB_DIR_OUT,
#endif /* CONFIG_KC_USB_US */
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
#ifndef CONFIG_KC_USB_US
	.bInterval =		0,
#endif /* CONFIG_KC_USB_US */
};

static struct usb_descriptor_header *nmea_hs_function[] = {
	(struct usb_descriptor_header *) &nmea_interface_desc,
#ifndef CONFIG_KC_USB_US
	(struct usb_descriptor_header *) &nmea_header_desc,
	(struct usb_descriptor_header *) &nmea_mdlm_desc,
#endif /* CONFIG_KC_USB_US */
	(struct usb_descriptor_header *) &nmea_hs_in_desc,
	(struct usb_descriptor_header *) &nmea_hs_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor nmea_ss_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor nmea_ss_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor nmea_ss_bulk_comp_desc = {
	.bLength =              sizeof(nmea_ss_bulk_comp_desc),
	.bDescriptorType =      USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *nmea_ss_function[] = {
	(struct usb_descriptor_header *) &nmea_interface_desc,
	(struct usb_descriptor_header *) &nmea_ss_in_desc,
	(struct usb_descriptor_header *) &nmea_ss_bulk_comp_desc,
	(struct usb_descriptor_header *) &nmea_ss_out_desc,
	(struct usb_descriptor_header *) &nmea_ss_bulk_comp_desc,
	NULL,
};

/* string descriptors: */
static struct usb_string nmea_string_defs[] = {
	[0].s = "NMEA Serial",
	{  } /* end of list */
};

static struct usb_gadget_strings nmea_string_table = {
	.language =	0x0409,	/* en-us */
	.strings =	nmea_string_defs,
};

static struct usb_gadget_strings *usb_nmea_strings[] = {
	&nmea_string_table,
	NULL,
};

static inline struct f_nmea *func_to_port(struct usb_function *f)
{
	return container_of(f, struct f_nmea, port_usb.func);
}

static unsigned int convert_acm_sigs_to_uart(unsigned acm_sig)
{
	unsigned int uart_sig = 0;

	acm_sig &= (ACM_CTRL_DTR | ACM_CTRL_RTS);
	if (acm_sig & ACM_CTRL_DTR)
		uart_sig |= TIOCM_DTR;

	if (acm_sig & ACM_CTRL_RTS)
		uart_sig |= TIOCM_RTS;

	return uart_sig;
}

static void usb_nmea_free_func(struct usb_function *f)
{
	/* Do nothing as nmea_alloc() doesn't alloc anything. */
}

static int usb_nmea_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_nmea *port = func_to_port(f);
	struct usb_composite_dev *cdev	= f->config->cdev;
	int rc = 0;

	if (port->port_usb.notify->driver_data) {
		dev_dbg(&cdev->gadget->dev,
			"reset port(%s)\n", port->name);
		usb_ep_disable(port->port_usb.notify);
	}

	if (!port->port_usb.notify->desc) {
		if (config_ep_by_speed(cdev->gadget, f,
				port->port_usb.notify)) {
			port->port_usb.notify->desc = NULL;
			return -EINVAL;
		}
	}

	rc = usb_ep_enable(port->port_usb.notify);
	if (rc) {
		dev_err(&cdev->gadget->dev, "can't enable %s, result %d\n",
				port->port_usb.notify->name, rc);
		return rc;
	}
	port->port_usb.notify->driver_data = port;

	if (port->port_usb.in->driver_data) {
		dev_dbg(&cdev->gadget->dev,
			"reset port(%s)\n", port->name);
		usb_nmea_disconnect(port);
	}
	if (!port->port_usb.in->desc || !port->port_usb.out->desc) {
		dev_dbg(&cdev->gadget->dev,
			"activate port(%s)\n", port->name);
		if (config_ep_by_speed(cdev->gadget, f, port->port_usb.in) ||
			config_ep_by_speed(cdev->gadget, f,
					port->port_usb.out)) {
			port->port_usb.in->desc = NULL;
			port->port_usb.out->desc = NULL;
			return -EINVAL;
		}
	}

	usb_nmea_connect(port);
	return rc;
}

static void usb_nmea_disable(struct usb_function *f)
{
	struct f_nmea	*port = func_to_port(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	dev_dbg(&cdev->gadget->dev,
		"port(%s) deactivated\n", port->name);

	usb_nmea_disconnect(port);
	usb_ep_disable(port->port_usb.notify);
	port->port_usb.notify->driver_data = NULL;
}

static void usb_nmea_free_req(struct usb_ep *ep, struct usb_request *req)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
		req = NULL;
	}
}

static void usb_nmea_free_requests(struct usb_ep *ep, struct list_head *head)
{
	struct usb_request	*req;

	while (!list_empty(head)) {
		req = list_entry(head->next, struct usb_request, list);
		list_del_init(&req->list);
		usb_nmea_free_req(ep, req);
	}
}

static struct usb_request *
usb_nmea_alloc_req(struct usb_ep *ep, unsigned len, gfp_t flags)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, flags);
	if (!req) {
		pr_err("usb alloc request failed\n");
		return 0;
	}

	req->length = len;
	req->buf = kmalloc(len, flags);
	if (!req->buf) {
		pr_err("request buf allocation failed\n");
		usb_ep_free_request(ep, req);
		return 0;
	}

	return req;
}

static int usb_nmea_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_nmea *port = func_to_port(f);
	int status;
	struct usb_ep *ep;

	if (nmea_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		nmea_string_defs[0].id = status;
	}

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	port->port_usb.data_id = status;
	nmea_interface_desc.bInterfaceNumber = status;

	status = -ENODEV;
	ep = usb_ep_autoconfig(cdev->gadget, &nmea_fs_in_desc);
	if (!ep)
		goto fail;
	port->port_usb.in = ep;
	ep->driver_data = cdev;

	ep = usb_ep_autoconfig(cdev->gadget, &nmea_fs_out_desc);
	if (!ep)
		goto fail;
	port->port_usb.out = ep;
	ep->driver_data = cdev;

	port->port_usb.notify = ep;
	ep->driver_data = cdev;
	/* allocate notification */
	port->port_usb.notify_req = usb_nmea_alloc_req(ep,
			sizeof(struct usb_cdc_notification) + 2, GFP_KERNEL);
	if (!port->port_usb.notify_req)
		goto fail;

	port->port_usb.notify_req->context = port;

	nmea_hs_in_desc.bEndpointAddress = nmea_fs_in_desc.bEndpointAddress;
	nmea_hs_out_desc.bEndpointAddress = nmea_fs_out_desc.bEndpointAddress;

	nmea_ss_in_desc.bEndpointAddress = nmea_fs_in_desc.bEndpointAddress;
	nmea_ss_out_desc.bEndpointAddress = nmea_fs_out_desc.bEndpointAddress;

	status = usb_assign_descriptors(f, nmea_fs_function, nmea_hs_function,
			nmea_ss_function, NULL);
	if (status)
		goto fail;

	dev_dbg(&cdev->gadget->dev, "usb serial port(%d): %s speed IN/%s OUT/%s\n",
		port->port_num,
		gadget_is_superspeed(c->cdev->gadget) ? "super" :
		gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
		port->port_usb.in->name, port->port_usb.out->name);
	return 0;

fail:
	if (port->port_usb.notify_req)
		usb_nmea_free_req(port->port_usb.notify,
				port->port_usb.notify_req);

	if (port->port_usb.notify)
		port->port_usb.notify->driver_data = NULL;
	if (port->port_usb.out)
		port->port_usb.out->driver_data = NULL;
	if (port->port_usb.in)
		port->port_usb.in->driver_data = NULL;

	pr_err("%s: can't bind, err %d\n", f->name, status);
	return status;
}

static void nmea_free_inst(struct usb_function_instance *fi)
{
	struct f_nmea_opts *opts;

	opts = container_of(fi, struct f_nmea_opts, func_inst);

	if (opts->port) {
		device_destroy(fnmea_classp, MKDEV(major, opts->port->minor));
		cdev_del(&opts->port->fnmea_cdev);
	}
	usb_nmea_chardev_deinit();
	kfree(opts->func_name);
	kfree(opts->port);
	kfree(opts);
}

static void usb_nmea_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_nmea *port = func_to_port(f);

	usb_free_all_descriptors(f);
	usb_nmea_free_req(port->port_usb.notify, port->port_usb.notify_req);
}

static int usb_nmea_alloc_requests(struct usb_ep *ep, struct list_head *head,
		int num, int size,
		void (*cb)(struct usb_ep *ep, struct usb_request *))
{
	int i;
	struct usb_request *req;

	pr_debug("ep:%pK head:%pK num:%d size:%d cb:%pK",
				ep, head, num, size, cb);

	for (i = 0; i < num; i++) {
		req = usb_nmea_alloc_req(ep, size, GFP_ATOMIC);
		if (!req) {
			pr_debug("req allocated:%d\n", i);
			return list_empty(head) ? -ENOMEM : 0;
		}
		req->complete = cb;
		list_add_tail(&req->list, head);
	}

	return 0;
}

static void usb_nmea_start_rx(struct f_nmea *port)
{
	struct list_head	*pool;
	struct usb_ep		*ep;
	unsigned long		flags;
	int ret;

	pr_debug("start RX(USB OUT)\n");
	if (!port) {
		pr_err("port is null\n");
		return;
	}

	spin_lock_irqsave(&port->port_lock, flags);
	if (!(port->is_connected && port->port_open)) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		pr_debug("can't start rx.\n");
		return;
	}

	pool = &port->read_pool;
	ep = port->port_usb.out;

	while (!list_empty(pool)) {
		struct usb_request	*req;

		req = list_entry(pool->next, struct usb_request, list);
		list_del_init(&req->list);
		req->length = BRIDGE_RX_BUF_SIZE;
		req->complete = usb_nmea_read_complete;
		spin_unlock_irqrestore(&port->port_lock, flags);
		ret = usb_ep_queue(ep, req, GFP_KERNEL);
		spin_lock_irqsave(&port->port_lock, flags);
		if (ret) {
			pr_err("port(%d):%pK usb ep(%s) queue failed\n",
					port->port_num, port, ep->name);
			list_add(&req->list, pool);
			break;
		}
	}

	spin_unlock_irqrestore(&port->port_lock, flags);
}

static void usb_nmea_read_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_nmea *port = ep->driver_data;
	unsigned long flags;

	pr_debug("ep:(%pK)(%s) port:%pK req_status:%d req->actual:%u\n",
			ep, ep->name, port, req->status, req->actual);
	if (!port) {
		pr_err("port is null\n");
		return;
	}

	spin_lock_irqsave(&port->port_lock, flags);
	if (!port->port_open || req->status || !req->actual) {
		list_add_tail(&req->list, &port->read_pool);
		spin_unlock_irqrestore(&port->port_lock, flags);
		return;
	}

	port->nbytes_from_host += req->actual;
	list_add_tail(&req->list, &port->read_queued);
	spin_unlock_irqrestore(&port->port_lock, flags);

	wake_up(&port->read_wq);
	return;
}

static void usb_nmea_write_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned long flags;
	struct f_nmea *port = ep->driver_data;

	pr_debug("ep:(%pK)(%s) port:%pK req_stats:%d\n",
			ep, ep->name, port, req->status);

	if (!port) {
		pr_err("port is null\n");
		return;
	}

	spin_lock_irqsave(&port->port_lock, flags);
	port->nbytes_to_host += req->actual;
	list_add_tail(&req->list, &port->write_pool);
	spin_unlock_irqrestore(&port->port_lock, flags);

	switch (req->status) {
	default:
		pr_debug("unexpected %s status %d\n", ep->name, req->status);
		/* FALL THROUGH */
	case 0:
		/* normal completion */
		break;

	case -ESHUTDOWN:
		/* disconnect */
		pr_debug("%s shutdown\n", ep->name);
		break;
	}

	return;
}

static void usb_nmea_start_io(struct f_nmea *port)
{
	int ret = -ENODEV;
	unsigned long	flags;

	pr_debug("port: %pK\n", port);

	spin_lock_irqsave(&port->port_lock, flags);
	if (!port->is_connected)
		goto start_io_out;

	port->current_rx_req = NULL;
	port->pending_rx_bytes = 0;
	port->current_rx_buf = NULL;

	ret = usb_nmea_alloc_requests(port->port_usb.out,
				&port->read_pool,
				BRIDGE_RX_QUEUE_SIZE, BRIDGE_RX_BUF_SIZE,
				usb_nmea_read_complete);
	if (ret) {
		pr_err("unable to allocate out requests\n");
		goto start_io_out;
	}

	ret = usb_nmea_alloc_requests(port->port_usb.in,
				&port->write_pool,
				BRIDGE_TX_QUEUE_SIZE, BRIDGE_TX_BUF_SIZE,
				usb_nmea_write_complete);
	if (ret) {
		usb_nmea_free_requests(port->port_usb.out, &port->read_pool);
		pr_err("unable to allocate IN requests\n");
		goto start_io_out;
	}

start_io_out:
	spin_unlock_irqrestore(&port->port_lock, flags);
	if (ret)
		return;

	usb_nmea_start_rx(port);
}

static void usb_nmea_stop_io(struct f_nmea *port)
{
	struct usb_ep	*in;
	struct usb_ep	*out;
	unsigned long	flags;

	pr_debug("port:%pK\n", port);

	in = port->port_usb.in;
	out = port->port_usb.out;

	/* disable endpoints, aborting down any active I/O */
	usb_ep_disable(out);
	out->driver_data = NULL;
	usb_ep_disable(in);
	in->driver_data = NULL;

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->current_rx_req != NULL) {
		kfree(port->current_rx_req->buf);
		usb_ep_free_request(out, port->current_rx_req);
	}

	port->pending_rx_bytes = 0;
	port->current_rx_buf = NULL;
	usb_nmea_free_requests(out, &port->read_queued);
	usb_nmea_free_requests(out, &port->read_pool);
	usb_nmea_free_requests(in, &port->write_pool);
	spin_unlock_irqrestore(&port->port_lock, flags);
}

int f_nmea_open(struct inode *inode, struct file *file)
{
	int ret;
	unsigned long flags;
	struct f_nmea *port;

	port = container_of(inode->i_cdev, struct f_nmea, fnmea_cdev);
	if (!port) {
		pr_err("Port is NULL.\n");
		return -EINVAL;
	}

	if (port && port->port_open) {
		pr_err("port is already opened.\n");
		return -EBUSY;
	}

	file->private_data = port;
	pr_debug("opening port(%s)(%pK)\n", port->name, port);
	ret = wait_event_interruptible(port->open_wq,
					port->is_connected);
	if (ret) {
		pr_debug("open interrupted.\n");
		return ret;
	}

	spin_lock_irqsave(&port->port_lock, flags);
	port->port_open = true;
	spin_unlock_irqrestore(&port->port_lock, flags);
	usb_nmea_start_rx(port);

	pr_debug("port(%s)(%pK) open is success\n", port->name, port);

	return 0;
}

int f_nmea_release(struct inode *inode, struct file *file)
{
	unsigned long flags;
	struct f_nmea *port;

	port = file->private_data;
	if (!port) {
		pr_err("port is NULL.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&port->port_lock, flags);
	port->port_open = false;
	port->cbits_updated = false;
	spin_unlock_irqrestore(&port->port_lock, flags);
	pr_debug("port(%s)(%pK) is closed.\n", port->name, port);

	return 0;
}

ssize_t f_nmea_read(struct file *file,
		       char __user *buf,
		       size_t count,
		       loff_t *ppos)
{
	unsigned long flags;
	struct f_nmea *port;
	struct usb_request *req;
	struct list_head *pool;
	struct usb_request *current_rx_req;
	size_t pending_rx_bytes, bytes_copied = 0, size;
	u8 *current_rx_buf;

	port = file->private_data;
	if (!port) {
		pr_err("port is NULL.\n");
		return -EINVAL;
	}

	pr_debug("read on port(%s)(%pK) count:%zu\n", port->name, port, count);
	spin_lock_irqsave(&port->port_lock, flags);
	current_rx_req = port->current_rx_req;
	pending_rx_bytes = port->pending_rx_bytes;
	current_rx_buf = port->current_rx_buf;
	port->current_rx_req = NULL;
	port->current_rx_buf = NULL;
	port->pending_rx_bytes = 0;
	bytes_copied = 0;

	if (list_empty(&port->read_queued) && !pending_rx_bytes) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		pr_debug("%s(): read_queued list is empty.\n", __func__);
		goto start_rx;
	}

	/*
	 * Consider below cases:
	 * 1. If available read buffer size (i.e. count value) is greater than
	 * available data as part of one USB OUT request buffer, then consider
	 * copying multiple USB OUT request buffers until read buffer is filled.
	 * 2. If available read buffer size (i.e. count value) is smaller than
	 * available data as part of one USB OUT request buffer, then copy this
	 * buffer data across multiple read() call until whole USB OUT request
	 * buffer is copied.
	 */
	while ((pending_rx_bytes || !list_empty(&port->read_queued)) && count) {
		if (pending_rx_bytes == 0) {
			pool = &port->read_queued;
			req = list_first_entry(pool, struct usb_request, list);
			list_del_init(&req->list);
			current_rx_req = req;
			pending_rx_bytes = req->actual;
			current_rx_buf = req->buf;
		}

		spin_unlock_irqrestore(&port->port_lock, flags);
		size = count;
		if (size > pending_rx_bytes)
			size = pending_rx_bytes;

		pr_debug("pending_rx_bytes:%zu count:%zu size:%zu\n",
					pending_rx_bytes, count, size);
		size -= copy_to_user(buf, current_rx_buf, size);
		port->nbytes_to_port_bridge += size;
		bytes_copied += size;
		count -= size;
		buf += size;

		spin_lock_irqsave(&port->port_lock, flags);
		if (!port->is_connected) {
			list_add_tail(&current_rx_req->list, &port->read_pool);
			spin_unlock_irqrestore(&port->port_lock, flags);
			return -EAGAIN;
		}

		/*
		 * partial data available, then update pending_rx_bytes,
		 * otherwise add USB request back to read_pool for next data.
		 */
		if (size < pending_rx_bytes) {
			pending_rx_bytes -= size;
			current_rx_buf += size;
		} else {
			list_add_tail(&current_rx_req->list, &port->read_pool);
			pending_rx_bytes = 0;
			current_rx_req = NULL;
			current_rx_buf = NULL;
		}
	}

	port->pending_rx_bytes = pending_rx_bytes;
	port->current_rx_buf = current_rx_buf;
	port->current_rx_req = current_rx_req;
	spin_unlock_irqrestore(&port->port_lock, flags);

start_rx:
	usb_nmea_start_rx(port);
	return bytes_copied;
}

ssize_t f_nmea_write(struct file *file,
		       const char __user *buf,
		       size_t count,
		       loff_t *ppos)
{
	int ret;
	unsigned long flags;
	struct f_nmea *port;
	struct usb_request *req;
	struct list_head *pool;
	unsigned xfer_size;
	struct usb_ep *in;

	port = file->private_data;
	if (!port) {
		pr_err("port is NULL.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&port->port_lock, flags);
	pr_debug("write on port(%s)(%pK)\n", port->name, port);

	if (!port->is_connected) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		pr_err("%s: cable is disconnected.\n", __func__);
		return -ENODEV;
	}

	if (list_empty(&port->write_pool)) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		pr_debug("%s: Request list is empty.\n", __func__);
		return 0;
	}

	in = port->port_usb.in;
	pool = &port->write_pool;
	req = list_first_entry(pool, struct usb_request, list);
	list_del_init(&req->list);
	spin_unlock_irqrestore(&port->port_lock, flags);

	pr_debug("%s: write buf size:%zu\n", __func__, count);
	if (count > BRIDGE_TX_BUF_SIZE)
		xfer_size = BRIDGE_TX_BUF_SIZE;
	else
		xfer_size = count;

	ret = copy_from_user(req->buf, buf, xfer_size);
	if (ret) {
		pr_err("copy_from_user failed: err %d\n", ret);
		ret = -EFAULT;
	} else {
		req->length = xfer_size;
		ret = usb_ep_queue(in, req, GFP_KERNEL);
		if (ret) {
			pr_err("EP QUEUE failed:%d\n", ret);
			ret = -EIO;
			goto err_exit;
		}
		spin_lock_irqsave(&port->port_lock, flags);
		port->nbytes_from_port_bridge += req->length;
		spin_unlock_irqrestore(&port->port_lock, flags);
	}

err_exit:
	if (ret) {
		spin_lock_irqsave(&port->port_lock, flags);
		/* USB cable is connected, add it back otherwise free request */
		if (port->is_connected)
			list_add(&req->list, &port->write_pool);
		else
			usb_nmea_free_req(in, req);
		spin_unlock_irqrestore(&port->port_lock, flags);
		return ret;
	}

	return xfer_size;
}

static unsigned int f_nmea_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	struct f_nmea *port;
	unsigned long flags;

	port = file->private_data;
	if (port && port->is_connected) {
		poll_wait(file, &port->read_wq, wait);
		spin_lock_irqsave(&port->port_lock, flags);
		if (!list_empty(&port->read_queued)) {
			mask |= POLLIN | POLLRDNORM;
			pr_debug("sets POLLIN for %s\n", port->name);
		}

		if (port->cbits_updated) {
			mask |= POLLPRI;
			pr_debug("sets POLLPRI for %s\n", port->name);
		}
		spin_unlock_irqrestore(&port->port_lock, flags);
	} else {
		pr_err("Failed due to NULL device or disconnected.\n");
		mask = POLLERR;
	}

	return mask;
}

static int f_nmea_tiocmget(struct f_nmea *port)
{
	struct cserial	*cser;
	unsigned int result = 0;

	if (!port) {
		pr_err("port is NULL.\n");
		return -ENODEV;
	}

	cser = &port->port_usb;
	if (cser->get_dtr)
		result |= (cser->get_dtr(cser) ? TIOCM_DTR : 0);

	if (cser->get_rts)
		result |= (cser->get_rts(cser) ? TIOCM_RTS : 0);

	if (cser->serial_state & TIOCM_CD)
		result |= TIOCM_CD;

	if (cser->serial_state & TIOCM_RI)
		result |= TIOCM_RI;
	return result;
}

static int f_nmea_tiocmset(struct f_nmea *port,
			unsigned int set, unsigned int clear)
{
	struct cserial *cser;
	int status = 0;

	if (!port) {
		pr_err("port is NULL.\n");
		return -ENODEV;
	}

	cser = &port->port_usb;
	if (set & TIOCM_RI) {
		if (cser->send_ring_indicator) {
			cser->serial_state |= TIOCM_RI;
			status = cser->send_ring_indicator(cser, 1);
		}
	}
	if (clear & TIOCM_RI) {
		if (cser->send_ring_indicator) {
			cser->serial_state &= ~TIOCM_RI;
			status = cser->send_ring_indicator(cser, 0);
		}
	}
	if (set & TIOCM_CD) {
		if (cser->send_carrier_detect) {
			cser->serial_state |= TIOCM_CD;
			status = cser->send_carrier_detect(cser, 1);
		}
	}
	if (clear & TIOCM_CD) {
		if (cser->send_carrier_detect) {
			cser->serial_state &= ~TIOCM_CD;
			status = cser->send_carrier_detect(cser, 0);
		}
	}

	return status;
}

static long f_nmea_ioctl(struct file *fp, unsigned cmd,
						unsigned long arg)
{
	long ret = 0;
	int i = 0;
	uint32_t val;
	struct f_nmea *port;

	port = fp->private_data;
	if (!port) {
		pr_err("port is null.\n");
		return POLLERR;
	}

	switch (cmd) {
	case TIOCMBIC:
	case TIOCMBIS:
	case TIOCMSET:
		pr_debug("TIOCMSET on port(%s)%pK\n", port->name, port);
		i = get_user(val, (uint32_t *)arg);
		if (i) {
			pr_err("Error getting TIOCMSET value\n");
			return i;
		}
		ret = f_nmea_tiocmset(port, val, ~val);
		break;
	case TIOCMGET:
		pr_debug("TIOCMGET on port(%s)%pK\n", port->name, port);
		ret = f_nmea_tiocmget(port);
		if (ret >= 0) {
			ret = put_user(ret, (uint32_t *)arg);
			port->cbits_updated = false;
		}
		break;
	default:
		pr_err("Received cmd:%d not supported\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static void usb_nmea_notify_modem(void *fport, int ctrl_bits)
{
	int temp;
	struct f_nmea *port = fport;

	if (!port) {
		pr_err("port is null\n");
		return;
	}

	pr_debug("port(%s): ctrl_bits:%x\n", port->name, ctrl_bits);

	temp = convert_acm_sigs_to_uart(ctrl_bits);

	if (temp == port->cbits_to_modem)
		return;

	port->cbits_to_modem = temp;
	port->cbits_updated = true;

	wake_up(&port->read_wq);
}

int usb_nmea_connect(struct f_nmea *port)
{
	unsigned long flags;
	int ret;
	struct cserial *cser;

	if (!port) {
		pr_err("port is NULL.\n");
		return -ENODEV;
	}

	pr_debug("port(%s) (%pK)\n", port->name, port);

	cser = &port->port_usb;
	cser->notify_modem = usb_nmea_notify_modem;

	ret = usb_ep_enable(cser->in);
	if (ret) {
		pr_err("usb_ep_enable failed eptype:IN ep:%pK, err:%d",
					cser->in, ret);
		return ret;
	}
	cser->in->driver_data = port;

	ret = usb_ep_enable(cser->out);
	if (ret) {
		pr_err("usb_ep_enable failed eptype:OUT ep:%pK, err: %d",
					cser->out, ret);
		cser->in->driver_data = 0;
		return ret;
	}
	cser->out->driver_data = port;

	spin_lock_irqsave(&port->port_lock, flags);
	cser->pending = false;
	cser->q_again = false;
	port->is_connected = true;
	spin_unlock_irqrestore(&port->port_lock, flags);

	usb_nmea_start_io(port);
	wake_up(&port->open_wq);
	return 0;
}

void usb_nmea_disconnect(struct f_nmea *port)
{
	unsigned long flags;

	usb_nmea_stop_io(port);

	/* lower DTR to modem */
	usb_nmea_notify_modem(port, 0);

	spin_lock_irqsave(&port->port_lock, flags);
	port->is_connected = false;
	port->nbytes_from_host = port->nbytes_to_host = 0;
	port->nbytes_to_port_bridge = 0;
	spin_unlock_irqrestore(&port->port_lock, flags);
}

static const struct file_operations f_nmea_fops = {
	.owner = THIS_MODULE,
	.open = f_nmea_open,
	.release = f_nmea_release,
	.read = f_nmea_read,
	.write = f_nmea_write,
	.poll = f_nmea_poll,
	.unlocked_ioctl = f_nmea_ioctl,
	.compat_ioctl = f_nmea_ioctl,
};

static struct f_nmea *f_nmea_alloc(char *func_name, int portno)
{
	int ret;
	dev_t dev;
	struct device *device;
	struct f_nmea *port;

	port = kzalloc(sizeof(struct f_nmea), GFP_KERNEL);
	if (!port) {
		ret = -ENOMEM;
		return  ERR_PTR(ret);
	}

	mutex_lock(&chardev_ida_lock);
	if (idr_is_empty(&chardev_ida.idr)) {
		ret = usb_nmea_alloc_chardev_region();
		if (ret) {
			mutex_unlock(&chardev_ida_lock);
			pr_err("alloc chardev failed\n");
			goto err_alloc_chardev;
		}
	}

	ret = ida_simple_get(&chardev_ida, 0, 0, GFP_KERNEL);
	if (ret >= NUM_INSTANCE) {
		ida_simple_remove(&chardev_ida, ret);
		mutex_unlock(&chardev_ida_lock);
		ret = -ENODEV;
		goto err_get_ida;
	}

	port->port_num = portno;
	port->minor = ret;
	mutex_unlock(&chardev_ida_lock);

	snprintf(port->name, sizeof(port->name), "%s%d", DEVICE_NAME, portno);
	spin_lock_init(&port->port_lock);

	init_waitqueue_head(&port->open_wq);
	init_waitqueue_head(&port->read_wq);
	INIT_LIST_HEAD(&port->read_pool);
	INIT_LIST_HEAD(&port->read_queued);
	INIT_LIST_HEAD(&port->write_pool);

	port->fnmea_wq = create_singlethread_workqueue(port->name);
	if (!port->fnmea_wq) {
		pr_err("Unable to create workqueue fnmea_wq for port:%s\n",
						port->name);
		ret = -ENOMEM;
		goto err_get_ida;
	}

	/* create char device */
	cdev_init(&port->fnmea_cdev, &f_nmea_fops);
	dev = MKDEV(major, port->minor);
	ret = cdev_add(&port->fnmea_cdev, dev, 1);
	if (ret) {
		pr_err("Failed to add cdev for port(%s)\n", port->name);
		goto err_cdev_add;
	}

	device = device_create(fnmea_classp, NULL, dev, NULL, port->name);
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		goto err_create_dev;
	}

	pr_info("port_name:%s (%pK) portno:(%d)\n",
			port->name, port, port->port_num);
	return port;

err_create_dev:
	cdev_del(&port->fnmea_cdev);
err_cdev_add:
	destroy_workqueue(port->fnmea_wq);
err_get_ida:
	usb_nmea_chardev_deinit();
err_alloc_chardev:
	kfree(port);

	return ERR_PTR(ret);
}

static void usb_nmea_chardev_deinit(void)
{

	if (idr_is_empty(&chardev_ida.idr)) {

		if (major) {
			unregister_chrdev_region(MKDEV(major, 0), minors);
			major = minors = 0;
		}

		if (!IS_ERR_OR_NULL(fnmea_classp))
			class_destroy(fnmea_classp);
	}
}

static int usb_nmea_alloc_chardev_region(void)
{
	int ret;
	dev_t dev;

	ret = alloc_chrdev_region(&dev,
			       0,
			       NUM_INSTANCE,
			       MODULE_NAME);
	if (IS_ERR_VALUE((unsigned long)ret)) {
		pr_err("alloc_chrdev_region() failed ret:%i\n", ret);
		return ret;
	}

	major = MAJOR(dev);
	minors = NUM_INSTANCE;

	fnmea_classp = class_create(THIS_MODULE, MODULE_NAME);
	if (IS_ERR(fnmea_classp)) {
		pr_err("class_create() failed ENOMEM\n");
		ret = -ENOMEM;
	}

	return 0;
}

static inline struct f_nmea_opts *to_f_nmea_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_nmea_opts,
			func_inst.group);
}

static struct f_nmea_opts *to_fi_nmea_opts(struct usb_function_instance *fi)
{
	return container_of(fi, struct f_nmea_opts, func_inst);
}

static void cserial_attr_release(struct config_item *item)
{
	struct f_nmea_opts *opts = to_f_nmea_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations cserial_item_ops = {
	.release	= cserial_attr_release,
};

static ssize_t usb_nmea_status_show(struct config_item *item, char *page)
{
	struct f_nmea *port = to_f_nmea_opts(item)->port;
	char *buf;
	unsigned long flags;
	int temp = 0;
	int ret;

	buf = kzalloc(sizeof(char) * 512, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	spin_lock_irqsave(&port->port_lock, flags);
	temp += scnprintf(buf + temp, 512 - temp,
			"###PORT:%s###\n"
			"port_no:%d\n"
			"func:%s\n"
			"nbytes_to_host: %lu\n"
			"nbytes_from_host: %lu\n"
			"nbytes_to_port_bridge:  %lu\n"
			"nbytes_from_port_bridge: %lu\n"
			"cbits_to_modem:  %u\n"
			"Port Opened: %s\n",
			port->name,
			port->port_num,
			to_f_nmea_opts(item)->func_name,
			port->nbytes_to_host,
			port->nbytes_from_host,
			port->nbytes_to_port_bridge,
			port->nbytes_from_port_bridge,
			port->cbits_to_modem,
			(port->port_open ? "Opened" : "Closed"));
	spin_unlock_irqrestore(&port->port_lock, flags);

	ret = scnprintf(page, temp, buf);
	kfree(buf);

	return ret;
}

static ssize_t usb_nmea_status_store(struct config_item *item,
			const char *page, size_t len)
{
	struct f_nmea *port = to_f_nmea_opts(item)->port;
	unsigned long flags;
	u8 stats;

	if (page == NULL) {
		pr_err("Invalid buffer");
		return len;
	}

	if (kstrtou8(page, 0, &stats) != 0 || stats != 0) {
		pr_err("(%u)Wrong value. enter 0 to clear.\n", stats);
		return len;
	}

	spin_lock_irqsave(&port->port_lock, flags);
	port->nbytes_to_host = port->nbytes_from_host = 0;
	port->nbytes_to_port_bridge = port->nbytes_from_port_bridge = 0;
	spin_unlock_irqrestore(&port->port_lock, flags);

	return len;
}

CONFIGFS_ATTR(usb_nmea_, status);
static struct configfs_attribute *cserial_attrs[] = {
	&usb_nmea_attr_status,
	NULL,
};

static struct config_item_type cserial_func_type = {
	.ct_item_ops	= &cserial_item_ops,
	.ct_attrs	= cserial_attrs,
	.ct_owner	= THIS_MODULE,
};

static int nmea_set_inst_name(struct usb_function_instance *f, const char *name)
{
	struct f_nmea_opts *opts =
		container_of(f, struct f_nmea_opts, func_inst);
	char *ptr, *str;
	size_t name_len, str_size;
	int ret;
	struct f_nmea *port;

	name_len = strlen(name) + 1;
	if (name_len > MAX_CDEV_INST_NAME)
		return -ENAMETOOLONG;

	/* expect name as cdev.<func>.<port_num> */
	str = strnchr(name, strlen(name), '.');
	if (!str) {
		pr_err("invalid input (%s)\n", name);
		return -EINVAL;
	}

	/* get function name */
	str_size = name_len - strlen(str);
	if (str_size > MAX_CDEV_FUNC_NAME)
		return -ENAMETOOLONG;

	ptr = kstrndup(name, str_size - 1, GFP_KERNEL);
	if (!ptr) {
		pr_err("error:%ld\n", PTR_ERR(ptr));
		return -ENOMEM;
	}

	opts->func_name = ptr;

	/* get port number */
	str = strrchr(name, '.');
	if (!str) {
		pr_err("err: port number not found\n");
		return -EINVAL;
	}
	pr_debug("str:%s\n", str);

	*str = '\0';
	str++;

	ret = kstrtou8(str, 0, &opts->port_num);
	if (ret) {
		pr_err("erro: not able to get port number\n");
		return -EINVAL;
	}

	pr_debug("gser: port_num:%d func_name:%s\n",
			opts->port_num, opts->func_name);

	port = f_nmea_alloc(opts->func_name, opts->port_num);
	if (IS_ERR(port)) {
		pr_err("Failed to create cdev port(%d)\n", opts->port_num);
		return -ENOMEM;
	}

	opts->port = port;

	return 0;
}

static struct usb_function_instance *nmea_alloc_inst(void)
{
	struct f_nmea_opts *opts;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);

	opts->func_inst.free_func_inst = nmea_free_inst;
	opts->func_inst.set_inst_name = nmea_set_inst_name;

	config_group_init_type_name(&opts->func_inst.group, "",
				    &cserial_func_type);
	return &opts->func_inst;
}

static struct usb_function *nmea_alloc(struct usb_function_instance *fi)
{
	struct f_nmea_opts *opts = to_fi_nmea_opts(fi);
	struct f_nmea *port = opts->port;

	port->port_usb.func.name = "nmea";
	port->port_usb.func.strings = usb_nmea_strings;
	port->port_usb.func.bind = usb_nmea_bind;
	port->port_usb.func.unbind = usb_nmea_unbind;
	port->port_usb.func.set_alt = usb_nmea_set_alt;
	port->port_usb.func.disable = usb_nmea_disable;
	port->port_usb.func.free_func = usb_nmea_free_func;

	return &port->port_usb.func;
}

DECLARE_USB_FUNCTION_INIT(nmea, nmea_alloc_inst, nmea_alloc);
MODULE_DESCRIPTION("USB Serial Character Driver");
MODULE_LICENSE("GPL v2");
