/*
 * \brief  Block driver to access Genode's block service
 * \author Stefan Kalkowski <stefan.kalkowski@genode-labs.com>
 * \author Martin Stein <martin.stein@genode-labs.com>
 * \date   2010-07-08
 */

/*
 * Copyright (C) 2011-2015 Genode Labs GmbH
 *
 * This file is part of the Genode OS framework, which is distributed
 * under the terms of the GNU General Public License version 2.
 */

/* Linux includes */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/semaphore.h>
#include <linux/interrupt.h>

enum {
	SMC_ID_BLK_DCOUNT    = 100,
	SMC_ID_BLK_BCOUNT    = 101,
	SMC_ID_BLK_BSIZE     = 102,
	SMC_ID_BLK_WRITEABLE = 103,
	SMC_ID_BLK_QSIZE     = 104,
	SMC_ID_BLK_IRQ       = 105,
};

#define SMC_1_ARGS             long arg_0
#define SMC_2_ARGS SMC_1_ARGS, long arg_1
#define SMC_3_ARGS SMC_2_ARGS, long arg_2
#define SMC_4_ARGS SMC_3_ARGS, long arg_3

#define SMC_1_FILL_ARG_REGS \
	register long arg_0_reg asm("r0") = arg_0;

#define SMC_2_FILL_ARG_REGS \
	SMC_1_FILL_ARG_REGS \
	register long arg_1_reg asm("r1") = arg_1;

#define SMC_3_FILL_ARG_REGS \
	SMC_2_FILL_ARG_REGS \
	register long arg_2_reg asm("r2") = arg_2;

#define SMC_4_FILL_ARG_REGS \
	SMC_3_FILL_ARG_REGS \
	register long arg_3_reg asm("r3") = arg_3;

#define SMC_5_FILL_ARG_REGS \
	SMC_4_FILL_ARG_REGS \
	register long arg_4_reg asm("r4") = arg_4;

#define SMC_6_FILL_ARG_REGS \
	SMC_5_FILL_ARG_REGS \
	register long arg_5_reg asm("r5") = arg_5;

#define SMC_1_ASM ".arch_extension sec\nsmc #0\n" : "+r" (arg_0_reg)
#define SMC_2_ASM SMC_1_ASM:  "r" (arg_1_reg)
#define SMC_3_ASM SMC_2_ASM,  "r" (arg_2_reg)
#define SMC_4_ASM SMC_3_ASM,  "r" (arg_3_reg)
#define SMC_5_ASM SMC_4_ASM,  "r" (arg_4_reg)
#define SMC_6_ASM SMC_5_ASM,  "r" (arg_5_reg)


long secure_monitor_call_1(SMC_1_ARGS)
{
	SMC_1_FILL_ARG_REGS
	asm volatile(SMC_1_ASM);
	return arg_0_reg;
}


long secure_monitor_call_2(SMC_2_ARGS)
{
	SMC_2_FILL_ARG_REGS
	asm volatile(SMC_2_ASM);
	return arg_0_reg;
}


long secure_monitor_call_3(SMC_3_ARGS)
{
	SMC_3_FILL_ARG_REGS
	asm volatile(SMC_3_ASM);
	return arg_0_reg;
}


long secure_monitor_call_4(SMC_4_ARGS)
{
	SMC_4_FILL_ARG_REGS
	asm volatile(SMC_4_ASM);
	return arg_0_reg;
}


unsigned genode_block_irq(unsigned idx) {
	return secure_monitor_call_2(SMC_ID_BLK_IRQ, idx); };


unsigned genode_block_count(void) {
	return secure_monitor_call_1(SMC_ID_BLK_DCOUNT); };


//const char *genode_block_name(unsigned idx)
//{
//	secure_monitor_call(SMC_BLOCK_NAME);
//	return "unknown";
//}


void genode_block_geometry(unsigned idx, unsigned long *blk_cnt,
                           unsigned long *blk_sz, int *writeable,
                           unsigned long *req_queue_sz)
{
	*blk_cnt      = secure_monitor_call_2(SMC_ID_BLK_BCOUNT,    idx);
	*blk_sz       = secure_monitor_call_2(SMC_ID_BLK_BSIZE,     idx);
	*writeable    = secure_monitor_call_2(SMC_ID_BLK_WRITEABLE, idx);
	*req_queue_sz = secure_monitor_call_2(SMC_ID_BLK_QSIZE,     idx);
}


//void genode_block_request(unsigned idx, unsigned long sz,
//                           void *req, unsigned long *offset, unsigned long queue_offset,
//                         unsigned long size, unsigned long long disc_offset,
//                         int write)
//{
//	secure_monitor_call(SMC_BLOCK_REQUEST);
//}
//
//enum Geometry {
//	KERNEL_SECTOR_SIZE = 512,      /* sector size used by kernel */
//	GENODE_BLK_MINORS  = 16        /* number of minor numbers */
//};


/*
 * The internal representation of our device.
 */
struct genode_blk_device {
	unsigned              blk_cnt;    /* Total block count */
	unsigned long         blk_sz;     /* Single block size */
	spinlock_t            lock;       /* For mutual exclusion */
	struct gendisk       *gd;         /* Generic disk structure */
	struct request_queue *queue;      /* The device request queue */
	struct semaphore      queue_wait; /* Used to block, when queue is full */
	short                 stopped;    /* Indicates queue availability */
	unsigned              idx;        /* drive index */
	unsigned              irq;        /* irq number */
};

enum { MAX_DISKS = 16 };
static struct genode_blk_device blk_devs[MAX_DISKS];

///*
// * Handle an I/O request.
// */
//static void genode_blk_request(struct request_queue *q)
//{
//	struct request *req;
//	unsigned long  queue_offset;
//	void          *buf;
//	unsigned long long  offset;
//	unsigned long  nbytes;
//	short          write;
//	struct genode_blk_device* dev;
//
//	while ((req = blk_fetch_request(q))) {
//		dev = req->rq_disk->private_data;
//		buf    = 0;
//		offset = blk_rq_pos(req) * KERNEL_SECTOR_SIZE;
//		nbytes = blk_rq_bytes(req);
//		write  = rq_data_dir(req) == WRITE;
//
//		if (req->cmd_type != REQ_TYPE_FS) {
//			printk(KERN_NOTICE "Skip non-fs request\n");
//			__blk_end_request_all(req, -EIO);
//			continue;
//		}
//
//		while (!buf) {
//			unsigned long flags;
//
//			if ((buf = genode_block_request(dev->idx, nbytes, req, &queue_offset)))
//				break;
//
//			/* stop_queue needs disabled interrupts */
//			local_irq_save(flags);
//			blk_stop_queue(q);
//
//			dev->stopped = 1;
//
//			/*
//			 * This function is called with the request queue lock held, unlock to
//			 * enable VCPU IRQs
//			 */
//			spin_unlock_irqrestore(q->queue_lock, flags);
//			/* block until new responses are available */
//			down(&dev->queue_wait);
//			spin_lock_irqsave(q->queue_lock, flags);
//
//			/* start_queue needs disabled interrupts */
//			blk_start_queue(q);
//			local_irq_restore(flags);
//		}
//
//		if (write) {
//			char               *ptr = (char*) buf;
//			struct req_iterator iter;
//			struct bio_vec     *bvec;
//
//			rq_for_each_segment(bvec, req, iter) {
//				void *buffer = page_address(bvec->bv_page) + bvec->bv_offset;
//				memcpy((void*)ptr, buffer, bvec->bv_len);
//				ptr += bvec->bv_len;
//			}
//		}
//
//		genode_block_submit(dev->idx, queue_offset, nbytes, offset, write);
//	}
//}
//
//
//static void FASTCALL
//genode_end_request(void *request, short write,
//                   void *buf, unsigned long sz) {
//	struct request *req = (struct request*) request;
//	struct genode_blk_device *dev = req->rq_disk->private_data;
//	char *ptr = (char*) buf;
//
//	if (!write) {
//		struct req_iterator iter;
//		struct bio_vec     *bvec;
//
//		rq_for_each_segment(bvec, req, iter) {
//			void *buffer = page_address(bvec->bv_page) + bvec->bv_offset;
//			memcpy(buffer, (void*)ptr, bvec->bv_len);
//			ptr += bvec->bv_len;
//		}
//	}
//
//	__blk_end_request_all(req, 0);
//
//	if (dev->stopped) {
//		dev->stopped = 0;
//		up(&dev->queue_wait);
//	}
//}
//
//
//static int genode_blk_getgeo(struct block_device *bdev, struct hd_geometry *geo)
//{
//	struct genode_blk_device *dev  = bdev->bd_disk->private_data;
//	unsigned long             size = dev->blk_cnt * dev->blk_sz *
//	                                 (dev->blk_sz / KERNEL_SECTOR_SIZE);
//	geo->cylinders = size >> 7;
//	geo->heads     = 4;
//	geo->sectors   = 32;
//	return 0;
//}
//
//
///*
// * The device operations structure.
// */
//static struct block_device_operations genode_blk_ops = {
//		.owner  = THIS_MODULE,
//		.getgeo = genode_blk_getgeo
//};
//
//
static irqreturn_t event_interrupt(int irq, void *data)
{
	printk(KERN_NOTICE "event_interrupt not implemented, irq %u\n", irq);
	return IRQ_HANDLED;
//	unsigned long flags;
//	struct genode_blk_device *dev = (struct genode_blk_device *)data;
//	spin_lock_irqsave(dev->queue->queue_lock, flags);
//	genode_block_collect_responses(dev->idx);
//	spin_unlock_irqrestore(dev->queue->queue_lock, flags);
//	return IRQ_HANDLED;
}


static int __init genode_blk_init(void)
{
	int      err;
	unsigned drive;
	unsigned drive_cnt;

	drive_cnt = genode_block_count();
	if (drive_cnt > MAX_DISKS) { drive_cnt = MAX_DISKS; }
	printk(KERN_NOTICE "genode block: drive count %u\n", drive_cnt);

	/**
	 * Loop through all Genode block devices and register them in Linux.
	 */
	for (drive = 0 ; drive < drive_cnt; drive++) {
//		int           major_num;
		int           writeable    = 0;
		unsigned long req_queue_sz = 0;

		/* Initialize device structure */
		memset (&blk_devs[drive], 0, sizeof(struct genode_blk_device));
		blk_devs[drive].idx = drive;
		spin_lock_init(&blk_devs[drive].lock);

		genode_block_geometry(drive, (unsigned long*)&blk_devs[drive].blk_cnt,
		                      &blk_devs[drive].blk_sz, &writeable, &req_queue_sz);

		/**
		 * Obtain an IRQ for the drive.
		 */
		blk_devs[drive].irq = genode_block_irq(drive);
		if ((err = request_irq(blk_devs[drive].irq, event_interrupt, 0,
		                       "genode block", &blk_devs[drive]))) {
			printk(KERN_WARNING "genode block: err %d on IRQ request\n", err);
			return err;
		}

		printk(KERN_NOTICE "genode block: drive %u\n", drive);
		printk(KERN_NOTICE "   block count %u\n", blk_devs[drive].blk_cnt);
		printk(KERN_NOTICE "   block size  %lu\n", blk_devs[drive].blk_sz);
		printk(KERN_NOTICE "   writeable   %u\n", writeable);
		printk(KERN_NOTICE "   queue size  %lu\n", req_queue_sz);
		printk(KERN_NOTICE "   irq         %u\n", blk_devs[drive].irq);
//
//		/*
//		 * Get a request queue.
//		 */
//		if(!(blk_devs[drive].queue = blk_init_queue(genode_blk_request,
//		                                           &blk_devs[drive].lock)))
//				return -ENOMEM;
//
//		/*
//		 * Align queue requests to hardware sector size.
//		 */
//		blk_queue_logical_block_size(blk_devs[drive].queue, blk_devs[drive].blk_sz);
//
//		/*
//		 * Important, limit number of sectors per request,
//		 * as Genode's block-session has a limited request-transmit-queue.
//		 */
//		blk_queue_max_hw_sectors(blk_devs[drive].queue, req_queue_sz / KERNEL_SECTOR_SIZE);
//		blk_devs[drive].queue->queuedata = &blk_devs[drive];
//
//		sema_init(&blk_devs[drive].queue_wait, 0);
//		blk_devs[drive].stopped = 0;
//
//		/*
//		 * Register block device and gain major number.
//		 */
//		major_num = register_blkdev(0, genode_block_name(drive));
//		if(major_num < 1) {
//				printk(KERN_WARNING "genode_blk: unable to get major number\n");
//				return -EBUSY;
//		}
//
//		/*
//		 * Allocate and setup generic disk structure.
//		 */
//		if(!(blk_devs[drive].gd = alloc_disk(GENODE_BLK_MINORS))) {
//				unregister_blkdev(major_num, genode_block_name(drive));
//				return -ENOMEM;
//		}
//		blk_devs[drive].gd->major        = major_num;
//		blk_devs[drive].gd->first_minor  = 0;
//		blk_devs[drive].gd->fops         = &genode_blk_ops;
//		blk_devs[drive].gd->private_data = &blk_devs[drive];
//		blk_devs[drive].gd->queue        = blk_devs[drive].queue;
//		strncpy(blk_devs[drive].gd->disk_name, genode_block_name(drive),
//		        sizeof(blk_devs[drive].gd->disk_name));
//		set_capacity(blk_devs[drive].gd, blk_devs[drive].blk_cnt *
//		                         (blk_devs[drive].blk_sz / KERNEL_SECTOR_SIZE));
//
//		/* Set it read-only or writeable */
//		if (!writeable)
//			set_disk_ro(blk_devs[drive].gd, 1);
//
//		if (drive == 0)
//			genode_block_register_callback(genode_end_request);
//
//		/* Make the block device available to the system */
//		add_disk(blk_devs[drive].gd);
	}

	printk(KERN_NOTICE "genode block: initialized\n");
	while(1);
	return 0;
}

/* vvv I checked all stuff downwards vvv */

static void __exit genode_blk_exit(void)
{
	printk(KERN_NOTICE "%s %u\n", __func__, __LINE__);
//	unsigned drive, drive_cnt = (genode_block_count() > MAX_DISKS)
//	                            ? MAX_DISKS : genode_block_count();
//	for (drive = 0 ; drive < drive_cnt; drive++) {
//		del_gendisk(blk_devs[drive].gd);
//		put_disk(blk_devs[drive].gd);
//		unregister_blkdev(blk_devs[drive].gd->major, genode_block_name(drive));
//		blk_cleanup_queue(blk_devs[drive].queue);
//	}
}

module_init(genode_blk_init);
module_exit(genode_blk_exit);

MODULE_LICENSE("GPL");
