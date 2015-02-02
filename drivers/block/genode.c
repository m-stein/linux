/*
 * \brief  Block driver using the SMC API of Genode's block service
 * \author Martin Stein     <martin.stein@genode-labs.com>
 * \author Stefan Kalkowski <stefan.kalkowski@genode-labs.com>
 * \date   2015-11-04
 */

/*
 * Copyright (C) 2015 Genode Labs GmbH
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
#include <linux/dma-mapping.h>
#include <genode_tz_vmm.h>


unsigned genode_block_irq(unsigned idx)
{
	return secure_monitor_call_3_1(SMC_BLOCK, SMC_BLOCK_IRQ, idx);
};


int genblk_collect(unsigned idx)
{
	return secure_monitor_call_3_1(SMC_BLOCK, SMC_BLOCK_COLLECT_REPLY, idx);
};


unsigned genode_block_count(void)
{
	return secure_monitor_call_2_1(SMC_BLOCK, SMC_BLOCK_DEVICE_COUNT);
};


void genode_block_submit(unsigned idx, unsigned long queue_offset,
                         unsigned long size, unsigned long long disc_offset,
                         int write, void * dst)
{
	secure_monitor_call_9_1(SMC_BLOCK, SMC_BLOCK_SUBMIT_REQUEST, idx,
	                        queue_offset, size,
	                        (unsigned long)(disc_offset >> 32),
	                        (unsigned long)disc_offset, write,
	                        (unsigned long)dst);
}

void genblk_smc_buffer(unsigned long buf_phys, int buf_size)
{
	secure_monitor_call_4_1(SMC_BLOCK, SMC_BLOCK_BUFFER, buf_phys, buf_size);
}

void genblk_smc_callback(void)
{
	secure_monitor_call_2_1(SMC_BLOCK, SMC_BLOCK_START_CALLBACK);
}


static dma_addr_t genblk_buf_phys;
static void *     genblk_buf;
enum {            GENBLK_BUF_SIZE = 1024 * 1024 };


void genblk_smc_name(unsigned dev, char * const name,
                     unsigned long const name_max_len)
{
	secure_monitor_call_3_1(SMC_BLOCK, SMC_BLOCK_NAME, dev);
	strncpy(name, genblk_buf, name_max_len);
}


void genode_block_geometry(unsigned idx, unsigned long *blk_cnt,
                           unsigned long *blk_sz, int *writeable,
                           unsigned long *req_queue_sz)
{
	*blk_cnt      = secure_monitor_call_3_1(SMC_BLOCK, SMC_BLOCK_BLOCK_COUNT, idx);
	*blk_sz       = secure_monitor_call_3_1(SMC_BLOCK, SMC_BLOCK_BLOCK_SIZE,  idx);
	*writeable    = secure_monitor_call_3_1(SMC_BLOCK, SMC_BLOCK_WRITEABLE,   idx);
	*req_queue_sz = secure_monitor_call_3_1(SMC_BLOCK, SMC_BLOCK_QUEUE_SIZE,  idx);
}


void* genblk_smc_request(unsigned idx, unsigned long sz,
                         void * req, unsigned long * offset)
{
	return (void *)secure_monitor_call_5_2(SMC_BLOCK, SMC_BLOCK_NEW_REQUEST,
	                                       idx, sz, (long)req, offset);
}

enum Geometry {
	KERNEL_SECTOR_SIZE = 512,      /* sector size used by kernel */
	GENODE_BLK_MINORS  = 16        /* number of minor numbers */
};

struct genblk_reply
{
	unsigned long req;
	unsigned long write;
	unsigned long data_size;
	unsigned long data[];

} __attribute__ ((__packed__));


/*
 * The internal representation of our device.
 */
struct genblk_device {
	unsigned              blk_cnt;    /* Total block count */
	unsigned long         blk_sz;     /* Single block size */
	spinlock_t            lock;       /* For mutual exclusion */
	struct gendisk       *gd;         /* Generic disk structure */
	struct request_queue *queue;      /* The device request queue */
	struct semaphore      queue_wait; /* Used to block, when queue is full */
	short                 stopped;    /* Indicates queue availability */
	unsigned              idx;        /* drive index */
	unsigned long         hwirq;      /* hwirq number */
};

static unsigned             genblk_dev_cnt;
enum {                      GENBLK_MAX_DEV_CNT = 16 };
static struct genblk_device genblk_devs[GENBLK_MAX_DEV_CNT];

/*
 * Handle an I/O request.
 */
static void genode_blk_request(struct request_queue *q)
{
	struct request *req;
	unsigned long  queue_offset;
	void          *buf;
	unsigned long long  offset;
	unsigned long  nbytes;
	short          write;
	struct genblk_device* dev;

	while ((req = blk_fetch_request(q))) {
		dev = req->rq_disk->private_data;
		buf    = 0;
		offset = blk_rq_pos(req) * KERNEL_SECTOR_SIZE;
		nbytes = blk_rq_bytes(req);
		write  = rq_data_dir(req) == WRITE;

		if (req->cmd_type != REQ_TYPE_FS) {
			printk(KERN_WARNING "genblk: skip non-fs request\n");
			__blk_end_request_all(req, -EIO);
			continue;
		}
		if (write && nbytes > GENBLK_BUF_SIZE) {
			printk(KERN_WARNING "genblk: skip oversized request\n");
			__blk_end_request_all(req, -EIO);
			continue;
		}
		while (!buf) {
			unsigned long flags;

			buf = genblk_smc_request(dev->idx, nbytes, req, &queue_offset);
			if (buf) { break; }

			/* stop_queue needs disabled interrupts */
			local_irq_save(flags);
			blk_stop_queue(q);

			dev->stopped = 1;

			/*
			 * This function is called with the request queue lock held, unlock to
			 * enable VCPU IRQs
			 */
			spin_unlock_irqrestore(q->queue_lock, flags);
			/* block until new responses are available */
			down(&dev->queue_wait);
			spin_lock_irqsave(q->queue_lock, flags);

			/* start_queue needs disabled interrupts */
			blk_start_queue(q);
			local_irq_restore(flags);
		}

		if (write) {
			char                * ptr = (char*)genblk_buf;
			struct req_iterator   iter;
			struct bio_vec        bvec;

			rq_for_each_segment(bvec, req, iter) {
				void *buffer = page_address(bvec.bv_page) + bvec.bv_offset;
				memcpy((void*)ptr, buffer, bvec.bv_len);
				ptr += bvec.bv_len;
			}
		}
		genode_block_submit(dev->idx, queue_offset, nbytes, offset, write, buf);
	}
}


static void
genblk_end_request(void *request, short write, void *buf, unsigned long sz)
{
	struct request *req = (struct request*) request;
	struct genblk_device *dev = req->rq_disk->private_data;
	char *ptr = (char*) buf;

	if (!write) {
		struct req_iterator iter;
		struct bio_vec      bvec;

		rq_for_each_segment(bvec, req, iter) {
			void *buffer = page_address(bvec.bv_page) + bvec.bv_offset;
			memcpy(buffer, (void*)ptr, bvec.bv_len);
			ptr += bvec.bv_len;
		}
	}

	__blk_end_request_all(req, 0);

	if (dev->stopped) {
		dev->stopped = 0;
		up(&dev->queue_wait);
	}
}


static int genode_blk_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct genblk_device *dev  = bdev->bd_disk->private_data;
	unsigned long             size = dev->blk_cnt * dev->blk_sz *
	                                 (dev->blk_sz / KERNEL_SECTOR_SIZE);
	geo->cylinders = size >> 7;
	geo->heads     = 4;
	geo->sectors   = 32;
	return 0;
}


/*
 * The device operations structure.
 */
static struct block_device_operations genode_blk_ops = {
		.owner  = THIS_MODULE,
		.getgeo = genode_blk_getgeo
};


unsigned long tzic_domain_irq(unsigned long const hwirq);
void          tzic_end_sw_irq(unsigned long const irq);


static irqreturn_t event_interrupt(int irq, void *data)
{
	/* lock interrupts and determine device */
	unsigned long flags;
	struct genblk_device * const dev = (struct genblk_device *)data;
	spin_lock_irqsave(dev->queue->queue_lock, flags);

	/* let Genode successively communicate all available request responses */
	while (1) {
		void * d;
		void * r;
		short w;
		unsigned long s;
		int const ret = genblk_collect(dev->idx);
		struct genblk_reply * const reply = (struct genblk_reply *)genblk_buf;

		/* no more responses available */
		if (ret == 0) {
			break;

		/* error during collect */
		} else if (ret < 0) {
			printk(KERN_ERR "genblk: failed to collect reply\n");
			break;
		}
		/* handle communicated response */
		d = (void *)reply->data;
		w = (short)reply->write;
		s = reply->data_size;
		r = (void *)reply->req;
		genblk_end_request(r, w, d, s);
	}
	/* end interrupt */
	tzic_end_sw_irq(dev->hwirq);
	spin_unlock_irqrestore(dev->queue->queue_lock, flags);
	return IRQ_HANDLED;
}

static u64 genblk_dma_mask;

static struct device_dma_parameters genblk_dma_params =
{
	.max_segment_size      = ~0,
	.segment_boundary_mask = ~0
};

static void genblk_pseudo_release(struct device *dev)
{
	printk(KERN_ERR "genblk: %s not implemented\n", __func__);
	while(1);
}

static int __init genode_blk_init(void)
{
	enum { NAME_MAX_LEN = 64 };
	int      err;
	unsigned drive;
	char     name[NAME_MAX_LEN];
	struct device * dev;

	/* initialize buffer */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		printk(KERN_ERR "genblk: failed to alloc pseudo device\n");
		return -EINVAL;
	}
	genblk_dma_mask        = ~0;
	dev->dma_mask          = &genblk_dma_mask;
	dev->coherent_dma_mask =  genblk_dma_mask;
	dev->dma_parms         = &genblk_dma_params;
	dev->release           =  genblk_pseudo_release;
	dev_set_name(dev, "genblk_pseudo");
	err = device_register(dev);
	if (err) {
		printk(KERN_ERR "genblk: failed to register pseudo device\n");
		return -EINVAL;
	}
	genblk_buf = dma_alloc_coherent(dev, GENBLK_BUF_SIZE, &genblk_buf_phys, GFP_KERNEL);
	if (!genblk_buf) {
		printk(KERN_ERR "genblk: failed to alloc buffer\n");
		return -ENOMEM;
	}
	genblk_smc_buffer(genblk_buf_phys, GENBLK_BUF_SIZE);

	genblk_dev_cnt = genode_block_count();
	if (genblk_dev_cnt > GENBLK_MAX_DEV_CNT) {
		genblk_dev_cnt = GENBLK_MAX_DEV_CNT; }

	/**
	 * Loop through all Genode block devices and register them in Linux.
	 */
	for (drive = 0 ; drive < genblk_dev_cnt; drive++) {
		int           major_num;
		int           writeable    = 0;
		unsigned long req_queue_sz = 0;
		unsigned long irq;
		unsigned long capacity;

		/* Initialize device structure */
		memset (&genblk_devs[drive], 0, sizeof(struct genblk_device));
		genblk_devs[drive].idx = drive;
		spin_lock_init(&genblk_devs[drive].lock);

		genode_block_geometry(drive, (unsigned long*)&genblk_devs[drive].blk_cnt,
		                      &genblk_devs[drive].blk_sz, &writeable, &req_queue_sz);

		/**
		 * Obtain an IRQ for the drive.
		 */
		genblk_devs[drive].hwirq = genode_block_irq(drive);
		irq = tzic_domain_irq(genblk_devs[drive].hwirq);
		if ((err = request_irq(irq, event_interrupt, 0,
		                       "genode block", &genblk_devs[drive]))) {
			printk(KERN_ERR "genblk: failed to obtain interrupt\n");
			return err;
		}

		/*
		 * Get a request queue.
		 */
		if(!(genblk_devs[drive].queue =
				blk_init_queue(genode_blk_request,
				               &genblk_devs[drive].lock)))
		{ return -ENOMEM; }

		/*
		 * Align queue requests to hardware sector size.
		 */
		blk_queue_logical_block_size(genblk_devs[drive].queue,
		                             genblk_devs[drive].blk_sz);

		/*
		 * Important, limit number of sectors per request,
		 * as Genode's block-session has a limited request-transmit-queue.
		 */
		blk_queue_max_hw_sectors(genblk_devs[drive].queue,
		                         req_queue_sz / KERNEL_SECTOR_SIZE);
		genblk_devs[drive].queue->queuedata = &genblk_devs[drive];

		sema_init(&genblk_devs[drive].queue_wait, 0);
		genblk_devs[drive].stopped = 0;

		/*
		 * Register block device and gain major number.
		 */
		genblk_smc_name(drive, name, NAME_MAX_LEN);
		major_num = register_blkdev(0, name);
		if(major_num < 1) {
			printk(KERN_ERR "genblk: failed to get major number\n");
			return -EBUSY;
		}

		printk(KERN_NOTICE "genblk: drive %u\n", drive);
		printk(KERN_NOTICE "   block count  %u\n", genblk_devs[drive].blk_cnt);
		printk(KERN_NOTICE "   block size   %lu\n", genblk_devs[drive].blk_sz);
		printk(KERN_NOTICE "   writeable    %u\n", writeable);
		printk(KERN_NOTICE "   queue size   %lu\n", req_queue_sz);
		printk(KERN_NOTICE "   pirq         %lu\n", genblk_devs[drive].hwirq);
		printk(KERN_NOTICE "   virq         %lu\n", irq);
		printk(KERN_NOTICE "   major number %u\n", major_num);
		printk(KERN_NOTICE "   name         \"%s\"\n", name);

		/*
		 * Allocate and setup generic disk structure.
		 */
		if(!(genblk_devs[drive].gd = alloc_disk(GENODE_BLK_MINORS))) {
			unregister_blkdev(major_num, name);
			printk(KERN_ERR "genblk: failed to alloc disc\n");
			return -ENOMEM;
		}
		genblk_devs[drive].gd->major        = major_num;
		genblk_devs[drive].gd->first_minor  = 0;
		genblk_devs[drive].gd->fops         = &genode_blk_ops;
		genblk_devs[drive].gd->private_data = &genblk_devs[drive];
		genblk_devs[drive].gd->queue        = genblk_devs[drive].queue;
		strncpy(genblk_devs[drive].gd->disk_name, name,
		        sizeof(genblk_devs[drive].gd->disk_name));
		capacity =
			genblk_devs[drive].blk_cnt *
			(genblk_devs[drive].blk_sz / KERNEL_SECTOR_SIZE);
		set_capacity(genblk_devs[drive].gd, capacity);

		/* Set it read-only or writeable */
		if (!writeable)
			set_disk_ro(genblk_devs[drive].gd, 1);

		if (drive == 0) { genblk_smc_callback(); }

		/* Make the block device available to the system */
		add_disk(genblk_devs[drive].gd);
	}
	return 0;
}

static void __exit genode_blk_exit(void)
{
	unsigned drive;
	for (drive = 0 ; drive < genblk_dev_cnt; drive++) {
		del_gendisk(genblk_devs[drive].gd);
		put_disk(genblk_devs[drive].gd);
		unregister_blkdev(genblk_devs[drive].gd->major,
		                  genblk_devs[drive].gd->disk_name);
		blk_cleanup_queue(genblk_devs[drive].queue);
	}
}

module_init(genode_blk_init);
module_exit(genode_blk_exit);

MODULE_LICENSE("GPL");
