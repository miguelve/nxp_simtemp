#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/ktime.h>
#include <linux/ioctl.h>
#include <linux/poll.h>

#define DEVICE_NAME "nxp_simtemp"
#define CLASS_NAME  "simtemp"

#define DEFAULT_SAMPLING_MS   1000   // Default execution period in milliseconds
#define MAX_SAMPLING_MS      10000   // Max sampling period in ms.
#define MIN_SAMPLING_MS        100   // Min sampling period in ms.
#define DEFAULT_HI_THRES    100000   // Default HI threshold
#define DEFAULT_LO_THRES    -20000   // Execution period in milliseconds
#define SAMPLES_NUM 1   // Max number of samples in buffer

#define NEW_SAMPLE_BIT              (1<<0)
#define LOW_THRESHOLD_CROSSED_BIT   (1<<1)
#define HI_THRESHOLD_CROSSED_BIT    (1<<2)

#define IOCTL_MAGIC  'x' // Unique identifier (must be unique per driver)

/* Example IOCTL command definitions */
#define IOCTL_SET_SAMPLING_MS   _IOW(IOCTL_MAGIC, 0, int)
#define IOCTL_SET_HI_THRESHOLD  _IOW(IOCTL_MAGIC, 1, int)
#define IOCTL_SET_LO_THRESHOLD  _IOW(IOCTL_MAGIC, 2, int)

static struct class*  simtemp_class  = NULL;

struct simtemp_sample {

    __u64 timestamp_ns;   // monotonic timestamp

    __s32 temp_mC;        // milli-degree Celsius (e.g., 44123 = 44.123 °C)

    __u32 flags;          // bit0=NEW_SAMPLE, bit1=THRESHOLD_CROSSED (extend as needed)

} __attribute__((packed));

typedef struct simtemp_sample simtemp_smp_t;

struct simtemp_dev {
    struct cdev cdev;
    dev_t devt;
    simtemp_smp_t sample_buff[SAMPLES_NUM];
    __u32 last_sample_idx;
    __u32 sampling_ms;
    __s32 hi_threshold;
    __s32 lo_threshold;
    struct task_struct *thread_st;
    wait_queue_head_t read_queue;
    bool data_ready;                 // flag to indicate data availability
};

static void push_sample(struct simtemp_dev *dev)
{
    simtemp_smp_t* p_sample;
    /* get pointer to last sample */
    p_sample = &(dev->sample_buff[dev->last_sample_idx]);
    p_sample->temp_mC = 24000;
    p_sample->flags |= NEW_SAMPLE_BIT;
    /* Add timestamp */
    p_sample->timestamp_ns = ktime_get_ns();

    /* TODO: Update last_sample_idx if storing multiple samples */

    /* Notify there is new data available */
    wake_up_interruptible(&dev->read_queue);
}

static simtemp_smp_t* pop_last_sample(struct simtemp_dev *dev)
{
    simtemp_smp_t* p_last_sample;
    /* get pointer to last sample */
    p_last_sample = &(dev->sample_buff[dev->last_sample_idx]);

    return p_last_sample;
}

/* This function generates a new sample at a specified rate */
static int simtemp_fn(void *data)
{
    struct simtemp_dev *dev = (struct simtemp_dev *)data;
    pr_info("simtemp_fn: started\n");

    while (!kthread_should_stop()) {
        push_sample(dev);

        /* Sleep for specified rate */
        msleep(dev->sampling_ms);
    }

    pr_info("simtemp_fn: stopping\n");
    return 0;
}

/* ---------- File Operations ---------- */

static int simtemp_open(struct inode *inodep, struct file *filep)
{
    struct simtemp_dev *dev;

    // Get the pointer to the containing struct from the cdev
    dev = container_of(inodep->i_cdev, struct simtemp_dev, cdev);

    // Store the entire device structure in private_data
    filep->private_data = dev;
    pr_info("%s: device opened\n", DEVICE_NAME);
    return 0;
}

static int simtemp_release(struct inode *inodep, struct file *filep)
{
    pr_info("%s: device closed\n", DEVICE_NAME);
    return 0;
}

static ssize_t simtemp_read(struct file *filep, char __user *buffer, size_t len, loff_t *offset)
{
    int bytes_read = 0;
    struct simtemp_dev *dev = filep->private_data;
    simtemp_smp_t* p_simtemp_sample = pop_last_sample(dev);

    /* If no data yet, block until it’s available */
    if ((p_simtemp_sample->flags & NEW_SAMPLE_BIT) == 0) {
        if (filep->f_flags & O_NONBLOCK)
            return -EAGAIN;

        if (wait_event_interruptible(dev->read_queue, (p_simtemp_sample->flags & NEW_SAMPLE_BIT) != NEW_SAMPLE_BIT))
            return -ERESTARTSYS;
    }

    if (copy_to_user(buffer, p_simtemp_sample, sizeof(simtemp_smp_t)) != 0)
        return -EFAULT;

    /* Clear new sample bit */
    p_simtemp_sample->flags &= !NEW_SAMPLE_BIT;

    bytes_read = sizeof(simtemp_smp_t);
    pr_info("%s: sent %d bytes to user\n", DEVICE_NAME, bytes_read);
    return bytes_read;
}

static ssize_t simtemp_write(struct file *filep, const char __user *buffer, size_t len, loff_t *offset)
{
    /* Write funtion not implemented */
    return 0;
}

/* ---------- IOCTL handler ---------- */
static long simtemp_unlocked_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    int tmp = 0;
    struct simtemp_dev *dev = filep->private_data;

    switch (cmd) {
    case IOCTL_SET_SAMPLING_MS:
        if (copy_from_user(&tmp, (int __user *)arg, sizeof(tmp)))
            return -EFAULT;
        pr_info("%s: IOCTL_SET_SAMPLING_MS = %d\n", DEVICE_NAME, tmp);
        dev->sampling_ms = tmp;
        break;

    case IOCTL_SET_HI_THRESHOLD:
        if (copy_to_user((int __user *)arg, &tmp, sizeof(tmp)))
            return -EFAULT;
        pr_info("%s: IOCTL_SET_HI_THRESHOLD = %d\n", DEVICE_NAME, tmp);
        dev->hi_threshold = tmp;
        break;

    case IOCTL_SET_LO_THRESHOLD:
        if (copy_to_user((int __user *)arg, &tmp, sizeof(tmp)))
            return -EFAULT;
        pr_info("%s: IOCTL_SET_LO_THRESHOLD = %d\n", DEVICE_NAME, tmp);
        dev->lo_threshold = tmp;
        break;

    default:
        return -ENOTTY; // Command not supported
    }

    return 0;
}

/* ---------- poll() handler ---------- */
/* This function notifies when there is a new sample.
    FUTURE ENHANCEMENT: Notify when there is unread data in
    the sampling buffer */
static __poll_t simtemp_poll(struct file *filep, poll_table *wait)
{
    __poll_t mask = 0;
    simtemp_smp_t* p_simtemp_sample;
    struct simtemp_dev *dev = filep->private_data;

    poll_wait(filep, &dev->read_queue, wait);
    p_simtemp_sample = pop_last_sample(dev);

    if((p_simtemp_sample->flags & NEW_SAMPLE_BIT) == NEW_SAMPLE_BIT)
        mask |= POLLIN | POLLRDNORM;  /* Readable */

    /* Clear new sample bit */
    p_simtemp_sample->flags &= !NEW_SAMPLE_BIT;

    return mask;
}

/* --------Sysfs: /sys/class/nxp_simtemp/sampling --------- */

/* show(): read current sampling configuration */
static ssize_t sampling_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
    struct simtemp_dev *sim_temp_dev_p = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", sim_temp_dev_p->sampling_ms);
}

/* store(): write new sampling configuration */
static ssize_t sampling_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    struct simtemp_dev *sim_temp_dev_p = dev_get_drvdata(dev);
    int val;
    dev_info(dev, "trying updating sampling interval\n");
    if (kstrtoint(buf, 10, &val))
        return -EINVAL;
    if(val < MIN_SAMPLING_MS || val > MAX_SAMPLING_MS)
        return -EINVAL;
        
    sim_temp_dev_p->sampling_ms = val;
    dev_info(dev, "Updated sampling interval: %d ms\n", val);
    
    return count;
}

static DEVICE_ATTR_RW(sampling);


/* ---------- File Operations Structure ---------- */

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = simtemp_open,
    .release = simtemp_release,
    .read = simtemp_read,
    .write = simtemp_write,
    .poll = simtemp_poll,
    .unlocked_ioctl = simtemp_unlocked_ioctl,
};

static int simtemp_probe(struct platform_device *pdev)
{
    struct simtemp_dev *dev;
    u32 dev_index = 0;  // default index
    int ret;

    pr_info("%s: device probed: name=%s\n", DEVICE_NAME, pdev->name);

    // Allocate per-device structure
    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
    {
        return -ENOMEM;
    }

    // Read 'dev-index' from device tree
    if (pdev->dev.of_node)
    {
        ret = of_property_read_u32(pdev->dev.of_node, "dev-index", &dev_index);
        if (ret)
            pr_warn("mydriver: 'dev-index' property missing, using default 0\n");
    }

    // Allocate a major number dynamically
    ret = alloc_chrdev_region(&dev->devt, 0, 1, DEVICE_NAME);
    if (ret < 0)
    {
        pr_err("%s: failed to allocate major number\n", DEVICE_NAME);
        return ret;
    }

    // Initialize cdev
    cdev_init(&dev->cdev, &fops);
    dev->cdev.owner = THIS_MODULE;
    ret = cdev_add(&dev->cdev, dev->devt, 1);
    if (ret < 0) {
        pr_err("Failed to add cdev\n");
        unregister_chrdev_region(dev->devt, 1);
        return ret;
    }

    // Create device class if not already created
    if (!simtemp_class)
        simtemp_class = class_create(DEVICE_NAME);

    // Create device
    device_create(simtemp_class, &pdev->dev, dev->devt, NULL, "%s%d",DEVICE_NAME,dev_index);

    dev->last_sample_idx = 0;
    dev->sampling_ms = DEFAULT_SAMPLING_MS;
    dev->hi_threshold = DEFAULT_HI_THRES;
    dev->lo_threshold = DEFAULT_LO_THRES;
    init_waitqueue_head(&dev->read_queue);
    dev->data_ready = false; // initially no data
    
    // Create /sys/devices/platform/nxp_simtemp/sampling
    ret = device_create_file(&pdev->dev, &dev_attr_sampling);
    if (ret) {
        dev_err(&pdev->dev, "Failed to create sysfs attribute\n");
        return ret;
    }

    dev->thread_st = kthread_run(simtemp_fn, (void*)dev, "simtemp%d",dev_index);
    if (IS_ERR(dev->thread_st)) {
        pr_err("mythread: failed to create thread\n");
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, dev);
    pr_info("%s: device %s initialized with dev-index=%d\n", DEVICE_NAME, pdev->name, dev_index);
    return 0; // 0 means success
}

static void simtemp_remove(struct platform_device *pdev)
{
    struct simtemp_dev *dev = platform_get_drvdata(pdev);
    
    device_remove_file(&pdev->dev, &dev_attr_sampling);

    if (dev->thread_st) {
        kthread_stop(dev->thread_st);
        pr_info("mythread: stopped\n");
    }

    class_destroy(simtemp_class);
    device_destroy(simtemp_class, dev->devt);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devt, 1);
    pr_info("%s: device removed\n",DEVICE_NAME);
}

static const struct of_device_id simtemp_of_match[] = {
    { .compatible = "nxp_simtemp" },
    {                             }
};

// Platform driver structure
static struct platform_driver simtemp_driver = {
    .probe = simtemp_probe,
    .remove = simtemp_remove,
    .driver = {
        .name = "nxp_simtemp",
        .owner = THIS_MODULE,
        .of_match_table = simtemp_of_match,
    },
};

// Register the driver
module_platform_driver(simtemp_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miguel Velazquez");
MODULE_DESCRIPTION("Sim Temperature Device Driver");
