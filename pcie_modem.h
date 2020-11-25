#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/skbuff.h>

#define PCDEV_MDM_COUNT 1
#define PCDEV_MDM_CONSOLE_IDX 0
#define PCDEV_MDM_NAME_MAX 64
#define PCDEV_MDM_DFT_RD_BUF_SIZE 2048
#define MAX_MPS_SIZE 1024

struct gs_port_stats
{
    unsigned long write_async_call;
    unsigned long write_bytes;
    unsigned long tx_inv_param;
    unsigned long tx_done;
    unsigned long tx_done_bytes;
    unsigned long tx_done_fail;

    unsigned long get_buf_call;
    unsigned long ret_buf_call;
};

struct gs_pcdev_mdm_port
{
    char *name;
    spinlock_t port_lock;
    void *fd;
    struct sk_buff_head skb_list;

    unsigned int port_num;
    unsigned open_count;
    bool openclose;
    pcdev_evt_e event_type;

    pcdev_write_done_cb_t write_done_cb;

    unsigned int read_buf_size;
    struct gs_port_stats pstats;
};

struct pcdev_mdm_port_manager
{
    unsigned int is_alloc;
    struct mutex open_close_lock; /* protect open/close */
    struct gs_pcdev_mdm_port *port;
    struct device *cdev;
    //acm_modem_rel_ind_cb_t rel_ind_cb;
    unsigned int rel_ind_cb_cnt;
};

/* cdev driver */
struct u_modem_driver
{
    struct kref kref; /* Reference management */
    struct cdev cdev;
    dev_t dev_no;
    struct module *owner;
    const char *driver_name;
    const char *name;
    int name_base;            /* offset of printed name */
    unsigned int major;       /* major device number */
    unsigned int minor_start; /* start of minor device number */
    int num;                  /* number of devices allocated */

    struct workqueue_struct *pmdm_work_queue;
};

long pcdev_mdm_ioctl(struct file *file, unsigned int cmd, unsigned long arg);