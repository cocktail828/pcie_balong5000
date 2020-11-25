#ifndef __PCIE_BALONG_DEV_H__
#define __PCIE_BALONG_DEV_H__

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/printk.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <asm-generic/sizes.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include "bsp_pcie.h"
#include "osl_types.h"

#define PCIE_WAIT_UNLOCK_MS (50)
#define PCIE_INTX_REG_OFFSET (0x1C)
#define PCIE_DEV_REMOVE_FLAG (0x0)
#define PCIE_DEV_LINKDOWN_FLAG (0x1)

#define PCIE_INTX_REG_BIT 5

#define PCIE_RC_PR_ERR(fmt, args...)                                                      \
    do                                                                                    \
    {                                                                                     \
        printk(KERN_ERR "[AP_PCIE_KERNEL][%d] %s:" fmt "\n", __LINE__, __func__, ##args); \
    } while (0)

#define PCIE_RC_TEST_PR_ERR(fmt, args...)                                                      \
    do                                                                                         \
    {                                                                                          \
        printk(KERN_ERR "[AP_KERNEL_PCIE_TEST][%d] %s:" fmt "\n", __LINE__, __func__, ##args); \
    } while (0)

enum pcie_rc_event
{
    PCIE_RC_GET_EP_DEV = 0,
    PCIE_RC_GET_FUNC_NUM,
    PCIE_RC_GET_BAR,
    PCIE_RC_INT_INIT,
    PCIE_RC_USER_INIT,
    PCIE_RC_EVENT_MAX
};

struct pcie_rc_debug_info
{
    char *pcie_rc_event_name;
    int init_flag;
};

struct pcie_irq_info
{
    irq_handler_t handler;
    const char *name;
    unsigned long irq_count;
    unsigned long irq_handler_count;
    void *irq_data;
    u32 irq_num;
};

struct pcie_dma_int_info
{
    struct semaphore dma_semaphore;
    pcie_callback dma_int_callback;
    void *dma_int_callback_args;
};

struct pcie_msi_info
{
    u32 msi_addr_h;
    u32 msi_addr_l;
    u32 msi_write_data;
    u32 msi_read_data;
};

struct rc_pcie_info
{
    struct pci_dev *balong_rc;
    struct pci_dev *balong_ep;
    struct pci_dev *balong_multi_ep[PCIE_FUNC_NUM];
    struct pci_saved_state *balong_multi_ep_save[PCIE_FUNC_NUM];
    struct pcie_msi_info rc_dma_msi_info;
    struct pcie_dma_int_info dma_int_info[PCIE_DMA_CHN_NUM][PCIE_DMA_DIRECTION_MAX];
    struct pcie_irq_info irq_chn[PCIE_EP_MSI_NUM];
    struct semaphore init_done_sema;
    spinlock_t spinlock; /* for controller */

    struct wakeup_source wake_lock;
    struct work_struct show_wake_user_work;
    struct work_struct bar_space_error_work;

    u64 rc_send_msi_count[PCIE_RC_MSI_NUM];
    u32 ep_config_space[PCIE_FUNC_NUM][PCIE_CFG_REG_NUM];

    u32 valid_ep_func_num;
    u32 dma_msi_irq;

    u32 ep_vendor_id;    /* EP's vendor id */
    u32 ep_device_id;    /* EP's device id */
    u32 ep_pcie_port_id; /* EP PCIE Port Channel */

    unsigned long ep_in_rc_msi_phys_addr; /* EP GIC MSI Address PCI Physical address((CPU domain) */
    void *ep_in_rc_msi_virt_addr;
    void *ep_msi_usr_virt_addr; /* EP to RC MSI Bar Virt Address */
    void *ep_dma_base_virt_addr;
    u32 ep_dma_base_addr_offset;
    void *ep_pcie_base_virt_addr;
    void *ep_intx_virt_addr;
    irq_handler_t handler;
    u32 dma_channel_state; /* 1 for busy, each bit corresponds to a DMA channel */
    u32 dma_isr_flags[PCIE_DMA_DIRECTION_MAX];
};

struct rc_pcie_flag
{
    unsigned long rm_state;
    u32 init_done;
    u32 remove_start;
    u32 res_init;
    u32 pm_status;
    u32 linkdown_detect;
    u32 first_user_flag;
    u32 first_user_id;
    u32 error_bar_content;
};

extern void *rc_get_normal_bar_virt_addr(enum pcie_bar_id_e bar_index);
extern u32 pcie_get_base_msi_irq_num(void);
void bsp_pcie_rc_linkdown_event_enable(void);
void bsp_pcie_rc_linkdown_event_disable(void);
void bsp_pcie_save_all_ep_cfg(void);
int bsp_pcie_restore_all_ep_cfg(void);
u32 bsp_pcie_get_port_id(void);

#endif
