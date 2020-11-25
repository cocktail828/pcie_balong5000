#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/syscore_ops.h>
#include "securec.h"
#include "pcie_cdev.h"
#include "pcie_cdev_desc.h"
#include "pcie_cdev_rc.h"
#include "pcie_cdev_dbg.h"

extern u64 g_pcdev_dma_mask;
extern struct pcdev_ctx g_pcdev_ctx;
extern struct pcie_cdev_name_type_tbl g_pcie_cdev_type_table[PCIE_CDEV_COUNT];
extern struct pcie_cdev_port_manager g_pcie_cdev_ports[PCIE_CDEV_COUNT];
char *g_pcdev_rc_irq_name = "pcdev_irq";
atomic_t g_pcdev_vote_cnt[vote_modem_bottom];
atomic_t g_pcdev_vote_fail_cnt[vote_modem_bottom];
atomic_t g_pcdev_unvote_cnt[vote_modem_bottom];

void pcdev_vote_dbg(void)
{
    int i;
    for (i = 0; i < vote_modem_bottom; i++)
    {
        PCDEV_ERR("mode(%d) vote_cnt:%d, fail_cnt:%d, unvote_cnt:%d\n",
                  i, atomic_read(&g_pcdev_vote_cnt[i]),
                  atomic_read(&g_pcdev_vote_fail_cnt[i]),
                  atomic_read(&g_pcdev_unvote_cnt[i]));
    }
}

static int pcie_cdev_vote_lock_rc(int mode)
{
    int ret = 0;

    if (in_interrupt())
    {
        PCDEV_ERR("can't send buf in atomic\n");
        WARN_ON_ONCE(1);
        g_pcdev_ctx.vote_in_interrupt++;
        return -EACCES;
    }

    atomic_inc(&g_pcdev_vote_cnt[mode]);
    // if (bsp_pcie_rc_vote_lock(PCIE_USER_CHAR_DEV, 1))
    //     atomic_inc(&g_pcdev_vote_fail_cnt[mode]);

    return ret;
}

static int pcie_cdev_vote_unlock_rc(int mode)
{
    atomic_inc(&g_pcdev_unvote_cnt[mode]);
    //return bsp_pcie_rc_vote_unlock(PCIE_USER_CHAR_DEV);
    return 0; // add by lee
}

static int pcie_cdev_vote_try_lock_rc(int mode)
{
    int ret;

    atomic_inc(&g_pcdev_vote_cnt[mode]);
    ret = bsp_pcie_rc_vote_lock(PCIE_USER_CHAR_DEV, 0);
    if (ret)
        atomic_inc(&g_pcdev_vote_fail_cnt[mode]);

    return ret;
}

int pcdev_rc_port_desc_match(unsigned int port_n)
{
    unsigned int i;
    union pcie_cdev_map *desc = NULL;
    union pcie_cdev_map *desc_addr = NULL;
    struct pcdev_ports_desc *ports_desc = (struct pcdev_ports_desc *)g_pcdev_ctx.virt_addr;
    unsigned int port_num = ports_desc->port_num;

    desc_addr = (union pcie_cdev_map *)(g_pcdev_ctx.virt_addr + sizeof(struct pcdev_ports_desc));
    for (i = 0; i < port_num; i++)
    {
        desc = desc_addr + i;

        if (desc->rc.port_id == g_pcie_cdev_type_table[port_n].type)
        {
            PCDEV_INFO("mhwpcie:rc:desc: %p\n", desc);
            g_pcie_cdev_ports[port_n].desc = desc;
            break;
        }
    }

    if (i >= ports_desc->port_num)
        return -EACCES;

    return 0;
}

static int pcie_cdev_send_irq_rc(void)
{
    return bsp_pcie_rc_send_msi(PCIE_RC_MSI_CHAR_DEV);
}

static irqreturn_t pcie_cdev_irq_handler_rc(int irq, void *dev_info)
{
    PCDEV_TRACE("in\n");
    return g_pcdev_ctx.irq_handler();
}

static int pcie_cdev_rc_init(void)
{
    int ret;

    PCDEV_TRACE("in\n");
    g_pcdev_ctx.phys_addr = (void *)bsp_pcie_rc_get_bar_addr(PCIE_BAR_CHAR_DEV);
    if (g_pcdev_ctx.phys_addr == NULL)
    {
        PCDEV_ERR("cannot resource\n");
        return -ENODEV;
    }

    g_pcdev_ctx.buffer_size = bsp_pcie_rc_get_bar_size(PCIE_BAR_CHAR_DEV);
    if (g_pcdev_ctx.buffer_size < MIN_IO_SIZE)
    {
        PCDEV_ERR("Invalid PCI region size, aborting\n");
        return -ENODEV;
    }

    PCDEV_TRACE("mhwpcie:rc:PCIE_BAR_CHAR_DEV phys_add:%p,size:%lx\n",
                g_pcdev_ctx.phys_addr, g_pcdev_ctx.buffer_size);

    g_pcdev_ctx.virt_addr = ioremap((unsigned long)g_pcdev_ctx.phys_addr,
                                    g_pcdev_ctx.buffer_size);
    if (g_pcdev_ctx.virt_addr == NULL)
    {
        PCDEV_ERR("cannot map\n");
        return -ENODEV;
    }

    PCDEV_TRACE("mhwpcie:rc:PCIE_BAR_CHAR_DEV virt_addr:%p", g_pcdev_ctx.virt_addr);
    ret = bsp_pcie_rc_msi_request(PCIE_EP_MSI_CHAR_DEV, pcie_cdev_irq_handler_rc,
                                  g_pcdev_rc_irq_name, NULL);
    if (ret)
    {
        PCDEV_ERR("irq request fail\n");
        ret = -ENODEV;
        return ret;
    }

    ret = bsp_pcie_rc_msi_enable(PCIE_EP_MSI_CHAR_DEV);
    if (ret)
    {
        PCDEV_ERR("irq enable fail\n");
        return -ENODEV;
    }

    g_pcdev_ctx.send_irq = pcie_cdev_send_irq_rc;

    PCDEV_TRACE("finish \n");

    return ret;
}

static void pcie_cdev_rc_exit(void)
{
    PCDEV_TRACE("in\n");

    bsp_pcie_rc_msi_free(PCIE_EP_MSI_CHAR_DEV);
    if (g_pcdev_ctx.virt_addr != NULL)
        iounmap(g_pcdev_ctx.virt_addr);

    g_pcdev_ctx.phys_addr = 0;
    g_pcdev_ctx.buffer_size = 0;
    g_pcdev_ctx.virt_addr = 0;

    PCDEV_TRACE("finish\n");
    return;
}

static int pcdev_pltform_probe(struct platform_device *pdev)
{
    PCDEV_TRACE("in\n");
    dma_set_mask_and_coherent(&pdev->dev, g_pcdev_dma_mask);
    g_pcdev_ctx.pdev = pdev;

    return 0;
}

static const struct of_device_id g_pcdev_match[] = {
    {.compatible = "hisilicon,pcdev_app"},
    {},
};

static struct platform_driver g_pcdev_pltfm_driver = {
    .probe = pcdev_pltform_probe,
    .driver = {
        .name = "pcdev",
        .of_match_table = g_pcdev_match,
    },
};

#define print_buff_size 1024
static char g_pmprint_buf[print_buff_size];
static int pcie_cdev_rc_cb(u32 usr_id, u32 cb_id, void *callback_args)
{
    int ret = 0;
    int i;
    unsigned int cnt = 0;

    switch (cb_id)
    {
    case PCIE_RC_CB_ENUM_DONE:
        for (i = 0; i < vote_modem_bottom; i++)
        {
            atomic_set(&g_pcdev_vote_cnt[i], 0);
            atomic_set(&g_pcdev_vote_fail_cnt[i], 0);
            atomic_set(&g_pcdev_unvote_cnt[i], 0);
        }
        ret = pcdev_init_cb();
        break;

    case PCIE_RC_CB_SUSPEND:
        memset_s(g_pmprint_buf, print_buff_size, 0, print_buff_size);

        cnt += snprintf_s((char *)g_pmprint_buf,
                          (size_t)(print_buff_size - 1),
                          "[C SR][PCDEV]( up )vote port:");
        for (i = 0; i < PCIE_CDEV_COUNT; i++)
        {
            cnt += snprintf_s((char *)g_pmprint_buf + cnt,
                              (size_t)(print_buff_size - cnt - 1),
                              "[%d]:%lld, ", i,
                              g_pcdev_ctx.vote_dbg[i].vote_port);
        }
        cnt += snprintf_s((char *)g_pmprint_buf + cnt,
                          (size_t)(print_buff_size - cnt - 1), "\n");
        PCDEV_INFO("%s\n", g_pmprint_buf);
        break;

    case PCIE_RC_CB_RESUME:
        memset_s(g_pmprint_buf, print_buff_size, 0, print_buff_size);

        cnt += snprintf_s((char *)g_pmprint_buf,
                          (size_t)(print_buff_size - 1),
                          "[C SR][PCDEV](down)vote port:");
        for (i = 0; i < PCIE_CDEV_COUNT; i++)
        {
            cnt += snprintf_s((char *)g_pmprint_buf + cnt,
                              (size_t)(print_buff_size - cnt - 1),
                              "[%d]:%lld, ", i,
                              g_pcdev_ctx.vote_dbg[i].vote_port);
        }
        cnt += snprintf_s((char *)g_pmprint_buf + cnt,
                          (size_t)(print_buff_size - cnt - 1), "\n");
        PCDEV_INFO("%s", g_pmprint_buf);
        break;

    case PCIE_RC_CB_EXIT:
        pcdev_exit();
        break;

    default:
        break;
    }
    return ret;
}

static int pcdev_clr_vote_flag(void)
{
    g_pcdev_ctx.vote_flag = 0xFFFF;
    return 0;
}

static void pcdev_set_vote_flag(void)
{
    g_pcdev_ctx.vote_flag = VOTE_FLAG;
}

static struct syscore_ops g_pcdev_ops = {
    .suspend = pcdev_clr_vote_flag,
    .resume = pcdev_set_vote_flag,
};

void pcdev_bsp_slice_getcurtime(u64 *pcurtime)
{
    if (bsp_slice_getcurtime(pcurtime))
        PCDEV_ERR("bsp_slice_getcurtime err\n");

    return;
}

int pcie_cdev_platform_rc_init(void)
{
    int ret = 0;
    struct pcie_callback_info pcie_cdev_callback_info = {0};

    printk(KERN_ERR "[pcdev]pcie_cdev_platform_rc_init in\n");

    memset_s(g_pcie_cdev_ports, sizeof(g_pcie_cdev_ports), 0,
             sizeof(struct pcie_cdev_port) * PCIE_CDEV_COUNT);

    memset_s(&g_pcdev_ctx, sizeof(g_pcdev_ctx), 0, sizeof(struct pcdev_ctx));

    g_pcdev_ctx.work_mode = PCIE_WORK_MODE_RC;
    g_pcdev_ctx.pcie_id = 0;
    g_pcdev_ctx.msg_level = PCDEV_LEVEL_ERR ||
                            PCDEV_LEVEL_WARN ||
                            PCDEV_LEVEL_TRACE ||
                            PCDEV_LEVEL_INFO ||
                            PCDEV_LEVEL_DBG;
    g_pcdev_ctx.print_port = 1;
    g_pcdev_ctx.pcdev_hw_init = pcie_cdev_rc_init;
    g_pcdev_ctx.pcdev_hw_exit = pcie_cdev_rc_exit;
    g_pcdev_ctx.pcdev_vote_lock = pcie_cdev_vote_lock_rc;
    g_pcdev_ctx.pcdev_vote_unlock = pcie_cdev_vote_unlock_rc;
    g_pcdev_ctx.pcdev_vote_try_lock = pcie_cdev_vote_try_lock_rc;
    g_pcdev_ctx.get_curtime = pcdev_bsp_slice_getcurtime;
    g_pcdev_ctx.pcie_first_user = bsp_is_pcie_first_user;

    PCDEV_ERR("init begin\n");

    pcdev_initwork_init();

    pcie_cdev_callback_info.callback = pcie_cdev_rc_cb;
    pcie_cdev_callback_info.callback_args = NULL;

    ret = bsp_pcie_rc_cb_register(PCIE_USER_CHAR_DEV, &pcie_cdev_callback_info);
    if (ret)
        PCDEV_ERR("pcie cb regiest fail\n");

    //bsp_pcie_rc_cb_run(PCIE_RC_CB_ENUM_DONE);
    bsp_pcie_rc_cb_run_with_id(PCIE_USER_CHAR_DEV, PCIE_RC_CB_ENUM_DONE);
    ret = platform_driver_register(&g_pcdev_pltfm_driver);
    if (ret)
    {
        PCDEV_ERR("platform_driver_register err\n");
        return ret;
    }

    register_syscore_ops(&g_pcdev_ops);
    PCDEV_TRACE("pcie_cdev_platform_rc_init finish\n");
    return ret;
}

#ifndef CONFIG_HISI_BALONG_MODEM_MODULE
device_initcall(pcie_cdev_platform_rc_init);
MODULE_LICENSE("GPL");

#endif
