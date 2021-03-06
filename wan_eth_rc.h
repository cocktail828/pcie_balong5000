#ifndef __WAN_ETH_EP_H__
#define __WAN_ETH_EP_H__

#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/version.h>

#include "mdrv_remote_eth.h"
#include "securec.h"
#include "bsp_pcie.h"
#include "wan_eth_event_buffer.h"
#include "wan_eth_table.h"

#define WETH_MAX_RX_NODE 2048
#define WETH_MAX_CPU_NUM 32
#define WETH_VOTE_TIMEOUT (100) /* ms unit */
#define WETH_NO_PKG_COUNT 15
#define WETH_VOTE_PROTECT_NUM 1024
#define WETH_CHN_NUM NR_CPUS
#define WETH_TX_CALC_TIME 10
#define WETH_MAX_NAP_QLEN 10240
#define WETH_RX_NUM 1024

struct rx_smp_node
{
    struct list_head list;
    int idx;
    struct rx_pcpu_ctx *pcpu;
    struct weth_ctx *eth_priv;
    struct sk_buff *skb_tail;
    unsigned int skb_num;
};

struct rx_pcpu_ctx
{
    struct list_head list;
    unsigned int cur_node_num;
    unsigned int cpu;
    unsigned int qlen_weight;
    unsigned int qlen_order;
    int cpu_run_cnt;
    int cpu_min_len;
    atomic_t qlen_list;
    struct notifier_block cpu_hotplug_notif;
    unsigned long state;
    struct tasklet_struct rx_list_push;
    struct tasklet_struct rx_pkg_push;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
    struct __call_single_data csd ____cacheline_aligned_in_smp;
#else
    struct call_single_data csd ____cacheline_aligned_in_smp;
#endif
    unsigned int stat_smp_call;
    unsigned int stat_direct_call;
};

struct weth_global_ctx
{
    unsigned int cur_cpu;
    unsigned int qlen_cmp_weight;
    unsigned int cpu_start;
    unsigned int cpu_end;
    int cpu_run_max;

    int debug_level;
    int debug_performance;

    unsigned int stat_smp_call_fail;
    unsigned int stat_smp_func_fail;
    unsigned int stat_max_find_cnt;
    unsigned int stat_max_find_idx;
    unsigned int stat_rx_packets_push;
    unsigned int stat_devid_mask_cnt;
    unsigned int stat_cpu_not_online;
    unsigned int stat_cpu_run_max;
    unsigned int stat_find_cpu_fail;
    unsigned int stat_find_cpu_ok;
    unsigned int stat_find_min_len_fail;
    unsigned int tx_timer;

    unsigned int cur_node_num;
    struct rx_smp_node rx_pcpu_node[WETH_MAX_RX_NODE];
    struct rx_pcpu_ctx *pcpu_debug[WETH_MAX_CPU_NUM];
    struct list_head rx_pcpu_head;
    spinlock_t rx_pcpu_lock;
};

struct weth_stats
{
    unsigned int stat_open;
    unsigned int stat_close;
    unsigned int stat_xmit_packets;
    unsigned int stat_len_err;
    unsigned int stat_liner_err;
    unsigned int stat_tx_timeout;
    unsigned int stat_set_mtu;
    unsigned int stat_set_mac;
    unsigned int stat_rx_done;
    unsigned int stat_poll;
    unsigned int stat_poll_none;
    unsigned int stat_tx_none;
    unsigned int stat_max_tx_num;
    unsigned int stat_cur_tx_num;
    unsigned int stat_max_rx_num;
    unsigned int stat_cur_rx_num;
    unsigned int stat_no_cmp_node;

    unsigned int stat_pcie_vote_fail;
    unsigned int stat_pcie_vote_work;

    unsigned int stat_tx_first_enq;
    unsigned int stat_tx_tail_enq;
    unsigned int stat_tx_submit_fail;

    unsigned int stat_rx_invalid_devid;
    unsigned int stat_rx_drop_by_invalid_devid;
    unsigned long long stat_rx_packets;
    unsigned long long stat_rx_bytes;
    unsigned long long stat_rx_errors;
    unsigned long long rx_packets_push;
    unsigned long long devid_mask;

    unsigned long long stat_tx_packets;
    unsigned long long stat_tx_bytes;
    unsigned long long stat_tx_errors;
    unsigned long long stat_tx_dropped;
    unsigned long long stat_tx_queue;
    unsigned long long tx_timer_start;
    unsigned long long tx_timer_cancel;
    unsigned long long tx_timer_running;
    unsigned long long force_schedule;
};

struct weth_chn
{
    struct napi_struct napi;
    struct sk_buff *rx_napi_skb;
    spinlock_t rx_napi_lock;
    struct weth_ctx *pctx;
    unsigned int rx_qlen;
    unsigned int extra_qlen;
    int last_sum_num;
    unsigned long last_jiffies;
};

struct weth_ctx
{
    struct net_device *ndev;
    spinlock_t rx_smp_lock;
    unsigned int devid;
    spinlock_t tx_lock;
    spinlock_t rx_lock;

    unsigned int rx_wpos;
    unsigned int rx_rpos;
    unsigned int rx_num;
    struct sk_buff *rx_skb[WETH_MAX_RX_NODE];
    unsigned int rx_pkg[WETH_MAX_RX_NODE];

    struct tasklet_struct xmit_push;
    struct delayed_work xmit_work;

    struct weth_chn chn[WETH_CHN_NUM];

    unsigned int tx_list_len;
    unsigned int rx_list_len;
    struct sk_buff *tx_tail_skb;
    struct sk_buff *rx_tail_skb;
    struct sk_buff *rx_skb_handle;

    struct hrtimer tx_timer;
    unsigned int tx_timer_duration;
    unsigned int max_list_len;
    unsigned long tx_last_jiff;

    int is_open;
    int gro_enable;
    struct weth_stats stats;
};

/* *********************Macro definition************************************** */
extern struct weth_global_ctx g_weth_global;
#define WETH_MUX_NUM 30

static inline unsigned int __skb_get_devid(struct sk_buff *skb)
{
    struct remote_eth_cb_map_s *map_info;

    map_info = (struct remote_eth_cb_map_s *)skb->cb;
    return (map_info->userfield0 & 0xFFFF);
}

/* *********************Macro definition****************************************** */
/* debug mode include */

#define PCIE_NIC_BAR_SIZE BAR_SIZE_256K
#define PCIE_NIC_BAR_MASK BAR_SIZE_256K
#define PCIE_NIC_BAR_NUM PCIE_BAR_ETH_DEV

#define MASK_16_BITS ((1 << 16) - 1)
#define MASK_8_BITS ((1 << 8) - 1)

#define ALIGN_64K(addr) (void *)(((addr) & ~PCIE_NIC_BAR_MASK) + PCIE_NIC_BAR_SIZE + 1)

/* skb buff MTU definition */
#define WETH_DMA_MASK 0xffffffffffffffff

#define NUM_R2E_DESC 4096
#define MIN_IO_SIZE 0x80

#define WETH_ETH_TYPE_IP 0x0008
#define WETH_ETH_TYPE_IPV6 0xDD86

/* ******************Structure definition********************************************* */

union weth_pcie_link_status
{
    struct
    {
        unsigned int open : 1;
        unsigned int rsv : 31;
    } bits;
    unsigned int bit32;
};

struct weth_map_ep
{
    union weth_pcie_link_status self_status;
    union weth_pcie_link_status peer_status;
    unsigned int enable_local_irq;
    unsigned int enable_remote_irq;

    /* add for table */
    struct weth_tab_addr_table_rc addr_table_rc;
    struct weth_tab_addr_table_ep addr_table_ep;

    /* event buffer */
    struct event_buffer tx_event;
    struct event_buffer rx_event;

} __attribute__((aligned(4)));

struct weth_map_rc
{
    union weth_pcie_link_status peer_status;
    union weth_pcie_link_status self_status;
    unsigned int enable_remote_irq;
    unsigned int enable_local_irq;

    /* add for table */
    struct weth_tab_addr_table_rc addr_table_rc;
    struct weth_tab_addr_table_ep addr_table_ep;

    /* event buffer */
    struct event_buffer rx_event;
    struct event_buffer tx_event;

} __attribute__((aligned(4)));

union weth_map
{
    struct weth_map_ep ep;
    struct weth_map_rc rc;
};

struct weth_rc_init_status
{
    unsigned int bar_init_success;
    unsigned int bar_config_fail;
    unsigned int dma_ll_init_fail;
    unsigned int dma_ll_addr_alloc_fail;
    unsigned int dma_ll_addr_init_success;
    unsigned int msi_intr_init_fail;
    unsigned int pcie_desc_init_fail;
    unsigned int peth_desc_ep_init_fail;
    unsigned int weth_rc_init_fail;
    unsigned int pcie_dma_callback_register_fail;
    unsigned int tab_init_fail;
    unsigned int event_init_fail;
};

struct weth_rc_tx_status
{
    unsigned long long rc_submit_tx;
    unsigned long long rc_link_not_up;
    unsigned long long rc_submit_skb_null;
    unsigned long long start_send;
    unsigned long long doing;
    unsigned long long unhandle;
    unsigned long long pending;
    unsigned long long dma_send;
    unsigned long long dma_send_packets;
    unsigned long long dma_send_bytes;
    unsigned long long dma_transfer_fail;
    unsigned long long dma_list_complete;
    unsigned long long dma_list_uncomplete;
    unsigned long long skb_pending;
    unsigned long long remote_eth_fail;
    unsigned long long remote_eth_drop;
    unsigned long long skb_free;
    unsigned long long send_data_complete;
    unsigned long long skb_pending_free;
    unsigned long long dma_write_callback;
    unsigned long long msi_send;
    unsigned long long no_capacity;
    unsigned long long send_pending;
    unsigned long long reschedule;
    unsigned long long data_send_irq;
    unsigned long long tx_queue_full;
    unsigned long long dma_send_pending;
    unsigned long long skb_free_due_to_tdfull;
    unsigned long long send_pending_write_cb;
    unsigned long long send_pending_resend;
    unsigned long long dma_resend;
    unsigned int phy_addr_null_drop;

    unsigned int debug_pkt_xmit;
    unsigned int debug_pkt_xmit_succ;
};

struct weth_rc_rx_status
{
    unsigned long long dma_rx_start;
    unsigned long long packets_received;
    unsigned long long bytes_received;
    unsigned long long peer_dma_tx_complete;
    unsigned long long rx_done_cb;
    unsigned long long dma_read_callback;
    unsigned long long msi_send;
    unsigned long long no_packets_received;
    unsigned long long data_read_irq;
    unsigned long long rx_try;
    unsigned long long rx_skb_refill_fail;
    unsigned long long ep_td_available;
    unsigned long long rx_buget_reach;
    unsigned long long ep_td_available_rx_buget;
    unsigned long long rx_queue_full;
    unsigned long long total_rx_packets;
    unsigned long long total_rx_bytes;
};

struct weth_rc_pm_status
{
    unsigned int suspend_fail;
    unsigned int suspend_sucess;
    unsigned int resume_sucess;
};

struct weth_rc_ctx
{
    /* kernel dev */
    struct platform_device *pdev;

    /* resource lock */
    spinlock_t rx_lock;
    spinlock_t tx_lock;

    /* netdev status */
    union weth_pcie_link_status *local_stat;
    union weth_pcie_link_status *remote_stat;

    unsigned int rx_wpos;
    unsigned int rx_rpos;
    unsigned int rx_num;
    unsigned int invalid_devid;
    unsigned int not_opened;
    struct sk_buff *rx_skb[NUM_R2E_DESC];

    /* pcie link bar */
    unsigned int init;
    unsigned int link;
    union weth_map *desc;

    /* address */
    void *phys_addr;
    void *org_phys_addr;
    void *virt_addr;
    unsigned int buffer_size;
    struct semaphore weth_rc_init_sem;
    struct tasklet_struct send_pending_packets;

    struct tasklet_struct tx_push;
    struct hrtimer tx_timer;
    struct workqueue_struct *xmit_queue;
    struct delayed_work xmit_work;
    struct timer_list vote_timer;
    struct platform_driver *rc_platform_driver;

    struct sk_buff *tx_tail_skb_all;
    unsigned int tx_list_len_all;
    spinlock_t tx_vote_lock;

    int dynamic_napi_weight;
    unsigned int tx_timer_duration;
    unsigned int max_list_len;
    unsigned int init_weight;
    unsigned int first_weight;
    unsigned int second_weight;
    unsigned int third_weight;

    unsigned int first_weight_thru;
    unsigned int second_weight_thru;
    unsigned int third_weight_thru;

    unsigned int no_data_duration;
    unsigned int no_data_jiffies;

    unsigned int evaluate_duration;
    unsigned int evaluate_jiffies;

    unsigned int init_change;
    unsigned int first_change;
    unsigned int second_change;
    unsigned int third_change;

    unsigned int is_wakeup;
    unsigned int stat_wait_vote_call;
    unsigned int stat_wait_vote_fail;
    unsigned int stat_nowait_vote_call;
    unsigned int stat_nowait_vote_fail;
    unsigned int stat_unvote_call;
    unsigned int stat_unvote_fail;

    unsigned int stat_vote_fail_free;
    unsigned int stat_work_no_skb;
    unsigned int stat_work_protect;
    int first_resume;
    unsigned int first_resume_called;
    unsigned int real_resume;

    int vote_lock_opt;
    int deep_sleep_vote;
    unsigned int last_total_tx_pkg;
    unsigned int cur_total_tx_pkg;
    unsigned int last_total_rx_pkg;
    unsigned int cur_total_rx_pkg;
    unsigned long long no_pkg_count;
    unsigned int trans_report_enable;

    /* debug stats */
    struct weth_rc_init_status init_stats;
    struct weth_rc_tx_status tx_stats;
    struct weth_rc_rx_status rx_stats;
    struct weth_rc_pm_status pm_stats;
};

#endif
