/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2012-2015. All rights reserved.
 * foss@huawei.com
 *
 * If distributed as part of the Linux kernel, the following license terms
 * apply:
 *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 and
 * * only version 2 as published by the Free Software Foundation.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * * GNU General Public License for more details.
 * *
 * * You should have received a copy of the GNU General Public License
 * * along with this program; if not, write to the Free Software
 * * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Otherwise, the following license terms apply:
 *
 * * Redistribution and use in source and binary forms, with or without
 * * modification, are permitted provided that the following conditions
 * * are met:
 * * 1) Redistributions of source code must retain the above copyright
 * *    notice, this list of conditions and the following disclaimer.
 * * 2) Redistributions in binary form must reproduce the above copyright
 * *    notice, this list of conditions and the following disclaimer in the
 * *    documentation and/or other materials provided with the distribution.
 * * 3) Neither the name of Huawei nor the names of its contributors may
 * *    be used to endorse or promote products derived from this software
 * *    without specific prior written permission.
 *
 * * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __PCIE_BALONG_H__
#define __PCIE_BALONG_H__

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/time.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/errno.h>
#include <asm-generic/sizes.h>

#include "hi_pcie_interface.h"
#include "bsp_pcie.h"
#include "mdrv_errno.h"
#include "bsp_dump.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ******************************** Definition ********************************* */
/* ****** work mode ********* */
/* ep mode */
#define PCIE_WORK_MODE_EP (0x0)
/* legacy ep mode */
#define PCIE_WORK_MODE_LEP (0x1)
/* rc mode */
#define PCIE_WORK_MODE_RC (0x4)

/* ****** link mode ********* */
#define PCIE_LINK_MODE_X1 (0x1)
#define PCIE_LINK_MODE_X2 (0x2)

#define pcie_trace(format, args...)                           \
    do                                                        \
    {                                                         \
        printk("[PCIE]:line: %d: " format, __LINE__, ##args); \
    } while (0)

/* this offset if for dbi directly */
#define PCIE_FUNC_ADDR_OFFSET(func) (func << 16U)

/* this offset if for dbi when the iatu cfg shift enable */
#define PCIE_CFG_SHIFT_BUS(x) (x << 20)
#define PCIE_CFG_SHIFT_DEV(x) (x << 15)
#define PCIE_CFG_SHIFT_FUN(x) (x << 12)
#define PCIE_CFG_SHIFT_REG(x) (x & 0xFFF)

#define PCIE_PORT_REG_ADDR(addr) (addr + 0x700U)

#if defined(PCIE_VERSION_AFTER_4_90_A)
#define PCIE_CS2_ADDR(addr) ((addr) | (0x01U << 20U))
#define PCIE_IATU_ADDR(addr, region, bound) ((0x03U << 20U) | (region << 9U) | (bound << 8U) | (addr))
#define PCIE_DMA_ADDR(addr) ((0x07U << 19U) | (addr))
#else
#define PCIE_CS2_ADDR(addr) ((addr) | (0x01U << 12U))
#define PCIE_IATU_ADDR(addr, region, bound) (addr)
#define PCIE_DMA_ADDR(addr) (addr)
#endif

#define PCIE_TLP_DIRECTION_OUTBOUND (0x0)
#define PCIE_TLP_DIRECTION_INBOUND (0x1)

#define PCIE_TLP_TYPE_MEM_RW (0x0)
#define PCIE_TLP_TYPE_IO_RW (0x2)
#define PCIE_TLP_TYPE_CFG0_RW (0x4)
#define PCIE_TLP_TYPE_CFG1_RW (0x5)
#define PCIE_TLP_TYPE_MSG (0x10)

#define PCIE_IATU_MATCH_MODE_ADDR (0x0)
#define PCIE_IATU_MATCH_MODE_BAR (0x1)

#define PCIE_TLP_INTB_DEASSERT_MSG_CODE (0x25)
/******************************************************************************/
/* ****************************** Configuration ******************************* */
#define BALONG_PCIE_MSI_PCI_LOWER_ADDR(x) (0x0)
#define BALONG_PCIE_MSI_PCI_UPPER_ADDR(x) (0x0)
#define BALONG_PCIE_MSI_PCI_SIZE (SZ_1M)
#define BALONG_PCIE_MSI_CPU_LOWER_ADDR(x) (x)
#define BALONG_PCIE_MSI_CPU_UPPER_ADDR(x) (0x0)

/* the last 4 irq is for intx after irq merge */
#define BALONG_MAX_MSI_NUM (32 * 8)
#define BALONG_PCIE_IRQ_DOMAIN_MAX_IRQ_NUM (BALONG_MAX_MSI_NUM + 4)

#if defined(BALONG_PCIE_FUNC_INT_MERGE)
#define BALONG_PCIE_INTA_INDEX (BALONG_MAX_MSI_NUM + 0)
#define BALONG_PCIE_INTB_INDEX (BALONG_MAX_MSI_NUM + 1)
#define BALONG_PCIE_INTC_INDEX (BALONG_MAX_MSI_NUM + 2)
#define BALONG_PCIE_INTD_INDEX (BALONG_MAX_MSI_NUM + 3)
#endif

#define BALONG_EP_MSI_BAR (0x3)
#define BALONG_EP_CONFIG_BAR (0x4)
#define BALONG_RC_MSI_BAR (0x5)

#define BALONG_PCIE_EP_CHAN_ID 0

#define PCIE_VENDOR_ID_OFFSET 8
#define PCIE_GEN3_EQUAL_BIT 16
#define PCIE_PIPE_LOOP_ENABLE_BIT 31
#define PCIE_LOOP_ENABLE_BIT 2

/* DMA Control 1 Reg, Bit6:Bit5 dma channel status */
#define DMA_DONE (0x3)
#define DMA_HALT (0x2)
#define DMA_RUNNING (0x1)

    /* Data Space Map   */
    /* Cfg0 Space   Addr: base          Size: 2 MB */
    /* Cfg1 Space   Addr: cfg0_end      Size: m MB */
    /* IO Space     Addr: cfg1_end      Size: n MB */
    /* Mem Space    Addr: io_end        Size: x MB */
    /******************************************************************************/
    enum pcie_time_stamp
    {
        PCIE_TIME_POWER_ENABLE = 0,
        PCIE_TIME_POWER_READY,
        PCIE_TIME_PLL_ENABLE,
        PCIE_TIME_PLL_READY,
        PCIE_TIME_CLOCK_ENABLE,
        PCIE_TIME_CLOCK_READY,
        PCIE_TIME_PHY_ENABLE,
        PCIE_TIME_PHY_READY,
        PCIE_TIME_LINK_ENABLE,
        PCIE_TIME_LINK_READY,
        PCIE_TIME_MAX,
    };

    enum pcie_capability_type
    {
        PCIE_CAPABILITY_TYPE_PCI_STANDARD,
        PCIE_CAPABILITY_TYPE_PCIE_EXTENDED,
    };

    struct balong_pcie_iatu_table
    {
        /* iATU Index Register */
        union
        {
            struct
            {
                u32 index : 31;    /* Defines which region is being accessed */
                u32 direction : 1; /* 0: Outbound; 1: Inbound */
            } attr;
            u32 value;
        } index;

        /* iATU Region Control 1 Register */
        union
        {
            struct
            {
                u32 type : 5;      /* bit[4:0]   ----  4b'0000: Memory read/write
                           *                  4b'0010: IO read/write
                           *                  4b'0100: Config Type 0 read/write
                           *                  4b'0101: Config Type 1 read/write
                           */
                u32 reserved : 15; /* u32 reserved:27; */
                u32 func_num : 12; /* function number from 0~8; */
            } attr;
            u32 value;
        } control1;

        /* iATU Region Control 2 Register */
        union
        {
            struct
            {
                u32 message_code : 8;
                u32 bar_index : 3;
                u32 reserved : 8;
                u32 func_num_en : 1; /* Function Number Enable */
                u32 reserved2 : 7;
                u32 dma_bypass_mode : 1; /* Only valid for outbound */
                u32 cfg_shift : 1;       /* CFG Shift Mode */
                u32 invert_mode : 1;
                u32 match_mode : 1;
                u32 enable : 1; /* Region Enable */
            } attr;
            u32 value;
        } control2;

        /* iATU Region Lower Base Address Register */
        u32 lower_addr;
        /* iATU Region Upper Base Address Register */
        u32 upper_addr;
        /* iATU Region Limit Address Register */
        u32 limit_addr;
        /* iATU Region Lower Target Address Register */
        u32 lower_target_addr;
        /* iATU Region Upper Target Address Register */
        u32 upper_target_addr;
        /* iATU Region Control 3 Register */
        u32 control3;
        /* iATU Region Upper Limit Address Register */
        u32 upper_limit_addr;
    };

    typedef struct
    {
        struct semaphore dma_semaphore;
        pcie_callback dma_int_callback;
        void *dma_int_callback_args;
    } pcie_dma_int_info_s;

    struct pcie_irq
    {
        u32 int_func;
        u32 int_misc;

        u32 int_pm;

        u32 int_link_down;
        u32 int_edma;
        int int_radm_a;
        int int_radm_b;
        int int_radm_c;
        int int_radm_d;

        u32 int_msi;

        u32 int_gic_msi;

        u32 int_dma;

        u32 dma_channel_state; /* 1 for busy, each bit corresponds to a DMA channel */
        u32 dma_isr_flags[PCIE_DMA_DIRECTION_MAX];
        pcie_dma_int_info_s dma_int_info[PCIE_DMA_CHN_NUM][PCIE_DMA_DIRECTION_MAX];
    };

    struct bar_attribute
    {
        u32 address;
        u32 size;
        u32 attribute;
    };

    struct pcie_ep_msi_info
    {
        u32 msg_data;
        u32 msg_addr_low;
        u32 is_init;
    };

    struct bar_config_info
    {
        u64 addr;
        u32 size;
        u32 flag;
    };

    struct balong_pcie_info
    {
        u32 id;
        u32 busnr;
        struct pcie_irq irqs;
        struct resource res_io;
        struct resource res_mem;
        struct resource res_cfg0;
        struct resource res_cfg1;

        u32 enabled;
        u32 vendor_id;
        u32 device_id;
        u32 work_mode;
        u32 port_mode;
        u32 max_speed_mode;
        u32 target_speed_mode;
        u32 clock_mode;
        u32 common_clock_mode;
        u32 output_clock_disable;
        u32 endpoint_disable;

        u32 msi_disable;
        u32 hotplug_disable;
        u32 pm_aspm_disable;
        u32 pm_aspm_l0s_disable;
        u32 pm_aspm_l1_disable;
        u32 pm_l1ss_disable;
        u32 pm_l1ss_l1_1_disable;
        u32 pm_l1ss_l1_2_disable;
        u32 phy_link_width;
        u32 phy_tx_vboost_lvl;
        u32 phy_firmware_enable;
        u32 compliance_test_enable;
        u32 compliance_emphasis_mode;

        unsigned long phys_pcie_cfg;
        unsigned long phys_pcie_data;

        unsigned long phys_ctrl_sc_addr;
        unsigned long phys_ctrl_sc_size;
        unsigned long phys_phy_sc_addr;
        unsigned long phys_phy_sc_size;
        unsigned long phys_rc_cfg_addr;
        unsigned long phys_rc_cfg_size;
        unsigned long phys_device_config_addr;
        unsigned long phys_device_config_size;
        unsigned long phys_device_io_size;
        unsigned long phys_device_mem_size;
        unsigned long phys_phy_reg_addr;
        unsigned long phys_phy_reg_size;
        unsigned long phys_dbi_cfg_addr;
        unsigned long phys_dbi_cfg_size;
        unsigned long phys_send_msi_addr;
        u32 phys_ep_target_addr_u;
        u32 phys_ep_target_addr_l;

        u32 gpio_perst;
        u32 perst_valid_value;

        u32 linked;
        struct pci_bus *bus;
        struct pci_dev *pdev;
        void *virt_ctrl_sc_addr;
        void *virt_phy_sc_addr;
        void *virt_rc_cfg_addr;
        void *virt_device_config_addr;
        void *virt_phy_reg_addr;
        void *virt_dbi_cfg_addr;
        void *virt_send_msi_addr;
        char *regulator_id;
        struct regulator *regulator;
        char *clock_sc_id;
        struct clk *clock_sc;
        char *clock_ctrl_id;
        struct clk *clock_ctrl;
        char *clock_aux_id;
        struct clk *clock_aux;
        char *clock_phy_id;
        struct clk *clock_phy;
        struct device_node *dev_node;
        struct platform_device *platform_dev;
        struct irq_domain *irq_domain;
        DECLARE_BITMAP(msi_irq_in_use, BALONG_MAX_MSI_NUM);
        unsigned long ctrl_cfg_assert_reset_addr;
        unsigned long ctrl_cfg_assert_reset_offset;
        u32 ctrl_cfg_assert_reset_value;
        unsigned long ctrl_cfg_deassert_reset_addr;
        unsigned long ctrl_cfg_deassert_reset_offset;
        u32 ctrl_cfg_deassert_reset_value;
        unsigned long phy_cfg_assert_reset_addr;
        unsigned long phy_cfg_assert_reset_offset;
        u32 phy_cfg_assert_reset_value;
        unsigned long phy_cfg_deassert_reset_addr;
        unsigned long phy_cfg_deassert_reset_offset;
        u32 phy_cfg_deassert_reset_value;
        unsigned long phy_assert_reset_addr;
        unsigned long phy_assert_reset_offset;
        u32 phy_assert_reset_value;
        unsigned long phy_deassert_reset_addr;
        unsigned long phy_deassert_reset_offset;
        u32 phy_deassert_reset_value;
        unsigned long ctrl_assert_reset_addr;
        unsigned long ctrl_assert_reset_offset;
        u32 ctrl_assert_reset_value;
        unsigned long ctrl_deassert_reset_addr;
        unsigned long ctrl_deassert_reset_offset;
        u32 ctrl_deassert_reset_value;

        struct bar_attribute bar_attr[PCIE_BAR_NUM];

        volatile u32 req_clk_count;
        volatile u32 xfer_pending_count;

        spinlock_t spinlock; /* for controller */
        spinlock_t spinlock_req_clk;

        struct timespec timespec_value[PCIE_TIME_MAX];

        struct bar_config_info ep_bar_config_info[PCIE_BAR_MAX_NUM];
    };

    int balong_pcie_read_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *value);
    int balong_pcie_write_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 value);
    int balong_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin);

    void pcie_set_target_speed(u32 id, u32 speed_mode);

    struct platform_device *balong_pcie_get_platform_device(void);

    extern u32 g_balong_pcie_num;
    extern struct balong_pcie_info *g_balong_pcie;
    extern int pcie_ep_send_msi(void);
    extern void pcie_ep_bar_print(void);
    extern struct pcie_ep_msi_info g_ep_msi;

    void dbi_enable(u32 id);
    void dbi_disable(u32 id);
    void pcie_app_req_clk(u32 id, u32 request);
    void pcie_app_req_xfer_pending(u32 id, u32 request);
    void pcie_set_iatu(u32 id, struct balong_pcie_iatu_table *iatu_table, u32 iatu_table_entry_num);

    void bsp_pcie_ltssm_enable(u32 id);
    void bsp_pcie_ltssm_disable(u32 id);

    void bsp_pcie_link_up_check(u32 id);
    void bsp_pcie_wait_for_linkup(u32 id);
    int bsp_pcie_is_card_present(u32 id);
    void bsp_pcie_try_change_speed(u32 id, u32 speed_mode);

    void bsp_pcie_platform_init(u32 id);
    void bsp_pcie_platform_exit(u32 id);

    int pcie_balong_speed_init(void);

    void bsp_pcie_hardware_init(u32 id);
    void bsp_pcie_hardware_exit(u32 id);

    int bsp_pcie_get_link_status(u32 id);

    int bsp_pcie_check_turnoff_status(u32 id);
    void bsp_pcie_enable_turnoff_check(u32 id);
    void bsp_pcie_disable_turnoff_check(u32 id);
    void bsp_pcie_ep_recover_bar_cfg(void);
    void bsp_pcie_linkdown_irq_disable(u32 id);
    void bsp_pcie_linkdown_irq_enable(u32 id);
    void bsp_pcie_show_link_status(u32 id);
    void bsp_pcie_ep_cfg_err_dbg(u32 id);
    void bsp_pcie_ep_l1ss_err_dbg(u32 id);

    struct pci_dev *pcie_get_rc(u32 id, u32 vendor_id, u32 device_id);
    struct pci_dev *pcie_get_dev(u32 id, u32 vendor_id, u32 device_id);

    void balong_pcie_msi_isr(struct irq_desc *desc);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef __PCIE_BALONG_H__ */
