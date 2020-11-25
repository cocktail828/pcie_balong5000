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

#ifndef __PCIE_EP_BALONG_H__
#define __PCIE_EP_BALONG_H__

#include "pcie_balong.h"
#include <linux/wakelock.h>
#include "osl_types.h"

#define GIC_MSI_INT_CFG 0x0U
#define GIC_MSI_INT_MASK_EN 0x500U
#define GIC_MSI_INT_MASK_DIS 0x520U
#define GIC_MSI_INT_MASK_STAT 0x540U
#define GIC_MSI_INT_EN 0x560U
#define GIC_MSI_INT_DIS 0x580U
#define GIC_MSI_INT_EN_STAT 0x5A0U
#define GIC_MSI_INT_STAT0 0x5C0U
#define GIC_MSI_INT_STAT1 0x5E0U
#define GIC_MSI_INT_FINAL_STAT 0x600U
#define GIC_MSI_INT_CLR 0x620U
#define GIC_MSI_INT_WARN_STAT 0x640U

#define GIC_MSI_GROUP_NUM 0x8U
#define GIC_MSI_GROUP_IRQ_NUM 0x20U
#define GIC_MSI_REG_OFFSET 0x4U

#define GIC_MSI_INT_WARN_CLR 0x660U

#define PCIE_EP_PR(fmt, args...)                               \
    do                                                         \
    {                                                          \
        printk("[PCIE_EP]%s:" fmt "\n", __FUNCTION__, ##args); \
    } while (0)
#define PCIE_EP_TEST_PR(fmt, args...)                               \
    do                                                              \
    {                                                               \
        printk("[PCIE_EP_TEST]%s:" fmt "\n", __FUNCTION__, ##args); \
    } while (0)

enum pcie_ep_event
{
    PCIE_EP_MSI_INIT = 0,
    PCIE_EP_BAR_CONFIG,
    PCIE_EP_USER_INIT,
    PCIE_EP_EVENT_MAX
};

struct pcie_ep_debug_info
{
    char *pcie_ep_event_name;
    unsigned int init_flag;
};

struct pcie_gic_msi_irq_info
{
    irq_handler_t handler;
    void *irq_data;
    unsigned long irq_count;
    unsigned long irq_handler_count;
    const char *name;
};

struct pcie_ep_gic_msi_info
{
    void *base_addr;            /* virtual base addr */
    unsigned long msi_phy_addr; /* phy base addr */
    struct pcie_gic_msi_irq_info irq_chn[PCIE_GIC_MSI_NUM];
    unsigned int irq;
};

struct pcie_ep_intx_info
{
    unsigned int base_addr;
    unsigned int reg_offset;
    unsigned int bit_offset;
    unsigned int reserved;
    void *virt_base_addr;
};

struct pcie_ep_msi_user_info
{
    void *ep_msi_base_virt;
    unsigned long ep_msi_base_phys;
    unsigned long ep_send_msi_count[PCIE_EP_MSI_NUM];
};

struct balong_ep_pcie_info
{
    struct pcie_ep_gic_msi_info pcie_gic_msi_info;
    struct pcie_ep_msi_user_info pcie_msi_user_info;
    struct pcie_ep_intx_info pcie_intx_info;
    spinlock_t spinlock; /* for controller */
    u32 pcie_pm_status;
    u32 pcie_linkdown_detect;
};

extern int pcie_ep_balong_msi_init(void);
void bsp_pcie_ep_recover_msi_cfg(void);
extern struct balong_ep_pcie_info g_pcie_ep_info;

#endif
