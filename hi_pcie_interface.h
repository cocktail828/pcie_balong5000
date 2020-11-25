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

#ifndef __HI_PCIE_INTERFACE_H__
#define __HI_PCIE_INTERFACE_H__

#include "bsp_pcie.h"
#include "bsp_version.h"
#include "bsp_sysctrl.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /********************************** Version ***********************************/

#define PCIE_VERSION_AFTER_4_90_A

    /******************************* Configuration ********************************/

#define BALONG_PCIE_INNER_CLK
#define CONFIG_BALONG_PCIE_L1SS_AUTO
#define BALONG_PCIE_FUNCTION_NUM (8)
#define BALONG_PCIE_MAX_DMA_CHANNEL_NUM (8)
#define BALONG_PCIE_FUNC_INT_MERGE

#define BALONG_PCIE_CTRL_BUG /* power on reset bugfix */

#define BALONG_PCIE_UPDATE_FIRMWARE
#define BALONG_PCIE_FIRMWARE_SRAM_ADDR (0xC000)
#define BALONG_PCIE_FIRMWARE_SRAM_SIZE (0x17FF)

    /******************************* Definition ********************************/
    static __inline__ u32 hi_pcie_get(void *__virt_addr, u32 __bit_offset, u32 __bit_width)
    {
        u32 __value = 0;

        __value = readl(__virt_addr);
        __value >>= __bit_offset;
        __value &= ((1 << __bit_width) - 1);
        return __value;
    }
    static __inline__ void hi_pcie_set(void *__virt_addr, u32 __bit_offset, u32 __bit_width, u32 __bit_value)
    {
        u32 __value = 0;

        __value = readl(__virt_addr);
        __value &= ~(((1 << __bit_width) - 1) << __bit_offset);
        __value |= (__bit_value << __bit_offset);
        writel(__value, (void *)__virt_addr);
    }

    static __inline__ void hi_pcie_dbi_enable(void *__virt_addr, u32 __pcie_id)
    {
        hi_pcie_set(__virt_addr + 0x000, 21, 1, 1);
        hi_pcie_set(__virt_addr + 0x004, 21, 1, 1);
    }

    static __inline__ void hi_pcie_dbi_disable(void *__virt_addr, u32 __pcie_id)
    {
        hi_pcie_set(__virt_addr + 0x000, 21, 1, 0);
        hi_pcie_set(__virt_addr + 0x004, 21, 1, 0);
    }

    static __inline__ void hi_pcie_set_work_mode(void *__virt_addr, u32 __pcie_id, u32 __pcie_work_mode)
    {
        hi_pcie_set(__virt_addr + 0x000, 28, 4, __pcie_work_mode);
    }

    static __inline__ void hi_pcie_state2_sel_enable(void *__virt_addr, u32 __pcie_id)
    {
        hi_pcie_set(__virt_addr + 0x020, 5, 2, 1);
    }

    static __inline__ void hi_pcie_state2_sel_disable(void *__virt_addr, u32 __pcie_id)
    {
        hi_pcie_set(__virt_addr + 0x020, 5, 2, 0);
    }

    static __inline__ u32 hi_pcie_get_turnoff_status(void *__virt_addr, u32 __pcie_id)
    {
        return hi_pcie_get(__virt_addr + 0x408, 0, 1);
    }

    static __inline__ void hi_pcie_ltssm_enable(void *__virt_addr, u32 __pcie_id)
    {
        hi_pcie_set(__virt_addr + 0x01C, 11, 1, 1);
    }

    static __inline__ void hi_pcie_ltssm_disable(void *__virt_addr, u32 __pcie_id)
    {
        hi_pcie_set(__virt_addr + 0x01C, 11, 1, 0);
    }

    static __inline__ void hi_pcie_ltssm_force_link(void *__virt_addr, u32 __pcie_id)
    {
        /* skip tx detect rx */
        hi_pcie_set(__virt_addr + 0x008, 16, 6, 0x2);
        hi_pcie_set(__virt_addr + 0x008, 15, 1, 0x1);
    }

    static __inline__ u32 hi_pcie_get_link_status(void *__virt_addr, u32 __pcie_id)
    {
        return hi_pcie_get(__virt_addr + 0x410, 0, 6);
    }

    static __inline__ u32 hi_pcie_is_linkup(void *__virt_addr, u32 __pcie_id)
    {
        return (hi_pcie_get(__virt_addr + 0x400, 5, 1) & hi_pcie_get(__virt_addr + 0x400, 15, 1));
    }

    static __inline__ void hi_pcie_linkdown_interrupt_clear(void *__virt_addr, u32 __pcie_id)
    {
        /* pcie_bridge_flush_not_clr */
        hi_pcie_set(__virt_addr + 0x02C, 11, 1, 1);
        /* pcie_link_req_rst_not_clr */
        hi_pcie_set(__virt_addr + 0x02C, 12, 1, 1);

        /* pcie_bridge_flush_not_clr */
        hi_pcie_set(__virt_addr + 0x02C, 11, 1, 0);
        /* pcie_link_req_rst_not_clr */
        hi_pcie_set(__virt_addr + 0x02C, 12, 1, 0);
    }

    static __inline__ void hi_soc_peri_usb_deassert_reset(void *__virt_addr, u32 __pcie_id)
    {
    }

    static __inline__ void hi_pcie_phy_deassert_reset(void *__phy_sc_virt_addr, void *__phy_virt_addr, u32 __pcie_id, u32 __firmware_enable)
    {
#ifdef BALONG_PCIE_UPDATE_FIRMWARE
        u32 i = 0;
        const bsp_version_info_s *version = bsp_get_version_info();

#include "hi_pcie_firmware_20180510.h"
#include "hi_pcie_firmware_20181210.h"

#endif

        /* phy_test_powerdown */
        hi_pcie_set(__phy_sc_virt_addr + 0x000, 22, 1, 0);
        /* phy_ana_pwr_en */
        hi_pcie_set(__phy_sc_virt_addr + 0x084, 1, 1, 1);
        /* pcs_rx_cdr_legacy_en */

        hi_pcie_set(__phy_sc_virt_addr + 0x098, 16, 1, 1);

        /* phy_reset */
        hi_pcie_set(__phy_sc_virt_addr + 0x004, 16, 1, 1);

#ifdef BALONG_PCIE_UPDATE_FIRMWARE

        if (__firmware_enable == PCIE_ENABLE)
        {
            /* sram_bypass */
            hi_pcie_set(__phy_sc_virt_addr + 0x0A0, 0, 1, 0);
        }

        /* phy_reset */
        hi_pcie_set(__phy_sc_virt_addr + 0x004, 16, 1, 0);

#ifdef BALONG_PCIE_TC_BUGFIX
        udelay(100);
        /* disable fast RX continuous data calibration */
        hi_pcie_set(__phy_virt_addr + (0x402D * 4), 2, 1, 1);
        hi_pcie_set(__phy_virt_addr + (0x402D * 4), 9, 1, 1);
        hi_pcie_set(__phy_virt_addr + (0x412D * 4), 2, 1, 1);
        hi_pcie_set(__phy_virt_addr + (0x412D * 4), 9, 1, 1);

        hi_pcie_set(__phy_virt_addr + (0x4047 * 4), 0, 1, 0);
        hi_pcie_set(__phy_virt_addr + (0x4147 * 4), 0, 1, 0);
#endif

        if (__firmware_enable == PCIE_ENABLE)
        {
            /* wait until sram_init_done */
            while (!hi_pcie_get(__phy_sc_virt_addr + 0x49C, 0, 1))
                ;

            if ((version) && ((version_cses_type_e)version->cses_type == TYPE_ES))
            {
                for (i = 0; i < sizeof(pcie_firmware_20180510) / sizeof(pcie_firmware_20180510[0]); i++)
                {
                    writel_relaxed(pcie_firmware_20180510[i], __phy_virt_addr + (BALONG_PCIE_FIRMWARE_SRAM_ADDR + i) * 4);
                }
            }
            else
            {
                for (i = 0; i < sizeof(pcie_firmware_20181210) / sizeof(pcie_firmware_20181210[0]); i++)
                {
                    writel_relaxed(pcie_firmware_20181210[i], __phy_virt_addr + (BALONG_PCIE_FIRMWARE_SRAM_ADDR + i) * 4);
                }
            }

            for (; i <= BALONG_PCIE_FIRMWARE_SRAM_SIZE; i++)
            {
                writel_relaxed(0, __phy_virt_addr + (BALONG_PCIE_FIRMWARE_SRAM_ADDR + i) * 4);
            }

            /* this bits is set to 1 by ECO, change them to default value */
            hi_pcie_set(__phy_virt_addr + (0x402D * 4), 2, 1, 0);
            hi_pcie_set(__phy_virt_addr + (0x412D * 4), 2, 1, 0);
            hi_pcie_set(__phy_virt_addr + (0x402D * 4), 9, 1, 0);
            hi_pcie_set(__phy_virt_addr + (0x412D * 4), 9, 1, 0);

            hi_pcie_set(__phy_virt_addr + (0x4047 * 4), 0, 1, 1);
            hi_pcie_set(__phy_virt_addr + (0x4147 * 4), 0, 1, 1);

            hi_pcie_set(__phy_virt_addr + (0x401E * 4), 0, 11, 0x801);
            hi_pcie_set(__phy_virt_addr + (0x411E * 4), 0, 11, 0x801);

            /* sram_ext_ld_done */
            hi_pcie_set(__phy_sc_virt_addr + 0x0A0, 4, 1, 1);
        }

#else
    /* phy_reset */
    hi_pcie_set(__phy_sc_virt_addr + 0x004, 16, 1, 0);
#endif
    }

    static __inline__ void hi_pcie_iso_set(void *__virt_addr, u32 __pcie_id, u32 __iso_enable)
    {
        void *sctrl_addr = bsp_sysctrl_addr_byindex(SYSCTRL_AO);
        if (__iso_enable)
        {
            hi_pcie_set(sctrl_addr + 0xD00, 8 + __pcie_id, 1, 0x1); /* enable iso */
        }
        else
        {
            hi_pcie_set(sctrl_addr + 0xD04, 8 + __pcie_id, 1, 0x1); /* disable iso */
        }
    }

    static __inline__ void hi_pcie_phy_fixup(void *__virt_addr, u32 __pcie_id)
    {
    }

    static __inline__ void hi_pcie_ctrl_fixup(void *__virt_addr, u32 __pcie_id)
    {
        /* por_n_ctrl */
        hi_pcie_set(__virt_addr + 0x058, 16, 2, 0x1);
        /* pcie_perst_in_n_ctrl */
        hi_pcie_set(__virt_addr + 0x030, 2, 2, 1);
    }

    static __inline__ u32 hi_pcie_is_under_reset(void *__virt_addr, u32 __pcie_id, u32 __lane_mask)
    {
        u32 lane_index;
        u32 phy_status = 0;

        for (lane_index = 0; lane_index < 32; lane_index++)
        {
            if ((1 << lane_index) & __lane_mask)
            {
                phy_status |= hi_pcie_get(__virt_addr + 0x400, 26 + lane_index, 1);
            }
        }
        return phy_status;
    }

    static __inline__ void hi_pcie_phy_init(void *__virt_addr, u32 __pcie_id)
    {
    }

    static __inline__ void hi_pcie_phy_tx_vboost_lvl(void *__virt_addr, u32 __pcie_id, u32 __pcie_value)
    {
        hi_pcie_set(__virt_addr + (0x21 * 4), 6, 3, __pcie_value);
        hi_pcie_set(__virt_addr + (0x21 * 4), 9, 1, 1);
    }

    static __inline__ void hi_pcie_set_apps_pm_xmt_turnoff(void *__virt_addr, u32 __pcie_id, u32 __pcie_value)
    {
        /* pcie_apps_pm_xmt_turnoff */
        hi_pcie_set(__virt_addr + 0x01C, 8, 1, __pcie_value);
    }

    static __inline__ void hi_pcie_set_app_ready_entr_l23(void *__virt_addr, u32 __pcie_id, u32 __pcie_value)
    {
        /* pcie_app_ready_entr_l23 */
        hi_pcie_set(__virt_addr + 0x01C, 2, 1, __pcie_value);
    }

    static __inline__ void hi_pcie_set_apps_pm_xmt_pme(void *__virt_addr, u32 __pcie_id, u32 __pcie_value)
    {
        /* pcie_apps_pm_xmt_pme */
        hi_pcie_set(__virt_addr + 0x024, 0, 8, __pcie_value);
    }

    static __inline__ u32 hi_pcie_get_radm_pm_to_ack_reg(void *__virt_addr, u32 __pcie_id)
    {
        /* radm_pm_to_ack_reg */
        return hi_pcie_get(__virt_addr + 0x404, 16, 1);
    }

    static __inline__ void hi_pcie_phy_sc_clk_init(void *__virt_addr, u32 __pcie_id, u32 __common_clk_mode)
    {
        /* for pcs_rxX_cmn_refclk_mode, lane0 & lane1 */
        if (__common_clk_mode)
        {
            hi_pcie_set(__virt_addr + 0x098, 10, 1, 0x1);
            hi_pcie_set(__virt_addr + 0x098, 12, 1, 0x1);
        }
        else
        {
            hi_pcie_set(__virt_addr + 0x098, 10, 1, 0x0);
            hi_pcie_set(__virt_addr + 0x098, 12, 1, 0x0);
        }
    }

    static __inline__ void hi_pcie_phy_clk_init(void *__virt_addr, u32 __pcie_id, u32 __clk_mode)
    {
        const bsp_version_info_s *version = bsp_get_version_info();

        if (__clk_mode == PCIE_CLOCK_MODE_INNER)
        {
            /* PLL, ref_clk = 38.4 MHz, VCO = 3.6 GHz, FOUTPOSTDIV = 100 MHz */
            hi_pcie_set(__virt_addr + 0xC10, 0, 1, 0);     /* PLL en */
            hi_pcie_set(__virt_addr + 0xC10, 1, 1, 0);     /* PLL bypass */
            hi_pcie_set(__virt_addr + 0xC10, 2, 6, 1);     /* PLL refdiv */
            hi_pcie_set(__virt_addr + 0xC10, 8, 12, 0x3E); /* PLL fbdiv */
            hi_pcie_set(__virt_addr + 0xC10, 20, 3, 0x4);  /* PLL postdiv1 */
            hi_pcie_set(__virt_addr + 0xC10, 23, 3, 0x6);  /* PLL postdiv2 */

            hi_pcie_set(__virt_addr + 0xC14, 0, 24, 0x800000); /* PLL fracdiv */
            hi_pcie_set(__virt_addr + 0xC14, 25, 1, 0);        /* PLL 4phase_pd */
            hi_pcie_set(__virt_addr + 0xC14, 26, 1, 0);        /* PLL clg gate */
            hi_pcie_set(__virt_addr + 0xC14, 27, 1, 1);        /* PLL dll en */
            hi_pcie_set(__virt_addr + 0xC14, 28, 1, 1);        /* PLL postdiv_pd */

            if ((version) && ((version_cses_type_e)version->cses_type == TYPE_ES))
            {
                hi_pcie_set(__virt_addr + 0xC40, 0, 16, 0x0000);  /* PLL cfg0 */
                hi_pcie_set(__virt_addr + 0xC40, 16, 16, 0x0000); /* PLL cfg1 */
                hi_pcie_set(__virt_addr + 0xC44, 0, 16, 0x0000);  /* PLL cfg2 */
                hi_pcie_set(__virt_addr + 0xC44, 16, 16, 0x00B4); /* PLL cfg3 */
                hi_pcie_set(__virt_addr + 0xC48, 0, 16, 0x0FA0);  /* PLL cfg4 */
                hi_pcie_set(__virt_addr + 0xC48, 16, 16, 0x2010); /* PLL cfg5 */
                hi_pcie_set(__virt_addr + 0xC4C, 0, 16, 0xFF20);  /* PLL cfg6 */
                hi_pcie_set(__virt_addr + 0xC4C, 16, 16, 0x2404); /* PLL cfg7 */
                hi_pcie_set(__virt_addr + 0xC50, 0, 16, 0x013F);  /* PLL cfg8 */
                hi_pcie_set(__virt_addr + 0xC50, 16, 16, 0x0000); /* PLL cfg9 */
                hi_pcie_set(__virt_addr + 0xC54, 0, 16, 0x0046);  /* PLL cfg10 */
            }
            else
            {
                hi_pcie_set(__virt_addr + 0xC40, 0, 16, 0x2000);  /* PLL cfg0 */
                hi_pcie_set(__virt_addr + 0xC40, 16, 16, 0x0004); /* PLL cfg1 */
                hi_pcie_set(__virt_addr + 0xC44, 0, 16, 0x00B4);  /* PLL cfg2 */
                hi_pcie_set(__virt_addr + 0xC44, 16, 16, 0x0000); /* PLL cfg3 */
                hi_pcie_set(__virt_addr + 0xC48, 0, 16, 0x2010);  /* PLL cfg4 */
                hi_pcie_set(__virt_addr + 0xC48, 16, 16, 0x0FA0); /* PLL cfg5 */
                hi_pcie_set(__virt_addr + 0xC4C, 0, 16, 0x2404);  /* PLL cfg6 */
                hi_pcie_set(__virt_addr + 0xC4C, 16, 16, 0xFF20); /* PLL cfg7 */
                hi_pcie_set(__virt_addr + 0xC50, 0, 16, 0x0000);  /* PLL cfg8 */
                hi_pcie_set(__virt_addr + 0xC50, 16, 16, 0x013F); /* PLL cfg9 */
                hi_pcie_set(__virt_addr + 0xC54, 0, 16, 0x0046);  /* PLL cfg10 */
            }

            hi_pcie_set(__virt_addr + 0xC10, 0, 1, 1); /* PLL en */

            /* wait until pll lock */
            while (!hi_pcie_get(__virt_addr + 0xE00, 4, 1))
                ;

            hi_pcie_set(__virt_addr + 0x000, 14, 1, 0); /* phy_ref_use_cio_pad */

            /* PHY port signal, switch to inner clock, phy_ref_use_pad
         * the clock is from phy's pad or inner PLL, 0 -- inner PLL; 1 -- phy's pad
         */
            hi_pcie_set(__virt_addr + 0x004, 8, 1, 0);
        }
        else if (__clk_mode == PCIE_CLOCK_MODE_OUTER_PHY)
        {
            hi_pcie_set(__virt_addr + 0x004, 8, 1, 1); /* phy_ref_use_pad */
        }
        else
        {
            hi_pcie_set(__virt_addr + 0x000, 14, 1, 1); /* phy_ref_use_cio_pad */
            hi_pcie_set(__virt_addr + 0x004, 8, 1, 0);  /* phy_ref_use_pad */
        }
    }

    static __inline__ void hi_pcie_ctrl_clk_init(void *__virt_addr, u32 __pcie_id, u32 __clk_mode)
    {
        if (__clk_mode == PCIE_CLOCK_MODE_INNER)
        {
            hi_pcie_set(__virt_addr + 0x05C, 12, 1, 1); /* gt_clk_pciephy_ref_soft */
        }
        else if (__clk_mode == PCIE_CLOCK_MODE_OUTER_PHY)
        {
            hi_pcie_set(__virt_addr + 0x05C, 21, 1, 1); /* pcieio_ie_en_soft */
            hi_pcie_set(__virt_addr + 0x05C, 22, 1, 0); /* pcieio_ie_polar */
        }
        else
        {
            hi_pcie_set(__virt_addr + 0x05C, 21, 1, 1); /* pcieio_ie_en_soft */
            hi_pcie_set(__virt_addr + 0x05C, 22, 1, 0); /* pcieio_ie_polar */
        }

        /* pcie_app_clk_req_n */
        hi_pcie_set(__virt_addr + 0x004, 23, 1, 1);
    }

    static __inline__ void hi_pcie_output_clk_enable(void *__virt_addr, u32 __pcie_id)
    {
        /* ds = 12 mA */
        hi_pcie_set(__virt_addr + 0x054, 4, 3, 5);
        /* cs = inner */
        hi_pcie_set(__virt_addr + 0x054, 20, 1, 1);
        /* pcie io ie overwrite value */
        hi_pcie_set(__virt_addr + 0x054, 13, 1, 0);
        /* pcie io ie overwrite en */
        hi_pcie_set(__virt_addr + 0x054, 12, 1, 1);
        /* pcie io oe overwrite en */
        hi_pcie_set(__virt_addr + 0x054, 14, 1, 0);
        /* gt_clk_pcieio_soft */
        hi_pcie_set(__virt_addr + 0x05C, 11, 1, 1);
        /* pcieio_oe_polar */
        hi_pcie_set(__virt_addr + 0x05C, 17, 1, 1);
    }

    static __inline__ void hi_pcie_l1ss_auto_gate_enable(void *__virt_addr, u32 __pcie_id)
    {
        /* pcieio_oe_en_hard_bypass */
        hi_pcie_set(__virt_addr + 0x05C, 19, 1, 0);
    }

    static __inline__ void hi_pcie_l1ss_auto_gate_disable(void *__virt_addr, u32 __pcie_id)
    {
        /* pcieio_oe_en_hard_bypass */
        hi_pcie_set(__virt_addr + 0x05C, 19, 1, 1);
    }

    static __inline__ void hi_pcie_output_clk_cs_switch(void *__virt_addr, u32 __pcie_id, u32 __is_inner)
    {
        if (__is_inner)
        {
            hi_pcie_set(__virt_addr + 0x054, 20, 1, 0);
        }
        else
        {
            hi_pcie_set(__virt_addr + 0x054, 20, 1, 1);
        }
    }

    static __inline__ void hi_pcie_app_clk_req_n(void *__virt_addr, u32 __pcie_id, u32 __request)
    {
        if (__request)
        {
            /* pcie_app_req_exit_l1 */
            hi_pcie_set(__virt_addr + 0x01C, 1, 1, 0);
            hi_pcie_set(__virt_addr + 0x01C, 3, 1, 1);
        }
        else
        {
            /* pcie_app_req_enter_l1 */
            hi_pcie_set(__virt_addr + 0x01C, 3, 1, 0);
            hi_pcie_set(__virt_addr + 0x01C, 1, 1, 1);
        }
    }

    static __inline__ void hi_pcie_app_xfer_pending(void *__virt_addr, u32 __pcie_id, u32 __request)
    {
        if (__request)
        {
            /* pcie_app_req_pending */
            hi_pcie_set(__virt_addr + 0x018, 8, 1, 1);
        }
        else
        {
            /* pcie_app_cancel_pending */
            hi_pcie_set(__virt_addr + 0x018, 8, 1, 0);
        }
    }

    static __inline__ void hi_pcie_trigger_int_msi(void *__virt_addr, u32 __pcie_id)
    {
        hi_pcie_set(__virt_addr + 0x1C, 5, 1, 1);
    }

    static __inline__ void hi_pcie_clear_int_msi(void *__virt_addr, u32 __pcie_id)
    {
        hi_pcie_set(__virt_addr + 0x1C, 5, 1, 0);
    }

    static __inline__ void hi_pcie_assert_perst(void *__virt_addr, u32 __pcie_id)
    {
        hi_pcie_set(__virt_addr + 0x030, 0, 1, 0);
        hi_pcie_set(__virt_addr + 0x030, 1, 1, 1);
    }

    static __inline__ void hi_pcie_deassert_perst(void *__virt_addr, u32 __pcie_id)
    {
        hi_pcie_set(__virt_addr + 0x030, 0, 1, 1);
        hi_pcie_set(__virt_addr + 0x030, 1, 1, 1);
    }

    static __inline__ u32 hi_pcie_check_l1ss_status(void *__virt_addr)
    {
        return hi_pcie_get(__virt_addr + 0x414, 15, 2);
    }

#ifdef __cplusplus
}
#endif

#endif
