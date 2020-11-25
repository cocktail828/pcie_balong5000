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

#ifndef __SOC_MEMMAP_COMM_H__
#define __SOC_MEMMAP_COMM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "product_config.h"

    /* ��2^n��С�ĺ꣬�ں�δ���壬��Ҫ�ŵ������� */

#ifndef SZ_3M
#define SZ_3M (0x00300000)
#endif

#ifdef __KERNEL__
#include <asm-generic/sizes.h>
#else
/************************************************************************
                * size *
************************************************************************/
#ifndef SZ_512
#define SZ_512 (0x00000200)
#endif

#ifndef SZ_1K
#define SZ_1K (0x00000400)
#endif

#ifndef SZ_2K
#define SZ_2K (0x00000800)
#endif

#ifndef SZ_4K
#define SZ_4K (0x00001000)
#endif

#ifndef SZ_8K
#define SZ_8K (0x00002000)
#endif

#ifndef SZ_16K
#define SZ_16K (0x00004000)
#endif

#ifndef SZ_32K
#define SZ_32K (0x00008000)
#endif

#ifndef SZ_64K
#define SZ_64K (0x00010000)
#endif

#ifndef SZ_128K
#define SZ_128K (0x00020000)
#endif

#ifndef SZ_256K
#define SZ_256K (0x00040000)
#endif

#ifndef SZ_512K
#define SZ_512K (0x00080000)
#endif

#ifndef SZ_1M
#define SZ_1M (0x00100000)
#endif

#ifndef SZ_2M
#define SZ_2M (0x00200000)
#endif

#ifndef SZ_4M
#define SZ_4M (0x00400000)
#endif

#ifndef SZ_8M
#define SZ_8M (0x00800000)
#endif

#ifndef SZ_16M
#define SZ_16M (0x01000000)
#endif

#ifndef SZ_32M
#define SZ_32M (0x02000000)
#endif

#ifndef SZ_64M
#define SZ_64M (0x04000000)
#endif

#ifndef SZ_128M
#define SZ_128M (0x08000000)
#endif

#ifndef SZ_256M
#define SZ_256M (0x10000000)
#endif

#ifndef SZ_512M
#define SZ_512M (0x20000000)
#endif
#endif

/************************************************************************
                * IP BASE ADDR *
************************************************************************/

/* SRAM */
#define HI_SRAM_MEM_BASE_ADDR (DRV_SRAM_ADDR)
#define HI_SRAM_MEM_SIZE (DRV_SRAM_SIZE)

#define HI_FPGA_IOC_BASEADDR 0xE0341000

#define HI_SYSCTRL_AO_REG_BASE_ADDR 0xFFF0A000
#define HI_SYSCTRL_AO_REG_SIZE SZ_4K

#define HI_AO_CRG_REG_BASE_ADDR HI_SYSCTRL_AO_REG_BASE_ADDR
#define HI_SYSSC_AO_BASE_ADDR HI_SYSCTRL_AO_REG_BASE_ADDR
#define HI_SYSSC_PD_BASE_ADDR HI_SYSCTRL_PD_REG_BASE_ADDR
#define HI_SYSSC_MDM_BASE_ADDR HI_SYSCTRL_AO_REG_BASE_ADDR

/* UART2 */
#define HI_UART2_REGBASE_ADDR (0xE4026000)
#define HI_UART2_REG_SIZE (SZ_4K)

#define PERI_CRG_REG_BASE 0xfff35000

/* bc_ctrl */
#define HI_BC_CTRL_REGBASE_ADDR 0xff200000
#define HI_BC_CTRL_REG_SIZE SZ_4K

/* UART0 */
#define HI_UART0_REGBASE_ADDR 0xE4024000
#define HI_UART0_REG_SIZE SZ_4K

/* UART1 */
#define HI_UART1_REGBASE_ADDR 0xE4025000
#define HI_UART1_REG_SIZE SZ_4K

/* HKADC SSI */
#define HI_HKADCSSI_REGBASE_ADDR (0XE82B8000)
#define HI_HKADCSSI_REG_SIZE (SZ_4K)

/* AP ��system controller */
#define HI_AP_SYSCTRL_BASE_ADDR (0xFFF0A000)
#define HI_AP_SYSCTRL_REG_SIZE (SZ_4K)

/* system controller */
#define HI_SYSCTRL_BASE_ADDR (0xE0200000)
#define HI_SYSCTRL_REG_SIZE (SZ_4K)

/*modem system controller */
#define HI_MODEM_SC_BASE_ADDR (0xE0200000)
#define HI_MODEM_SC_REG_SIZE (SZ_4K)

#define HI_SYSCRG_BASE_ADDR HI_SYSCTRL_BASE_ADDR
#define HI_SYSSC_BASE_ADDR HI_SYSCTRL_BASE_ADDR
#define HI_PWRCTRL_BASE_ADDR HI_SYSCTRL_BASE_ADDR

/*sysctrl pd*/
#define HI_SYSCTRL_PD_REG_BASE_ADDR (0xE4001000)
#define HI_SYSCTRL_PD_REG_SIZE SZ_4K

#define HI_PD_CRG_BASE_ADDR 0xe4000000
#define HI_BBIC_CRG_BASE_ADDR 0xE4E00000
#define HI_SYSCTRL_PCIE_REG_BASE_ADDR 0xe4100000

/* EDMA */
#define HI_EDMA_CH4_REGBASE_ADDR (0xFDF30000)
#define HI_EDMA_CH4_REG_SIZE (SZ_4K)

/* watchdog(WDT) */
#define HI_WDT_BASE_ADDR (0xE0201000)
#define HI_WDT_REG_SIZE (SZ_4K)

/* watchdog(WDT1) */
#define HI_WDT1_BASE_ADDR (0xE0211000)
#define HI_WDT1_REG_SIZE (SZ_4K)

/* NANDC reg */
#define HI_NANDC_REGBASE_ADDR 0xFFFA0000
#define HI_NANDC_REG_SIZE SZ_128K

/* CICOM0 */
#define HI_CICOM0_REGBASE_ADDR (0xE0440000)
#define HI_CICOM0_REG_SIZE (SZ_4K)

/* CICOM1 */
#define HI_CICOM1_REGBASE_ADDR (0xE043f000)
#define HI_CICOM1_REG_SIZE (SZ_4K)
/* HDLC */
#define HI_HDLC_REGBASE_ADDR (0xE0454000)
#define HI_HDLC_REG_SIZE (SZ_4K)

/* UPACC */
#define HI_UPACC_BASE_ADDR (0xE0453000)
#define HI_UPACC_REG_SIZE (SZ_4K)

/* CIPHER */
#define HI_CIPHER_BASE_ADDR (0xE0452000)
#define HI_CIPHER_REG_SIZE (SZ_4K)

#define HI_NAND_MEM_BUFFER_ADDR 0xFFF80000
#define HI_NAND_MEM_BUFFER_SIZE SZ_128K

/* shared  DDR */
#define HI_SHARED_DDR_BASE_ADDR (DDR_SHARED_MEM_ADDR)
#define HI_SHARED_DDR_SIZE (DDR_SHARED_MEM_SIZE)

/* dsp subsystem */
#define HI_DSP_SUBSYSTEM_BASE_ADDR (0xE1000000)
#define HI_DSP_SUBSYSTEM_SIZE (SZ_16M)

/*chicago hifi3*/
/*ITCM24KB(hifi:0x3c020000~0x3c025fff,sys:0xec020000~0xec025fff)*/
/*DTCM128KB(hifi:0x3c000000~0x3c01ffff,sys:0xec000000~0xec01ffff)*/
#define HI_HIFI2DMEM0_BASE_ADDR 0xFFE00000
#define HI_HIFI2DMEM0_SIZE 0x10000

#define HI_HIFI2DMEM1_BASE_ADDR 0xFFE10000
#define HI_HIFI2DMEM1_SIZE 0x10000

#define HI_HIFI2IMEM0_BASE_ADDR 0xFFE20000
#define HI_HIFI2IMEM0_SIZE 0x6000

#define HI_SPI_MST0_REGBASE_ADDR 0xe401d000
#define HI_SPI_MST1_REGBASE_ADDR 0xe401e000

#define HI_LTESIO_REGBASE_ADDR 0xE5033000
#define HI_LTESIO_SIZE SZ_4K

/* efusec */
#define HI_EFUSE_REGBASE_ADDR (0xFFF03000)
#define HI_EFUSE_REG_SIZE (SZ_4K)
#define HI_EFUSE_BASE_ADDR HI_EFUSE_REGBASE_ADDR

/*****************************BBP BEGIN*****************************************/
/*tl*/
#define HI_BBP_SRC_BASE_ADDR (0xE1000000)
#define HI_BBP_SRC_SIZE SZ_1M

#define HI_BBP_DMA_BASE_ADDR (0xE1FCC000)
#define HI_BBP_DMA_SIZE SZ_1M

#define HI_BBP_DBG_BASE_ADDR (0xE1FC4000)
#define HI_BBP_DBG_SIZE SZ_1M

#define HI_BBP_INT_BASE_ADDR (0xE1700000)
#define HI_BBP_INT_SIZE SZ_4K

#define HI_BBP_STU_BASE_ADDR (0xE170e000)
#define HI_BBP_STU_SIZE SZ_4K

#define HI_BBP_TSTU_BASE_ADDR (0xE1d00000)
#define HI_BBP_TSTU_SIZE SZ_8K
/*gu*/
#define HI_GBBP_REG_BASE_ADDR (0xE1800000)
#define HI_GBBP_REG_SIZE SZ_512K

#define HI_GBBP1_REG_BASE_ADDR (0xE1880000)
#define HI_GBBP1_REG_SIZE SZ_512K

#define HI_WBBP_REG_BASE_ADDR (0xE1900000)
#define HI_WBBP_REG_REG_SIZE SZ_1M

#define HI_BBP_CDMA_BASE_ADDR (0xE12E0000)
#define HI_BBP_GSDR_BASE_ADDR (0xE1840000)

#define HI_CTU_BASE_ADDR (0xE1f80000)
#define HI_CTU_SIZE SZ_32K
#if defined(BSP_CONFIG_BOARD_SFT)
/*tl*/
#define HI_BBP_LTEDRX_BASE_ADDR (0xE1FB0000)

#define HI_BBP_TDSDRX_BASE_ADDR (0xE1FB1400)

/*gu*/
#define HI_BBP_COMM_ON_BASE_ADDR (0xE1FB0000)

#define HI_GBBP_DRX_REG_BASE_ADDR (0xE1FB0800)

#define HI_GBBP1_DRX_REG_BASE_ADDR (0xE1FB0C00)

#define HI_WBBP_DRX_REG_BASE_ADDR (0xE1FB0000)

#define HI_BBP_CDMA_ON_BASE_ADDR (0xE1FB0000)

#define HI_BBP_GLB_ON_BASE_ADDR (0xE1FB0000)

#else
/*tl*/
#define HI_BBP_LTEDRX_BASE_ADDR (0xFFF12000)

#define HI_BBP_TDSDRX_BASE_ADDR (0xFFF13400)

/*gu*/
#define HI_BBP_COMM_ON_BASE_ADDR (0xFFF12000)

#define HI_GBBP_DRX_REG_BASE_ADDR (0xFFF12800)

#define HI_GBBP1_DRX_REG_BASE_ADDR (0xFFF12C00)

#define HI_WBBP_DRX_REG_BASE_ADDR (0xFFF12000)

#define HI_BBP_CDMA_ON_BASE_ADDR (0xFFF12000)

#define HI_BBP_GLB_ON_BASE_ADDR (0xFFF12000)

#endif

#define HI_BBP_CTU_BASE_ADDR HI_CTU_BASE_ADDR
#define HI_BBPMASTER_REG_BASE_ADDR HI_WBBP_REG_BASE_ADDR
/*****************************BBP END*****************************************/

/* CoreSignt Core1 PTM */
#define HI_CORESIGHT_PTM1_BASE_ADDR 0xEC0BC000
#define HI_CORESIGHT_PTM1_SIZE SZ_4K

#if defined(BSP_CONFIG_BOARD_SFT) && defined(BSP_CONFIG_HI3650)
#define HI_BBP_SYSTIME_BASE_ADDR (0xFFF08000)
#else
#define HI_BBP_SYSTIME_BASE_ADDR (HI_AP_SYSCTRL_BASE_ADDR)
#endif
#define HI_BBP_SYSTIME_SIZE SZ_8K

#define HI_XG2RAM_HARQ_BASE_ADDR (0xEA000000)

/* reset fpga addr */
#define HI_RESET_FPGA_ADDR (0x21fd607c)

#ifdef __cplusplus
}
#endif

#endif /* __SOC_MEMMAP_COMM_H__ */
