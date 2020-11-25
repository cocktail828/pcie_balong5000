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

#ifndef __SOC_ONCHIP_H__
#define __SOC_ONCHIP_H__

#include "soc_memmap.h"

#define FEATURE_RSA_LOCAL // ��ʾʹ�ñ��ص�RSA�㷨������ԭ���ĺ�SOC_ONCHIPROM_V7R2
/*--------------------------------------------------------------*
 * �궨��                                                    *
 *--------------------------------------------------------------*/

#define ACORE_RESET
#define IO_WR(addr, data) writel(data, (void *)(uintptr_t)(addr))
#define IO_RD(addr) readl((void *)(uintptr_t)(addr))
//from asic_base_addr.h of chip
#define SYS_PD_BASE (0xC4001000)
#define CRG_PD_BASE (0xC4000000)
#define SYS_AO_BASE (0xCDF00000)

#define CRG_PD_PD_CRG_SSRSTEN1 (CRG_PD_BASE + 0x360)
#define SYS_AO_PWR_CTRL6 (SYS_AO_BASE + 0xC18)
#define SYS_AO_PWR_STAT1 (SYS_AO_BASE + 0xE04)
#define SYS_PD_SC_TOP_CTRL26 (SYS_PD_BASE + 0x468)
#define CRG_PD_PD_CRG_CLKEN3 (CRG_PD_BASE + 0x20)
#define SYS_AO_PWR_CTRL5 (SYS_AO_BASE + 0xC14)
#define CRG_PD_PD_CRG_SSRSTDIS1 (CRG_PD_BASE + 0x364)
#define SYS_PD_SC_PERI_CTRL87 (SYS_PD_BASE + 0x95c)
#define SC_AO_STAT0 (SYS_AO_BASE + 0x600)
#define SC_AO_ABS_TIMER (SYS_AO_BASE + 0x614)

/* boot mode */
#define SC_BOOTMODE_BITPOS (0)
#define SC_BOOTMODE_BITWIDTH (3)
#define SC_BOOTMODE_BITMASK (((1 << SC_BOOTMODE_BITWIDTH) - 1) << SC_BOOTMODE_BITPOS)

#define BOOT_MODE_FMC (0x0 << SC_BOOTMODE_BITPOS)
#define BOOT_MODE_EMMC0 (0x1 << SC_BOOTMODE_BITPOS)
#define BOOT_MODE_AP_PCIE (0x3 << SC_BOOTMODE_BITPOS)
#define BOOT_MODE_AP_UART (0x4 << SC_BOOTMODE_BITPOS)
#define BOOT_MODE_TEST (0x5 << SC_BOOTMODE_BITPOS)
#define BOOT_MODE_FMC_PAD (0x6 << SC_BOOTMODE_BITPOS)
#define BOOT_MODE_FMC_DOUBLEBOOT (0x7 << SC_BOOTMODE_BITPOS) // just for compile
#define BOOT_MODE_AP_PCIE1 (0x7 << SC_BOOTMODE_BITPOS)       // just for compile
#define HI_LPMCU_TCM_MCORE_SIZE (256 * 1024)
#define HI_LPMCU_TCM_MCORE_ADDR (0x40000)
#define HI_LPMCU_TCM_ACORE_ADDR (0xFFF40000)
#define CONVERT_TCM_ADDR(addr) ((addr)-HI_LPMCU_TCM_MCORE_ADDR + HI_LPMCU_TCM_ACORE_ADDR)

#define HI_DDR_MCORE_ADDR (0x20000000)
#define HI_DDR_ACORE_ADDR (0x0)
#define CONVERT_DDR_ADDR(addr) ((addr)-HI_DDR_MCORE_ADDR + HI_DDR_ACORE_ADDR)

#if defined(__FASTBOOT__) || defined(__ONCHIP__)
#define M3_TCM_ADDR HI_LPMCU_TCM_ACORE_ADDR
#else
#define M3_TCM_ADDR HI_LPMCU_TCM_MCORE_ADDR
#endif
#define M3_TCM_SIZE HI_LPMCU_TCM_MCORE_SIZE

/* ��������������TCM����,BootLoader��������Ҫ������ */
#define M3_TCM_SHARE_DATA_SIZE sizeof(tOcrShareData)
#define M3_TCM_SHARE_DATA_ADDR (M3_TCM_ADDR + HI_LPMCU_TCM_MCORE_SIZE - M3_TCM_SHARE_DATA_SIZE)

/* OnChipRom���й���־���Ծٱ�־����tOcrShareData�ṹ���� */
#define OCR_INITED_FLAG_ADDR (M3_TCM_ADDR + HI_LPMCU_TCM_MCORE_SIZE - 4)
#define OCR_AUTO_ENUM_FLAG_ADDR (M3_TCM_ADDR + HI_LPMCU_TCM_MCORE_SIZE - 8)
#define OCR_INITED_FLAG_VALUE (0x23456789)
#define OCR_UNINITED_FLAG_VALUE (0xA55A6789)

#define SC_AO_STAT0_ACORE (0xEDF00600)
#define SC_AUTO_ENUM_EN_BITPOS 3
#define SC_AUTO_ENUM_EN_BITWIDTH 1
#define SC_AUTO_ENUM_EN_BITMASK (((1 << SC_AUTO_ENUM_EN_BITWIDTH) - 1) << SC_AUTO_ENUM_EN_BITPOS)
#define SC_AUTO_ENUM_EN (0 << SC_AUTO_ENUM_EN_BITPOS)
#define AUTO_ENUM_FLAG_VALUE 0x82822828

#define ROOT_CA_WITHOUT_MD5

#define EFUSE_GROUP_INDEX_SECURE (31)
#define EFUSE_BIT_FLAG_SEC_EN ((0x1 << 3) | (0x1 << 4))

/* ֻ��OEM CA�в���OEM ID��HWID,��CA���� */
#define KEY_OEMID_INDEX sizeof(KEY_STRUCT)
#define KEY_HWID_INDEX (KEY_OEMID_INDEX + 0x04)

#define BL_LEN_INDEX (144 * 4)           /* bootload.bin�ļ�����(Byte)������bootload.bin�ļ��е�ƫ���ֽ��� */
#define ROOT_CA_INDEX (BL_LEN_INDEX + 4) /* ��CA��Image��λ�� */

#define ROOT_CA_LEN (0) /* CA֤��ĳ��� 260+128 Byte */
#define OEM_CA_LEN (0)  /* CA֤��ĳ��� 268+128 Byte */
#define IDIO_LEN (0)    /* ǩ���ĳ��� 128 Byte */

#define MAX_N_LENGTH 32

#define SHA256_HASH_SIZE 8 /* HASHǩ��ռ��word�� */

#define DX_SEC_BOOT
#define DX_SEC_BOOT_EN
#define VRL_TABLE_SIZE (4 * 1024)

#define NAND_ARGS_FULL_AVAIL 0x3a
#define NAND_ARGS_PART_AVAIL 0x45

#ifndef __ASSEMBLY__
/*--------------------------------------------------------------*
 * ���ݽṹ                                                     *
 *--------------------------------------------------------------*/

/* ��Կ��ʽ */
typedef struct
{
    unsigned short eLen;          /* E������64bitsΪ��λ��ȡֵ��ΧΪ0~15��0��ʾ16 */
    unsigned short nLen;          /* N������32bitsΪ��λ��ȡֵ��ΧΪ0~31��0��ʾ32 */
    unsigned int e[MAX_N_LENGTH]; /* ��e(��Կ) */
    unsigned int n[MAX_N_LENGTH]; /* ģ�� */
    unsigned int c[MAX_N_LENGTH]; /* ���������� */
} KEY_STRUCT;

/* ��ȫУ�麯��ָ�� */
typedef unsigned int (*secCheckPtr)(unsigned int vrl_addr, unsigned int image_addr);

typedef int (*efuseReadPtr)(unsigned int group, unsigned int *pBuf, unsigned int num);

/* WDT reboot����ָ�� */
typedef void (*wdtRebootPtr)(unsigned int delay_ms);

typedef void (*printfPtr)(const void *pucBuffer);

/* �������ݽṹ��,����AXI����,�������¼�Ԫ��(��������ǰ��) */
typedef struct tagOcrShareData
{
    printfPtr buf_printf;
    wdtRebootPtr wdtReboot;
    efuseReadPtr efuse_read;
    unsigned int emmc_base_addr;

    secCheckPtr secure_verification;

    int errNo;

    /* �������ݶ� */
    unsigned int RMA_Enable;
    unsigned int SOC_ID[8];
    unsigned int ProvisionKey[4];

    unsigned int ulEnumFlag;      /* �Ծ���λ��־,AXI Mem Top - 8 */
    unsigned int ulOcrInitedFlag; /* AXI Mem Top - 4 */
} tOcrShareData;                  /* AXI Mem Top */

#endif /* __ASSEMBLY__ */

#endif
