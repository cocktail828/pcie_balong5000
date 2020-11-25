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
#ifndef __PCIE_CDEV_DBG_H__
#define __PCIE_CDEV_DBG_H__

#include <linux/kernel.h>
#include "bsp_print.h"
#include "bsp_slice.h"
#include "pcie_cdev.h"
#include "osl_types.h"

#define THIS_MODU mod_pcdev

extern struct pcdev_ctx g_pcdev_ctx;

/* pcdev msg level */
#define PCDEV_LEVEL_ERR BIT(0)
#define PCDEV_LEVEL_WARN BIT(1)
#define PCDEV_LEVEL_TRACE BIT(2)
#define PCDEV_LEVEL_INFO BIT(3)
#define PCDEV_LEVEL_DBG BIT(4)

#define PCDEV_SHOW(level, fmt, ...)                                                  \
    do                                                                               \
    {                                                                                \
        if (g_pcdev_ctx.msg_level & PCDEV_LEVEL_ERR)                                 \
            printk(KERN_ERR "[%d] %s: " fmt, __LINE__, __func__, ##__VA_ARGS__);     \
        else if (g_pcdev_ctx.msg_level & PCDEV_LEVEL_WARN)                           \
            printk(KERN_WARNING "[%d] %s: " fmt, __LINE__, __func__, ##__VA_ARGS__); \
        else if (g_pcdev_ctx.msg_level & PCDEV_LEVEL_TRACE)                          \
            printk(KERN_NOTICE "[%d] %s: " fmt, __LINE__, __func__, ##__VA_ARGS__);  \
        else if (g_pcdev_ctx.msg_level & PCDEV_LEVEL_INFO)                           \
            printk(KERN_INFO "[%d] %s: " fmt, __LINE__, __func__, ##__VA_ARGS__);    \
        else if (g_pcdev_ctx.msg_level & PCDEV_LEVEL_DBG)                            \
            printk(KERN_DEBUG "[%d] %s: " fmt, __LINE__, __func__, ##__VA_ARGS__);   \
    } while (0)

#define PCDEV_ERR(fmt, ...) PCDEV_SHOW(PCDEV_LEVEL_ERR, fmt, ##__VA_ARGS__)
#define PCDEV_WARN(fmt, ...) PCDEV_SHOW(PCDEV_LEVEL_WARN, fmt, ##__VA_ARGS__)
#define PCDEV_TRACE(fmt, ...) PCDEV_SHOW(PCDEV_LEVEL_TRACE, fmt, ##__VA_ARGS__)
#define PCDEV_INFO(fmt, ...) PCDEV_SHOW(PCDEV_LEVEL_INFO, fmt, ##__VA_ARGS__)

#define PCDEV_LINE PCDEV_TRACE("PCDEV_LINE\n")

#define print_pkt(_pnum, _buf, _len)                                                                                                            \
    do                                                                                                                                          \
    {                                                                                                                                           \
        int j, count;                                                                                                                           \
        static char buf[512];                                                                                                                   \
        count = _len > 64 ? 64 : _len;                                                                                                          \
        snprintf(buf, sizeof(buf), "buf addr: 0x%lx, len: %d, port %d\n", (uintptr_t)_buf, (int)_len, _pnum);                                   \
                                                                                                                                                \
        for (j = 0; j < count; j += 16)                                                                                                         \
            snprintf(buf + strlen(buf), sizeof(buf), "%03x: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", \
                     j, _buf[j], _buf[j + 1], _buf[j + 2], _buf[j + 3], _buf[j + 4], _buf[j + 5], _buf[j + 6],                                  \
                     _buf[j + 7], _buf[j + 8], _buf[j + 9], _buf[j + 0xa], _buf[j + 0xb], _buf[j + 0xc],                                        \
                     _buf[j + 0xd], _buf[j + 0xe], _buf[j + 0xf]);                                                                              \
        PCDEV_TRACE("%s", buf);                                                                                                                 \
    } while (0)

#endif
