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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include "securec.h"
#include "pcie_cdev.h"
#include "pcie_cdev_dbg.h"
#include "bsp_pcdev.h"
#include "bsp_softtimer.h"
#include <linux/module.h>

#include <linux/timer.h>

#include <linux/jiffies.h>

//struct timer_list mytimer;

#define PDEV_TEST_MAX_BUFSIZE 2048
#define PDEV_TEST_MAX_TXBUF_SIZE 0X100

extern struct pcdev_ctx g_pcdev_ctx;
extern struct pcie_cdev_port_manager g_pcie_cdev_ports[PCIE_CDEV_COUNT];

static char *g_st_pcdev_buf = "abcdef";
static char *g_st_pcdev_buf2 = NULL;
static void *g_st_pcdev_filep[pcdev_port_bottom] = {NULL};
static struct task_struct *g_pcie_cdev_stress_task = NULL;
static unsigned int g_pcie_cdev_stress_task_running = 0;
static unsigned int g_pcie_cdev_stress_run_count = 0;

static u32 g_pcdev_test_count = 20;
static u32 g_pcdev_wake_timer_ms = 2000;
static volatile int g_pcdev_timer_enable = 0x1;
static u32 g_pcdev_wake_timer_count = 0;
static struct softtimer_list g_pcdev_off_timer;
#if 0
/* ************************************  ST  *************************************** */

void st_pcdev_write_cb_0(char *pVirAddr, unsigned int size, unsigned int MuxId, void *pDrvPriv)
{
    return;
}

void st_pcdev_read_cb_0(void)
{
    pcdev_wr_async_info_s rw_info;
    int ret;

    unsigned int port_n = 0;

    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_GET_RD_BUFF, (void *)&rw_info);

    if (rw_info.size > 0 && rw_info.size <= PDEV_TEST_MAX_BUFSIZE && rw_info.p_vaddr) {
        print_pkt(port_n, rw_info.p_vaddr, rw_info.size);
        ret = memset_s(rw_info.p_vaddr, PDEV_TEST_MAX_BUFSIZE, 0, rw_info.size);
        if (ret) {
            PCDEV_ERR("memset_s failed, line: %d \n", __LINE__);
        }
    } else {
        PCDEV_ERR("err   buf:%lx  size:%d \n", (uintptr_t)rw_info.p_vaddr, rw_info.size);
    }

    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_RETURN_BUFF, (void *)&rw_info);
    return;
}

void st_pcdev_write_cb_1(char *pVirAddr, unsigned int size, unsigned int MuxId, void *pDrvPriv)
{
    return;
}

void st_pcdev_read_cb_1(void)
{
    pcdev_wr_async_info_s rw_info;
    int ret;

    unsigned int port_n = 1;

    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_GET_RD_BUFF, (void *)&rw_info);

    if (rw_info.size > 0 && rw_info.size <= PDEV_TEST_MAX_BUFSIZE && rw_info.p_vaddr) {
        ret = memset_s(rw_info.p_vaddr, PDEV_TEST_MAX_BUFSIZE, 0, rw_info.size);
        if (ret) {
            PCDEV_ERR("memset_s failed, line: %d \n", __LINE__);
        }
    } else {
        printk("st_pcdev_read_cb_1  err   buf:%lx  size:%d \n", (uintptr_t)rw_info.p_vaddr, rw_info.size);
    }
    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_RETURN_BUFF, (void *)&rw_info);
    return;
}

void st_pcdev_event_cb_0(pcdev_evt_e evt)
{
    PCDEV_ERR("event:%d \n", evt);
    return;
}

void st_pcdev_event_cb_1(pcdev_evt_e evt)
{
    PCDEV_ERR("event:%d \n", evt);
    return;
}

int st_pcdev_event_send(unsigned int port_n, pcdev_evt_e evt)
{
    if (!g_st_pcdev_filep[port_n]) {
        PCDEV_ERR("filep is null \n");
        return -1;
    }

    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SEND_EVT, (void *)&evt);
    return 0;
}

void st_pcdev_enum_done_cb_0(void)
{
    PCDEV_ERR("in \n");
    return;
}

void st_pcdev_enum_done_cb_1(void)
{
    PCDEV_ERR("in \n");
    return;
}

void st_pcdev_enumdone_regist(void)
{
    mdrv_pcdev_reg_enumdonecb(st_pcdev_enum_done_cb_0);
    mdrv_pcdev_reg_enumdonecb(st_pcdev_enum_done_cb_1);
}

void st_pcdev_open_0(void)
{
    unsigned int port_n = 0;
    void *pcdev_filep = NULL;

    pcdev_filep = bsp_pcdev_open(port_n);
    if (pcdev_filep == NULL) {
        PCDEV_ERR("st_pcdev_open port(%d) fail\n", port_n);
        return;
    }
    g_st_pcdev_filep[port_n] = pcdev_filep;
    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SET_READ_CB, st_pcdev_read_cb_0);
    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SET_WRITE_CB, st_pcdev_write_cb_0);
    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SET_EVT_CB, st_pcdev_event_cb_0);

    return;
}

void st_pcdev_open_1(void)
{
    unsigned int port_n = 1;
    void *pcdev_filep = NULL;

    pcdev_filep = bsp_pcdev_open(port_n);
    if (pcdev_filep == NULL) {
        PCDEV_ERR("port(%d) fail\n", port_n);
        return;
    }
    g_st_pcdev_filep[port_n] = pcdev_filep;
    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SET_READ_CB, st_pcdev_read_cb_1);
    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SET_WRITE_CB, st_pcdev_write_cb_1);
    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SET_EVT_CB, st_pcdev_event_cb_1);

    return;
}

int st_pcdev_send(unsigned int port_n)
{
    pcdev_wr_async_info_s rw_info;

    if (!g_st_pcdev_filep[port_n]) {
        return -1;
    }

    rw_info.p_vaddr = g_st_pcdev_buf;
    rw_info.size = 8;
    rw_info.p_priv = NULL;
    rw_info.p_paddr = NULL;

    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_WRITE_ASYNC, &rw_info);

    return 0;
}

int st_pcdev_send_n(unsigned int port_n, unsigned int packet_n)
{
    pcdev_wr_async_info_s rw_info;
    unsigned int i;

    if (!g_st_pcdev_filep[port_n]) {
        return -1;
    }

    rw_info.p_vaddr = g_st_pcdev_buf;
    rw_info.size = 8;
    rw_info.p_priv = NULL;
    rw_info.p_paddr = NULL;

    for (i = 0; i < packet_n; i++) {
        bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_WRITE_ASYNC, &rw_info);
    }

    return 0;
}

int st_pcdev_sync_send_n(unsigned int port_n, unsigned int packet_n)
{
    unsigned int i;

    if (!g_st_pcdev_filep[port_n]) {
        return -1;
    }

    for (i = 0; i < packet_n; i++) {
        bsp_pcdev_write(g_st_pcdev_filep[port_n], g_st_pcdev_buf, 8);
    }

    return 0;
}

int pcdev_stress_test_thread(void *data)
{
    pcdev_wr_async_info_s rw_info;
    unsigned int i;

    if (!g_st_pcdev_filep[0]) {
        return -1;
    }
    if (!g_st_pcdev_filep[1]) {
        return -1;
    }

    rw_info.p_vaddr = g_st_pcdev_buf;
    rw_info.size = 8;
    rw_info.p_priv = NULL;
    rw_info.p_paddr = NULL;

    for (i = 0;; i++) {
        if (!g_pcie_cdev_stress_task_running) {
            break;
        }
        bsp_pcdev_ioctl(g_st_pcdev_filep[0], PCDEV_IOCTL_WRITE_ASYNC, &rw_info);
        bsp_pcdev_ioctl(g_st_pcdev_filep[1], PCDEV_IOCTL_WRITE_ASYNC, &rw_info);
        if (!(i % 200)) {
            msleep(10);
        }
        g_pcie_cdev_stress_run_count++;
    }

    return 0;
}

int st_pcdev_stress_start(void)
{
    if (g_pcie_cdev_stress_task_running) {
        return 0;
    }

    g_pcie_cdev_stress_task = kthread_run(pcdev_stress_test_thread, NULL, "PCDEV_Test");
    if (IS_ERR(g_pcie_cdev_stress_task)) {
        g_pcie_cdev_stress_task = NULL;
        PCDEV_ERR("kthread_run %s fail\n", "PCDEV_Test");
        return -1;
    }

    g_pcie_cdev_stress_run_count = 0;
    g_pcie_cdev_stress_task_running = 1;

    return 0;
}

int st_pcdev_stress_stop(void)
{
    g_pcie_cdev_stress_task_running = 0;

    if (g_pcie_cdev_stress_task == NULL)
        return kthread_stop(g_pcie_cdev_stress_task);

    return 0;
}

void pcie_stress_test_info(void)
{
    PCDEV_ERR("stress_run_count:%d\n ", g_pcie_cdev_stress_run_count);
}

void st_pcdev_close_0(void)
{
    unsigned int port_n = 0;
    void *pcdev_filep = NULL;
    int ret;

    pcdev_filep = g_st_pcdev_filep[port_n];
    if (pcdev_filep == NULL) {
        PCDEV_ERR("port(%d) is closed\n", port_n);
        return;
    }

    g_st_pcdev_filep[port_n] = NULL;

    ret = bsp_pcdev_close(pcdev_filep);
    if (ret) {
        PCDEV_ERR("port(%d) bsp_pcdev_close fail\n", port_n);
    }

    return;
}

void st_pcdev_realloc_0(unsigned int buf_size)
{
    unsigned int port_n = 0;
    void *pcdev_filep = NULL;
    pcdev_read_buf_info_s buf_info;
    int ret;

    pcdev_filep = bsp_pcdev_open(port_n);
    if (pcdev_filep == NULL) {
        PCDEV_ERR("port(%d) fail\n", port_n);
        return;
    }

    buf_info.buf_size = buf_size;
    bsp_pcdev_ioctl(pcdev_filep, PCDEV_IOCTL_RELLOC_READ_BUFF, (void *)&buf_info);

    ret = bsp_pcdev_close(pcdev_filep);
    if (ret) {
        PCDEV_ERR("port(%d) bsp_pcdev_close fail\n", port_n);
    }

    return;
}
#endif
void st_pcdev_write_cb_test_at(char *pVirAddr, char *pPhyAddr, unsigned int size)
{
    PCDEV_ERR("buf:%lx  size:%d  paddr:%lx\n", (uintptr_t)pVirAddr, size, (uintptr_t)pPhyAddr);
    if (g_st_pcdev_buf2 != NULL)
    {
        kfree(g_st_pcdev_buf2);
        g_st_pcdev_buf2 = NULL;
    }
    return;
}

void st_pcdev_read_cb_test_at(void)
{
    pcdev_wr_async_info_s rw_info;
    int ret;

    unsigned int port_n = pcdev_ttyGS0;

    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_GET_RD_BUFF, (void *)&rw_info);

    if (rw_info.size > 0 && rw_info.size <= PDEV_TEST_MAX_BUFSIZE && rw_info.p_vaddr)
    {
        print_pkt1(port_n, rw_info.p_vaddr, rw_info.size);
        ret = memset_s(rw_info.p_vaddr, PDEV_TEST_MAX_BUFSIZE, 0, rw_info.size);
        if (ret)
        {
            PCDEV_ERR("memset_s failed, line: %d \n", __LINE__);
        }
    }
    else
    {
        PCDEV_ERR("err   buf:%lx  size:%d \n", (uintptr_t)rw_info.p_vaddr, rw_info.size);
    }
    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_RETURN_BUFF, (void *)&rw_info);
}

void st_pcdev_open_test_at(void)
{
    unsigned int port_n = pcdev_ttyGS0;
    void *pcdev_filep = NULL;

    pcdev_filep = bsp_pcdev_open(port_n);
    if (pcdev_filep == NULL)
    {
        PCDEV_ERR("port(%d) fail\n", port_n);
        return;
    }
    g_st_pcdev_filep[port_n] = pcdev_filep;
    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SET_READ_CB, st_pcdev_read_cb_test_at);
    bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SET_WRITE_CB, st_pcdev_write_cb_test_at);

    return;
}

void st_pcdev_close_test_at(void)
{
    unsigned int port_n = pcdev_ttyGS0;
    void *pcdev_filep = NULL;
    int ret;

    pcdev_filep = g_st_pcdev_filep[port_n];
    if (pcdev_filep == NULL)
    {
        PCDEV_ERR("port(%d) is closed\n", port_n);
        return;
    }

    g_st_pcdev_filep[port_n] = NULL;

    ret = bsp_pcdev_close(pcdev_filep);
    if (ret)
    {
        PCDEV_ERR("port(%d) bsp_pcdev_close fail\n", port_n);
    }

    return;
}

void st_pcdev_open_send_test_at(int num)
{
    unsigned int port_n = pcdev_ttyGS0;
    void *pcdev_filep = NULL;
    pcdev_wr_async_info_s rw_info;
    int ret;

    pcdev_filep = g_st_pcdev_filep[port_n];
    if (pcdev_filep == NULL)
    {
        pcdev_filep = bsp_pcdev_open(port_n);
        if (pcdev_filep == NULL)
        {
            PCDEV_ERR("port(%d) fail\n", port_n);
            return;
        }
        g_st_pcdev_filep[port_n] = pcdev_filep;
        bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SET_READ_CB, st_pcdev_read_cb_test_at);
        bsp_pcdev_ioctl(g_st_pcdev_filep[port_n], PCDEV_IOCTL_SET_WRITE_CB, st_pcdev_write_cb_test_at);
    }

    while (num > 0)
    {
        g_st_pcdev_buf2 = kzalloc(PDEV_TEST_MAX_TXBUF_SIZE, GFP_KERNEL);
        ret = memcpy_s(g_st_pcdev_buf2, PDEV_TEST_MAX_TXBUF_SIZE, "ati\r\n", sizeof("ati\r\n"));
        if (ret)
        {
            PCDEV_ERR("memcpy_s fail\n");
        }
        print_pkt(port_n, g_st_pcdev_buf2, 64);
        rw_info.p_vaddr = g_st_pcdev_buf2;
        rw_info.size = sizeof("ati\r\n") + 1;
        rw_info.p_priv = NULL;
        rw_info.p_paddr = NULL;

        bsp_pcdev_ioctl(pcdev_filep, PCDEV_IOCTL_WRITE_ASYNC, &rw_info);
        num--;
    }
    return;
}
