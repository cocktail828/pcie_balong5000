/* MD5: 90285a8a07adfd7dd1900c36756dc1a1*/
/*
 * copyright (C) Huawei Technologies Co., Ltd. 2012-2015. All rights reserved.
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

#if !defined(__PRODUCT_CONFIG_LTEVPHY_H__)
#define __PRODUCT_CONFIG_LTEVPHY_H__

#ifndef XTENSA_CORE_X_CACHE
#define XTENSA_CORE_X_CACHE Phoenix_NX
#endif

#ifndef XTENSA_CORE_X_SYSTEM
#define XTENSA_CORE_X_SYSTEM "RH-2018.5"
#endif

#ifndef XTENSA_CORE
#define XTENSA_CORE "$(CFG_XTENSA_CORE_X_CACHE)"
#endif

#ifndef XTENSA_SYSTEM
#define XTENSA_SYSTEM "$(ROOT_XTENSA_PATH_W)/XtDevTools/install/builds/$(CFG_XTENSA_CORE_X_SYSTEM)/$(CFG_XTENSA_CORE)/config"
#endif

#ifndef TENSILICA_BUILDS
#define TENSILICA_BUILDS "$(ROOT_XTENSA_PATH_W)/XtDevTools/install/builds/$(CFG_XTENSA_CORE_X_SYSTEM)"
#endif

#ifndef TENSILICA_TOOLS
#define TENSILICA_TOOLS "$(ROOT_XTENSA_PATH_W)/XtDevTools/install/tools/$(CFG_XTENSA_CORE_X_SYSTEM)"
#endif

#ifndef XTENSA_PREDICT_BUG
#define XTENSA_PREDICT_BUG
#endif

#ifndef TL_PHY_ASIC
#define TL_PHY_ASIC
#endif

#ifndef XTENSA_INST_PREFETCH_BUG
#endif

#ifndef XTENSA_DATA_PREFETCH_BUG
#define XTENSA_DATA_PREFETCH_BUG
#endif

#ifndef DSP1_DTCM_BASE
#define DSP1_DTCM_BASE 0xE3400000
#endif

#ifndef DSP1_DTCM_GLB_BASE
#define DSP1_DTCM_GLB_BASE 0xE3000000
#endif

#ifndef DSP1_ITCM_BASE
#define DSP1_ITCM_BASE 0xE3500000
#endif

#ifndef DSP1_DTCM_SIZE
#define DSP1_DTCM_SIZE 0x80000
#endif

#ifndef DSP1_ITCM_SIZE
#define DSP1_ITCM_SIZE 0x60000
#endif

#ifndef DSP1_L2M_BASE
#define DSP1_L2M_BASE 0xE3700000
#endif

#ifndef DSP1_L2M_SIZE
#define DSP1_L2M_SIZE 0x80000
#endif

#ifndef DSP1_L2C_BASE
#define DSP1_L2C_BASE 0xE3780000
#endif

#ifndef DSP1_L2C_SIZE
#define DSP1_L2C_SIZE 0x80000
#endif

#ifndef DSP1_SRAM_BASE
#define DSP1_SRAM_BASE 0x0
#endif

#ifndef DSP1_SRAM_SIZE
#define DSP1_SRAM_SIZE 0x0
#endif

#ifndef DSP1_DDR_BASE
#define DSP1_DDR_BASE ((DDR_CBBE_IMAGE_ADDR) + 0x10000)
#endif

#ifndef DSP1_DDR_SIZE
#define DSP1_DDR_SIZE 0xC00000
#endif

#ifndef DSP1_TOTAL_IMG_SIZE
#define DSP1_TOTAL_IMG_SIZE (((DSP1_DTCM_SIZE) + (DSP1_ITCM_SIZE) + (DSP1_L2M_SIZE) + (DSP1_DDR_SIZE)))
#endif

#ifndef FEATURE_LTEV_EASYRF
#define FEATURE_LTEV_EASYRF FEATURE_ON
#endif

#ifndef LTEV_PUB_DTCM_GLB_MINUS_LOCAL
#define LTEV_PUB_DTCM_GLB_MINUS_LOCAL 0x00400000
#endif

#endif /*__PRODUCT_CONFIG_H__*/
