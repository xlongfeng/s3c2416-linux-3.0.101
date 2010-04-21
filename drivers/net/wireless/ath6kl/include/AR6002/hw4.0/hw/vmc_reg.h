// ------------------------------------------------------------------
// Copyright (c) 2004-2007 Atheros Corporation.  All rights reserved.
// 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// Software distributed under the License is distributed on an "AS
// IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
//
//
// ------------------------------------------------------------------
//===================================================================
// Author(s): ="Atheros"
//===================================================================


#ifdef WLAN_HEADERS

#include "vmc_wlan_reg.h"


#ifndef BT_HEADERS

#define MC_BCAM_VALID_ADDRESS WLAN_MC_BCAM_VALID_ADDRESS
#define MC_BCAM_VALID_OFFSET WLAN_MC_BCAM_VALID_OFFSET
#define MC_BCAM_VALID_BIT_MSB WLAN_MC_BCAM_VALID_BIT_MSB
#define MC_BCAM_VALID_BIT_LSB WLAN_MC_BCAM_VALID_BIT_LSB
#define MC_BCAM_VALID_BIT_MASK WLAN_MC_BCAM_VALID_BIT_MASK
#define MC_BCAM_VALID_BIT_GET(x) WLAN_MC_BCAM_VALID_BIT_GET(x)
#define MC_BCAM_VALID_BIT_SET(x) WLAN_MC_BCAM_VALID_BIT_SET(x)
#define MC_BCAM_COMPARE_ADDRESS WLAN_MC_BCAM_COMPARE_ADDRESS
#define MC_BCAM_COMPARE_OFFSET WLAN_MC_BCAM_COMPARE_OFFSET
#define MC_BCAM_COMPARE_KEY_MSB WLAN_MC_BCAM_COMPARE_KEY_MSB
#define MC_BCAM_COMPARE_KEY_LSB WLAN_MC_BCAM_COMPARE_KEY_LSB
#define MC_BCAM_COMPARE_KEY_MASK WLAN_MC_BCAM_COMPARE_KEY_MASK
#define MC_BCAM_COMPARE_KEY_GET(x) WLAN_MC_BCAM_COMPARE_KEY_GET(x)
#define MC_BCAM_COMPARE_KEY_SET(x) WLAN_MC_BCAM_COMPARE_KEY_SET(x)
#define MC_BCAM_TARGET_ADDRESS WLAN_MC_BCAM_TARGET_ADDRESS
#define MC_BCAM_TARGET_OFFSET WLAN_MC_BCAM_TARGET_OFFSET
#define MC_BCAM_TARGET_INST_MSB WLAN_MC_BCAM_TARGET_INST_MSB
#define MC_BCAM_TARGET_INST_LSB WLAN_MC_BCAM_TARGET_INST_LSB
#define MC_BCAM_TARGET_INST_MASK WLAN_MC_BCAM_TARGET_INST_MASK
#define MC_BCAM_TARGET_INST_GET(x) WLAN_MC_BCAM_TARGET_INST_GET(x)
#define MC_BCAM_TARGET_INST_SET(x) WLAN_MC_BCAM_TARGET_INST_SET(x)
#define APB_ADDR_ERROR_CONTROL_ADDRESS WLAN_APB_ADDR_ERROR_CONTROL_ADDRESS
#define APB_ADDR_ERROR_CONTROL_OFFSET WLAN_APB_ADDR_ERROR_CONTROL_OFFSET
#define APB_ADDR_ERROR_CONTROL_QUAL_ENABLE_MSB WLAN_APB_ADDR_ERROR_CONTROL_QUAL_ENABLE_MSB
#define APB_ADDR_ERROR_CONTROL_QUAL_ENABLE_LSB WLAN_APB_ADDR_ERROR_CONTROL_QUAL_ENABLE_LSB
#define APB_ADDR_ERROR_CONTROL_QUAL_ENABLE_MASK WLAN_APB_ADDR_ERROR_CONTROL_QUAL_ENABLE_MASK
#define APB_ADDR_ERROR_CONTROL_QUAL_ENABLE_GET(x) WLAN_APB_ADDR_ERROR_CONTROL_QUAL_ENABLE_GET(x)
#define APB_ADDR_ERROR_CONTROL_QUAL_ENABLE_SET(x) WLAN_APB_ADDR_ERROR_CONTROL_QUAL_ENABLE_SET(x)
#define APB_ADDR_ERROR_CONTROL_ENABLE_MSB WLAN_APB_ADDR_ERROR_CONTROL_ENABLE_MSB
#define APB_ADDR_ERROR_CONTROL_ENABLE_LSB WLAN_APB_ADDR_ERROR_CONTROL_ENABLE_LSB
#define APB_ADDR_ERROR_CONTROL_ENABLE_MASK WLAN_APB_ADDR_ERROR_CONTROL_ENABLE_MASK
#define APB_ADDR_ERROR_CONTROL_ENABLE_GET(x) WLAN_APB_ADDR_ERROR_CONTROL_ENABLE_GET(x)
#define APB_ADDR_ERROR_CONTROL_ENABLE_SET(x) WLAN_APB_ADDR_ERROR_CONTROL_ENABLE_SET(x)
#define APB_ADDR_ERROR_STATUS_ADDRESS WLAN_APB_ADDR_ERROR_STATUS_ADDRESS
#define APB_ADDR_ERROR_STATUS_OFFSET WLAN_APB_ADDR_ERROR_STATUS_OFFSET
#define APB_ADDR_ERROR_STATUS_WRITE_MSB WLAN_APB_ADDR_ERROR_STATUS_WRITE_MSB
#define APB_ADDR_ERROR_STATUS_WRITE_LSB WLAN_APB_ADDR_ERROR_STATUS_WRITE_LSB
#define APB_ADDR_ERROR_STATUS_WRITE_MASK WLAN_APB_ADDR_ERROR_STATUS_WRITE_MASK
#define APB_ADDR_ERROR_STATUS_WRITE_GET(x) WLAN_APB_ADDR_ERROR_STATUS_WRITE_GET(x)
#define APB_ADDR_ERROR_STATUS_WRITE_SET(x) WLAN_APB_ADDR_ERROR_STATUS_WRITE_SET(x)
#define APB_ADDR_ERROR_STATUS_ADDRESS_MSB WLAN_APB_ADDR_ERROR_STATUS_ADDRESS_MSB
#define APB_ADDR_ERROR_STATUS_ADDRESS_LSB WLAN_APB_ADDR_ERROR_STATUS_ADDRESS_LSB
#define APB_ADDR_ERROR_STATUS_ADDRESS_MASK WLAN_APB_ADDR_ERROR_STATUS_ADDRESS_MASK
#define APB_ADDR_ERROR_STATUS_ADDRESS_GET(x) WLAN_APB_ADDR_ERROR_STATUS_ADDRESS_GET(x)
#define APB_ADDR_ERROR_STATUS_ADDRESS_SET(x) WLAN_APB_ADDR_ERROR_STATUS_ADDRESS_SET(x)
#define AHB_ADDR_ERROR_CONTROL_ADDRESS WLAN_AHB_ADDR_ERROR_CONTROL_ADDRESS
#define AHB_ADDR_ERROR_CONTROL_OFFSET WLAN_AHB_ADDR_ERROR_CONTROL_OFFSET
#define AHB_ADDR_ERROR_CONTROL_ENABLE_MSB WLAN_AHB_ADDR_ERROR_CONTROL_ENABLE_MSB
#define AHB_ADDR_ERROR_CONTROL_ENABLE_LSB WLAN_AHB_ADDR_ERROR_CONTROL_ENABLE_LSB
#define AHB_ADDR_ERROR_CONTROL_ENABLE_MASK WLAN_AHB_ADDR_ERROR_CONTROL_ENABLE_MASK
#define AHB_ADDR_ERROR_CONTROL_ENABLE_GET(x) WLAN_AHB_ADDR_ERROR_CONTROL_ENABLE_GET(x)
#define AHB_ADDR_ERROR_CONTROL_ENABLE_SET(x) WLAN_AHB_ADDR_ERROR_CONTROL_ENABLE_SET(x)
#define AHB_ADDR_ERROR_STATUS_ADDRESS WLAN_AHB_ADDR_ERROR_STATUS_ADDRESS
#define AHB_ADDR_ERROR_STATUS_OFFSET WLAN_AHB_ADDR_ERROR_STATUS_OFFSET
#define AHB_ADDR_ERROR_STATUS_MAC_MSB WLAN_AHB_ADDR_ERROR_STATUS_MAC_MSB
#define AHB_ADDR_ERROR_STATUS_MAC_LSB WLAN_AHB_ADDR_ERROR_STATUS_MAC_LSB
#define AHB_ADDR_ERROR_STATUS_MAC_MASK WLAN_AHB_ADDR_ERROR_STATUS_MAC_MASK
#define AHB_ADDR_ERROR_STATUS_MAC_GET(x) WLAN_AHB_ADDR_ERROR_STATUS_MAC_GET(x)
#define AHB_ADDR_ERROR_STATUS_MAC_SET(x) WLAN_AHB_ADDR_ERROR_STATUS_MAC_SET(x)
#define AHB_ADDR_ERROR_STATUS_MBOX_MSB WLAN_AHB_ADDR_ERROR_STATUS_MBOX_MSB
#define AHB_ADDR_ERROR_STATUS_MBOX_LSB WLAN_AHB_ADDR_ERROR_STATUS_MBOX_LSB
#define AHB_ADDR_ERROR_STATUS_MBOX_MASK WLAN_AHB_ADDR_ERROR_STATUS_MBOX_MASK
#define AHB_ADDR_ERROR_STATUS_MBOX_GET(x) WLAN_AHB_ADDR_ERROR_STATUS_MBOX_GET(x)
#define AHB_ADDR_ERROR_STATUS_MBOX_SET(x) WLAN_AHB_ADDR_ERROR_STATUS_MBOX_SET(x)
#define AHB_ADDR_ERROR_STATUS_ADDRESS_MSB WLAN_AHB_ADDR_ERROR_STATUS_ADDRESS_MSB
#define AHB_ADDR_ERROR_STATUS_ADDRESS_LSB WLAN_AHB_ADDR_ERROR_STATUS_ADDRESS_LSB
#define AHB_ADDR_ERROR_STATUS_ADDRESS_MASK WLAN_AHB_ADDR_ERROR_STATUS_ADDRESS_MASK
#define AHB_ADDR_ERROR_STATUS_ADDRESS_GET(x) WLAN_AHB_ADDR_ERROR_STATUS_ADDRESS_GET(x)
#define AHB_ADDR_ERROR_STATUS_ADDRESS_SET(x) WLAN_AHB_ADDR_ERROR_STATUS_ADDRESS_SET(x)
#define BCAM_CONFLICT_ERROR_ADDRESS WLAN_BCAM_CONFLICT_ERROR_ADDRESS
#define BCAM_CONFLICT_ERROR_OFFSET WLAN_BCAM_CONFLICT_ERROR_OFFSET
#define BCAM_CONFLICT_ERROR_IPORT_FLAG_MSB WLAN_BCAM_CONFLICT_ERROR_IPORT_FLAG_MSB
#define BCAM_CONFLICT_ERROR_IPORT_FLAG_LSB WLAN_BCAM_CONFLICT_ERROR_IPORT_FLAG_LSB
#define BCAM_CONFLICT_ERROR_IPORT_FLAG_MASK WLAN_BCAM_CONFLICT_ERROR_IPORT_FLAG_MASK
#define BCAM_CONFLICT_ERROR_IPORT_FLAG_GET(x) WLAN_BCAM_CONFLICT_ERROR_IPORT_FLAG_GET(x)
#define BCAM_CONFLICT_ERROR_IPORT_FLAG_SET(x) WLAN_BCAM_CONFLICT_ERROR_IPORT_FLAG_SET(x)
#define BCAM_CONFLICT_ERROR_DPORT_FLAG_MSB WLAN_BCAM_CONFLICT_ERROR_DPORT_FLAG_MSB
#define BCAM_CONFLICT_ERROR_DPORT_FLAG_LSB WLAN_BCAM_CONFLICT_ERROR_DPORT_FLAG_LSB
#define BCAM_CONFLICT_ERROR_DPORT_FLAG_MASK WLAN_BCAM_CONFLICT_ERROR_DPORT_FLAG_MASK
#define BCAM_CONFLICT_ERROR_DPORT_FLAG_GET(x) WLAN_BCAM_CONFLICT_ERROR_DPORT_FLAG_GET(x)
#define BCAM_CONFLICT_ERROR_DPORT_FLAG_SET(x) WLAN_BCAM_CONFLICT_ERROR_DPORT_FLAG_SET(x)
#define CPU_PERF_CNT_ADDRESS WLAN_CPU_PERF_CNT_ADDRESS
#define CPU_PERF_CNT_OFFSET WLAN_CPU_PERF_CNT_OFFSET
#define CPU_PERF_CNT_EN_MSB WLAN_CPU_PERF_CNT_EN_MSB
#define CPU_PERF_CNT_EN_LSB WLAN_CPU_PERF_CNT_EN_LSB
#define CPU_PERF_CNT_EN_MASK WLAN_CPU_PERF_CNT_EN_MASK
#define CPU_PERF_CNT_EN_GET(x) WLAN_CPU_PERF_CNT_EN_GET(x)
#define CPU_PERF_CNT_EN_SET(x) WLAN_CPU_PERF_CNT_EN_SET(x)
#define CPU_INST_FETCH_ADDRESS WLAN_CPU_INST_FETCH_ADDRESS
#define CPU_INST_FETCH_OFFSET WLAN_CPU_INST_FETCH_OFFSET
#define CPU_INST_FETCH_CNT_MSB WLAN_CPU_INST_FETCH_CNT_MSB
#define CPU_INST_FETCH_CNT_LSB WLAN_CPU_INST_FETCH_CNT_LSB
#define CPU_INST_FETCH_CNT_MASK WLAN_CPU_INST_FETCH_CNT_MASK
#define CPU_INST_FETCH_CNT_GET(x) WLAN_CPU_INST_FETCH_CNT_GET(x)
#define CPU_INST_FETCH_CNT_SET(x) WLAN_CPU_INST_FETCH_CNT_SET(x)
#define CPU_DATA_FETCH_ADDRESS WLAN_CPU_DATA_FETCH_ADDRESS
#define CPU_DATA_FETCH_OFFSET WLAN_CPU_DATA_FETCH_OFFSET
#define CPU_DATA_FETCH_CNT_MSB WLAN_CPU_DATA_FETCH_CNT_MSB
#define CPU_DATA_FETCH_CNT_LSB WLAN_CPU_DATA_FETCH_CNT_LSB
#define CPU_DATA_FETCH_CNT_MASK WLAN_CPU_DATA_FETCH_CNT_MASK
#define CPU_DATA_FETCH_CNT_GET(x) WLAN_CPU_DATA_FETCH_CNT_GET(x)
#define CPU_DATA_FETCH_CNT_SET(x) WLAN_CPU_DATA_FETCH_CNT_SET(x)
#define CPU_RAM1_CONFLICT_ADDRESS WLAN_CPU_RAM1_CONFLICT_ADDRESS
#define CPU_RAM1_CONFLICT_OFFSET WLAN_CPU_RAM1_CONFLICT_OFFSET
#define CPU_RAM1_CONFLICT_CNT_MSB WLAN_CPU_RAM1_CONFLICT_CNT_MSB
#define CPU_RAM1_CONFLICT_CNT_LSB WLAN_CPU_RAM1_CONFLICT_CNT_LSB
#define CPU_RAM1_CONFLICT_CNT_MASK WLAN_CPU_RAM1_CONFLICT_CNT_MASK
#define CPU_RAM1_CONFLICT_CNT_GET(x) WLAN_CPU_RAM1_CONFLICT_CNT_GET(x)
#define CPU_RAM1_CONFLICT_CNT_SET(x) WLAN_CPU_RAM1_CONFLICT_CNT_SET(x)
#define CPU_RAM2_CONFLICT_ADDRESS WLAN_CPU_RAM2_CONFLICT_ADDRESS
#define CPU_RAM2_CONFLICT_OFFSET WLAN_CPU_RAM2_CONFLICT_OFFSET
#define CPU_RAM2_CONFLICT_CNT_MSB WLAN_CPU_RAM2_CONFLICT_CNT_MSB
#define CPU_RAM2_CONFLICT_CNT_LSB WLAN_CPU_RAM2_CONFLICT_CNT_LSB
#define CPU_RAM2_CONFLICT_CNT_MASK WLAN_CPU_RAM2_CONFLICT_CNT_MASK
#define CPU_RAM2_CONFLICT_CNT_GET(x) WLAN_CPU_RAM2_CONFLICT_CNT_GET(x)
#define CPU_RAM2_CONFLICT_CNT_SET(x) WLAN_CPU_RAM2_CONFLICT_CNT_SET(x)
#define CPU_RAM3_CONFLICT_ADDRESS WLAN_CPU_RAM3_CONFLICT_ADDRESS
#define CPU_RAM3_CONFLICT_OFFSET WLAN_CPU_RAM3_CONFLICT_OFFSET
#define CPU_RAM3_CONFLICT_CNT_MSB WLAN_CPU_RAM3_CONFLICT_CNT_MSB
#define CPU_RAM3_CONFLICT_CNT_LSB WLAN_CPU_RAM3_CONFLICT_CNT_LSB
#define CPU_RAM3_CONFLICT_CNT_MASK WLAN_CPU_RAM3_CONFLICT_CNT_MASK
#define CPU_RAM3_CONFLICT_CNT_GET(x) WLAN_CPU_RAM3_CONFLICT_CNT_GET(x)
#define CPU_RAM3_CONFLICT_CNT_SET(x) WLAN_CPU_RAM3_CONFLICT_CNT_SET(x)
#define CPU_RAM4_CONFLICT_ADDRESS WLAN_CPU_RAM4_CONFLICT_ADDRESS
#define CPU_RAM4_CONFLICT_OFFSET WLAN_CPU_RAM4_CONFLICT_OFFSET
#define CPU_RAM4_CONFLICT_CNT_MSB WLAN_CPU_RAM4_CONFLICT_CNT_MSB
#define CPU_RAM4_CONFLICT_CNT_LSB WLAN_CPU_RAM4_CONFLICT_CNT_LSB
#define CPU_RAM4_CONFLICT_CNT_MASK WLAN_CPU_RAM4_CONFLICT_CNT_MASK
#define CPU_RAM4_CONFLICT_CNT_GET(x) WLAN_CPU_RAM4_CONFLICT_CNT_GET(x)
#define CPU_RAM4_CONFLICT_CNT_SET(x) WLAN_CPU_RAM4_CONFLICT_CNT_SET(x)


#endif
#endif



