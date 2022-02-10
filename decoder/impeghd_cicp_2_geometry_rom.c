/* 	Copyright (c) [2020]-[2021] Ittiam Systems Pvt. Ltd.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted (subject to the limitations in the
   disclaimer below) provided that the following conditions are met:
   •	Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   •	Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
   •	Neither the names of Dolby Laboratories, Inc. (or its affiliates),
   Ittiam Systems Pvt. Ltd. nor the names of its contributors may be used
   to endorse or promote products derived from this software without
   specific prior written permission.

   NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED
   BY THIS LICENSE. YOUR USE OF THE SOFTWARE MAY REQUIRE ADDITIONAL PATENT
   LICENSE(S) BY THIRD PARTIES, INCLUDING, WITHOUT LIMITATION, DOLBY
   LABORATORIES, INC. OR ANY OF ITS AFFILIATES. THIS SOFTWARE IS PROVIDED
   BY ITTIAM SYSTEMS LTD. AND ITS CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
   IN NO EVENT SHALL ITTIAM SYSTEMS LTD OR ITS CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------
*/

#include <impeghd_type_def.h>
#include "impeghd_error_codes.h"
#include "ia_core_coder_bitbuffer.h"
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"

/**
 * @defgroup CICPROM CICP ROM Tables
 * @ingroup  CICPROM
 * @brief CICP ROM Tables
 *
 * @{
 */

const WORD32 ia_channel_names[USED_CHANNEL] = {
    CH_M_L030, CH_M_R030, CH_M_000,  CH_LFE1,   CH_M_L110, CH_M_R110, CH_M_L022, CH_M_R022,
    CH_M_L135, CH_M_R135, CH_M_180,  CH_M_L090, CH_M_R090, CH_M_L060, CH_M_R060, CH_U_L030,
    CH_U_R030, CH_U_000,  CH_U_L135, CH_U_R135, CH_U_180,  CH_U_L090, CH_U_R090, CH_T_000,
    CH_LFE2,   CH_L_L045, CH_L_R045, CH_L_000,  CH_U_L110, CH_U_R110, CH_U_L045, CH_U_R045,
    CH_M_L045, CH_M_R045, CH_LFE3,   CH_M_L150, CH_M_R150};
const WORD32 ia_ele_ordering[][3] = {
    /* center */
    {CH_L_000, CH_M_000, CH_U_000},
    {CH_L_000, CH_M_L022, CH_U_000},
    {CH_L_000, CH_M_R022, CH_U_000},
    /* front left */
    {CH_L_L045, CH_M_L030, CH_U_L030},
    {CH_L_L045, CH_M_L030, CH_U_L045},
    {CH_L_L045, CH_M_L045, CH_U_L030},
    {CH_L_L045, CH_M_L045, CH_U_L045},
    {CH_L_L045, CH_M_L060, CH_U_L030},
    {CH_L_L045, CH_M_L060, CH_U_L045},
    /* front right */
    {CH_L_R045, CH_M_R030, CH_U_R030},
    {CH_L_R045, CH_M_R030, CH_U_R045},
    {CH_L_R045, CH_M_R045, CH_U_R030},
    {CH_L_R045, CH_M_R045, CH_U_R045},
    {CH_L_R045, CH_M_R060, CH_U_R030},
    {CH_L_R045, CH_M_R060, CH_U_R045},
    /* surround center */
    {CH_M_180, CH_U_180, -1},
    /* surround left */
    {CH_M_L090, CH_U_L090, -1},
    {CH_M_L110, CH_U_L110, -1},
    {CH_M_L135, CH_U_L135, -1},
    {CH_M_L090, CH_U_L110, -1},
    {CH_M_L090, CH_U_L135, -1},
    {CH_M_L110, CH_U_L090, -1},
    {CH_M_L110, CH_U_L135, -1},
    {CH_M_L135, CH_U_L090, -1},
    {CH_M_L135, CH_U_L135, -1},
    /* surround right */
    {CH_M_R090, CH_U_R090, -1},
    {CH_M_R110, CH_U_R110, -1},
    {CH_M_R135, CH_U_R135, -1},
    {CH_M_R090, CH_U_R110, -1},
    {CH_M_R090, CH_U_R135, -1},
    {CH_M_R110, CH_U_R090, -1},
    {CH_M_R110, CH_U_R135, -1},
    {CH_M_R135, CH_U_R090, -1},
    {CH_M_R135, CH_U_R135, -1},
    {-1, -1, -1},
};
const WORD32 impgehd_cicp_get_num_ls[] = {0, 1, 2,  3, 4,  5,  6,  8,  0,  3, 4,
                                          7, 8, 24, 8, 12, 10, 12, 14, 12, 14};

const ia_cicp_ls_geo_str ia_cicp_ls_geo_tbls[CICP_MAX_CH] = {
    /* ls_azimuth, ls_elevation, ls_azimuth_start, ls_azimuth_end,
       ls_elevation_start, ls_elevation_end, lfe_flag, screen_rel_flag */

    /* CH_M_L030 */ {+30, 0, +23, +37, -9, +20, 0, 0},
    /* CH_M_R030 */ {-30, 0, -37, -23, -9, +20, 0, 0},
    /* CH_M_000  */ {0, 0, -7, +7, -9, +20, 0, 0},
    /* CH_LFE1   */ {0, 0, 0, 0, 0, 0, 1, 0},
    /* CH_M_L110 */ {+110, 0, +101, +124, -45, +20, 0, 0},
    /* CH_M_R110 */ {-110, 0, -124, -101, -45, +20, 0, 0},
    /* CH_M_L022 */ {+22, 0, +8, +22, -9, +20, 0, 0},
    /* CH_M_R022 */ {-22, 0, -22, -8, -9, +20, 0, 0},
    /* CH_M_L135 */ {+135, 0, 125, 142, -45, +20, 0, 0},
    /* CH_M_R135 */ {-135, 0, -142, -125, -45, +20, 0, 0},
    /* CH_M_180  */ {180, 0, 158, -158, -45, +20, 0, 0},
    /* dummy     */ {0, 0, 0, 0, 0, 0, 0, 0},
    /* dummy     */ {0, 0, 0, 0, 0, 0, 0, 0},
    /* CH_M_L090 */ {+90, 0, +76, +100, -45, +20, 0, 0},
    /* CH_M_R090 */ {-90, 0, -100, -76, -45, +20, 0, 0},
    /* CH_M_L060 */ {+60, 0, +53, +75, -9, +20, 0, 0},
    /* CH_M_R060 */ {-60, 0, -75, -53, -9, +20, 0, 0},
    /* CH_U_L030 */ {+30, +35, +11, +37, +21, +60, 0, 0},
    /* CH_U_R030 */ {-30, +35, -37, -11, +21, +60, 0, 0},
    /* CH_U_000  */ {0, +35, -10, +10, +21, +60, 0, 0},
    /* CH_U_L135 */ {+135, +35, +125, +157, +21, +60, 0, 0},
    /* CH_U_R135 */ {-135, +35, -157, -125, +21, +60, 0, 0},
    /* CH_U_180  */ {180, +35, +158, -158, +21, +60, 0, 0},
    /* CH_U_L090 */ {+90, +35, +67, +100, +21, +60, 0, 0},
    /* CH_U_R090 */ {-90, +35, -100, -67, +21, +60, 0, 0},
    /* CH_T_000  */ {0, +90, -180, +180, +61, +90, 0, 0},
    /* CH_LFE2   */ {+45, -15, 0, 0, 0, 0, 1, 0},
    /* CH_L_L045 */ {+45, -15, +11, +75, -45, -10, 0, 0},
    /* CH_L_R045 */ {-45, -15, -75, -11, -45, -10, 0, 0},
    /* CH_L_000  */ {0, -15, -10, +10, -45, -10, 0, 0},
    /* CH_U_L110 */ {+110, +35, +101, +124, +21, +60, 0, 0},
    /* CH_U_R110 */ {-110, +35, -124, -101, +21, +60, 0, 0},
    /* CH_U_L045 */ {+45, +35, +38, +66, +21, +60, 0, 0},
    /* CH_U_R045 */ {-45, +35, -66, -38, +21, +60, 0, 0},
    /* CH_M_L045 */ {+45, 0, +38, +52, -9, +20, 0, 0},
    /* CH_M_R045 */ {-45, 0, -52, -38, -9, +20, 0, 0},
    /* CH_LFE3   */ {-45, -15, 0, 0, 0, 0, 1, 0},
    /* CH_M_LSCR */ {+60, 0, 0, 0, 0, 0, 0, 1},
    /* CH_M_RSCR */ {-60, 0, 0, 0, 0, 0, 0, 1},
    /* CH_M_LSCH */ {+30, 0, 0, 0, 0, 0, 0, 1},
    /* CH_M_RSCH */ {-30, 0, 0, 0, 0, 0, 0, 1},
    /* CH_M_L150 */ {+150, 0, 143, 157, -45, +20, 0, 0},
    /* CH_M_R150 */ {-150, 0, -157, -143, -45, +20, 0, 0}};

const WORD32 ia_cicp_idx_1_ls_set[1] = {CH_M_000};
const WORD32 ia_cicp_idx_2_ls_set[2] = {CH_M_L030, CH_M_R030};
const WORD32 ia_cicp_idx_3_ls_set[3] = {CH_M_L030, CH_M_R030, CH_M_000};
const WORD32 ia_cicp_idx_4_ls_set[4] = {CH_M_L030, CH_M_R030, CH_M_000, CH_M_180};
const WORD32 ia_cicp_idx_5_ls_set[5] = {CH_M_L030, CH_M_R030, CH_M_000, CH_M_L110, CH_M_R110};
const WORD32 ia_cicp_idx_6_ls_set[6] = {CH_M_L030, CH_M_R030, CH_M_000,
                                        CH_LFE1,   CH_M_L110, CH_M_R110};
const WORD32 ia_cicp_idx_7_ls_set[8] = {CH_M_L030, CH_M_R030, CH_M_000,  CH_LFE1,
                                        CH_M_L110, CH_M_R110, CH_M_L060, CH_M_R060};
const WORD32 ia_cicp_idx_9_ls_set[3] = {CH_M_L030, CH_M_R030, CH_M_180};
const WORD32 ia_cicp_idx_10_ls_set[4] = {CH_M_L030, CH_M_R030, CH_M_L110, CH_M_R110};
const WORD32 ia_cicp_idx_11_ls_set[7] = {CH_M_L030, CH_M_R030, CH_M_000, CH_LFE1,
                                         CH_M_L110, CH_M_R110, CH_M_180};
const WORD32 ia_cicp_idx_12_ls_set[8] = {CH_M_L030, CH_M_R030, CH_M_000,  CH_LFE1,
                                         CH_M_L110, CH_M_R110, CH_M_L135, CH_M_R135};
const WORD32 ia_cicp_idx_13_ls_set[24] = {
    CH_M_L060, CH_M_R060, CH_M_000,  CH_LFE2,   CH_M_L135, CH_M_R135, CH_M_L030, CH_M_R030,
    CH_M_180,  CH_LFE3,   CH_M_L090, CH_M_R090, CH_U_L045, CH_U_R045, CH_U_000,  CH_T_000,
    CH_U_L135, CH_U_R135, CH_U_L090, CH_U_R090, CH_U_180,  CH_L_000,  CH_L_L045, CH_L_R045};
const WORD32 ia_cicp_idx_14_ls_set[8] = {CH_M_L030, CH_M_R030, CH_M_000,  CH_LFE1,
                                         CH_M_L110, CH_M_R110, CH_U_L030, CH_U_R030};
const WORD32 ia_cicp_idx_15_ls_set[12] = {CH_M_L030, CH_M_R030, CH_M_000,  CH_LFE2,
                                          CH_M_L135, CH_M_R135, CH_LFE3,   CH_M_L090,
                                          CH_M_R090, CH_U_L045, CH_U_R045, CH_U_180};
const WORD32 ia_cicp_idx_16_ls_set[10] = {CH_M_L030, CH_M_R030, CH_M_000,  CH_LFE1,   CH_M_L110,
                                          CH_M_R110, CH_U_L030, CH_U_R030, CH_U_L110, CH_U_R110};
const WORD32 ia_cicp_idx_17_ls_set[12] = {CH_M_L030, CH_M_R030, CH_M_000,  CH_LFE1,
                                          CH_M_L110, CH_M_R110, CH_U_L030, CH_U_R030,
                                          CH_U_000,  CH_U_L110, CH_U_R110, CH_T_000};
const WORD32 ia_cicp_idx_18_ls_set[14] = {CH_M_L030, CH_M_R030, CH_M_000,  CH_LFE1,   CH_M_L110,
                                          CH_M_R110, CH_M_L150, CH_M_R150, CH_U_L030, CH_U_R030,
                                          CH_U_000,  CH_U_L110, CH_U_R110, CH_T_000};
const WORD32 ia_cicp_idx_19_ls_set[12] = {CH_M_L030, CH_M_R030, CH_M_000,  CH_LFE1,
                                          CH_M_L135, CH_M_R135, CH_M_L090, CH_M_R090,
                                          CH_U_L030, CH_U_R030, CH_U_L135, CH_U_R135};
const WORD32 ia_cicp_idx_20_ls_set[14] = {CH_M_L030, CH_M_R030, CH_M_000,  CH_LFE1,   CH_M_L135,
                                          CH_M_R135, CH_M_L090, CH_M_R090, CH_U_L045, CH_U_R045,
                                          CH_U_L135, CH_U_R135, CH_M_LSCR, CH_M_RSCR};

const WORD32 *ia_cicp_idx_ls_set_map_tbl[] = {NULL,
                                              &ia_cicp_idx_1_ls_set[0],
                                              &ia_cicp_idx_2_ls_set[0],
                                              &ia_cicp_idx_3_ls_set[0],
                                              &ia_cicp_idx_4_ls_set[0],
                                              &ia_cicp_idx_5_ls_set[0],
                                              &ia_cicp_idx_6_ls_set[0],
                                              &ia_cicp_idx_7_ls_set[0],
                                              NULL,
                                              &ia_cicp_idx_9_ls_set[0],
                                              &ia_cicp_idx_10_ls_set[0],
                                              &ia_cicp_idx_11_ls_set[0],
                                              &ia_cicp_idx_12_ls_set[0],
                                              &ia_cicp_idx_13_ls_set[0],
                                              &ia_cicp_idx_14_ls_set[0],
                                              &ia_cicp_idx_15_ls_set[0],
                                              &ia_cicp_idx_16_ls_set[0],
                                              &ia_cicp_idx_17_ls_set[0],
                                              &ia_cicp_idx_18_ls_set[0],
                                              &ia_cicp_idx_19_ls_set[0],
                                              &ia_cicp_idx_20_ls_set[0]};
/** @} */ /* End of CICPROM */
