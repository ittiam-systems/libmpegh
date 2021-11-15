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

#include <math.h>
#include <stdio.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_interface.h"
#include "impd_drc_rom.h"
#include "impd_drc_selection_process.h"
#include "ia_core_coder_constants.h"

/**
 * @defgroup DRCROM DRCROM
 * @ingroup  DRCROM
 * @brief DRC ROM Tables
 *
 * @{
 */

const ia_drc_delta_param_table_entry_struct
    ia_drc_gain_tbls_prof_0_1[NUM_GAIN_TBL_PROF_0_1_ENTRIES] = {
        {2, 0x003, -0.125f},  {2, 0x002, 0.125f},   {3, 0x001, -0.250f},  {3, 0x002, 0.000f},
        {4, 0x000, -2.000f},  {5, 0x002, -0.500f},  {5, 0x00F, -0.375f},  {5, 0x00E, 1.000f},
        {6, 0x019, -0.625f},  {6, 0x018, 0.250f},   {6, 0x006, 0.375f},   {7, 0x00F, -1.000f},
        {7, 0x034, -0.875f},  {7, 0x036, -0.750f},  {7, 0x037, 0.500f},   {8, 0x01D, 0.625f},
        {9, 0x039, -1.875f},  {9, 0x0D5, -1.125f},  {9, 0x0D7, 0.750f},   {9, 0x0D4, 0.875f},
        {10, 0x070, -1.500f}, {10, 0x1AC, -1.375f}, {10, 0x1AD, -1.250f}, {11, 0x0E2, -1.750f},
        {11, 0x0E3, -1.625f}

};

const ia_drc_delta_param_table_entry_struct ia_drc_gain_tbls_prof_2[NUM_GAIN_TBL_PROF_2_ENTRIES] =
    {{3, 0x7, -0.125000f},    {4, 0xC, -0.625000f},    {4, 0x9, -0.500000f},
     {4, 0x5, -0.375000f},    {4, 0x3, -0.250000f},    {4, 0x1, 0.000000f},
     {4, 0xB, 0.125000f},     {5, 0x11, -0.875000f},   {5, 0xE, -0.750000f},
     {5, 0x5, 0.250000f},     {5, 0x4, 0.375000f},     {5, 0x8, 0.500000f},
     {5, 0x0, 0.625000f},     {5, 0xD, 0.750000f},     {5, 0xF, 0.875000f},
     {5, 0x10, 1.000000f},    {5, 0x1B, 1.125000f},    {6, 0x2B, -1.250000f},
     {6, 0x28, -1.125000f},   {6, 0x2, -1.000000f},    {6, 0x12, 1.250000f},
     {6, 0x18, 1.375000f},    {6, 0x29, 1.500000f},    {7, 0x6A, -4.000000f},
     {7, 0x54, -1.750000f},   {7, 0x68, -1.625000f},   {7, 0x26, -1.500000f},
     {7, 0x6, -1.375000f},    {7, 0x32, 1.625000f},    {8, 0xD2, -2.250000f},
     {8, 0xAB, -2.125000f},   {8, 0xAA, -2.000000f},   {8, 0x4F, -1.875000f},
     {8, 0x4E, 1.750000f},    {8, 0xD7, 1.875000f},    {8, 0xE, 2.000000f},
     {9, 0x1AD, -3.625000f},  {9, 0x1AC, -3.375000f},  {9, 0x1A6, -3.250000f},
     {9, 0xCD, -3.125000f},   {9, 0xCE, -2.750000f},   {9, 0x1A7, -2.625000f},
     {9, 0x1F, -2.500000f},   {9, 0xCC, -2.375000f},   {10, 0x3C, -3.500000f},
     {10, 0x19E, -3.000000f}, {10, 0x19F, -2.875000f}, {11, 0x7A, -3.875000f},
     {11, 0x7B, -3.750000f}

};

const ia_drc_delta_param_table_entry_struct
    ia_drc_slope_code_tbl_entries_by_size[NUM_SLOPE_TBL_ENTRIES] = {
        {1, 0x001, 0.0f},     {2, 0x000, -0.005f},  {4, 0x007, 0.005f},  {5, 0x00A, -0.1953f},
        {5, 0x009, -0.0781f}, {5, 0x00D, -0.0312f}, {5, 0x00B, 0.0312f}, {6, 0x018, -3.0518f},
        {6, 0x011, 0.0781f},  {7, 0x032, -0.4883f}, {7, 0x020, 1.2207f}, {7, 0x033, 3.0518f},
        {8, 0x042, -1.2207f}, {9, 0x087, 0.1953f},  {9, 0x086, 0.4883f},
};

const WORD32 ia_drc_characteristic_order_default[][3] = {
    {1, 2, -1}, {2, 3, 1},   {3, 4, 2},  {4, 5, 3},   {5, 6, 4},  {6, 5, -1},
    {7, 9, -1}, {8, 10, -1}, {9, 7, -1}, {10, 8, -1}, {11, 10, 9}};

const WORD32 ia_drc_measurement_system_def_tbl[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const WORD32 ia_drc_ameasurement_system_bs1770_3_tbl[] = {0, 0, 8, 0, 1, 3, 0, 5, 6, 7, 4, 2};
const WORD32 ia_drc_measurement_system_user_tbl[] = {0, 0, 1, 0, 8, 5, 0, 2, 3, 4, 6, 7};
const WORD32 ia_drc_measurement_system_expert_tbl[] = {0, 0, 3, 0, 1, 8, 0, 4, 5, 6, 7, 2};
const WORD32 ia_drc_measurement_system_rms_a_tbl[] = {0, 0, 5, 0, 1, 3, 0, 8, 6, 7, 4, 2};
const WORD32 ia_drc_measurement_system_rms_b_tbl[] = {0, 0, 5, 0, 1, 3, 0, 6, 8, 7, 4, 2};
const WORD32 ia_drc_measurement_system_rms_c_tbl[] = {0, 0, 5, 0, 1, 3, 0, 6, 7, 8, 4, 2};
const WORD32 ia_drc_measurement_system_rms_d_tbl[] = {0, 0, 3, 0, 1, 7, 0, 4, 5, 6, 8, 2};
const WORD32 ia_drc_measurement_system_rms_e_tbl[] = {0, 0, 1, 0, 7, 5, 0, 2, 3, 4, 6, 8};

const WORD32 ia_drc_measurement_method_prog_loudness_tbl[] = {0, 0, 1, 0, 0, 0, 0, 2, 3, 4, 0, 0};
const WORD32 ia_drc_measurement_method_peak_loudness_tbl[] = {0, 7, 0, 0, 0, 0, 6, 5, 4, 3, 2, 1};

const ia_drc_loc_sys_interface_struct ia_drc_loc_sys_interface[] = {
    {0, 1, {0, 0, 0}, 0, 0}, {0, 1, {0, 0, 0}, 0, 0}, {0, 1, {0, 0, 0}, 0, 0},
    {0, 1, {0, 0, 0}, 0, 0}, {0, 1, {0, 0, 0}, 0, 0}, {0, 1, {0, 0, 0}, 0, 0},
    {0, 1, {0, 0, 0}, 0, 0}, {0, 1, {1, 0, 0}, 0, 0}, {0, 1, {0, 0, 0}, 0, 0},
    {0, 1, {0, 0, 0}, 0, 0}, {0, 1, {1, 0, 0}, 0, 0}, {0, 1, {1, 0, 0}, 0, 0},
    {0, 1, {1, 0, 0}, 0, 0}, {0, 1, {2, 0, 0}, 0, 0}, {0, 1, {0, 0, 0}, 0, 0},
    {0, 1, {3, 0, 0}, 0, 0}, {0, 1, {1, 0, 0}, 0, 0}, {0, 1, {0, 0, 0}, 0, 0},
    {0, 1, {2, 0, 0}, 0, 0}, {0, 1, {0, 0, 0}, 0, 0}};

const ia_drc_loc_loudness_norm_ctrl_interface_struct ia_drc_loc_loudness_norm_ctrl_interface[] = {
    {1, -23.0f}, {1, -23.0f}, {1, -23.0f}, {1, -23.0f}, {1, -20.0f}, {1, -23.0f}, {1, -31.0f},
    {1, -31.0f}, {1, -24.0f}, {1, -16.0f}, {1, -30.0f}, {1, -30.0f}, {1, -30.0f}, {1, -30.0f},
    {1, -30.0f}, {1, -10.0f}, {1, -30.0f}, {1, -30.0f}, {1, -24.0f}, {1, -24.0f}};
const ia_drc_loc_loudness_norm_param_interface_struct ia_drc_loc_loudness_norm_param_interface[] =
    {

        {0, 0, IA_DRC_LOUDNESS_DEVIATION_MAX_DEFAULT,
         IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS, IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3,
         IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, IA_DRC_LOUDNESS_DEVIATION_MAX_DEFAULT,
         IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS, IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3,
         IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, IA_DRC_LOUDNESS_DEVIATION_MAX_DEFAULT,
         IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS, IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3,
         IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, IA_DRC_LOUDNESS_DEVIATION_MAX_DEFAULT,
         IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS, IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3,
         IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, IA_DRC_LOUDNESS_DEVIATION_MAX_DEFAULT,
         IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS, IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3,
         IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, IA_DRC_LOUDNESS_DEVIATION_MAX_DEFAULT,
         IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS, IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3,
         IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 1, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 0.0f},
        {0, 1, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 0.0f},
        {0, 1, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 0.0f},
        {0, 1, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 0.0f},
        {0, 0, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f},
        {0, 0, 0, IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS,
         IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3, IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF, 20,
         IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT, 0.0f, 1.0f}

};
const ia_drc_loc_drc_interface_struct ia_drc_loc_dyn_range_ctrl_interface[] = {
    {1,
     3,
     {MATCH_EFFECT_TYPE, MATCH_DYNAMIC_RANGE, MATCH_DRC_CHARACTERISTIC},
     IA_DRC_SHORT_TERM_LOUDNESS_TO_AVG,
     1,
     5.0f,
     3.0f,
     10.0f,
     3},
    {1,
     3,
     {MATCH_EFFECT_TYPE, MATCH_DYNAMIC_RANGE, MATCH_DRC_CHARACTERISTIC},
     IA_DRC_TOP_OF_LOUDNESS_RANGE_TO_AVG,
     0,
     5.0f,
     3.0f,
     10.0f,
     3},
};
const ia_drc_loc_requested_drc_effect_struct ia_drc_loc_requested_drc_effect_type_str[] = {

    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {1,
     1,
     {EFFECT_TYPE_REQUESTED_NONE, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {1,
     1,
     {EFFECT_TYPE_REQUESTED_NONE, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {1,
     1,
     {EFFECT_TYPE_REQUESTED_GENERAL_COMPR, EFFECT_TYPE_REQUESTED_NOISY,
      EFFECT_TYPE_REQUESTED_LOWLEVEL, EFFECT_TYPE_REQUESTED_LIMITED,
      EFFECT_TYPE_REQUESTED_DIALOG}},
    {1,
     1,
     {EFFECT_TYPE_REQUESTED_GENERAL_COMPR, EFFECT_TYPE_REQUESTED_NOISY,
      EFFECT_TYPE_REQUESTED_LOWLEVEL, EFFECT_TYPE_REQUESTED_LIMITED,
      EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_NIGHT, EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LOWLEVEL,
      EFFECT_TYPE_REQUESTED_LIMITED, EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_GENERAL_COMPR, EFFECT_TYPE_REQUESTED_NOISY,
      EFFECT_TYPE_REQUESTED_LOWLEVEL, EFFECT_TYPE_REQUESTED_LIMITED,
      EFFECT_TYPE_REQUESTED_DIALOG}},
    {4,
     1,
     {EFFECT_TYPE_REQUESTED_GENERAL_COMPR, EFFECT_TYPE_REQUESTED_NOISY,
      EFFECT_TYPE_REQUESTED_LOWLEVEL, EFFECT_TYPE_REQUESTED_LIMITED,
      EFFECT_TYPE_REQUESTED_DIALOG}}

};

const ia_drc_loc_drc_parameter_interface_struct ia_drc_loc_drc_parameter_interface[] = {
    {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0},
    {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0},
    {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0},
    {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}, {1.0f, 1.0f, 0}};
const ia_drc_loc_mpegh_parameter_interface_struct ia_drc_loc_mpegh_parameter_interface[] = {
    {0, {0, 0, 0}, 0, {0, 0, 0}, {1, 1, 1}, -1},  {2, {1, 2, 0}, 0, {0, 0, 0}, {1, 1, 1}, -1},
    {2, {1, 2, 0}, 0, {0, 0, 0}, {1, 1, 1}, -1},  {2, {1, 2, 0}, 2, {1, 2, 0}, {1, 2, 1}, -1},
    {2, {1, 2, 0}, 2, {1, 2, 0}, {1, 2, 1}, -1},  {0, {0, 0, 0}, 2, {1, 2, 0}, {1, 2, 1}, 1},
    {0, {0, 0, 0}, 0, {0, 0, 0}, {1, 1, 1}, -1},  {0, {0, 0, 0}, 0, {0, 0, 0}, {1, 1, 1}, -1},
    {0, {0, 0, 0}, 0, {0, 0, 0}, {1, 1, 1}, -1},  {0, {0, 0, 0}, 0, {0, 0, 0}, {1, 1, 1}, -1},
    {0, {0, 0, 0}, 1, {2, 8, 0}, {1, 1, 1}, -1},  {0, {0, 0, 0}, 1, {2, 4, 0}, {1, 1, 1}, -1},
    {2, {10, 7, 0}, 2, {2, 3, 0}, {1, 1, 1}, -1}, {2, {10, 2, 0}, 2, {1, 5, 0}, {1, 2, 1}, -1},
    {2, {1, 2, 0}, 2, {1, 2, 0}, {1, 2, 1}, -1},  {2, {1, 3, 0}, 2, {1, 3, 0}, {1, 2, 1}, 1},
    {2, {1, 7, 0}, 2, {1, 3, 0}, {1, 2, 1}, 1},   {2, {1, 5, 0}, 2, {5, 3, 0}, {1, 2, 1}, 1},
    {0, {0, 0, 0}, 0, {0, 0, 0}, {0, 0, 0}, -1},  {0, {0, 0, 0}, 1, {1, 0, 0}, {1, 0, 0}, 1}};

const ia_filter_bank_params_struct ia_drc_normal_cross_freq[FILTER_BANK_PARAMETER_COUNT] = {
    {2.0f / 1024.0f, 0.0000373252f, 0.9913600345f},
    {3.0f / 1024.0f, 0.0000836207f, 0.9870680830f},
    {4.0f / 1024.0f, 0.0001480220f, 0.9827947083f},
    {5.0f / 1024.0f, 0.0002302960f, 0.9785398263f},
    {6.0f / 1024.0f, 0.0003302134f, 0.9743033527f},
    {2.0f / 256.0f, 0.0005820761f, 0.9658852897f},
    {3.0f / 256.0f, 0.0012877837f, 0.9492662926f},
    {2.0f / 128.0f, 0.0022515827f, 0.9329321561f},
    {3.0f / 128.0f, 0.0049030350f, 0.9010958535f},
    {2.0f / 64.0f, 0.0084426929f, 0.8703307793f},
    {3.0f / 64.0f, 0.0178631928f, 0.8118317459f},
    {2.0f / 32.0f, 0.0299545822f, 0.7570763753f},
    {3.0f / 32.0f, 0.0604985076f, 0.6574551915f},
    {2.0f / 16.0f, 0.0976310729f, 0.5690355937f},
    {3.0f / 16.0f, 0.1866943331f, 0.4181633458f},
    {2.0f / 8.0f, 0.2928932188f, 0.2928932188f},
};

const FLOAT32 ia_drc_impd_drc_table[AUDIO_CODEC_SUBBAND_COUNT_STFT256] = {
    INFINITY,          63095776178.907753f, 3981074046.015824f, 790760988.782753f,
    251188772.366017f, 103201616.344517f,   49893641.693518f,   26988223.071905f,
    15848938.912827f,  9910377.195577f,     6511581.300959f,    4453302.889236f,
    3148075.735594f,   2288086.861654f,     1702841.631031f,    1293395.804391f,
    1000000.367439f,   785316.545085f,      625302.481860f,     504065.687683f,
    410852.974409f,    338235.540543f,      280984.395867f,     235356.282462f,
    198630.136038f,    168800.104557f,      144368.510408f,     124203.871400f,
    107442.035463f,    93416.359396f,       81607.752213f,      71608.496720f,
    63095.752995f,     55811.948268f,       49550.120545f,      44142.863627f,
    39453.916446f,     35371.712157f,       31804.392437f,      28675.926325f,
    25923.068265f,     23492.958457f,       21341.218279f,      19430.429851f,
    17728.915523f,     16209.752968f,       14849.976408f,      13629.925686f,
    12532.713396f,     11543.786743f,       10650.565789f,      9842.143568f,
    9109.036526f,      8442.976082f,        7836.733911f,       7283.974999f,
    6779.133640f,      6317.308475f,        5894.173372f,       5505.901533f,
    5149.100684f,      4820.757581f,        4518.190361f,       4239.007532f,
    3981.072583f,      3742.473376f,        3521.495608f,       3316.599761f,
    3126.401018f,      2949.651750f,        2785.226196f,       2632.107045f,
    2489.373652f,      2356.191685f,        2231.803993f,       2115.522554f,
    2006.721352f,      1904.830070f,        1809.328500f,       1719.741566f,
    1635.634911f,      1556.610951f,        1482.305359f,       1412.383925f,
    1346.539742f,      1284.490685f,        1225.977152f,       1170.760029f,
    1118.618864f,      1069.350221f,        1022.766194f,       978.693066f,
    936.970099f,       897.448436f,         859.990108f,        824.467133f,
    790.760698f,       758.760416f,         728.363649f,        699.474895f,
    672.005221f,       645.871759f,         620.997231f,        597.309526f,
    574.741307f,       553.229655f,         532.715738f,        513.144512f,
    494.464445f,       476.627264f,         459.587719f,        443.303368f,
    427.734384f,       412.843370f,         398.595189f,        384.956811f,
    371.897171f,       359.387034f,         347.398876f,        335.906766f,
    324.886266f,       314.314330f,         304.169218f,        294.430407f,
    285.078518f,       276.095243f,         267.463274f,        259.166247f,
    251.188680f,       243.515919f,         236.134089f,        229.030046f,
    222.191335f,       215.606147f,         209.263282f,        203.152114f,
    197.262554f,       191.585026f,         186.110430f,        180.830121f,
    175.735880f,       170.819889f,         166.074715f,        161.493280f,
    157.068847f,       152.795003f,         148.665634f,        144.674918f,
    140.817302f,       137.087492f,         133.480439f,        129.991325f,
    126.615548f,       123.348719f,         120.186643f,        117.125314f,
    114.160902f,       111.289747f,         108.508349f,        105.813361f,
    103.201578f,       100.669938f,         98.215504f,         95.835468f,
    93.527138f,        91.287938f,          89.115395f,         87.007141f,
    84.960908f,        82.974516f,          81.045877f,         79.172987f,
    77.353923f,        75.586838f,          73.869958f,         72.201582f,
    70.580074f,        69.003861f,          67.471433f,         65.981338f,
    64.532179f,        63.122615f,          61.751353f,         60.417151f,
    59.118812f,        57.855186f,          56.625164f,         55.427679f,
    54.261704f,        53.126247f,          52.020355f,         50.943110f,
    49.893623f,        48.871042f,          47.874542f,         46.903329f,
    45.956636f,        45.033724f,          44.133879f,         43.256413f,
    42.400660f,        41.565979f,          40.751750f,         39.957374f,
    39.182274f,        38.425889f,          37.687680f,         36.967126f,
    36.263722f,        35.576980f,          34.906429f,         34.251612f,
    33.612088f,        32.987431f,          32.377227f,         31.781078f,
    31.198595f,        30.629405f,          30.073145f,         29.529464f,
    28.998023f,        28.478491f,          27.970550f,         27.473890f,
    26.988213f,        26.513228f,          26.048654f,         25.594217f,
    25.149654f,        24.714708f,          24.289131f,         23.872681f,
    23.465123f,        23.066232f,          22.675787f,         22.293574f,
    21.919386f,        21.553020f,          21.194283f,         20.842983f,
    20.498936f,        20.161964f,          19.831892f,         19.508552f,
    19.191779f,        18.881414f,          18.577301f,         18.279291f,
    17.987237f,        17.700996f,          17.420431f,         17.145406f,
    16.875790f,        16.611458f,          16.352284f,         16.098148f};
/** @} */ /* End of DRCROM */