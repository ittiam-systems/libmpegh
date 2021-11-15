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
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_rom.h"

/**
 * @defgroup DRCProcessing DRCProcessing
 * @ingroup  DRCProcessing
 * @brief DRC Decoder processing
 *
 * @{
 */

/**
 *  impd_drc_init_tbls
 *
 *  \brief Initialise drc tables
 *
 *  \param [out] str_tables  Pointer to drc tables struct
 *  \param [in]  num_gain_max_values   number of gain max values
 *
 *  \return VOID
 *
 */
VOID impd_drc_init_tbls(ia_drc_tables_struct *str_tables, const WORD32 num_gain_max_values)
{
  impd_drc_create_delta_time_code_tbl(str_tables->delta_time_code_table, num_gain_max_values);
  return;
}

/**
 *  impd_drc_create_delta_gain_code_tbl
 *
 *  \brief Create delta gain code table
 *
 *  \param [out] ppstr_delta_gain_code_tbl  Pointer to gain code table struct
 *  \param [in]  num_gain_max_values   number of gain max values
 *
 *  \return VOID
 *
 */
VOID impd_drc_create_delta_gain_code_tbl(
    ia_drc_delta_param_table_entry_struct const **ppstr_delta_gain_code_tbl, WORD32 *num_entries,
    const WORD32 gain_coding_profile)
{
  if (gain_coding_profile != IA_DRC_GAIN_CODING_PROFILE_CLIPPING)
  {
    *ppstr_delta_gain_code_tbl = ia_drc_gain_tbls_prof_0_1;
    *num_entries = NUM_GAIN_TBL_PROF_0_1_ENTRIES;
  }
  else
  {
    *ppstr_delta_gain_code_tbl = ia_drc_gain_tbls_prof_2;
    *num_entries = NUM_GAIN_TBL_PROF_2_ENTRIES;
  }
}

/**
 *  impd_drc_create_delta_time_code_tbl
 *
 *  \brief Create delta time code table
 *
 *  \param [out] pstr_delta_time_code_tbl_item  Pointer to time code table struct
 *  \param [in]  num_gain_max_values   number of gain max values
 *
 *  \return VOID
 *
 */
VOID impd_drc_create_delta_time_code_tbl(
    ia_drc_delta_param_table_entry_struct *pstr_delta_time_code_tbl_item,
    const WORD32 num_gain_max_values)
{
  WORD32 n, k;
  WORD8 val = 1;

  k = 2 * num_gain_max_values - 14;
  while ((1 << val) < 2 * num_gain_max_values)
  {
    val++;
  }

  for (n = k; n >= 0; n--)
  {
    pstr_delta_time_code_tbl_item[n + 14].size = 2 + val;
    pstr_delta_time_code_tbl_item[n + 14].code = (0x3 << val) + n;
    pstr_delta_time_code_tbl_item[n + 14].value = (FLOAT32)(n + 14);
  }

  for (n = 7; n >= 0; n--)
  {
    pstr_delta_time_code_tbl_item[n + 6].size = 5;
    pstr_delta_time_code_tbl_item[n + 6].code = 0x10 + n;
    pstr_delta_time_code_tbl_item[n + 6].value = (FLOAT32)(n + 6);
  }

  for (n = 3; n >= 0; n--)
  {
    pstr_delta_time_code_tbl_item[n + 2].size = 4;
    pstr_delta_time_code_tbl_item[n + 2].code = 0x4 + n;
    pstr_delta_time_code_tbl_item[n + 2].value = (FLOAT32)(n + 2);
  }

  pstr_delta_time_code_tbl_item[1].size = 2;
  pstr_delta_time_code_tbl_item[1].code = 0x0;
  pstr_delta_time_code_tbl_item[1].value = (FLOAT32)1;

  pstr_delta_time_code_tbl_item[0].size = -1;
  pstr_delta_time_code_tbl_item[0].code = -1;
  pstr_delta_time_code_tbl_item[0].value = (FLOAT32)-1;
}

/**
 *  impd_drc_get_delta_tmin
 *
 *  \brief Update delta tmin
 *
 *  \param [in]  sampling_rate   sampling rate
 *
 *  \return WORD32 delta tmin
 *
 */
WORD32
impd_drc_get_delta_tmin(const WORD32 sampling_rate)
{
  WORD32 lower_bound;
  WORD32 delta_tmin = 1;
  lower_bound = (WORD32)(0.5f + 0.0005f * sampling_rate);

  while (delta_tmin <= lower_bound)
  {
    delta_tmin = delta_tmin << 1;
  }
  return delta_tmin;
}
/** @} */ /* End of DRCProcessing */