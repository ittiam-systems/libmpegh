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

#ifndef IMPD_DRC_TABLES_H
#define IMPD_DRC_TABLES_H
#include "impd_drc_common.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  WORD32 size;
  WORD32 code;
  FLOAT32 value;
} ia_drc_delta_param_table_entry_struct;

typedef struct
{
  ia_drc_delta_param_table_entry_struct
      delta_time_code_table[N_DELTA_TIME_CODE_TABLE_ENTRIES_MAX];
} ia_drc_tables_struct;

VOID impd_drc_init_tbls(ia_drc_tables_struct *str_tables, const WORD32 num_gain_max_values);

void impd_drc_create_delta_time_code_tbl(
    ia_drc_delta_param_table_entry_struct *delta_time_code_tbl_item,
    const WORD32 num_gain_max_values);

void impd_drc_create_delta_gain_code_tbl(
    ia_drc_delta_param_table_entry_struct const **delta_time_code_tbl, WORD32 *num_entries,
    const WORD32 gain_coding_profile);

WORD32
impd_drc_get_delta_tmin(const WORD32 sampling_rate);

#ifdef __cplusplus
}
#endif
#endif
