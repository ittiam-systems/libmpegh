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

#ifndef IMPEGHD_UNI_DRC_STRUCT_H
#define IMPEGHD_UNI_DRC_STRUCT_H
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_common.h"
#include "impd_drc_struct.h"
#include "impd_drc_interface.h"
#include "impd_drc_gain_dec.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_multi_band.h"
#include "impd_drc_process_audio.h"
#include "impd_drc_peak_limiter.h"
#include "impd_drc_selection_process.h"
typedef struct
{
  ia_drc_bits_dec_struct str_bitstream_dec;
  ia_drc_config *pstr_drc_config;
  ia_drc_gain_dec_struct str_gain_dec[2];
  ia_drc_gain_struct str_drc_gain;
  ia_drc_interface_struct str_drc_interface;
  ia_drc_loudness_info_set_struct str_loud_info;
  ia_drc_sel_pro_struct str_select_proc;
  ia_drc_sel_proc_params_struct str_uni_drc_sel_proc_params;
  WORD32 multiband_drc_present[3];
  WORD32 num_sets[3];
} ia_drc_payload_struct;

IA_ERRORCODE impd_drc_read_uni_drc_gain(ia_drc_gain_struct *pstr_uni_drc_gain,
                                        ia_drc_config *drc_config, ia_bit_buf_struct *it_bit_buff,
                                        ia_drc_bits_dec_struct *pstr_drc_uni_bs_dec);

IA_ERRORCODE impd_drc_parse_config(ia_drc_config *, ia_drc_loudness_info_set_struct *,
                                   ia_bit_buf_struct *, ia_drc_params_bs_dec_struct *);

IA_ERRORCODE impd_drc_mpegh3da_parse_loudness_info_set(ia_drc_loudness_info_set_struct *,
                                                       ia_bit_buf_struct *);

#endif /* IMPEGHD_UNI_DRC_STRUCT_H */
