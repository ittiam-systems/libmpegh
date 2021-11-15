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

#ifndef IMPD_DRC_MULTI_BAND_H
#define IMPD_DRC_MULTI_BAND_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{

  FLOAT32 overlap_weight[AUDIO_CODEC_SUBBAND_COUNT_MAX];

} ia_drc_band_overlap_params_struct;

typedef struct
{
  ia_drc_band_overlap_params_struct str_band_overlap_params[BAND_COUNT_MAX];
} ia_drc_group_overlap_params_struct;

typedef struct
{
  ia_drc_group_overlap_params_struct str_grp_overlap_params[CHANNEL_GROUP_COUNT_MAX];
} ia_drc_overlap_params_struct;

IA_ERRORCODE impd_drc_generate_overlap_weights(
    ia_drc_group_overlap_params_struct *pstr_group_overlap_params,
    ia_drc_uni_drc_coeffs_struct *str_p_loc_drc_coefficients_uni_drc,
    ia_drc_instructions_struct *str_drc_instruction_str, WORD32 sub_band_domain_mode,
    WORD32 index);

IA_ERRORCODE
impd_drc_init_overlap_weight(ia_drc_overlap_params_struct *pstr_overlap_params,
                             ia_drc_uni_drc_coeffs_struct *str_p_loc_drc_coefficients_uni_drc,
                             ia_drc_instructions_struct *str_drc_instruction_str,
                             WORD32 sub_band_domain_mode);

#ifdef __cplusplus
}
#endif
#endif
