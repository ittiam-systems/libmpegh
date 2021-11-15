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

#ifndef IMPD_DRC_FILTER_BANK_H
#define IMPD_DRC_FILTER_BANK_H

#define FILTER_BANK_PARAMETER_COUNT 16
#define CASCADE_ALLPASS_COUNT_MAX 9

typedef struct
{
  FLOAT32 freq_cross_norm;
  FLOAT32 gamma;
  FLOAT32 delta;

} ia_filter_bank_params_struct;

typedef struct
{
  FLOAT32 x_p[MAX_CHANNEL_COUNT * 2];
  FLOAT32 y_p[MAX_CHANNEL_COUNT * 2];
  FLOAT32 a0;
  FLOAT32 a1;
  FLOAT32 a2;
  FLOAT32 b0;
  FLOAT32 b1;
  FLOAT32 b2;
} ia_iir_filter_struct;
typedef struct
{
  ia_iir_filter_struct str_lp;
  ia_iir_filter_struct str_hp;
} ia_two_band_filt_struct;

typedef struct
{
  ia_iir_filter_struct str_lp_stage_1;
  ia_iir_filter_struct str_lp_stage_2;
  ia_iir_filter_struct str_hp_stage_1;
  ia_iir_filter_struct str_hp_stage_2;
  ia_iir_filter_struct str_ap_stage_2;
} ia_three_band_filt_struct;

typedef struct
{
  ia_iir_filter_struct str_lp_stage_1;
  ia_iir_filter_struct str_lp_stage_3_low;
  ia_iir_filter_struct str_lp_stage_3_high;
  ia_iir_filter_struct str_hp_stage_1;
  ia_iir_filter_struct str_hp_stage_3_low;
  ia_iir_filter_struct str_hp_stage_3_high;
  ia_iir_filter_struct str_ap_stage_2_low;
  ia_iir_filter_struct str_ap_stage_2_high;
} ia_four_band_filt_struct;

typedef struct
{
  ia_iir_filter_struct str_ap_stage;
} ia_all_pass_filter_sturct;

typedef struct
{
  ia_all_pass_filter_sturct str_ap_cascade_filt[CASCADE_ALLPASS_COUNT_MAX];
  WORD32 num_filter;
} ia_all_pass_cascade_struct;

typedef struct
{
  ia_two_band_filt_struct str_two_band_filt_bank;
  ia_three_band_filt_struct str_three_band_filt_bank;
  ia_four_band_filt_struct str_four_band_filt_bank;
  ia_all_pass_cascade_struct str_all_pass_cascade;
  WORD32 complexity;
  WORD32 num_bands;
} ia_drc_filter_bank_struct;

typedef struct
{
  ia_drc_filter_bank_struct str_drc_filter_bank[8];
  WORD32 complexity;
  WORD32 num_filt_banks;
  WORD32 num_ph_align_ch_grps;
} ia_filter_banks_struct;

WORD32
impd_drc_init_all_filter_banks(ia_filter_banks_struct *ia_filter_banks_struct,
                               ia_drc_uni_drc_coeffs_struct *str_p_loc_drc_coefficients_uni_drc,
                               ia_drc_instructions_struct *str_drc_instruction_str);
VOID impd_drc_two_band_filter_process(FLOAT32 *audio_out[],
                                      ia_two_band_filt_struct *str_two_band_filt_bank, WORD32 c,
                                      WORD32 size, FLOAT32 *audio_in);
VOID impd_drc_three_band_filter_process(FLOAT32 *audio_out[],
                                        ia_three_band_filt_struct *str_three_band_filt_bank,
                                        WORD32 c, WORD32 size, FLOAT32 *audio_in);
VOID impd_drc_four_band_filter_process(FLOAT32 *audio_out[],
                                       ia_four_band_filt_struct *str_four_band_filt_bank,
                                       WORD32 c, WORD32 size, FLOAT32 *audio_in);
VOID impd_drc_all_pass_cascade_process(ia_all_pass_cascade_struct *str_all_pass_cascade, WORD32 c,
                                       WORD32 size, FLOAT32 *audio_in);

#endif
