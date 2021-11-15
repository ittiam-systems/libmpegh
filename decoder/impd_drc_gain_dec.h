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

#ifndef IMPD_DRC_GAIN_DEC_H
#define IMPD_DRC_GAIN_DEC_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  ia_drc_node_struct str_node;
  ia_drc_node_struct prev_node;
  FLOAT32 lpcm_gains[2 * AUDIO_CODEC_FRAME_SIZE_MAX + MAX_SIGNAL_DELAY];

} ia_drc_interp_buf_struct;

typedef struct
{
  ia_drc_interp_buf_struct *pstr_buf_interp;
  WORD32 buf_interp_cnt;
} ia_drc_gain_buffer_struct;

typedef struct
{
  ia_drc_gain_buffer_struct pstr_gain_buf[SEL_DRC_COUNT];
} ia_drc_gain_buffers_struct;

typedef struct
{
  ia_drc_ducking_modifiers_struct *pstr_ducking_modifiers;
  ia_drc_gain_modifiers_struct *pstr_gain_modifiers;

  WORD32 characteristic_idx;
  WORD32 clipping_flag;
  WORD32 delta_tmin;
  WORD32 ducking_flag;
  WORD32 gain_interp_type;
  WORD32 gain_modification_flag;
  WORD32 limiter_peak_target_present;

  FLOAT32 boost;
  FLOAT32 compress;
  FLOAT32 limiter_peak_target;
  FLOAT32 loud_norm_gain_db;
} ia_drc_interp_params_struct;

typedef struct
{
  WORD32 downmix_instrns_idx;
  WORD32 drc_coeff_idx;
  WORD32 drc_instrns_idx;
} ia_drc_sel_struct;

typedef struct
{
  ia_drc_sel_struct sel_drc_array[SEL_DRC_COUNT];
  WORD32 audio_delay_samples;
  WORD32 channel_offset;
  WORD32 delay_mode;
  WORD32 delta_tmin_def;
  WORD32 drc_frame_size;
  WORD32 drc_set_counter;
  WORD32 gain_delay_samples;
  WORD32 multiband_sel_drc_idx;
  WORD32 num_ch_process;
  WORD32 sample_rate;
  WORD32 sub_band_domain_mode;
} ia_drc_params_struct;

#ifdef __cplusplus
}
#endif
#endif
