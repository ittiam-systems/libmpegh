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

#ifndef _FORMAT_DMXT_MAT_DATA_H_
#define _FORMAT_DMXT_MAT_DATA_H_

#define IA_DMX_MTX_HISTORY_CNT_MAX (512)
#define IA_DMX_MTX_GAIN_TABLE_SIZE_MAX (274)
#define IA_DMX_MTX_GAIN_ZERO (-256.0f)
#define IA_DMX_MAX_NUM_BANDS (100)
#define IA_MAX_NUM_EQ (12)
#define IA_DMX_NUM_ERB_BANDS (58)

#define DMX_SPK_CENTRE 0
#define DMX_SPK_SYMMETRIC 1
#define DMX_SPK_SINGLE 2
#define DMX_SPK_NONE 3

typedef struct
{
  WORD32 min_gain;
  WORD32 max_gain;
  WORD32 precision_level;
  WORD32 raw_code_non_zeros;
  WORD32 gain_lg_param;
  FLOAT32 history[IA_DMX_MTX_HISTORY_CNT_MAX];
  WORD32 history_cnt;
  FLOAT32 gain_tab[IA_DMX_MTX_GAIN_TABLE_SIZE_MAX];
  WORD32 gain_tab_size;
} ia_dmx_mtx_gain_coder_t;

typedef struct
{
  FLOAT32 freq;
  FLOAT32 q_fac;
  FLOAT32 gain;
} ia_eq_pk_filt;

typedef struct
{
  UWORD32 num_pk_filt;
  FLOAT32 global_gain;
  ia_eq_pk_filt *pk_filt;
} ia_eq_params;

typedef struct
{
  UWORD32 num_eq;
  WORD32 *eq_map;
  ia_eq_params *eq_params;
} ia_eq_config;

typedef struct
{
  WORD32 is_seperable[MAX_NUM_SPEAKERS];
  WORD32 is_symmetric[MAX_NUM_SPEAKERS];
  WORD32 comp_dmx_mtx[MAX_NUM_SPEAKERS][MAX_NUM_SPEAKERS];
  WORD32 flat_comp_mtx[MAX_NUM_SPEAKERS * MAX_NUM_SPEAKERS];
  FLOAT32 eq_gains[IA_DMX_MAX_NUM_BANDS][IA_MAX_NUM_EQ];
  ia_speaker_info_str inp_spk_conf[MAX_NUM_SPEAKERS];
  ia_speaker_info_str out_spk_conf[MAX_NUM_SPEAKERS];
  ia_speaker_info_str *ptr_inp_spk_comp_conf[MAX_NUM_SPEAKERS];
  ia_speaker_info_str *ptr_out_spk_comp_conf[MAX_NUM_SPEAKERS];
  ia_dmx_mtx_gain_coder_t gain_coder;
  ia_eq_config eq_conf;
} ia_dmx_mtx_scr_t;

typedef struct
{
  WORD32 num_in_ch;
  WORD32 num_out_ch;
  WORD32 cicp_out_idx;
  WORD32 cicp_in_idx;
  WORD32 samp_freq;
  ia_speaker_config_3d *spk_layout;
  FLOAT32 dmx_mtx[MAX_NUM_SPEAKERS * MAX_NUM_SPEAKERS];
} ia_dmx_mtx_params;

IA_ERRORCODE impeghd_decode_downmix_matrix(ia_format_conv_param *fc_params,
                                           ia_dmx_mtx_params *params,
                                           ia_bit_buf_struct *ptr_bit_buf, pWORD8 ptr_scratch);

#endif
