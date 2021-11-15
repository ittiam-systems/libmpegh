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

#ifndef _IA_CORE_CODER_STEREO_LPD_H_
#define _IA_CORE_CODER_STEREO_LPD_H_

#define STEREO_LPD_DFT_SIZE (512 + 160)
#define STEREO_LPD_DEC_DFT_NB NUM_FRAMES
#define STEREO_LPD_BAND_MAX 20
#define STEREO_LPD_FLT_MIN 1e-10f
#define STEREO_LPD_MAX_ILD_IDX 30

#define STERO_LPD_DMX_LIMIT 15.8489f

typedef struct
{
  WORD32 fs;
  WORD32 ccfl;
  WORD32 dft_size;
  WORD32 frame_size;
  WORD32 overlap_size;
} ia_usac_slpd_config_struct, *ia_usac_slpd_config_handle;

typedef struct
{
  WORD32 res_mode;
  WORD32 q_mode;
  WORD32 ipd_mode;
  WORD32 pred_mode;
  WORD32 cod_mode;
  WORD32 ild_idx[STEREO_LPD_DEC_DFT_NB][STEREO_LPD_BAND_MAX];
  WORD32 ipd_idx[STEREO_LPD_DEC_DFT_NB][STEREO_LPD_BAND_MAX];
  WORD32 pred_gain_idx[STEREO_LPD_DEC_DFT_NB][STEREO_LPD_BAND_MAX];
  WORD32 cod_gain_idx[STEREO_LPD_DEC_DFT_NB];
} ia_usac_slpd_bitstream_struct, *ia_usac_slpd_bitstream_handle;

typedef struct
{

  FLOAT32 ild[STEREO_LPD_DEC_DFT_NB][STEREO_LPD_BAND_MAX];
  FLOAT32 ipd[STEREO_LPD_DEC_DFT_NB][STEREO_LPD_BAND_MAX];
  FLOAT32 pred_gain[STEREO_LPD_DEC_DFT_NB][STEREO_LPD_BAND_MAX];
  FLOAT32 cod_gain[STEREO_LPD_DEC_DFT_NB];
  FLOAT32 res_sig[STEREO_LPD_DEC_DFT_NB][STEREO_LPD_DFT_SIZE];
} ia_usac_slpd_param_struct, *ia_usac_slpd_param_handle;

typedef struct
{
  WORD32 band_limits[STEREO_LPD_BAND_MAX + 1];
  WORD32 num_bands;
  WORD32 ipd_band_max;
  WORD32 cod_band_max;
  WORD32 num_dft_lines;
} ia_usac_slpd_band_info_struct, *ia_usac_slpd_band_info_handle;

typedef struct
{
  WORD16 fac_fb;
  WORD32 bpf_pitch[NUM_SUBFR_SUPERFRAME + 1];
  FLOAT32 dft_prev_dmx[STEREO_LPD_DFT_SIZE];
  FLOAT32 bpf_gain[NUM_SUBFR_SUPERFRAME + 1];
  FLOAT32 time_buff_dmx[LEN_SUPERFRAME / 2 + LEN_SUPERFRAME];

  const FLOAT32 *p_slpd_sin_table;
  const FLOAT32 *win;
  FLOAT32 prev_left[2 * MAX_PITCH + LEN_SUPERFRAME / 2 + FAC_LENGTH];
  FLOAT32 prev_right[2 * MAX_PITCH + LEN_SUPERFRAME / 2 + FAC_LENGTH];
  FLOAT32 old_noise_pf_left[(2 * FILTER_DELAY + 1) * 2 + LTPF_MAX_DELAY];
  FLOAT32 old_noise_pf_right[(2 * FILTER_DELAY + 1) * 2 + LTPF_MAX_DELAY];

  ia_usac_slpd_config_struct lpd_stereo_config;
  ia_usac_slpd_bitstream_struct lpd_stereo_bitstream;
  ia_usac_slpd_param_struct lpd_stereo_parameter;
  ia_usac_slpd_band_info_struct lpd_stereo_bandinfo;
} ia_usac_slpd_dec_data_struct, *ia_usac_slpd_dec_data_handle;

typedef struct
{
  FLOAT32 time_buff_left[(2 * MAX_PITCH + LEN_SUPERFRAME / 2 + LEN_SUPERFRAME + FAC_LENGTH)];
  FLOAT32 time_buff_right[(2 * MAX_PITCH + LEN_SUPERFRAME / 2 + LEN_SUPERFRAME + FAC_LENGTH)];
  FLOAT32 out_buff[LEN_SUPERFRAME + LTPF_MAX_DELAY];
  FLOAT32 dft_left[STEREO_LPD_DFT_SIZE];
  FLOAT32 dft_right[STEREO_LPD_DFT_SIZE];
  FLOAT32 dft_dmx[STEREO_LPD_DFT_SIZE];
  FLOAT32 time_buff[STEREO_LPD_DFT_SIZE];
} ia_core_coder_slpd_apply_scr_t;

VOID ia_core_coder_slpd_init(ia_usac_slpd_dec_data_handle slpd_dec_data, WORD32 fs,
                             WORD32 full_band_lpd, WORD32 ccfl);

VOID ia_core_coder_slpd_set_past(ia_usac_slpd_dec_data_handle slpd_dec_data, FLOAT32 *synth_left,
                                 FLOAT32 *synth_right, WORD32 length);

IA_ERRORCODE ia_core_coder_slpd_data(ia_usac_slpd_dec_data_handle slpd_dec_data,
                                     ia_bit_buf_struct *it_bit_buff, WORD32 td_start_flag);
VOID ia_core_coder_slpd_apply(ia_usac_slpd_dec_data_handle slpd_dec_data, FLOAT32 *synth_left,
                              FLOAT32 *synth_right, WORD32 td_start_flag, FLOAT32 *ptr_scratch);

#endif /* _IA_CORE_CODER_STEREO_LPD_H_ */
