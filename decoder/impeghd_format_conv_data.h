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

#ifndef _FORMAT_CONV_DATA_H_
#define _FORMAT_CONV_DATA_H_
typedef struct
{
  WORD32 num_in_ch;
  WORD32 num_out_ch;
  FLOAT32 target_energy_prev[FC_MAX_CHANNELS][FC_ERB_BANDS];
  FLOAT32 target_energy[FC_MAX_CHANNELS][FC_ERB_BANDS];
  FLOAT32 realized_energy_prev[FC_MAX_CHANNELS][FC_ERB_BANDS];
  FLOAT32 realized_energy[FC_MAX_CHANNELS][FC_ERB_BANDS];
  FLOAT32 ***downmix_mat;
} ia_format_conv_dmx_state;

typedef struct converter_pr
{
  WORD32 in_out_src[FC_MAX_IN_OUT];
  WORD32 in_out_dst[FC_MAX_IN_OUT];
  FLOAT32 in_out_gain[FC_MAX_IN_OUT];
  FLOAT32 eq[N_EQ][FC_MAXBANDS];
  WORD32 in_out_proc[FC_MAX_IN_OUT];
} ia_format_conv_data_state;

typedef struct
{
  WORD32 samp_rate;
  ia_format_conv_data_state *data_state;
  ia_format_conv_dmx_state *active_dmx_stft;
  WORD32 num_in_ch;
  WORD32 num_out_ch;
  FLOAT32 **downmix_mat;
  FLOAT32 downmix_mat_freq[FC_BANDS][FC_MAX_CHANNELS][FC_MAX_CHANNELS];
  // FLOAT32 ***downmix_mat_freq;
  WORD32 *in_ch;
  const WORD32 *out_ch;
  ia_format_conv_data_state init_scratch;
} ia_format_conv_param;

typedef struct
{
  WORD32 num_in_ch;
  WORD32 num_out_ch;
  WORD32 *in_ch;
  WORD32 *out_ch;
} ia_ds_param;

typedef struct
{
  ia_format_conv_param fc_params;
  FLOAT32 *downmix_mat_ptr[FC_MAX_CHANNELS];
  FLOAT32 **downmix_mat_freq_ptr[FC_BANDS];
  FLOAT32 downmix_mat_freq_ch_ptr[FC_MAX_CHANNELS * FC_BANDS * FC_MAX_CHANNELS];
  FLOAT32 downmix_mat[FC_MAX_CHANNELS * FC_MAX_CHANNELS];
  FLOAT32 **active_downmix_mat_ptr[FC_MAX_CHANNELS];
  FLOAT32 *active_downmix_mat_ch_ptr[FC_MAX_CHANNELS * FC_MAX_CHANNELS];
  FLOAT32 active_downmix_mat_freq_ptr[FC_MAX_CHANNELS * FC_BANDS * FC_MAX_CHANNELS];
  FLOAT32 *stft_input_buf_ptr[FC_MAX_CHANNELS];
  FLOAT32 stft_input_buf[FC_MAX_CHANNELS * FC_STFT_FRAME];
  FLOAT32 *stft_output_buf_ptr[FC_MAX_CHANNELS];
  FLOAT32 stft_output_buf[FC_MAX_CHANNELS * FC_STFT_FRAME];
  FLOAT32 *conv_mix_mat_ptr[FC_MAX_CHANNELS]; // Not Used
  WORD32 *mix_mat_ptr[FC_MAX_CHANNELS];
  ia_format_conv_data_state format_conv_params_int;
  ia_format_conv_dmx_state active_dmx_stft;
} ia_format_conv_struct;

typedef struct
{
  ia_ds_param ds_params;
  FLOAT32 *stft_input_buf_ptr[FC_MAX_CHANNELS];
  FLOAT32 stft_input_buf[FC_MAX_CHANNELS * FC_STFT_FRAME];
  FLOAT32 *stft_output_buf_ptr[FC_MAX_CHANNELS];
  FLOAT32 stft_output_buf[FC_MAX_CHANNELS * FC_STFT_FRAME];
} ia_domain_switcher_struct;

typedef struct
{
  FLOAT32 scratch_ds[MAX_NUM_CHANNELS * FC_STFT_FRAMEx2 * 4];
  FLOAT32 scratch_fc[MAX_NUM_CHANNELS * FC_STFT_FRAMEx2 * 4];
  FLOAT32 scratch_fc_ercon[MAX_NUM_CHANNELS * FC_STFT_FRAMEx2 * 4];
  FLOAT32 earcon_fc[FC_MAX_CHANNELS * FC_BANDS * FC_MAX_CHANNELS];
  FLOAT32 earcon_fc_data[FC_OUT_MAX_CH];
  FLOAT32 fc_array[FC_MAX_CHANNELS * FC_BANDS * FC_MAX_CHANNELS];
  FLOAT32 fc_data[FC_OUT_MAX_CH];
  ia_cicp_ls_geo_str str_azi_elev[CICP_MAX_CH];
  ia_cicp_ls_geo_str str_azi_elev_ec[CICP_MAX_CH];

} ia_format_converter_scratch;
IA_ERRORCODE impeghd_format_conv_dmx_init(ia_format_conv_dmx_state *active_dmx,
                                          ia_format_conv_param *format_conv_params);
IA_ERRORCODE impeghd_format_conv_init_data(FLOAT32 *ptr_scratch,
                                           ia_cicp_ls_geo_str *azi_elev_array,
                                           ia_format_conv_param *params,
                                           ia_speaker_config_3d *ref_spk_layout);

IA_ERRORCODE impeghd_format_conv_post_proc_dmx_mtx(ia_format_conv_param *params);

IA_ERRORCODE impeghd_format_conv_map_lfes(ia_format_conv_param *params,
                                          const ia_cicp_ls_geo_str **pp_cicp_in_geometry,
                                          const ia_cicp_ls_geo_str **pp_cicp_out_geometry);

#endif
