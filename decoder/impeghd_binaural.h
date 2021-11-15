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

#ifndef IMPEGHD_BINAURAL_H
#define IMPEGHD_BINAURAL_H

#define MAX_SAMPLING_FREQ (48000)
#define BINAURAL_FFT_MAXLEN (16384)
#define MAX_BINAURAL_REPRESENTATION (16)
#define CICP2GEOMETRY_MAX_LOUDSPEAKERS_BRIR (32)
#define CICP2GEOMETRY_CICP_LOUDSPEAKER_TABLESIZE (43)
#define MAX_NUM_BRIR_PAIRS (21)
#define MAX_BRIR_SIZE (MAX_SAMPLING_FREQ) // 2^18
#define MAX_NUM_DIFFUSE_BLOCKS (2)        // Can be restricted to 1
#define MAX_LENGTH_DIRECT_FILTER (8192)
#define MIN_LENGTH_DIRECT_FILTER (1024)
#define MAX_FACTORS (23)
#define MAX_PERM (209)
#define NFACTOR (11)
#define ENERGY_THRESHOLD (-50)
#define DIRECT_ENERGY_THRESH (-15)
#define DIFFUSE_ENERGY_THRESH (-19)
#define DIFFUSE_FC_THRESH (-20)
#define DIRECT_FC_THRESH (-20)
#define FFT_IFFT_SCRATCH 65536
#define BINURAL_MAX_3D_SPEAKER (32767)

typedef struct
{
  WORD32 cicp_loudspeaker_idx;
  WORD32 az;
  WORD32 el;
  WORD32 lfe;
  WORD32 screen_relative;
  WORD32 loudspeaker_type;
} ia_binural_channel_geometry;

static const WORD32 brir_sampling_frequency_table[32] = {
    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025,
    8000,  7350,  -1,    -1,    57600, 51200, 40000, 38400, 34150, 28800, 25600,
    20000, 19200, 17075, 14400, 12800, 9600,  -1,    -1,    -1,    0};

typedef struct
{
  WORD32 num_channel;
  WORD32 begin_delay;
  WORD32 len_direct;
  WORD32 num_diffuse_block;
  FLOAT32 ptr_direct_fc[2][MAX_NUM_BRIR_PAIRS];
  FLOAT32 ptr_diffuse_fc[2][MAX_NUM_DIFFUSE_BLOCKS];
  FLOAT32 ptr_taps_direct[2][MAX_NUM_BRIR_PAIRS][MAX_LENGTH_DIRECT_FILTER];
  FLOAT32 ptr_inv_diffuse_weight[MAX_NUM_BRIR_PAIRS];
  FLOAT32 ptr_taps_diffuse[MAX_NUM_DIFFUSE_BLOCKS][2][MAX_LENGTH_DIRECT_FILTER];
} ia_td_binaural_ren_param_str;

typedef struct
{
  WORD32 n_taps;
  FLOAT32 taps[2][MAX_NUM_BRIR_PAIRS][MAX_BRIR_SIZE];
  FLOAT32 cut_freq[2][MAX_NUM_BRIR_PAIRS];
  UWORD32 all_cut_freq;
} ia_binaural_fir_data_str;

typedef struct
{
  WORD32 num_ch;
  WORD32 num_lfes;
  UWORD16 spk_layout_type;
  WORD16 cicp_spk_layout_idx;
  UWORD32 num_spks;
  UWORD16 cicp_spk_idx[CICP2GEOMETRY_MAX_LOUDSPEAKERS_BRIR];
  ia_flex_spk_data_str str_flex_spk_data;
  ia_binural_channel_geometry geometry[CICP2GEOMETRY_MAX_LOUDSPEAKERS_BRIR];
} ia_binaural_ren_spk_cfg_3d;

typedef struct
{
  UWORD32 brir_sampling_frequency;
  UWORD16 is_hoa_data;
  WORD16 hoa_order_binaural;
  WORD16 brir_pairs;
  WORD16 binaural_data_format_id;
  ia_interface_speaker_config_3d setup_spk_config_3d;
  ia_binaural_fir_data_str ia_binaural_fir_data_str;
  ia_td_binaural_ren_param_str td_binaural_ren_param;
} ia_binaural_representation_str;

typedef struct
{
  FLOAT32 spectrum_r[MAX_LENGTH_DIRECT_FILTER];
  FLOAT32 spectrum_i[MAX_LENGTH_DIRECT_FILTER];
  FLOAT32 spectrum_mag[MAX_LENGTH_DIRECT_FILTER];
  FLOAT32 spectrum_block[MAX_LENGTH_DIRECT_FILTER];
  FLOAT32 diffusion_filter_0[MAX_NUM_BRIR_PAIRS];
  FLOAT32 diffusion_filter_1[MAX_NUM_BRIR_PAIRS];
  FLOAT32 diffuse_weigh_0[MAX_NUM_BRIR_PAIRS];
  FLOAT32 diffuse_weigh_1[MAX_NUM_BRIR_PAIRS];
  FLOAT32 diffusion_filter_wmc[MAX_NUM_DIFFUSE_BLOCKS][MAX_LENGTH_DIRECT_FILTER];
  FLOAT32 diffuse_filter[MAX_LENGTH_DIRECT_FILTER];
  FLOAT32 fft_ifft_scratch[FFT_IFFT_SCRATCH];
  FLOAT32 xi[BINAURAL_FFT_MAXLEN];
  FLOAT32 complex_scratch[BINAURAL_FFT_MAXLEN];
  FLOAT32 taps_direct[MAX_NUM_DIFFUSE_BLOCKS][MAX_NUM_BRIR_PAIRS][MAX_LENGTH_DIRECT_FILTER];
  FLOAT32 work_buffer[BINAURAL_FFT_MAXLEN];

} ia_binaural_scratch;
typedef struct
{
  FLOAT32 *ptr_scratch;
  UWORD16 num_binaural_representation;
  ia_binaural_representation_str binaural_rep;
  ia_binaural_representation_str *ptr_binaural_rep[MAX_BINAURAL_REPRESENTATION];
  ia_binaural_scratch *pstr_local_scratch;
} ia_binaural_renderer;

IA_ERRORCODE impeghd_read_brir_info(ia_binaural_renderer *pstr_binaural_rendering,
                                    ia_bit_buf_struct *pstr_bit_buf);

VOID impeghd_cplx_fft_16k(FLOAT32 *ptr_in_buf_real, FLOAT32 *ptr_in_buf_imag,
                          FLOAT32 *ptr_scratch_buf, WORD32 fft_len);

VOID impeghd_cplx_ifft_8k(FLOAT32 *ptr_in_buf_real, FLOAT32 *ptr_in_buf_imag,
                          FLOAT32 *ptr_scratch_buf, WORD32 fft_len);

#endif /* IMPEGHD_BINAURAL_H */
