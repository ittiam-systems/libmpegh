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

#ifndef IMPEGHD_BINAURAL_RENDERER_H
#define IMPEGHD_BINAURAL_RENDERER_H

#define MAX_NUM_BRIR_CHANNELS (32)
#define BINAURAL_TEST_LGR_SIGNAL (1024)
#define BINAURAL_NB_OUTPUT (2)

#define BINAURAL_FFT_MAXLEN (16384)
#define BINAURAL_DIR_MAXLEN (8192)
#define BINAURAL_DIF_MAXLEN (BINAURAL_DIR_MAXLEN)
#define BINAURAL_DIF_MAXBLOCKS (2) // can be restricted to 1

#define BINAURAL_RENDERER_PERSISTEN_BUF_SIZE                                                     \
    ((BINAURAL_FFT_MAXLEN * BINAURAL_NB_OUTPUT * sizeof(WORD32)) + \
     (BINAURAL_FFT_MAXLEN * MAX_NUM_BRIR_CHANNELS * BINAURAL_NB_OUTPUT * sizeof(WORD32)) + \
     ()

typedef struct
{
  WORD32 fft_len;
  WORD32 ifft_len;
  WORD32 fft_size;
  WORD32 direct_size;
  WORD32 diffuse_size;
  WORD32 direct_process_size[BINAURAL_NB_OUTPUT][MAX_NUM_BRIR_CHANNELS];
  WORD32 diffuse_process_size[MAX_NUM_DIFFUSE_BLOCKS][BINAURAL_NB_OUTPUT];
  WORD32 spec_mul_size;
  WORD32 filter_size;
  WORD32 num_input;
  WORD32 num_output;
  WORD32 output_samples_size;
  WORD32 memory_index;
  WORD32 processed_samples;
  WORD32 diffuse_blocks;
  WORD32 write_idx;
  FLOAT32 taps_direct[BINAURAL_NB_OUTPUT][MAX_NUM_BRIR_CHANNELS][BINAURAL_FFT_MAXLEN];
  FLOAT32 input_mem_buf[MAX_NUM_BRIR_CHANNELS][BINAURAL_FFT_MAXLEN];
  FLOAT32 output_mem_buf[BINAURAL_NB_OUTPUT][BINAURAL_FFT_MAXLEN];
  FLOAT32 taps_diffuse[MAX_NUM_DIFFUSE_BLOCKS][BINAURAL_NB_OUTPUT][BINAURAL_FFT_MAXLEN];
  FLOAT32 prev_in_buf[MAX_NUM_DIFFUSE_BLOCKS][BINAURAL_FFT_MAXLEN];
  FLOAT32 diffuse_weight[MAX_NUM_BRIR_CHANNELS];
} ia_binaural_ren_pers_mem_str;

typedef struct
{
  ia_binaural_ren_pers_mem_str str_binaural;
  WORD32 channel_map[MAX_NUM_BRIR_PAIRS];
  FLOAT32 binaural_signal_in_renderer[MAX_NUM_BRIR_CHANNELS][BINAURAL_TEST_LGR_SIGNAL];
  FLOAT32 binaural_signal_out_renderer[BINAURAL_NB_OUTPUT][BINAURAL_DIR_MAXLEN];
  FLOAT32 *ptr_binaural_signal_in_renderer[MAX_NUM_BRIR_CHANNELS];
  FLOAT32 *ptr_binaural_signal_out_renderer[BINAURAL_NB_OUTPUT];
  FLOAT32 *ptr_work;
} ia_binaural_render_struct;

typedef struct
{
  WORD32 fs_input;
  WORD32 num_speaker_expected;
  WORD32 cicp_spk_idx;
  WORD32 num_channel_is_input;
} ia_binaural_in_stream_cfg_str;

IA_ERRORCODE impeghd_binaural_renderer_init(ia_binaural_renderer *binaural_info_handle,
                                            ia_binaural_in_stream_cfg_str *wav_param,
                                            ia_binaural_render_struct *binaural_handle);

WORD32 impeghd_binaural_struct_process_ld_ola(FLOAT32 **ptr_src, FLOAT32 **ptr_dst,
                                              WORD32 samples_len, FLOAT32 *ptr_work,
                                              ia_binaural_ren_pers_mem_str *ptr_binaural,
                                              FLOAT32 *ptr_scratch_buf);

#endif /* IMPEGHD_BINAURAL_RENDERER_H */
