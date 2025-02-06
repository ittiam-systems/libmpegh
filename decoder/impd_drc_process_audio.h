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

#ifndef IMPD_DRC_PROCESS_AUDIO_H
#define IMPD_DRC_PROCESS_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  WORD32 multiband_audio_sig_count;
  WORD32 frame_size;
  FLOAT32 **non_interleaved_audio;

} ia_drc_audio_band_buffer_struct;

typedef struct ia_drc_gain_dec_struct
{
  ia_drc_audio_band_buffer_struct audio_band_buffer;
  ia_drc_gain_buffers_struct drc_gain_buffers;
  ia_drc_params_struct ia_drc_params_struct;
  ia_filter_banks_struct ia_filter_banks_struct;
  ia_drc_overlap_params_struct str_overlap_params;
  WORD32 audio_num_chan;
} ia_drc_gain_dec_struct;

WORD32
impd_drc_get_gain(ia_drc_gain_buffers_struct *drc_gain_buffers,
                  ia_drc_gain_dec_struct *p_drc_gain_dec_structs, ia_drc_config *pstr_drc_config,
                  ia_drc_gain_struct *pstr_drc_gain, FLOAT32 compress, FLOAT32 boost,
                  WORD32 characteristic_idx, FLOAT32 loud_norm_gain_db, WORD32 sel_drc_index);
WORD32 impd_drc_td_process(FLOAT32 *audio_in_out_buf[],
                           ia_drc_gain_dec_struct *p_drc_gain_dec_structs,
                           ia_drc_config *pstr_drc_config, ia_drc_gain_struct *pstr_drc_gain,
                           FLOAT32 loud_norm_gain_db, FLOAT32 boost, FLOAT32 compress,
                           WORD32 drc_characteristic,
                           UWORD32 *ptr_inactive_sig);

WORD32 impd_drc_fd_process(FLOAT32 *audio_real_buff[], FLOAT32 *audio_imag_buff[],
                           ia_drc_gain_dec_struct *p_drc_gain_dec_structs,
                           ia_drc_config *pstr_drc_config, ia_drc_gain_struct *pstr_drc_gain,
                           FLOAT32 loud_norm_gain_db, FLOAT32 boost, FLOAT32 compress,
                           WORD32 drc_characteristic,
                           UWORD32 *ptr_inactive_sig);

WORD32
impd_drc_apply_gains_subband(FLOAT32 *deinterleaved_audio_re[], FLOAT32 *deinterleaved_audio_im[],
                             ia_drc_instructions_struct *pstr_drc_instruction_arr,
                             ia_drc_gain_dec_struct *pstr_drc_gain_dec, WORD32 sel_drc_idx,
                             UWORD32 *ptr_inactive_sig);

WORD32 impd_drc_filter_banks_process(FLOAT32 *audio_io_buf[],
                                     ia_drc_instructions_struct *pstr_drc_instruction_arr,
                                     ia_drc_gain_dec_struct *pstr_drc_gain_dec,
                                     WORD32 sel_drc_idx);

#ifdef __cplusplus
}
#endif
#endif
