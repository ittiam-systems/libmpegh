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

#ifndef IA_CORE_CODER_CREATE_H
#define IA_CORE_CODER_CREATE_H

typedef struct
{
  struct ia_bit_buf_struct dec_bit_buf;
  ia_frame_data_struct str_frame_data;
  ia_usac_data_struct str_usac_data;
  impeghd_hoa_dec_struct str_hoa_dec_handle;
  ia_hoa_frame_struct str_hoa_frame_data;
  ia_obj_ren_dec_state_struct str_obj_ren_dec_state;
  ia_drc_payload_struct str_drc_payload;
  UWORD8 drc_persistent_buf[MAX_UNIDRC_PERSISTENT_SIZE];
  ia_peak_limiter_struct str_peak_limiter;
  ia_ele_intrctn str_element_interaction;
  ia_local_setup_struct str_local_setup_interaction;
  ia_scene_disp_data str_scene_displacement;
  ia_interaction_data_struct str_interaction_config;
  ia_enh_obj_md_frame_str str_enh_obj_md_frame;
  ia_format_conv_struct str_format_converter;
  ia_domain_switcher_struct str_domain_switcher;
  ia_binaural_render_struct binaural_handle;
  ia_binaural_renderer str_binaural_rendering;
  FLOAT32 *ptr_binaural_output[MAX_TIME_CHANNELS];
  ia_resampler_struct str_resampler[MAX_TIME_CHANNELS];
  ia_format_conv_struct str_earcon_format_converter;
  ia_resampler_struct str_earcon_resampler[MAX_TIME_CHANNELS];
} ia_dec_data_struct;

WORD32 ia_core_coder_frm_data_init(ia_audio_specific_config_struct *pstr_audio_conf,
                                   ia_dec_data_struct *pstr_dec_data);

IA_ERRORCODE ia_core_coder_decode_create(ia_mpegh_dec_api_struct *dec_handle,
                                         ia_dec_data_struct *pstr_dec_data,
                                         WORD32 tracks_for_decoder);

#endif
