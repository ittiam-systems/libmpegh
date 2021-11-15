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

#ifndef IA_CORE_CODER_BIT_EXTRACT_H
#define IA_CORE_CODER_BIT_EXTRACT_H

typedef struct
{
  WORD32 common_tw;
  WORD32 common_window;
  WORD32 core_mode[2];
  WORD32 tns_data_present[2];
  WORD32 tns_active;
  WORD32 common_tns;
  WORD32 tns_on_lr;
  WORD32 tns_present_both;
  WORD32 common_max_sfb;
  UWORD8 max_sfb[2];
  WORD32 max_sfb_ste;
  WORD32 max_sfb_ste_clear;
  WORD32 pred_dir;
  WORD32 complex_coef;
  WORD32 use_prev_frame;
  UWORD8 ms_mask_present[2];
  WORD32 common_ltpf;
} ia_usac_tmp_core_coder_struct;

VOID ia_core_coder_cplx_prev_mdct_dmx(ia_sfb_info_struct *pstr_sfb_info, FLOAT32 *l_spec,
                                      FLOAT32 *r_spec, FLOAT32 *dmx_re_prev, WORD32 pred_dir,
                                      WORD32 save_zeros);

IA_ERRORCODE ia_core_coder_ics_info(ia_usac_data_struct *usac_data, WORD32 widx, UWORD8 *max_sfb,
                                    ia_bit_buf_struct *it_bit_buff, WORD32 window_sequence_last);

IA_ERRORCODE ia_core_coder_read_tns_tcx(WORD32 sfb_per_sbk,
                                        ia_tns_frame_info_struct *pstr_tns_frame_info,
                                        ia_bit_buf_struct *it_bit_buff, WORD32 k);

IA_ERRORCODE ia_core_coder_read_tns_u(ia_sfb_info_struct *pstr_sfb_info,
                                      ia_tns_frame_info_struct *pstr_tns_frame_info,
                                      ia_bit_buf_struct *it_bit_buff);

IA_ERRORCODE
ia_core_coder_data(WORD32 id, ia_usac_data_struct *usac_data, WORD32 elem_idx, WORD32 chan_offset,
                   ia_bit_buf_struct *it_bit_buff, WORD32 nr_core_coder_channels,
                   ia_usac_decoder_config_struct *pstr_usac_dec_config,
                   ia_usac_tmp_core_coder_struct *pstr_core_coder);

IA_ERRORCODE ia_core_coder_data_process(ia_usac_data_struct *usac_data, WORD32 elem_idx,
                                        WORD32 chan_offset, WORD32 nr_core_coder_channels,
                                        ia_usac_tmp_core_coder_struct *pstr_core_coder);

IA_ERRORCODE ia_core_coder_lpd_channel_stream(ia_usac_data_struct *usac_data,
                                              ia_td_frame_data_struct *pstr_td_frame_data,
                                              ia_bit_buf_struct *it_bit_buff, FLOAT32 *synth,
                                              WORD32 elem_idx);
VOID ia_core_coder_acelp_decoding(WORD32 k, ia_usac_data_struct *usac_data,
                                  ia_td_frame_data_struct *pstr_td_frame_data,
                                  ia_bit_buf_struct *it_bit_buff, WORD32 chan,
                                  WORD32 full_band_lpd);

WORD32 ia_core_coder_win_seq_select(WORD32 window_sequence_curr, WORD32 window_sequence_last);

IA_ERRORCODE
ia_core_coder_fd_channel_stream(ia_usac_data_struct *usac_data,
                                ia_usac_tmp_core_coder_struct *pstr_core_coder, UWORD8 *max_sfb,
                                WORD32 window_sequence_last, WORD32 chn,
                                WORD32 noise_filling_config,
                                ia_usac_igf_config_struct *igf_config, WORD32 ch,
                                ia_bit_buf_struct *it_bit_buff, WORD32 id);
VOID ia_core_coder_calc_grp_offset(ia_sfb_info_struct *ptr_sfb_info, pUWORD8 group);

#endif /* IA_CORE_CODER_BIT_EXTRACT_H */
