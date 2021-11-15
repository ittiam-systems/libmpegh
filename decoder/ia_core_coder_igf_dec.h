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

#ifndef _IA_CORE_CODER_IGF_DEC_H_
#define _IA_CORE_CODER_IGF_DEC_H_

#define TEN_IN_Q15 327680
#define SFE_GROUP_SIZE 2
#define CTX_OFFSET 3
#define MIN_ENC_SEPARATE -12
#define MAX_ENC_SEPARATE +12
#define SYMBOLS_IN_TABLE (1 + (MAX_ENC_SEPARATE - MIN_ENC_SEPARATE + 1) + 1)

#define CF_OFFSET_SE_01 (+2)
#define CF_OFFSET_SE_10 (-4)

typedef struct
{
  WORD32 igf_curr_q[MAX_SHORT_WINDOWS][NSFB_LONG];
} ia_core_coder_igf_level_scr_t;

WORD32 ia_core_coder_get_igf_levels(ia_bit_buf_struct *it_bit_buff,
                                    ia_usac_data_struct *usac_data,
                                    ia_usac_igf_config_struct *igf_config,
                                    WORD32 num_window_groups, WORD32 chn, WORD32 igf_frame_id,
                                    WORD8 igf_win_type);

VOID ia_core_coder_get_igf_data(ia_bit_buf_struct *it_bit_buff, ia_usac_data_struct *usac_data,
                                ia_usac_igf_config_struct *igf_config, WORD32 chn,
                                WORD32 igf_frame_id, WORD32 igf_win_type);
VOID ia_core_coder_igf_tnf(ia_usac_data_struct *usac_data, ia_usac_igf_config_struct *igf_config,
                           WORD32 chn, WORD32 igf_frame_id, WORD32 igf_win_type, WORD32 win,
                           FLOAT32 *coef, FLOAT32 *scratch);
VOID ia_core_coder_igf_stereofilling(FLOAT32 *coef, FLOAT32 *dmx_prev, ia_sfb_info_struct *info,
                                     WORD32 noise_filling, WORD32 band_quantized_to_zero,
                                     WORD32 stereo_filling, WORD32 win_tot, WORD32 sfb,
                                     WORD32 noise_filling_start_offset, WORD32 grp,
                                     WORD32 max_noise_sfb);
VOID ia_core_coder_igf_mono(ia_usac_data_struct *usac_data, ia_usac_igf_config_struct *igf_config,
                            WORD32 chn, WORD32 igf_win_type, WORD32 igf_frame_id,
                            WORD32 num_groups, WORD16 *group_len, WORD32 bins_per_sbk, WORD32 win,
                            FLOAT32 *coef, FLOAT32 *scratch);
VOID ia_core_coder_igf_init(ia_usac_data_struct *usac_data,
                            ia_usac_dec_element_config_struct *ptr_usac_ele_config,
                            WORD32 ele_type, WORD32 id, UINT32 sample_rate, WORD32 chan);

VOID ia_core_coder_igf_apply_stereo(ia_usac_data_struct *usac_data, WORD32 chn,
                                    ia_usac_igf_config_struct *igf_config,
                                    const WORD32 igf_win_type, const WORD8 *mask,
                                    const WORD32 hasmask, const WORD32 num_groups,
                                    const WORD16 *group_len, const WORD32 bins_per_sbk,
                                    const WORD32 sfb_per_sbk);

VOID ia_core_coder_igf_get_swb_offset(WORD32 igf_win_type, WORD16 *swb_offset,
                                      ia_sfb_info_struct *pstr_usac_winmap);

FLOAT32 ia_core_coder_igf_get_random_sign(UWORD32 *seed);
VOID ia_core_coder_igf_stereo_pred_data(ia_usac_data_struct *usac_data, WORD32 num_window_groups,
                                        WORD32 igf_win_type, ia_bit_buf_struct *it_bit_buff,
                                        WORD32 chn, WORD32 elem_idx);

#endif
