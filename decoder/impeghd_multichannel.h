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

#ifndef IMPEGHD_MULTICHANNEL_H_
#define IMPEGHD_MULTICHANNEL_H_ 1

#define CODE_BOOK_BETA_LAV 65
#define CODE_BOOK_ALPHA_LAV 121
#define DEFAULT_BETA (48)
#define DEFAULT_ALPHA (0)

IA_ERRORCODE impeghd_mc_init(ia_usac_decoder_config_struct *pstr_usac_dec_config,
                             jmp_buf *xmpeghd_jmp_buf, WORD32 num_ch, WORD32 start_ch,
                             WORD32 start_element, WORD32 mct_cnt);

IA_ERRORCODE impeghd_mc_parse(ia_multichannel_data *ptr_mc_data, UWORD8 *ptr_bitbuf,
                              UWORD32 bitbuf_len, const WORD32 indep_flag,
                              ia_sfb_info *ptr_sfb_info[MAX_TIME_CHANNELS],
                              ia_bit_buf_struct *ptr_dec_bit_buf);

VOID impeghd_mc_save_prev(ia_multichannel_data *ptr_mc_data, FLOAT32 *ptr_spec, const WORD32 ch,
                          const WORD32 num_windows, const WORD32 bins_per_sbk,
                          const WORD32 zero_spec_save);

VOID impeghd_mc_get_prev_dmx(ia_multichannel_data *ptr_mc_data, FLOAT32 *ptr_prev_spec1,
                             FLOAT32 *ptr_prev_spec2, FLOAT32 *ptr_prev_dmx,
                             const WORD32 bands_per_window, const WORD32 *ptr_mask,
                             const WORD32 *ptr_coeff_sfb_idx, const WORD32 num_samples,
                             const WORD32 pair, const WORD32 total_sfb,
                             const WORD16 *ptr_sfb_offset);

VOID impeghd_mc_stereofilling_add(FLOAT32 *ptr_coef, FLOAT32 *ptr_dmx_prev,
                                  FLOAT32 *ptr_scale_factors, const WORD32 total_sfb,
                                  const WORD32 group_len, const WORD32 bins_per_sbk,
                                  const WORD16 *ptr_sbk_sfb_top);

VOID impeghd_mc_process(ia_multichannel_data *ptr_mc_data, FLOAT32 *ptr_dmx, FLOAT32 *ptr_res,
                        WORD32 *ptr_coeff_sfb_idx, WORD32 *ptr_mask, WORD32 bands_per_window,
                        WORD32 total_sfb, WORD32 pair, const WORD16 *ptr_sfb_offset,
                        WORD32 num_samples);

#endif /* IMPEGHD_MULTICHANNEL_H_*/
