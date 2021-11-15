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

#ifndef IA_CORE_CODER_ARITH_DEC_H
#define IA_CORE_CODER_ARITH_DEC_H

#define ARITH_ESCAPE 16
#define INVALID_BIT_COUNT -1

typedef struct
{
  WORD32 low;
  WORD32 high;
  WORD32 value;
} ia_state_arith;

typedef struct
{
  WORD32 scratch_prev_tmp[516];
  WORD32 scratch_tmp[516];
  WORD32 scratch_x_ac_dec[1024];
  WORD32 scratch_map_context[1032];
} ia_arith_dec_scratch;

IA_ERRORCODE ia_core_coder_ac_spectral_data(
    ia_usac_data_struct *usac_data, WORD32 max_spec_coefficients, WORD32 noise_level,
    WORD32 arth_size, ia_bit_buf_struct *it_bit_buff, UWORD8 max_sfb, WORD32 max_noise_sfb,
    ia_usac_igf_config_struct *igf_config, WORD32 igf_num_tiles, WORD32 igf_all_zero,
    WORD32 reset, WORD32 noise_filling, WORD32 chn, WORD32 ch, WORD32 fdp_spacing_idx);

IA_ERRORCODE ia_core_coder_arith_data(ia_td_frame_data_struct *pstr_td_frame_data, WORD32 *quant,
                                      ia_usac_data_struct *usac_data,
                                      ia_bit_buf_struct *it_bit_buff, WORD32 first_tcx_flag,
                                      WORD32 k);

WORD32 ia_core_coder_arith_decode(ia_bit_buf_struct *it_bit_buff, WORD32 bit_count, WORD32 *m,
                                  ia_state_arith *s, UWORD16 const *cum_freq, WORD32 cfl);

WORD32 ia_core_coder_arith_first_symbol(ia_bit_buf_struct *it_bit_buff, ia_state_arith *s);

#endif
