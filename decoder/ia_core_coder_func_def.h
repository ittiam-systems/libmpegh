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

#ifndef IA_CORE_CODER_FUNC_DEF_H
#define IA_CORE_CODER_FUNC_DEF_H

#define restrict

VOID ia_core_coder_qn_data(WORD32 nk_mode, WORD32 *qn, ia_bit_buf_struct *it_bit_buff,
                           WORD32 fblpd);
VOID ia_core_coder_memset(FLOAT32 *x, WORD32 n);
VOID ia_core_coder_mem_cpy(const FLOAT32 *x, FLOAT32 *y, WORD32 n);

VOID ia_core_coder_vec_cnst_mul(FLOAT32 a, FLOAT32 *x, FLOAT32 *z, WORD32 n);
WORD32 ia_core_coder_lpd_dec(ia_usac_data_struct *usac_data, ia_usac_lpd_decoder_handle st,
                             ia_td_frame_data_struct *pstr_td_frame_data, FLOAT32 *fsynth,
                             WORD32 first_lpd_flag, WORD32 short_fac_flag,
                             WORD32 bpf_control_info, WORD32 elem_idx, WORD32 chan);
IA_ERRORCODE ia_core_coder_lpd_bpf(ia_usac_data_struct *usac_data, WORD32 is_short_flag,
                                   FLOAT32 *out_buffer, ia_usac_lpd_decoder_handle st);

VOID ia_core_coder_lpd_dec_update(ia_usac_lpd_decoder_handle tddec,
                                  ia_usac_data_struct *usac_data, WORD32 i_ch, WORD32 elem_idx);

WORD32 ia_core_coder_init_acelp_data(ia_usac_data_struct *usac_data,
                                     ia_usac_lpd_decoder_handle st, WORD32 chn);

WORD32 ia_core_coder_acelp_alias_cnx(ia_usac_data_struct *usac_data,
                                     ia_td_frame_data_struct *pstr_td_frame_data, WORD32 k,
                                     FLOAT32 A[], FLOAT32 stab_fac, ia_usac_lpd_decoder_handle st,
                                     ia_usac_td_config_handle td_config, FLOAT32 *synth_fb,
                                     FLOAT32 *pitch_buffer, FLOAT32 *voice_factors,
                                     FLOAT32 *scratch);
WORD32 ia_core_coder_tcx_mdct(ia_usac_data_struct *usac_data,
                              ia_td_frame_data_struct *pstr_td_frame_data, WORD32 frame_index,
                              FLOAT32 *a, FLOAT32 *synth_fb, WORD32 long_frame,
                              ia_usac_lpd_decoder_handle st, WORD32 elem_idx, FLOAT32 *scratch);

VOID ia_core_coder_fac_decoding(WORD32 fac_len, WORD32 k, WORD32 *fac_prm,
                                ia_bit_buf_struct *it_bit_buff);

IA_ERRORCODE ia_core_coder_lpc_to_mdct(FLOAT32 *lpc_coeffs, WORD32 lpc_order, FLOAT32 *mdct_gains,
                                       WORD32 lg, FLOAT32 *ptr_scratch);
VOID ia_core_coder_alg_vec_dequant(ia_td_frame_data_struct *pstr_td_frame_data,
                                   WORD32 first_lpd_flag, FLOAT32 *lsf, WORD32 *mod,
                                   WORD32 num_div);

VOID ia_core_coder_interpolate_lsp_params(FLOAT32 *lsp_old, FLOAT32 *lsp_new,
                                          FLOAT32 *lp_flt_coff_a, WORD32 nb_subfr);
VOID ia_core_coder_interpolate_lpc_coef(FLOAT32 *lsf_old, FLOAT32 *lsf_new, FLOAT32 *a,
                                        WORD32 nb_subfr, WORD32 m);
VOID ia_core_coder_noise_shaping(FLOAT32 *r, WORD32 lg, WORD32 M, FLOAT32 *gain1, FLOAT32 *gain2,
                                 FLOAT32 *ptr_scratch);
VOID ia_core_coder_tw_create(VOID);

VOID ia_core_coder_huffman_decode(WORD32 it_bit_buff, WORD16 *h_index, WORD16 *len,
                                  const UWORD16 *input_table, const UWORD32 *idx_table);

#endif
