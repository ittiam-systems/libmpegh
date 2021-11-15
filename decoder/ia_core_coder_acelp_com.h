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

#ifndef IA_CORE_CODER_ACELP_COM_H
#define IA_CORE_CODER_ACELP_COM_H

#define LEN_ABS_LEADER 37
#define LEN_SIGN_LEADER 226
#define LEN_I3 9
#define LEN_I4 28

extern const FLOAT32 ia_core_coder_int_leave_gain_table[256];

VOID ia_core_coder_rotated_gosset_mtx_dec(WORD32 qn, WORD32 code_book_idx, WORD32 *kv, WORD32 *y);

VOID ia_core_coder_residual_tool(FLOAT32 *a, FLOAT32 *x, FLOAT32 *y, WORD32 l, WORD32 loop_count,
                                 WORD32 incr_val);

VOID ia_core_coder_synthesis_tool(FLOAT32 *a, FLOAT32 *x, FLOAT32 *y, WORD32 l, FLOAT32 *mem,
                                  FLOAT32 *scratch);

VOID ia_core_coder_synthesis_tool1(FLOAT32 *a, FLOAT32 *x, WORD32 l);

VOID ia_core_coder_lpc_wt_synthesis_tool(FLOAT32 *a, FLOAT32 *x, WORD32 l);

VOID ia_core_coder_preemphsis_tool(FLOAT32 *signal, FLOAT32 mu, WORD32 len, FLOAT32 mem);

VOID ia_core_coder_deemphsis_tool(FLOAT32 *signal, WORD32 len, FLOAT32 mem);

VOID ia_core_coder_lsp_to_lp_conversion(FLOAT32 *lsp, FLOAT32 *a);

VOID ia_core_coder_lpc_coeff_wt_apply(FLOAT32 *a, FLOAT32 *ap);

VOID ia_core_coder_acelp_decode_pulses_per_track(WORD32 *index, const WORD16 nbbits,
                                                 FLOAT32 *code);

IA_ERRORCODE ia_core_coder_bass_post_filter(FLOAT32 *synth_sig, FLOAT32 *synth_fb, WORD32 fac_fb,
                                            WORD32 *pitch, FLOAT32 *pitch_gain,
                                            FLOAT32 *synth_out, WORD32 len_fr, WORD32 len2,
                                            FLOAT32 *bpf_prev);
#endif /* IA_CORE_CODER_ACELP_COM_H */
