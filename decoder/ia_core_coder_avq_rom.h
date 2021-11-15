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

#ifndef IA_CORE_CODER_AVQ_ROM_H
#define IA_CORE_CODER_AVQ_ROM_H
extern const WORD32 ia_core_coder_7_factorial[8];
extern const WORD32 ia_core_coder_index_table_iso_code[LEN_ABS_LEADER];
extern const UWORD8 ia_core_coder_data_table_iso_code[LEN_SIGN_LEADER];
extern const UWORD32 ia_core_coder_is_signed_leader[LEN_SIGN_LEADER];
extern const WORD32 ia_core_coder_num_table_iso_code[], ia_core_coder_a3_pos_abs_leaders[],
    ia_core_coder_a4_pos_abs_leaders[];
extern const UWORD8 ia_core_coder_tab_da_absolute_leader[][8];
extern const UWORD32 ia_core_coder_i3_cardinality_offset_table[];
extern const UWORD32 ia_core_coder_i4_cardinality_offset_tab[];
extern const FLOAT32 ia_core_coder_interpol_filt[INTER_LP_FIL_LEN];
extern const FLOAT32 ia_core_coder_fir_lp2_filt[1 + 2 * (FILTER_DELAY) + 1];
extern const FLOAT32 ia_core_coder_fir_lp_filt[1 + FILTER_DELAY];
extern const FLOAT32 ia_core_coder_lsf_init[ORDER];

#endif /*IA_CORE_CODER_AVQ_ROM_H*/