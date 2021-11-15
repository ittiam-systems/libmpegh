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

#ifndef IMPEGHD_TBE_ROM_H
#define IMPEGHD_TBE_ROM_H
extern const FLOAT32 ia_core_coder_ap_filter1[AP_NUM];
extern const FLOAT32 ia_core_coder_ap_filter2[AP_NUM];
extern const FLOAT32 ia_core_coder_wac[LPC_WHTN_ORDER + 1];
extern const FLOAT32 ia_core_coder_recip_order[15];
extern const FLOAT32 ia_core_coder_tbe_interpol_frac[NUM_TBE_FRAME * 2];
extern const FLOAT32 ia_core_coder_gain_frame_5bit[32];
extern const FLOAT32 ia_core_coder_sub_gain_5bit[128];
extern const FLOAT32 ia_core_coder_tbe_exc_filter_cb1_7b[1280];
extern const FLOAT32 ia_core_coder_tbe_exc_filter_cb2_4b[96];
extern const FLOAT32 ia_core_coder_tbe_lsf_cb1_7b[128 * LPC_ORDER_TBE];
extern const FLOAT32 ia_core_coder_tbe_lsf_cb2_7b[128 * LPC_ORDER_TBE];
extern const FLOAT32 ia_core_coder_sub_win_tbe[TBE_OVERLAP_LEN + 1];
extern const FLOAT32 ia_core_coder_subwin_tbe_fb[2 * TBE_OVERLAP_LEN + 1];
extern const FLOAT32 ia_core_coder_tbe_win_flatten[LEN_TBE_FRAME / 2];
extern const FLOAT32 ia_core_coder_gaus_dico[256];
extern const FLOAT32 ia_core_coder_lsp_prev_interp[10];
extern const FLOAT32 ia_core_coder_tdr_filter_interp[L_FILT_MAX + 1];
#endif /*IMPEGHD_TBE_ROM_H*/