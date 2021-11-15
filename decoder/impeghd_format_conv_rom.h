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

#ifndef IMPEGHD_FORMAT_CONV_ROM_H
#define IMPEGHD_FORMAT_CONV_ROM_H
#define IMPEGHD_CLIP(val, ULIM, LLIM)                                                            \
  ((val) = (val) > (ULIM) ? (ULIM) : ((val) < (LLIM)) ? (LLIM) : (val))
extern const WORD32 ia_dmx_rules_fc[][8];
extern const WORD32 ia_erb_idx_freq_fc[58];
extern const FLOAT32 ia_eq_88k_freq_fc[58];
extern const FLOAT32 ia_eq_64k_freq_fc[58];
extern const FLOAT32 ia_eq_48k_freq_fc[58];
extern const FLOAT32 ia_eq_44k_freq_fc[58];
extern const FLOAT32 ia_eq_32k_freq_fc[58];
extern const FLOAT32 ia_eq_24k_freq_fc[58];
extern const FLOAT32 ia_eq_22k_freq_fc[58];
extern const FLOAT32 ia_eq_16k_freq_fc[58];
extern const FLOAT32 ia_eq_12k_freq_fc[58];
extern const FLOAT32 ia_eq_11k_freq_fc[58];
extern const FLOAT32 ia_eq_8k_freq_fc[58];
extern const FLOAT32 ia_peak_filter_params_fc[5][12];
extern const FLOAT32 *ia_eq_freq_fc[12];
extern const WORD32 ia_comp_tmplt_inp_idx[];
extern const WORD32 ia_comp_tmplt_out_idx[];
extern const WORD32 *ia_comp_tmplt_data[];
extern const FLOAT32 ia_eq_precisions[4];
extern const FLOAT32 ia_eq_min_ranges[2][4];
extern const WORD32 ia_erb_freq_idx_256_58[58];
extern const FLOAT32 ia_nrm_stft_erb_58[58];

#endif
