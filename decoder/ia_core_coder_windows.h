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

#ifndef IA_CORE_CODER_WINDOWS_H
#define IA_CORE_CODER_WINDOWS_H

extern const FLOAT32 ia_core_coder_sine_window64[64];
extern const FLOAT32 ia_core_coder_sine_window96[96];
extern const FLOAT32 ia_core_coder_sine_window128[128];
extern const FLOAT32 ia_core_coder_sine_window192[192];
extern const FLOAT32 ia_core_coder_sine_window256[256];
extern const FLOAT32 ia_core_coder_sine_window1024[1024];

extern const FLOAT32 ia_core_coder_sine_window384[384];
extern const FLOAT32 ia_core_coder_sine_window512[512];
extern const FLOAT32 ia_core_coder_kbd_window64[64];
extern const FLOAT32 ia_core_coder_kbd_window96[96];
extern const FLOAT32 ia_core_coder_kbd_window128[128];
extern const FLOAT32 ia_core_coder_kbd_window192[192];
extern const FLOAT32 ia_core_coder_kbd_window256[256];
extern const FLOAT32 ia_core_coder_kbd_window384[384];
extern const FLOAT32 ia_core_coder_kbd_window512[512];

extern const FLOAT32 ia_core_coder_kbd_window128[128];
extern const FLOAT32 ia_core_coder_kbd_win960[960];
extern const FLOAT32 ia_core_coder_kbd_win1024[1024];
extern const FLOAT32 kbd_win256[256];
extern const FLOAT32 ia_core_coder_kbd_win4[4];
extern const FLOAT32 ia_core_coder_kbd_win16[16];
extern const FLOAT32 ia_core_coder_kbd_win768[768];
extern const FLOAT32 ia_core_coder_kbd_win192[192];
extern const FLOAT32 ia_core_coder_kbd_win48[48];

IA_ERRORCODE ia_core_coder_calc_window(FLOAT32 **win, WORD32 len, WORD32 wfun_select);

WORD32 ia_core_coder_acelp_imdct(FLOAT32 *imdct_in, WORD32 npoints, WORD16 transform_kernel_type,
                                 FLOAT32 *scratch, FLOAT32 *ptr_fft_scratch);

typedef struct
{
  WORD32 lfac;
  WORD32 n_flat_ls;
  WORD32 n_trans_ls;
  WORD32 n_long;
  WORD32 n_short;

} offset_lengths;

#endif
