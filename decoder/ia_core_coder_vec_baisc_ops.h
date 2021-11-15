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

#ifndef IA_CORE_CODER_VEC_BAISC_OPS_H
#define IA_CORE_CODER_VEC_BAISC_OPS_H

VOID ia_core_coder_windowing_long1(FLOAT32 *src1, FLOAT32 *src2, const FLOAT32 *win_fwd,
                                   const FLOAT32 *win_rev, FLOAT32 *dest,
                                   WORD16 transform_kernel_type, WORD32 vlen);

VOID ia_core_coder_windowing_long2(FLOAT32 *src1, const FLOAT32 *win_fwd, FLOAT32 *fac_data_out,
                                   FLOAT32 *over_lap, FLOAT32 *p_out_buffer,
                                   offset_lengths *ia_core_coder_drc_offset);

VOID ia_core_coder_windowing_long3(FLOAT32 *src1, const FLOAT32 *win_fwd, FLOAT32 *over_lap,
                                   FLOAT32 *p_out_buffer, const FLOAT32 *win_rev,
                                   offset_lengths *ia_core_coder_drc_offset,
                                   WORD32 transform_kernel_type);

VOID ia_core_coder_windowing_short1(FLOAT32 *src1, FLOAT32 *src2, FLOAT32 *fp,
                                    offset_lengths *ia_core_coder_drc_offset);

VOID ia_core_coder_windowing_short2(FLOAT32 *src1, FLOAT32 *win_fwd, FLOAT32 *fp,
                                    offset_lengths *ia_core_coder_drc_offset,
                                    WORD32 transform_kernel_type);

VOID ia_core_coder_windowing_short3(FLOAT32 *src1, FLOAT32 *win_rev, FLOAT32 *fp, WORD32 n_short,
                                    WORD32 transform_kernel_type);

VOID ia_core_coder_windowing_short4(FLOAT32 *src1, FLOAT32 *win_fwd, FLOAT32 *fp,
                                    FLOAT32 *win_fwd1, WORD32 nshort, WORD32 flag,
                                    WORD32 transform_kernel_type);

#endif
