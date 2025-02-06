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

#ifndef IA_CORE_CODER_INTERFACE_H
#define IA_CORE_CODER_INTERFACE_H

enum
{
  LN = 2048,
  SN = 256,
  LN2 = LN / 2,
  SN2 = SN / 2,
  LN4 = LN / 4,
  SN4 = SN / 4,
  NSHORT = LN / SN,
  MAX_SBK = NSHORT,

  MAXBANDS = 16 * NSHORT,
  MAXFAC = 121,
  MIDFAC = (MAXFAC - 1) / 2,
  SF_OFFSET = 100,

  LEN_TAG = 4,
  LEN_MAX_SFBS = 4,
  LEN_MAX_SFBL = 6,
  LEN_SAMP_IDX = 4,
  LEN_PC_COMM = 8,
};

IA_ERRORCODE ia_core_coder_search_asi(VOID *handle, UWORD8 *buffer,
                                        WORD32 buffer_len);
IA_ERRORCODE ia_core_code_mdp(VOID *handle, WORD32 preset_id);

IA_ERRORCODE ia_core_coder_dec_main(VOID *handle, WORD8 *inbuffer, WORD8 *outbuffer,
                                    WORD32 *out_bytes, WORD32 frames_done, WORD32 pcmsize,
                                    WORD32 *num_channel_out);

void ia_core_coder_calc_pre_twid_dec(FLOAT32 *ptr_x, FLOAT32 *r_ptr, FLOAT32 *i_ptr,
                                     WORD32 nlength, WORD16 transform_kernel_type,
                                     const FLOAT32 *cos_ptr, const FLOAT32 *sin_ptr);

VOID ia_core_coder_calc_post_twid_dec(FLOAT32 *xptr, FLOAT32 *r_ptr, FLOAT32 *i_ptr,
                                      WORD32 nlength, WORD16 transform_kernel_type,
                                      const FLOAT32 *cos_ptr, const FLOAT32 *sin_ptr

                                      );
#endif /* #ifndef IA_CORE_CODER_INTERFACE_H */
