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


#ifndef IA_CORE_CODER_BASIC_OP_H
#define IA_CORE_CODER_BASIC_OP_H

#define add_d(a, b) ((a) + (b))
#define sub_d(a, b) ((a) - (b))
#define ia_cbrt_calc(a) cbrt(1.0f / a)

static PLATFORM_INLINE WORD32 msu32x16in32_dual(WORD32 a, WORD16 c1, WORD32 b,
                                                WORD16 c2) {
  WORD32 result;
  WORD32 temp_result;
  UWORD32 a_lsb;
  WORD32 a_msb;
  UWORD32 b_lsb;
  WORD32 b_msb;

  a_lsb = a & 65535;
  a_msb = a >> 16;

  b_lsb = b & 65535;
  b_msb = b >> 16;
  temp_result = ((UWORD32)a_lsb * (UWORD32)c1);
  temp_result = temp_result - (UWORD32)b_lsb * (UWORD32)c2;
  temp_result = ((WORD32)temp_result) >> 16;
  result = temp_result + ((a_msb * (WORD32)c1) - (b_msb * (WORD32)c2));

  return (result);
}

static PLATFORM_INLINE WORD32 mac32x16in32_dual(WORD32 a, WORD16 c1, WORD32 b,
                                                WORD16 c2) {
  WORD32 result;
  WORD32 temp_result;
  UWORD32 a_lsb;
  WORD32 a_msb;
  UWORD32 b_lsb;
  WORD32 b_msb;

  a_lsb = a & 65535;
  a_msb = a >> 16;

  b_lsb = b & 65535;
  b_msb = b >> 16;
  temp_result = (UWORD32)a_lsb * (UWORD32)c1;
  temp_result = temp_result + (UWORD32)b_lsb * (UWORD32)c2;
  temp_result = ((UWORD32)temp_result) >> 16;
  result = temp_result + ((a_msb * (WORD32)c1)) + ((b_msb * (WORD32)c2));
  return (result);
}

static PLATFORM_INLINE WORD64 mac32x32in64_dual(WORD32 a, WORD32 b, WORD64 c) {
  WORD64 result;
  WORD64 temp_result;

  temp_result = (WORD64)a * (WORD64)b;
  result = c + (temp_result);
  return (result);
}
#endif
