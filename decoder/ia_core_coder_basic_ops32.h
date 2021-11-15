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

#ifndef IA_CORE_CODER_BASIC_OPS32_H
#define IA_CORE_CODER_BASIC_OPS32_H

static PLATFORM_INLINE WORD32 ia_core_coder_mul32_sh_sat(WORD32 a, WORD32 b, WORD8 shift)
{

  WORD32 result;
  WORD64 temp_result;

  temp_result = (WORD64)a * (WORD64)b;
  if ((temp_result >> shift) >= MAX_32)
    return MAX_32;
  if ((temp_result >> shift) <= MIN_32)
    return MIN_32;
  result = (WORD32)(temp_result >> shift);

  return (result);
}
static PLATFORM_INLINE WORD32 ia_core_coder_min32(WORD32 a, WORD32 b)
{
  WORD32 min_val;

  min_val = (a < b) ? a : b;

  return min_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_max32(WORD32 a, WORD32 b)
{
  WORD32 max_val;

  max_val = (a > b) ? a : b;

  return max_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_shl32(WORD32 a, WORD b)
{
  WORD32 out_val;

  b = ((UWORD32)(b << 24) >> 24);
  if (b > 31)
    out_val = 0;
  else
    out_val = (WORD32)a << b;

  return out_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_shr32(WORD32 a, WORD b)
{
  WORD32 out_val;

  b = ((UWORD32)(b << 24) >> 24);
  if (b >= 31)
  {
    if (a < 0)
      out_val = -1;
    else
      out_val = 0;
  }
  else
  {
    out_val = (WORD32)a >> b;
  }

  return out_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_shl32_sat(WORD32 a, WORD b)
{
  WORD32 out_val;
  if (a > (MAX_32 >> b))
    out_val = MAX_32;
  else if (a < (MIN_32 >> b))
    out_val = MIN_32;
  else
    out_val = a << b;
  return (out_val);
}

static PLATFORM_INLINE WORD32 ia_core_coder_shl32_dir(WORD32 a, WORD b)
{
  WORD32 out_val;

  if (b < 0)
  {
    out_val = ia_core_coder_shr32(a, -b);
  }
  else
  {
    out_val = ia_core_coder_shl32(a, b);
  }

  return out_val;
}
static PLATFORM_INLINE WORD64 ia_core_coder_shl32in64_dir(WORD64 a, WORD b)
{
  WORD64 out_val;

  if (b < 0)
  {
    out_val = (WORD64)a >> -b;
  }
  else
  {
    out_val = (WORD64)a << b;
  }

  return out_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_shl32_dir_sat(WORD32 a, WORD b)
{
  WORD32 out_val;

  if (b < 0)
  {
    out_val = ia_core_coder_shr32(a, -b);
  }
  else
  {
    out_val = ia_core_coder_shl32_sat(a, b);
  }

  return out_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_shr32_dir(WORD32 a, WORD b)
{
  WORD32 out_val;

  if (b < 0)
  {
    out_val = ia_core_coder_shl32(a, -b);
  }
  else
  {
    out_val = ia_core_coder_shr32(a, b);
  }

  return out_val;
}

static PLATFORM_INLINE WORD32 shr32_dir_sat(WORD32 a, WORD b)
{
  WORD32 out_val;

  if (b < 0)
  {
    out_val = ia_core_coder_shl32_sat(a, -b);
  }
  else
  {
    out_val = ia_core_coder_shr32(a, b);
  }

  return out_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_mult16x16in32(WORD16 a, WORD16 b)
{
  WORD32 product;

  product = (WORD32)a * (WORD32)b;

  return product;
}

static PLATFORM_INLINE WORD32 mult16x16in32_32(WORD32 a, WORD32 b)
{
  WORD32 product;

  product = (WORD32)a * (WORD32)b;

  return product;
}

static PLATFORM_INLINE WORD32 ia_core_coder_mult16x16in32_shl(WORD16 a, WORD16 b)
{
  WORD32 product;

  product = ia_core_coder_shl32(ia_core_coder_mult16x16in32(a, b), 1);

  return product;
}

static PLATFORM_INLINE WORD32 ia_core_coder_mult16x16in32_shl_sat(WORD16 a, WORD16 b)
{
  WORD32 product;
  product = (WORD32)a * (WORD32)b;
  if (product != (WORD32)0x40000000L)
  {
    product = ia_core_coder_shl32(product, 1);
  }
  else
  {
    product = MAX_32;
  }
  return product;
}

static PLATFORM_INLINE WORD32 ia_core_coder_add32(WORD32 a, WORD32 b)
{
  WORD32 sum;

  sum = (WORD32)a + (WORD32)b;

  return sum;
}

static PLATFORM_INLINE WORD32 ia_core_coder_sub32(WORD32 a, WORD32 b)
{
  WORD32 diff;

  diff = (WORD32)a - (WORD32)b;

  return diff;
}

static PLATFORM_INLINE WORD32 ia_core_coder_add32_sat(WORD32 a, WORD32 b)
{
  WORD64 sum;

  sum = (WORD64)a + (WORD64)b;

  if (sum >= MAX_32)
    return MAX_32;
  if (sum <= MIN_32)
    return MIN_32;

  return (WORD32)sum;
}

static PLATFORM_INLINE WORD32 ia_core_coder_add32_sat3(WORD32 a, WORD32 b, WORD32 c)
{
  WORD64 sum;

  sum = (WORD64)a + (WORD64)b;

  sum = (WORD64)sum + (WORD64)c;

  if (sum > MAX_32)
  {
    sum = MAX_32;
  }
  if (sum < MIN_32)
  {
    sum = MIN_32;
  }

  return (WORD32)sum;
}

static PLATFORM_INLINE WORD32 ia_core_coder_sub32_sat(WORD32 a, WORD32 b)
{
  WORD64 diff;

  diff = (WORD64)a - (WORD64)b;

  if (diff >= MAX_32)
    return MAX_32;
  if (diff <= MIN_32)
    return MIN_32;

  return (WORD32)diff;
}

static PLATFORM_INLINE WORD ia_core_coder_norm32(WORD32 a)
{
  WORD norm_val;

  if (a == 0)
  {
    norm_val = 31;
  }
  else
  {
    if (a == (WORD32)0xffffffffL)
    {
      norm_val = 31;
    }
    else
    {
      if (a < 0)
      {
        a = ~a;
      }
      for (norm_val = 0; a < (WORD32)0x40000000L; norm_val++)
      {
        a <<= 1;
      }
    }
  }

  return norm_val;
}

static PLATFORM_INLINE WORD ia_core_coder_pnorm32(WORD32 a)
{
  WORD norm_val;

  if (a == 0)
  {
    norm_val = 31;
  }
  else
  {
    for (norm_val = 0; a < (WORD32)0x40000000L; norm_val++)
    {
      a <<= 1;
    }
  }

  return norm_val;
}

static PLATFORM_INLINE WORD bin_expo32(WORD32 a)
{
  WORD bin_expo_val;

  bin_expo_val = 31 - ia_core_coder_norm32(a);

  return bin_expo_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_abs32(WORD32 a)
{
  WORD32 abs_val;

  abs_val = a;

  if (a < 0)
  {
    abs_val = -a;
  }

  return abs_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_abs32_nrm(WORD32 a)
{
  WORD32 abs_val;

  abs_val = a;

  if (a < 0)
  {
    abs_val = ~a;
  }

  return abs_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_abs32_sat(WORD32 a)
{
  WORD32 abs_val;

  abs_val = a;

  if (a == MIN_32)
  {
    abs_val = MAX_32;
  }
  else if (a < 0)
  {
    abs_val = -a;
  }

  return abs_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_negate32(WORD32 a)
{
  WORD32 neg_val;

  neg_val = -a;

  return neg_val;
}

static PLATFORM_INLINE WORD32 ia_core_coder_negate32_sat(WORD32 a)
{
  WORD32 neg_val;

  if (a == MIN_32)
  {
    neg_val = MAX_32;
  }
  else
  {
    neg_val = -a;
  }
  return neg_val;
}

static PLATFORM_INLINE WORD32 div32(WORD32 a, WORD32 b, WORD *q_format)
{
  WORD32 quotient;
  UWORD32 mantissa_nr, mantissa_dr;
  WORD16 sign = 0;

  LOOPINDEX i;
  WORD q_nr, q_dr;

  if ((a < 0) && (0 != b))
  {
    if (a == MIN_32)
    {
      a = MAX_32;
    }
    else
    {
      a = -a;
    }
    sign = (WORD16)(sign ^ -1);
  }

  if (b < 0)
  {
    b = -b;
    sign = (WORD16)(sign ^ -1);
  }

  if (0 == b)
  {
    *q_format = 0;
    return (a);
  }

  quotient = 0;

  q_nr = ia_core_coder_norm32(a);
  mantissa_nr = (UWORD32)a << (q_nr);
  q_dr = ia_core_coder_norm32(b);
  mantissa_dr = (UWORD32)b << (q_dr);
  *q_format = (WORD)(30 + q_nr - q_dr);

  for (i = 0; i < 31; i++)
  {
    quotient = quotient << 1;

    if (mantissa_nr >= mantissa_dr)
    {
      mantissa_nr = mantissa_nr - mantissa_dr;
      quotient += 1;
    }

    mantissa_nr = (UWORD32)mantissa_nr << 1;
  }

  if (sign < 0)
  {
    quotient = -quotient;
  }

  return quotient;
}

static PLATFORM_INLINE WORD32 ia_core_coder_mac16x16in32_sat(WORD32 a, WORD16 b, WORD16 c)
{
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32(b, c);

  acc = ia_core_coder_add32_sat(a, acc);

  return acc;
}

static PLATFORM_INLINE WORD32 mac16x16hin32(WORD32 a, WORD32 b, WORD32 c)
{
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32((WORD16)b, (WORD16)(c >> 16));

  acc = ia_core_coder_add32(a, acc);

  return acc;
}

static PLATFORM_INLINE WORD32 ia_core_coder_mac16x16in32_shl(WORD32 a, WORD16 b, WORD16 c)
{
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32_shl(b, c);

  acc = ia_core_coder_add32(a, acc);

  return acc;
}

static PLATFORM_INLINE WORD32 ia_core_coder_mac16x16in32_shl_sat(WORD32 a, WORD16 b, WORD16 c)
{
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32_shl_sat(b, c);

  acc = ia_core_coder_add32_sat(a, acc);

  return acc;
}

static PLATFORM_INLINE WORD32 msu16x16in32(WORD32 a, WORD16 b, WORD16 c)
{
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32(b, c);

  acc = ia_core_coder_sub32(a, acc);

  return acc;
}

static PLATFORM_INLINE WORD32 msu16x16in32_shl(WORD32 a, WORD16 b, WORD16 c)
{
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32_shl(b, c);

  acc = ia_core_coder_sub32(a, acc);

  return acc;
}

static PLATFORM_INLINE WORD32 msu16x16in32_shl_sat(WORD32 a, WORD16 b, WORD16 c)
{
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32_shl_sat(b, c);

  acc = ia_core_coder_sub32_sat(a, acc);

  return acc;
}

static PLATFORM_INLINE WORD32 add32_shr(WORD32 a, WORD32 b)
{
  WORD32 sum;

  a = ia_core_coder_shr32(a, 1);
  b = ia_core_coder_shr32(b, 1);

  sum = ia_core_coder_add32(a, b);

  return sum;
}

static PLATFORM_INLINE WORD32 sub32_shr(WORD32 a, WORD32 b)
{
  WORD32 diff;

  a = ia_core_coder_shr32(a, 1);
  b = ia_core_coder_shr32(b, 1);

  diff = ia_core_coder_sub32(a, b);

  return diff;
}
#endif
