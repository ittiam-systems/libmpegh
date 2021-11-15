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


#ifndef IA_CORE_CODER_BASIC_OPS40_H
#define IA_CORE_CODER_BASIC_OPS40_H
#define lo64(a) (((unsigned *)&a)[0]) /* low 32 bits of a long long */
#define hi64(a) (((WORD32 *)&a)[1])   /* high 32 bits of a long long */

// normalize input to 32 bits, return denormalizing info
static PLATFORM_INLINE WORD16 norm40(WORD40 *in) {
  WORD16 expo;
  WORD32 tempo;

  if (0 == (*in)) return 31;

  if (((*in) <= (WORD40)0x7fffffffLL) && ((WORD40)(*in) >= (WORD40)0xFFFFFFFF80000000LL))
  {
    tempo = (WORD32)(*in);
    expo = ia_core_coder_norm32(tempo);
    *in = tempo << expo;

    return (expo);
  }

  tempo = (WORD32)((*in) >> 31);
  expo = 31 - (ia_core_coder_norm32(tempo));
  *in = (*in) >> expo;

  return (-expo);
}

// adds two numbers and right shifts by 1
static PLATFORM_INLINE WORD32 add32_shr40(WORD32 a, WORD32 b) {
  WORD40 sum;

  sum = (WORD40)a + (WORD40)b;
  sum = sum >> 1;

  return ((WORD32)sum);
}

// subtracts and right shifts by one
static PLATFORM_INLINE WORD32 sub32_shr40(WORD32 a, WORD32 b) {
  WORD40 sum;

  sum = (WORD40)a - (WORD40)b;
  sum = sum >> 1;

  return ((WORD32)sum);
}


// multiply WORD32 with WORD16 return bits 46 to 15
static PLATFORM_INLINE WORD32 ia_core_coder_mult32x16in32_shl(WORD32 a, WORD16 b) {
  WORD32 result;
  __asm__(

      "        SMULWB  %0,  %1,  %2 \n\t"
      "        MOV     %0, %0, LSL   #1 \n\t"
      : "=r"((WORD32)result)
      : "r"((WORD32)a), "r"((WORD16)b));
  return result;
}

// multiply WORD32 with higher 16 bits of second data and return bits 46 to 15
static PLATFORM_INLINE WORD32 mult32x16hin32_shl(WORD32 a, WORD32 b) {
  WORD32 product;
  WORD64 temp_product;

  temp_product = (WORD64)a * (WORD64)(b >> 16);
  product = (WORD32)(temp_product >> 16);

  return (product << 1);
}

// multiply WORD32 with WORD16 return bits 47 to 16
static PLATFORM_INLINE WORD32 ia_core_coder_mult32x16in32(WORD32 a, WORD16 b) {
  WORD32 result;
  __asm__(

      "        SMULWB  %0,  %1,  %2 \n\t"
      : "=r"((WORD32)result)
      : "r"((WORD32)a), "r"((WORD16)b));
  return (result);
}

// multiply WORD32 with WORD16 return bits 46 to 15 with saturation
static PLATFORM_INLINE WORD32 ia_core_coder_mult32x16in32_shl_sat(WORD32 a,
                                                             WORD16 b) {
  WORD32 result;

  __asm__(

      "        SMULWB  %0,  %1,  %2 \n\t"
      "        QADD  %0,  %0,  %0 \n\t"
      : "=r"((WORD32)result)
      : "r"((WORD32)a), "r"((WORD16)b));
  return (result);
}

// multiply WORD32 with WORD32 return bits 62 to 31
static PLATFORM_INLINE WORD32 ia_core_coder_mult32_shl(WORD32 a, WORD32 b) {
  WORD32 resultHi;

  __asm__(

      "			smull r3, %0,  %1,  %2 \n\t"
      "        MOV     %0, %0, LSL   #1 \n\t"
      : "=r"((WORD32)resultHi)
      : "r"((WORD32)a), "r"((WORD32)b)
      : "cc", "r3");

  return resultHi;
}

// multiply WORD32 with WORD32 return bits 63 to 32
static PLATFORM_INLINE WORD32 ia_core_coder_mult32(WORD32 a, WORD32 b) {
  WORD32 resultHi;
  __asm__(

      "			smull r3, %0,  %1,  %2 \n\t"
      : "=r"((WORD32)resultHi)
      : "r"((UWORD32)a), "r"((UWORD32)b)
      : "r3");
  return resultHi;
}

// multiply WORD32 with WORD32 return bits 62 to 31 with saturation
static PLATFORM_INLINE WORD32 ia_core_coder_mult32_shl_sat(WORD32 a, WORD32 b) {
  WORD32 result;

  if (a == (WORD32)0x80000000 && b == (WORD32)0x80000000) {
    result = 0x7fffffff;
  } else {
    result = ia_core_coder_mult32_shl(a, b);
  }

  return (result);
}

// multiply WORD32 with WORD16 add bits 47 to 16 to accumulator
static PLATFORM_INLINE WORD32 ia_core_coder_mac32x16in32(WORD32 a, WORD32 b,
                                                    WORD16 c) {
  WORD32 acc;
  __asm__(

      "		SMLAWB  %0, %2, %3, %1 \n\t"
      : "=r"((WORD32)acc)
      : "r"((WORD32)a), "r"((WORD32)b), "r"((WORD32)c));
  return acc;
}

// multiply WORD32 with WORD16 add bits 46 to 15 to accumulator
static PLATFORM_INLINE WORD32 ia_core_coder_mac32x16in32_shl(WORD32 a, WORD32 b,
                                                        WORD16 c) {
  WORD32 result;

  result = a + ia_core_coder_mult32x16in32_shl(b, c);

  return (result);
}

// multiply WORD32 with WORD16 add bits 46 to 15 to accumulator with saturation
static PLATFORM_INLINE WORD32 mac32x16in32_shl_sat(WORD32 a, WORD32 b,
                                                   WORD16 c) {
  return (ia_core_coder_add32_sat(a, ia_core_coder_mult32x16in32_shl_sat(b, c)));
}

// multiply WORD32 with WORD32 add bits 63 to 32 to accumulator
static PLATFORM_INLINE WORD32 ia_core_coder_mac32(WORD32 a, WORD32 b, WORD32 c) {
  WORD32 result;

  result = a + ia_core_coder_mult32(b, c);

  return (result);
}

// multiply WORD32 with WORD32 add bits 62 to 31 to accumulator
static PLATFORM_INLINE WORD32 mac32_shl(WORD32 a, WORD32 b, WORD32 c) {
  WORD32 result;

  result = a + ia_core_coder_mult32_shl(b, c);

  return (result);
}

// multiply WORD32 with WORD32 add bits 62 to 31 to accumulator with saturation
static PLATFORM_INLINE WORD32 mac32_shl_sat(WORD32 a, WORD32 b, WORD32 c) {
  return (ia_core_coder_add32_sat(a, ia_core_coder_mult32_shl_sat(b, c)));
}

// multiply WORD32 with WORD16 sub bits 47 to 16 from accumulator
static PLATFORM_INLINE WORD32 msu32x16in32(WORD32 a, WORD32 b, WORD16 c) {
  WORD32 result;

  result = a - ia_core_coder_mult32x16in32(b, c);

  return (result);
}

// multiply WORD32 with WORD16 sub bits 46 to 15 from accumulator
static PLATFORM_INLINE WORD32 msu32x16in32_shl(WORD32 a, WORD32 b, WORD16 c) {
  WORD32 result;

  result = a - ia_core_coder_mult32x16in32_shl(b, c);

  return (result);
}

// multiply WORD32 with WORD16 sub bits 46 to 15 from accumulator with
// saturation
static PLATFORM_INLINE WORD32 msu32x16in32_shl_sat(WORD32 a, WORD32 b,
                                                   WORD16 c) {
  return (ia_core_coder_sub32_sat(a, ia_core_coder_mult32x16in32_shl_sat(b, c)));
}

// multiply WORD32 with WORD32 sub bits 63 to 32 from accumulator
static PLATFORM_INLINE WORD32 msu32(WORD32 a, WORD32 b, WORD32 c) {
  WORD32 result;

  result = a - ia_core_coder_mult32(b, c);

  return (result);
}

// multiply WORD32 with WORD32 sub bits 62 to 31 from accumulator
static PLATFORM_INLINE WORD32 msu32_shl(WORD32 a, WORD32 b, WORD32 c) {
  WORD32 result;

  result = a - ia_core_coder_mult32_shl(b, c);

  return (result);
}

// multiply WORD32 with WORD32 sub bits 62 to 31 from accumulator with
// saturation
static PLATFORM_INLINE WORD32 msu32_shl_sat(WORD32 a, WORD32 b, WORD32 c) {
  return (ia_core_coder_sub32_sat(a, ia_core_coder_mult32_shl_sat(b, c)));
}

// returns normalized 32 bit accumulated result
static PLATFORM_INLINE WORD32 mac3216_arr40(WORD32 *x, WORD16 *y,
                                            LOOPINDEX length, WORD16 *q_val) {
  LOOPINDEX i;
  WORD40 sum = 0;

  for (i = 0; i < length; i++) {
    sum += (WORD40)(ia_core_coder_mult32x16in32(x[i], y[i]));
  }

  *q_val = norm40(&sum);

  return (WORD32)sum;
}

// returns normalized 32 bit accumulated result
static PLATFORM_INLINE WORD32 mac32_arr40(WORD32 *x, WORD32 *y,
                                          LOOPINDEX length, WORD16 *q_val) {
  LOOPINDEX i;
  WORD40 sum = 0;

  for (i = 0; i < length; i++) {
    sum += (WORD40)(ia_core_coder_mult32(x[i], y[i]));
  }

  *q_val = norm40(&sum);

  return ((WORD32)sum);
}

// returns normalized 32 bit accumulated result
static PLATFORM_INLINE WORD32 mac16_arr40(WORD16 *x, WORD16 *y,
                                          LOOPINDEX length, WORD16 *q_val) {
  LOOPINDEX i;
  WORD40 sum = 0;

  for (i = 0; i < length; i++) {
    sum += (WORD40)((WORD32)x[i] * (WORD32)y[i]);
  }

  *q_val = norm40(&sum);

  return ((WORD32)sum);
}

// returns normalized 32 bit accumulated result
static PLATFORM_INLINE WORD32 add32_arr40(WORD32 *in_arr, LOOPINDEX length,
                                          WORD16 *q_val) {
  LOOPINDEX i;
  WORD40 sum = 0;

  for (i = 0; i < length; i++) {
    sum += (WORD40)in_arr[i];
  }

  *q_val = norm40(&sum);

  return ((WORD32)sum);
}

// multiply WORD32 with WORD32 return WORD64
static PLATFORM_INLINE WORD64 ia_core_coder_mult32x32in64(WORD32 a, WORD32 b) {
  WORD64 result;

  result = (WORD64)a * (WORD64)b;

  return (result);
}

// multiply WORD32 with WORD32 and accumulate the 64 bit result
static PLATFORM_INLINE WORD64 ia_core_coder_mac32x32in64(WORD64 sum, WORD32 a,
                                                    WORD32 b) {
  sum += (WORD64)a * (WORD64)b;

  return (sum);
}

static PLATFORM_INLINE WORD64 ia_core_coder_mac32x32in64_7(WORD64 sum,
                                                      const WORD32 *a,
                                                      const WORD16 *b) {
  sum = (WORD64)a[0] * (WORD64)b[0];
  sum += (WORD64)a[1] * (WORD64)b[1];
  sum += (WORD64)a[2] * (WORD64)b[2];
  sum += (WORD64)a[3] * (WORD64)b[3];
  sum += (WORD64)a[4] * (WORD64)b[4];
  sum += (WORD64)a[5] * (WORD64)b[5];
  sum += (WORD64)a[6] * (WORD64)b[6];

  return (sum);
}

static PLATFORM_INLINE WORD64 ia_core_coder_mac32x32in64_n(WORD64 sum,
                                                      const WORD32 *a,
                                                      const WORD16 *b,
                                                      WORD32 n) {
  WORD32 k;

  sum += (WORD64)a[0] * (WORD64)b[0];
  for (k = 1; k < n; k++) sum += (WORD64)a[k] * (WORD64)b[k];
  return (sum);
}

static PLATFORM_INLINE WORD64 ia_core_coder_mult64(WORD32 a, WORD32 b) {
  WORD64 result;
  result = (WORD64)a * (WORD64)b;
  return (result);
}

static PLATFORM_INLINE WORD64 ia_core_coder_mult64_in64(WORD64 a, WORD32 b)
{
	WORD64 result;
	result = a * b;
	return (result);
}

static PLATFORM_INLINE WORD64 ia_core_coder_mult64_sat(WORD64 a, WORD64 b) {
  WORD64 result;

  if (a > 0 && b > 0 && a > (WORD64)MAX_64 / b)
    return (WORD64)MAX_64;
  if (a < 0 && b > 0 && a < (WORD64)MIN_64 / b)
    return (WORD64)MIN_64;
  if (a > 0 && b < 0 && b < (WORD64)MIN_64 / a)
    return (WORD64)MIN_64;
  if (a < 0 && b < 0 && a < (WORD64)MAX_64 / b)
    return (WORD64)MAX_64;

  result = a * b;
  return (result);
}


static PLATFORM_INLINE WORD64 ia_core_coder_add64(WORD64 a, WORD64 b) {
  WORD64 result;
  result = a + b;
  return (result);
}

static PLATFORM_INLINE WORD64 ia_core_coder_add64_sat(WORD64 a, WORD64 b) {
  WORD64 result , comp;
  result = (a < 0) ? (WORD64)MIN_64 : (WORD64)MAX_64;
  comp = result - a;
  if ((a < 0) == (b > comp)) 
	result = a + b;

  return (result);
}

static PLATFORM_INLINE WORD32 ia_core_coder_sat64_32(WORD64 a) {
	WORD32 result;
	if (a >= MAX_32) {
		result = MAX_32;
	}
	else if (a <= MIN_32) {
		result = MIN_32;
	}
	else
		result = (WORD32)a;

	return (result);
}

static PLATFORM_INLINE WORD64 ia_core_coder_sub64(WORD64 a, WORD64 b) {
  WORD64 diff;

  diff = (WORD64)a - (WORD64)b;

  return diff;
}

static PLATFORM_INLINE WORD64 ia_core_coder_sub64_sat(WORD64 a, WORD64 b) {
  WORD64 diff;

  diff = ia_core_coder_sub64(a, b);

  if ((((WORD64)a ^ (WORD64)b) & (WORD64)MIN_64) != 0) {
    if (((WORD64)diff ^ (WORD64)a) & (WORD64)MIN_64) {
      diff = (a < 0L) ? (WORD64)MIN_64 : (WORD64)MAX_64;
    }
  }

  return (diff);
}

static PLATFORM_INLINE WORD32 ia_core_coder_mul32_sh(WORD32 a, WORD32 b,
                                                WORD8 shift) {
  WORD32 result;
  WORD64 temp_result;

  temp_result = (WORD64)a * (WORD64)b;
  result = (WORD32)(temp_result >> shift);

  return (result);
}





#endif
