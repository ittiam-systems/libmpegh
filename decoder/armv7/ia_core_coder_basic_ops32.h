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
// returns the minima of 2 32 bit variables
static PLATFORM_INLINE WORD32 ia_core_coder_min32(WORD32 a, WORD32 b) {
  WORD32 min_val;

  __asm__ __volatile__(
      "  		CMP		%1, 	%2 \n\t"
      "		MOVGT	%0, 	%2 \n\t"
      "		MOVLE	%0, 	%1 \n\t"
      : "=r"((WORD32)min_val)
      : "r"((WORD32)a), "r"((WORD32)b)
      : "cc");

  return min_val;
}

// returns the maxima of 2 32 bit variables
static PLATFORM_INLINE WORD32 ia_core_coder_max32(WORD32 a, WORD32 b) {
  WORD32 max_val;
  __asm__ __volatile__(
      "  		CMP		%1, 	%2 \n\t"
      "		MOVLE	%0, 	%2 \n\t"
      "		MOVGT	%0, 	%1 \n\t"
      : "=r"((WORD32)max_val)
      : "r"((WORD32)a), "r"((WORD32)b)
      : "cc");

  return max_val;
}

// shifts a 32-bit value left by specificed bits
static PLATFORM_INLINE WORD32 ia_core_coder_shl32(WORD32 a, WORD b) {
  WORD32 out_val;
  __asm__(

      "          MOV   %0,  %1,  LSL %2 \n\t"
      : "=r"((WORD32)out_val)
      : "r"((WORD32)a), "r"((WORD32)b));

  return (out_val);
}

// shifts a 32-bit value right by specificed bits
static PLATFORM_INLINE WORD32 ia_core_coder_shr32(WORD32 a, WORD b) {
  WORD32 out_val;
  __asm__(

      "          MOV   %0,  %1,  ASR %2 \n\t"
      : "=r"((WORD32)out_val)
      : "r"((WORD32)a), "r"((WORD32)b));

  return out_val;
}

// shifts a 32-bit value left by specificed bits and saturates it to 32 bits
static PLATFORM_INLINE WORD32 ia_core_coder_shl32_sat(WORD32 a, WORD b) {
  WORD32 out_val = a;
  // WORD32 dummy1=0/*,dummy2=0*/;

  __asm__ __volatile__(
      "			RSBS   r3,  %2,  #31 \n\t"
      "		    MOVS  r3,  %1,  ASR r3 \n\t"
      "		    CMNLT r3,  #1 \n\t"
      "		    MOVLT %0,  #0x80000000 \n\t"
      "		    MOVGT %0,  #0x7fffffff \n\t"
      "           MOVEQ %0,  %1,  LSL %2 \n\t"
      : "=r"((WORD32)out_val)
      : "r"((WORD32)a), "r"((WORD32)b)
      : "cc", "r3");

  return (out_val);
}

// shifts a 32-bit value left by specificed bits, shifts
// it right if specified no. of bits is negative

static PLATFORM_INLINE WORD32 ia_core_coder_shl32_dir(WORD32 a, WORD b) {
  WORD32 out_val = 0;
  // WORD32	dummy=0;

  __asm__ __volatile__(

      "		RSBS    r3,  %2,  #0 \n\t"
      "		MOVMI   %0,  %1,  LSL %2 \n\t"
      "		MOVPL   %0,  %1,  ASR r3 \n\t"
      : "=r"((WORD32)out_val)
      : "r"((WORD32)a), "r"((WORD)b)
      : "cc", "r3");
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


// shifts a 32-bit value left by specificed bits with sat,
// shifts it right if specified no. of bits is negative

static PLATFORM_INLINE WORD32 ia_core_coder_shl32_dir_sat(WORD32 a, WORD b) {
  WORD32 out_val;

  if (b < 0) {
    out_val = ia_core_coder_shr32(a, -b);
  } else {
    out_val = ia_core_coder_shl32_sat(a, b);
  }

  return out_val;
}

// shifts a 32-bit value right by specificed bits, shifts
// it left if specified no. of bits is negative
static PLATFORM_INLINE WORD32 ia_core_coder_shr32_dir(WORD32 a, WORD b) {
  WORD32 out_val = 0;
  __asm__ __volatile__(
      "		RSBS    r3,  %2,  #0 \n\t"
      "		MOVMI   %0,  %1,  ASR %2 \n\t"
      "		MOVPL   %0,  %1,  LSL r3 \n\t"
      : "=r"((WORD32)out_val)
      : "r"((WORD32)a), "r"((WORD32)b)
      : "cc", "r3");

  return out_val;
}

// shifts a 32-bit value right by specificed bits, shifts
// it left with sat if specified no. of bits is negative
static PLATFORM_INLINE WORD32 shr32_dir_sat(WORD32 a, WORD b) {
  WORD32 out_val;

  if (b < 0) {
    out_val = ia_core_coder_shl32_sat(a, -b);
  } else {
    out_val = ia_core_coder_shr32(a, b);
  }

  return out_val;
}

// multiplies two 16 bit numbers and returns their 32-bit result
static PLATFORM_INLINE WORD32 ia_core_coder_mult16x16in32(WORD16 a, WORD16 b) {
  WORD32 product;
  __asm__(

      "                SMULBB   %0 ,  %1,  %2 \n\t"
      : "=r"((WORD32)product)
      : "r"((WORD16)a), "r"((WORD16)b));
  return product;
}

// multiplies two 16 bit numbers and returns their 32-bit result
static PLATFORM_INLINE WORD32 mult16x16in32_32(WORD16 a, WORD16 b) {
  WORD32 product;
  __asm__(

      "                SMULBB   %0 ,  %1,  %2 \n\t"
      : "=r"((WORD32)product)
      : "r"((WORD16)a), "r"((WORD16)b));
  return product;
}


// multiplies two 16 bit numbers and returns their 32-bit
// result after removing 1 redundant sign bit
static PLATFORM_INLINE WORD32 ia_core_coder_mult16x16in32_shl(WORD16 a, WORD16 b) {
  WORD32 product;
  __asm__(

      "                SMULBB   %0 ,  %1,  %2 \n\t"
      "				MOV		%0,	%0,	LSL #1 \n\t"
      : "=r"((WORD32)product)
      : "r"((WORD16)a), "r"((WORD16)b));
  return product;
}

// multiplies two 16 bit numbers and returns their 32-bit
// result after removing 1 redundant sign bit with saturation
static PLATFORM_INLINE WORD32 ia_core_coder_mult16x16in32_shl_sat(WORD16 a,
                                                             WORD16 b) {
  WORD32 product;
  __asm__(

      "                SMULBB   %0 ,  %1,  %2 \n\t"
      "				QADD		%0,	%0,	%0 \n\t"
      : "=r"((WORD32)product)
      : "r"((WORD16)a), "r"((WORD16)b));
  return product;
}

// adds 2 32 bit variables
static PLATFORM_INLINE WORD32 ia_core_coder_add32(WORD32 a, WORD32 b) {
  WORD32 sum;
  __asm__(

      "                ADD   %0 ,  %1,  %2 \n\t"
      : "=r"((WORD32)sum)
      : "r"((WORD32)a), "r"((WORD32)b));
  return (sum);
}

// subtract 2 32 bit variables
static PLATFORM_INLINE WORD32 ia_core_coder_sub32(WORD32 a, WORD32 b) {
  WORD32 diff;
  __asm__(

      "                SUB   %0 ,  %1,  %2 \n\t"
      : "=r"((WORD32)diff)
      : "r"((WORD32)a), "r"((WORD32)b));
  return (diff);
}

// adds 2 32 bit variables with saturation
static PLATFORM_INLINE WORD32 ia_core_coder_add32_sat(WORD32 a, WORD32 b) {
  WORD32 sum;
  __asm__(

      "                QADD   %0 ,  %1,  %2 \n\t"
      : "=r"((WORD32)sum)
      : "r"((WORD32)a), "r"((WORD32)b));
  return (sum);
}

static PLATFORM_INLINE WORD32 ia_core_coder_add32_sat3(WORD32 a, WORD32 b,
                                                  WORD32 c) {
  WORD64 sum;

  sum = (WORD64)a + (WORD64)b;

  sum = (WORD64)sum + (WORD64)c;

  if (sum > MAX_32) {
    sum = MAX_32;
  }
  if (sum < MIN_32) {
    sum = MIN_32;
  }

  return (WORD32)sum;
}

// subtract 2 32 bit variables
static PLATFORM_INLINE WORD32 ia_core_coder_sub32_sat(WORD32 a, WORD32 b) {
  WORD32 diff;
  __asm__(

      "                QSUB   %0 ,  %1,  %2 \n\t"
      : "=r"((WORD32)diff)
      : "r"((WORD32)a), "r"((WORD32)b));
  return (diff);
}

// returns number of redundant sign bits in a 32-bit value.
// return zero for a value of zero
static PLATFORM_INLINE WORD ia_core_coder_norm32(WORD32 a) {
  WORD32 norm_val;
  __asm__(
      "		eor 	%0 , %1,   %1,asr #31 \n\t"
      "		CLZ 	%0,   %0 \n\t"
      "		SUB 	%0,   %0, #1 \n\t"
      : "=r"((WORD32)norm_val)
      : "r"((WORD32)a));
  return norm_val;
}

static PLATFORM_INLINE WORD ia_core_coder_pnorm32(WORD32 a) {
  WORD32 norm_val;
  __asm__(

      "		CLZ 	%0,   %1 \n\t"
      "		SUB 	%0,   %0, #1 \n\t"
      : "=r"((WORD32)norm_val)
      : "r"((WORD32)a));
  return norm_val;
}

// returns the position of the most significant bit for negative numbers.
// ignores leading zeros to determine the position of most significant bit.
static PLATFORM_INLINE WORD bin_expo32(WORD32 a) {
  WORD bin_expo_val;

  bin_expo_val = 31 - ia_core_coder_norm32(a);

  return bin_expo_val;
}

// returns the absolute value of 32-bit number
static PLATFORM_INLINE WORD32 ia_core_coder_abs32(WORD32 a) {
	WORD32 abs_val;
  __asm__ __volatile__("       MOVS 	%0 ,  %1  \n\t"
                       "       IT MI \n\t"
                       "       RSBMI 	%0 ,  %1 ,  #0  \n\t"
                       : "=r"((WORD32)abs_val)
                       : "r"((WORD32)a)
                       : "cc");
  return abs_val;
}

// returns the absolute value of 32-bit number
static PLATFORM_INLINE WORD32 ia_core_coder_abs32_nrm(WORD32 a) {
	 WORD32 abs_val;
  __asm__("        eor     %0 , %1,   %1,asr #31 \n\t" : "=r"((WORD32)abs_val) : "r"((WORD32)a));
  return abs_val;
}

// returns the absolute value of 32-bit number with saturation
static PLATFORM_INLINE WORD32 ia_core_coder_abs32_sat(WORD32 a) {
  WORD32 abs_val;
  __asm__ __volatile__(

      "		MOVS 	%0 ,  %1  \n\t"
      "		RSBMIS 	%0 ,  %1 ,  #0  \n\t"
      "       MOVMI 	%0 ,  #0x7fffffff  \n\t"
      : "=r"((WORD32)abs_val)
      : "r"((WORD32)a)
      : "cc");

  return abs_val;
}

// returns the negated value of 32-bit number
static PLATFORM_INLINE WORD32 ia_core_coder_negate32(WORD32 a) {
  WORD32 neg_val;
  __asm__("        RSB %0, %1, #0 \n\t"
          : "=r"((WORD32)neg_val)
          : "r"((WORD32)a));
  return neg_val;
}

// returns the negated value of 32-bit number with saturation
static PLATFORM_INLINE WORD32 ia_core_coder_negate32_sat(WORD32 a) {
  WORD32 neg_val;
  __asm__(
      "       RSBS %0, %1, #0 \n\t"
      "		MVNVS %0, #0x80000000 \n\t"
      : "=r"((WORD32)neg_val)
      : "r"((WORD32)a)
      : "cc");
  return neg_val;
}

// divides 2 32 bit variables and returns the quotient
static PLATFORM_INLINE WORD32 div32(WORD32 a, WORD32 b, WORD *q_format) {
  WORD32 quotient;
  UWORD32 mantissa_nr, mantissa_dr;
  WORD16 sign = 0;

  LOOPINDEX i;
  WORD q_nr, q_dr;

  mantissa_nr = a;
  mantissa_dr = b;
  quotient = 0;

  if ((a < 0) && (0 != b)) {
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

  if (b < 0) {
    b = -b;
    sign = (WORD16)(sign ^ -1);
  }

  if (0 == b) {
    *q_format = 0;
    return (a);
  }

  quotient = 0;

  q_nr = ia_core_coder_norm32(a);
  mantissa_nr = (UWORD32)a << (q_nr);
  q_dr = ia_core_coder_norm32(b);
  mantissa_dr = (UWORD32)b << (q_dr);
  *q_format = (WORD)(30 + q_nr - q_dr);

  for (i = 0; i < 31; i++) {
    quotient = quotient << 1;

    if (mantissa_nr >= mantissa_dr) {
      mantissa_nr = mantissa_nr - mantissa_dr;
      quotient += 1;
    }

    mantissa_nr = (UWORD32)mantissa_nr << 1;
  }

  if (sign < 0) {
    quotient = -quotient;
  }

  return quotient;
}

// multiplies two 16 bit numbers and accumulates their result in a 32 bit
// variable
static PLATFORM_INLINE WORD32 ia_core_coder_mac16x16in32(WORD32 a, WORD16 b,
                                                    WORD16 c) {
  WORD32 acc;
  __asm__(

      "		SMLABB  %0, %2, %3, %1 \n\t"
      : "=r"((WORD32)acc)
      : "r"((WORD32)a), "r"((WORD32)b), "r"((WORD32)c)

          );

  return acc;
}

static PLATFORM_INLINE WORD32 ia_core_coder_mac16x16in32_sat(WORD32 a, WORD16 b,
                                                        WORD16 c) {
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32(b, c);

  acc = ia_core_coder_add32_sat(a, acc);

  return acc;
}

// multiplies lower 16 bit of one data with upper 16 bit of
// other and accumulates their result in a 32 bit variable
static PLATFORM_INLINE WORD32 mac16x16hin32(WORD32 a, WORD32 b, WORD32 c) {
	WORD32 acc;
  __asm__(

      "		SMLATT  %0, %2, %3, %1 \n\t"
      : "=r"((WORD32)acc)
      : "r"((WORD32)a), "r"((WORD32)b), "r"((WORD32)c)

          );

  return acc;
}

// multiplies two 16 bit numbers and accumulates their result in a 32 bit
// variable
static PLATFORM_INLINE WORD32 ia_core_coder_mac16x16in32_shl(WORD32 a, WORD16 b,
                                                        WORD16 c) {
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32_shl(b, c);

  acc = ia_core_coder_add32(a, acc);

  return acc;
}

// multiplies two 16 bit numbers and accumulates their
// result in a 32 bit variable with saturation

static PLATFORM_INLINE WORD32 ia_core_coder_mac16x16in32_shl_sat(WORD32 a, WORD16 b,
                                                            WORD16 c) {
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32_shl_sat(b, c);

  acc = ia_core_coder_add32_sat(a, acc);

  return acc;
}

// multiplies two 16 bit numbers and subtracts their
// result from a 32 bit variable
static PLATFORM_INLINE WORD32 msu16x16in32(WORD32 a, WORD16 b, WORD16 c) {
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32(b, c);

  acc = ia_core_coder_sub32(a, acc);

  return acc;
}

// multiplies two 16 bit numbers and subtracts their
// result from a 32 bit variable after removing a redundant sign bit in the
// product
static PLATFORM_INLINE WORD32 msu16x16in32_shl(WORD32 a, WORD16 b, WORD16 c) {
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32_shl(b, c);

  acc = ia_core_coder_sub32(a, acc);

  return acc;
}

// multiplies two 16 bit numbers and subtracts their
// result from a 32 bit variable with saturation
// after removing a redundant sign bit in the product
static PLATFORM_INLINE WORD32 msu16x16in32_shl_sat(WORD32 a, WORD16 b,
                                                   WORD16 c) {
  WORD32 acc;

  acc = ia_core_coder_mult16x16in32_shl_sat(b, c);

  acc = ia_core_coder_sub32_sat(a, acc);

  return acc;
}

// adding two 32 bit numbers and taking care of overflow
// by downshifting both numbers before addition
static PLATFORM_INLINE WORD32 add32_shr(WORD32 a, WORD32 b) {
  WORD32 sum;

  a = ia_core_coder_shr32(a, 1);
  b = ia_core_coder_shr32(b, 1);

  sum = ia_core_coder_add32(a, b);

  return sum;
}

// subtracting two 32 bit numbers and taking care of
// overflow by downshifting both numbers before addition

static PLATFORM_INLINE WORD32 sub32_shr(WORD32 a, WORD32 b) {
  WORD32 diff;

  a = ia_core_coder_shr32(a, 1);
  b = ia_core_coder_shr32(b, 1);

  diff = ia_core_coder_sub32(a, b);

  return diff;
}
#endif
