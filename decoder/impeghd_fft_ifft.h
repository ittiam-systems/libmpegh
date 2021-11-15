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

#ifndef IMPEGHD_FFT_IFFT_H
#define IMPEGHD_FFT_IFFT_H

#define PLATFORM_INLINE __inline

// u = -1 * ((2 * M_PI) / 7);
#define C70 (-0.1666667014f) //(cos(u) + cos(2 * u) + cos(3 * u)) / 3;
#define C71 (0.7901564837f)  //(2 * cos(u) - cos(2 * u) - cos(3 * u)) / 3;
#define C72 (0.0558542535f)  //(cos(u) - 2 * cos(2 * u) + cos(3 * u)) / 3;
#define C73 (0.7343022227f)  //(cos(u) + cos(2 * u) - 2 * cos(3 * u)) / 3;
#define C74 (-0.4409585893f) //(sin(u) + sin(2 * u) - sin(3 * u)) / 3;
#define C75 (-0.3408728838f) //(2 * sin(u) - sin(2 * u) + sin(3 * u)) / 3;
#define C76 (0.5339693427f)  //(sin(u) - 2 * sin(2 * u) - sin(3 * u)) / 3;
#define C77 (-0.8748422265f) //(sin(u) + sin(2 * u) + 2 * sin(3 * u)) / 3;

#define DIG_REV(i, m, j)                                                                         \
  do                                                                                             \
  {                                                                                              \
    unsigned _ = (i);                                                                            \
    _ = ((_ & 0x33333333) << 2) | ((_ & ~0x33333333) >> 2);                                      \
    _ = ((_ & 0x0F0F0F0F) << 4) | ((_ & ~0x0F0F0F0F) >> 4);                                      \
    _ = ((_ & 0x00FF00FF) << 8) | ((_ & ~0x00FF00FF) >> 8);                                      \
    (j) = _ >> (m);                                                                              \
  } while (0)

#define SLPD_MAX_FFT_SIZE 336

#define CPLX_MPY_FFT(re, im, a, b, c, d)                                                         \
  do                                                                                             \
  {                                                                                              \
    re = ((a * c) - (b * d));                                                                    \
    im = ((a * d) + (b * c));                                                                    \
  } while (0)

#define CPLX_MPY_IFFT(re, im, a, b, c, d)                                                        \
  do                                                                                             \
  {                                                                                              \
    re = ((a * c) + (b * d));                                                                    \
    im = (-(a * d) + (b * c));                                                                   \
  } while (0)

static PLATFORM_INLINE char norm32(WORD32 a)
{
  char norm_val;

  if (a == 0)
  {
    norm_val = 31;
  }
  else
  {
    if (a == (int)0xffffffffL)
    {
      norm_val = 31;
    }
    else
    {
      if (a < 0)
      {
        a = ~a;
      }
      for (norm_val = 0; a < (int)0x40000000L; norm_val++)
      {
        a <<= 1;
      }
    }
  }

  return norm_val;
}

static PLATFORM_INLINE FLOAT32 mult32X32float(FLOAT32 a, FLOAT32 b)
{
  FLOAT32 result;

  result = a * b;

  return result;
}

static PLATFORM_INLINE FLOAT32 mac32X32float(FLOAT32 a, FLOAT32 b, FLOAT32 c)
{
  FLOAT32 result;

  result = a + b * c;

  return result;
}

IA_ERRORCODE ia_core_coder_real_fft(FLOAT32 *fft_data, const FLOAT32 *sin_table, WORD32 len,
                                    WORD32 isign, FLOAT32 *ptr_scratch);

VOID impeghd_rad2_cplx_fft(FLOAT32 *Re, FLOAT32 *Im, WORD32 nPass, FLOAT32 *ptr_scratch);
VOID impeghd_rad2_cplx_ifft(FLOAT32 *Re, FLOAT32 *Im, WORD32 nPass, FLOAT32 *ptr_scratch);

VOID impeghd_mix_rad_ifft_3nx7(FLOAT32 *inp, WORD32 len, FLOAT32 *ptr_scratch);

#endif /*IMPEGHD_FFT_IFFT_H*/
