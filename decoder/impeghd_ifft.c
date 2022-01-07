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

#define _USE_MATH_DEFINES
#include <math.h>

#include <impeghd_type_def.h>
#include "impeghd_fft_ifft_rom.h"
#include <impeghd_fft_ifft.h>
#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup FFT FFT
 * @ingroup  FFT
 * @brief FFT functions
 *
 * @{
 */

/**
*  impeghd_ifft_7
*
*  \brief 7 point IFFT
*
*  \param [in]  inp    Pointer to input data
*  \param [out] op     Pointer to output data
*
*
*
*/
static PLATFORM_INLINE VOID impeghd_ifft_7(FLOAT32 *inp, FLOAT32 *op)
{
  FLOAT32 x0r, x1r, x2r, x3r, x4r, x5r, x6r, x7r, x8r;
  FLOAT32 x0i, x1i, x2i, x3i, x4i, x5i, x6i, x7i, x8i;
  FLOAT32 y0r, y1r, y2r, y3r, y4r, y5r, y6r, y7r, y8r;
  FLOAT32 y0i, y1i, y2i, y3i, y4i, y5i, y6i, y7i, y8i;

  /*
  * Node 1 of Winograd FFT for 7 point
  *
  * 1   0   0   0   0   0   0
  * 0   1   0   0   0   0   1
  * 0   1   0   0   0   0  -1
  * 0   0   1   0   0   1   0
  * 0   0   1   0   0  -1   0
  * 0   0   0   1   1   0   0
  * 0   0   0  -1   1   0   0
  *
  */

  x0r = inp[0];
  x0i = inp[1];
  x1r = ia_add_flt(inp[2], inp[12]);
  x1i = ia_add_flt(inp[3], inp[13]);
  x2r = ia_sub_flt(inp[2], inp[12]);
  x2i = ia_sub_flt(inp[3], inp[13]);
  x3r = ia_add_flt(inp[4], inp[10]);
  x3i = ia_add_flt(inp[5], inp[11]);
  x4r = ia_sub_flt(inp[4], inp[10]);
  x4i = ia_sub_flt(inp[5], inp[11]);
  x5r = ia_add_flt(inp[8], inp[6]);
  x5i = ia_add_flt(inp[9], inp[7]);
  x6r = ia_sub_flt(inp[8], inp[6]);
  x6i = ia_sub_flt(inp[9], inp[7]);

  /*
  * Node 2 of Winograd FFT for 7 point
  *
  * 1   0   0   0   0   0   0
  * 0   1   0   1   0   1   0
  * 0   1   0  -1   0   0   0
  * 0  -1   0   0   0   1   0
  * 0   0   0   1   0  -1   0
  * 0   0   1   0   1   0   1
  * 0   0   1   0  -1   0   0
  * 0   0  -1   0   0   0   1
  * 0   0   0   0   1   0  -1
  *
  */

  y0r = x0r;
  y0i = x0i;
  y1r = ia_add_flt(ia_add_flt(x1r, x3r), x5r);
  y1i = ia_add_flt(ia_add_flt(x1i, x3i), x5i);
  y2r = ia_sub_flt(x1r, x3r);
  y2i = ia_sub_flt(x1i, x3i);
  y3r = ia_sub_flt(x5r, x1r);
  y3i = ia_sub_flt(x5i, x1i);
  y4r = ia_sub_flt(x3r, x5r);
  y4i = ia_sub_flt(x3i, x5i);
  y5r = ia_add_flt(ia_add_flt(x2r, x4r), x6r);
  y5i = ia_add_flt(ia_add_flt(x2i, x4i), x6i);
  y6r = ia_sub_flt(x2r, x4r);
  y6i = ia_sub_flt(x2i, x4i);
  y7r = ia_sub_flt(x6r, x2r);
  y7i = ia_sub_flt(x6i, x2i);
  y8r = ia_sub_flt(x4r, x6r);
  y8i = ia_sub_flt(x4i, x6i);

  /*
  * Node 3 of Winograd FFT for 7 point
  *
  * 1    1    0    0    0     0     0     0     0
  * 1  c70    0    0    0     0     0     0     0
  * 0    0  c71    0    0     0     0     0     0
  * 0    0    0  c72    0     0     0     0     0
  * 0    0    0    0  c73     0     0     0     0
  * 0    0    0    0    0  jc74     0     0     0
  * 0    0    0    0    0     0  jc75     0     0
  * 0    0    0    0    0     0     0  jc76     0
  * 0    0    0    0    0     0     0     0  jc77
  *
  */
  x0r = ia_add_flt(y0r, y1r);
  x0i = ia_add_flt(y0i, y1i);
  x1r = ia_mac_flt(y0r, C70, y1r);
  x1i = ia_mac_flt(y0i, C70, y1i);
  x2r = ia_mul_flt(C71, y2r);
  x2i = ia_mul_flt(C71, y2i);
  x3r = ia_mul_flt(C72, y3r);
  x3i = ia_mul_flt(C72, y3i);
  x4r = ia_mul_flt(C73, y4r);
  x4i = ia_mul_flt(C73, y4i);
  x5r = ia_mul_flt(C74, y5i);
  x5i = ia_mul_flt(-C74, y5r);
  x6r = ia_mul_flt(C75, y6i);
  x6i = ia_mul_flt(-C75, y6r);
  x7r = ia_mul_flt(C76, y7i);
  x7i = ia_mul_flt(-C76, y7r);
  x8r = ia_mul_flt(C77, y8i);
  x8i = ia_mul_flt(-C77, y8r);

  /*
  * Node 4 of Winograd FFT for 7 point
  *
  * 1   0   0   0   0   0   0   0   0
  * 0   1   1   0   1   0   0   0   0
  * 0   1  -1  -1   0   0   0   0   0
  * 0   1   0   1  -1   0   0   0   0
  * 0   0   0   0   0   1   1   0   1
  * 0   0   0   0   0   1  -1  -1   0
  * 0   0   0   0   0   1   0   1  -1
  *
  */

  y0r = x0r;
  y0i = x0i;
  y1r = ia_add_flt(ia_add_flt(x1r, x2r), x4r);
  y1i = ia_add_flt(ia_add_flt(x1i, x2i), x4i);
  y2r = ia_sub_flt(ia_sub_flt(x1r, x2r), x3r);
  y2i = ia_sub_flt(ia_sub_flt(x1i, x2i), x3i);
  y3r = ia_sub_flt(ia_add_flt(x1r, x3r), x4r);
  y3i = ia_sub_flt(ia_add_flt(x1i, x3i), x4i);
  y4r = ia_add_flt(ia_add_flt(x5r, x6r), x8r);
  y4i = ia_add_flt(ia_add_flt(x5i, x6i), x8i);
  y5r = ia_sub_flt(ia_sub_flt(x5r, x6r), x7r);
  y5i = ia_sub_flt(ia_sub_flt(x5i, x6i), x7i);
  y6r = ia_sub_flt(ia_add_flt(x5r, x7r), x8r);
  y6i = ia_sub_flt(ia_add_flt(x5i, x7i), x8i);

  /*
  * Node 5 of Winograd FFT for 7 point
  *
  * 1   0   0   0   0   0   0
  * 0   1   0   0   1   0   0
  * 0   0   0   1   0   0   1
  * 0   0   1   0   0  -1   0
  * 0   0   1   0   0   1   0
  * 0   0   0   1   0   0  -1
  * 0   1   0   0  -1   0   0
  *
  */
  x0r = y0r;
  x0i = y0i;
  x1r = ia_add_flt(y1r, y4r);
  x1i = ia_add_flt(y1i, y4i);
  x2r = ia_add_flt(y3r, y6r);
  x2i = ia_add_flt(y3i, y6i);
  x3r = ia_sub_flt(y2r, y5r);
  x3i = ia_sub_flt(y2i, y5i);
  x4r = ia_add_flt(y2r, y5r);
  x4i = ia_add_flt(y2i, y5i);
  x5r = ia_sub_flt(y3r, y6r);
  x5i = ia_sub_flt(y3i, y6i);
  x6r = ia_sub_flt(y1r, y4r);
  x6i = ia_sub_flt(y1i, y4i);

  op[0] = x0r;
  op[1] = x0i;
  op[2] = x1r;
  op[3] = x1i;
  op[4] = x2r;
  op[5] = x2i;
  op[6] = x3r;
  op[7] = x3i;
  op[8] = x4r;
  op[9] = x4i;
  op[10] = x5r;
  op[11] = x5i;
  op[12] = x6r;
  op[13] = x6i;

  return;
}

/**
 *  impeghd_ifft_3
 *
 *  \brief Inverse fast fourier transform
 *
 *  \param [in]    inp    Input buffer
 *  \param [out]  op    Output buffer
 *
 *
 *
 */
static PLATFORM_INLINE VOID impeghd_ifft_3(FLOAT32 *inp, FLOAT32 *op)
{
  FLOAT32 add_r, sub_r;
  FLOAT32 add_i, sub_i;
  FLOAT32 x01r, x01i, temp;

  FLOAT32 p1, p2, p3, p4;
  FLOAT32 sinmu;
  sinmu = -0.866025403784439f;

  x01r = ia_add_flt(inp[0], inp[2]);
  x01i = ia_add_flt(inp[1], inp[3]);

  add_r = ia_add_flt(inp[2], inp[4]);
  add_i = ia_add_flt(inp[3], inp[5]);

  sub_r = ia_sub_flt(inp[2], inp[4]);
  sub_i = ia_sub_flt(inp[3], inp[5]);

  p1 = add_r / (FLOAT32)2.0;
  p4 = add_i / (FLOAT32)2.0;
  p2 = ia_mul_flt(sub_i, sinmu);
  p3 = ia_mul_flt(sub_r, sinmu);

  temp = ia_sub_flt(inp[0], p1);

  op[0] = ia_add_flt(x01r, inp[4]);
  op[1] = ia_add_flt(x01i, inp[5]);
  op[2] = ia_add_flt(temp, p2);
  op[3] = ia_sub_flt(ia_sub_flt(inp[1], p3), p4);
  op[4] = ia_sub_flt(temp, p2);
  op[5] = ia_sub_flt(ia_add_flt(inp[1], p3), p4);

  return;
}

/**
 *  impeghd_rad2_cplx_ifft
 *
 *  \brief Radian based complex inverse fast fourier transform
 *
 *  \param [in,out]  ptr_real  Real buffer
 *  \param [in,out]  ptr_imag  Imaginary buffer
 *  \param [in]    n_points  Number of points
 *  \param [in]    ptr_scratch  Pointer to scratch buffer for intermediate processing
 *
 *
 *
 */
VOID impeghd_rad2_cplx_ifft(FLOAT32 *ptr_real, FLOAT32 *ptr_imag, WORD32 n_points,
                            FLOAT32 *ptr_scratch)
{
  WORD32 i, j, k, n_stages, h2;
  FLOAT32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;
  WORD32 del, nodespacing, in_loop_cnt;
  WORD32 not_power_4;
  WORD32 dig_rev_shift;
  WORD32 m_points = n_points;
  FLOAT32 *ptr_x = ptr_scratch;
  FLOAT32 *y = ptr_scratch + 2048;
  FLOAT32 *ptr_y = y;
  const FLOAT32 *ptr_w;

  dig_rev_shift = norm32(m_points) + 1 - 16;
  n_stages = 30 - norm32(m_points);
  not_power_4 = n_stages & 1;

  n_stages = n_stages >> 1;

  ptr_w = ia_fft_twiddle_table_float;

  for (i = 0; i < n_points; i++)
  {
    ptr_x[2 * i] = ptr_real[i];
    ptr_x[2 * i + 1] = ptr_imag[i];
  }

  /**********************IFFT******************************************/
  for (i = 0; i < n_points; i += 4)
  {
    FLOAT32 *inp = ptr_x;

    DIG_REV(i, dig_rev_shift, h2);
    if (not_power_4)
    {
      h2 += 1;
      h2 &= ~1;
    }
    inp += (h2);

    x0r = *inp;
    x0i = *(inp + 1);
    inp += (n_points >> 1);

    x1r = *inp;
    x1i = *(inp + 1);
    inp += (n_points >> 1);

    x2r = *inp;
    x2i = *(inp + 1);
    inp += (n_points >> 1);

    x3r = *inp;
    x3i = *(inp + 1);

    x0r = ia_add_flt(x0r, x2r);
    x0i = ia_add_flt(x0i, x2i);
    x2r = ia_msu_flt(x0r, x2r, 2);
    x2i = ia_msu_flt(x0i, x2i, 2);
    x1r = ia_add_flt(x1r, x3r);
    x1i = ia_add_flt(x1i, x3i);
    x3r = ia_msu_flt(x1r, x3r, 2);
    x3i = ia_msu_flt(x1i, x3i, 2);

    x0r = ia_add_flt(x0r, x1r);
    x0i = ia_add_flt(x0i, x1i);
    x1r = ia_msu_flt(x0r, x1r, 2);
    x1i = ia_msu_flt(x0i, x1i, 2);
    x2r = ia_sub_flt(x2r, x3i);
    x2i = ia_add_flt(x2i, x3r);
    x3i = ia_mac_flt(x2r, x3i, 2);
    x3r = ia_msu_flt(x2i, x3r, 2);

    *ptr_y++ = x0r;
    *ptr_y++ = x0i;
    *ptr_y++ = x2r;
    *ptr_y++ = x2i;
    *ptr_y++ = x1r;
    *ptr_y++ = x1i;
    *ptr_y++ = x3i;
    *ptr_y++ = x3r;
  }
  ptr_y -= 2 * n_points;
  del = 4;
  nodespacing = 64;
  in_loop_cnt = n_points >> 4;
  for (i = n_stages - 1; i > 0; i--)
  {
    const FLOAT32 *twiddles = ptr_w;
    FLOAT32 *data = ptr_y;
    FLOAT32 W1, W2, W3, W4, W5, W6;
    WORD32 sec_loop_cnt;

    for (k = in_loop_cnt; k != 0; k--)
    {
      x0r = (*data);
      x0i = (*(data + 1));
      data += (del << 1);

      x1r = (*data);
      x1i = (*(data + 1));
      data += (del << 1);

      x2r = (*data);
      x2i = (*(data + 1));
      data += (del << 1);

      x3r = (*data);
      x3i = (*(data + 1));
      data -= 3 * (del << 1);

      x0r = ia_add_flt(x0r, x2r);
      x0i = ia_add_flt(x0i, x2i);
      x2r = ia_msu_flt(x0r, x2r, 2);
      x2i = ia_msu_flt(x0i, x2i, 2);
      x1r = ia_add_flt(x1r, x3r);
      x1i = ia_add_flt(x1i, x3i);
      x3r = ia_msu_flt(x1r, x3r, 2);
      x3i = ia_msu_flt(x1i, x3i, 2);

      x0r = ia_add_flt(x0r, x1r);
      x0i = ia_add_flt(x0i, x1i);
      x1r = ia_msu_flt(x0r, x1r, 2);
      x1i = ia_msu_flt(x0i, x1i, 2);
      x2r = ia_sub_flt(x2r, x3i);
      x2i = ia_add_flt(x2i, x3r);
      x3i = ia_mac_flt(x2r, x3i, 2);
      x3r = ia_msu_flt(x2i, x3r, 2);

      *data = x0r;
      *(data + 1) = x0i;
      data += (del << 1);

      *data = x2r;
      *(data + 1) = x2i;
      data += (del << 1);

      *data = x1r;
      *(data + 1) = x1i;
      data += (del << 1);

      *data = x3i;
      *(data + 1) = x3r;
      data += (del << 1);
    }
    data = ptr_y + 2;

    sec_loop_cnt = (nodespacing * del);
    sec_loop_cnt = (sec_loop_cnt / 4) + (sec_loop_cnt / 8) - (sec_loop_cnt / 16) +
                   (sec_loop_cnt / 32) - (sec_loop_cnt / 64) + (sec_loop_cnt / 128) -
                   (sec_loop_cnt / 256);

    for (j = nodespacing; j <= sec_loop_cnt; j += nodespacing)
    {
      W1 = *(twiddles + j);
      W4 = *(twiddles + j + 257);
      W2 = *(twiddles + (j << 1));
      W5 = *(twiddles + (j << 1) + 257);
      W3 = *(twiddles + j + (j << 1));
      W6 = *(twiddles + j + (j << 1) + 257);

      // twiddles += nodespacing;
      for (k = in_loop_cnt; k != 0; k--)
      {
        FLOAT32 tmp;
        /*x0 is loaded later to avoid register crunch*/

        data += (del << 1);

        x1r = *data;
        x1i = *(data + 1);
        data += (del << 1);

        x2r = *data;
        x2i = *(data + 1);
        data += (del << 1);

        x3r = *data;
        x3i = *(data + 1);
        data -= 3 * (del << 1);

        tmp = ia_mac_flt(ia_mul_flt(x1r, W1), x1i, W4);
        x1i = ia_mac_flt(ia_negate_flt(ia_mul_flt(x1r, W4)), x1i, W1);
        x1r = tmp;

        tmp = ia_mac_flt(ia_mul_flt(x2r, W2), x2i, W5);
        x2i = ia_mac_flt(ia_negate_flt(ia_mul_flt(x2r, W5)), x2i, W2);
        x2r = tmp;

        tmp = ia_mac_flt(ia_mul_flt(x3r, W3), x3i, W6);
        x3i = ia_mac_flt(ia_negate_flt(ia_mul_flt(x3r, W6)), x3i, W3);
        x3r = tmp;

        x0r = (*data);
        x0i = (*(data + 1));

        x0r = ia_add_flt(x0r, (x2r));
        x0i = ia_add_flt(x0i, (x2i));
        x2r = ia_msu_flt(x0r, x2r, 2);
        x2i = ia_msu_flt(x0i, x2i, 2);
        x1r = ia_add_flt(x1r, x3r);
        x1i = ia_add_flt(x1i, x3i);
        x3r = ia_msu_flt(x1r, x3r, 2);
        x3i = ia_msu_flt(x1i, x3i, 2);

        x0r = ia_add_flt(x0r, (x1r));
        x0i = ia_add_flt(x0i, (x1i));
        x1r = ia_msu_flt(x0r, x1r, 2);
        x1i = ia_msu_flt(x0i, x1i, 2);
        x2r = ia_sub_flt(x2r, (x3i));
        x2i = ia_add_flt(x2i, (x3r));
        x3i = ia_mac_flt(x2r, x3i, 2);
        x3r = ia_msu_flt(x2i, x3r, 2);

        *data = x0r;
        *(data + 1) = x0i;
        data += (del << 1);

        *data = x2r;
        *(data + 1) = x2i;
        data += (del << 1);

        *data = x1r;
        *(data + 1) = x1i;
        data += (del << 1);

        *data = x3i;
        *(data + 1) = x3r;
        data += (del << 1);
      }
      data -= 2 * n_points;
      data += 2;
    }
    for (; j <= (nodespacing * del) >> 1; j += nodespacing)
    {
      W1 = *(twiddles + j);
      W4 = *(twiddles + j + 257);
      W2 = *(twiddles + (j << 1));
      W5 = *(twiddles + (j << 1) + 257);
      W3 = *(twiddles + j + (j << 1) - 256);
      W6 = *(twiddles + j + (j << 1) + 1);
      // twiddles += nodespacing;

      for (k = in_loop_cnt; k != 0; k--)
      {
        FLOAT32 tmp;
        /*x0 is loaded later to avoid register crunch*/

        data += (del << 1);

        x1r = *data;
        x1i = *(data + 1);
        data += (del << 1);

        x2r = *data;
        x2i = *(data + 1);
        data += (del << 1);

        x3r = *data;
        x3i = *(data + 1);
        data -= 3 * (del << 1);

        tmp = ia_mac_flt(ia_mul_flt(x1r, W1), x1i, W4);
        x1i = ia_mac_flt(ia_negate_flt(ia_mul_flt(x1r, W4)), x1i, W1);
        x1r = tmp;

        tmp = ia_mac_flt(ia_mul_flt(x2r, W2), x2i, W5);
        x2i = ia_mac_flt(ia_negate_flt(ia_mul_flt(x2r, W5)), x2i, W2);
        x2r = tmp;

        tmp = ia_msu_flt(ia_mul_flt(x3r, W6), x3i, W3);
        x3i = ia_mac_flt(ia_mul_flt(x3r, W3), x3i, W6);
        x3r = tmp;

        x0r = (*data);
        x0i = (*(data + 1));

        x0r = ia_add_flt(x0r, x2r);
        x0i = ia_add_flt(x0i, x2i);
        x2r = ia_msu_flt(x0r, x2r, 2);
        x2i = ia_msu_flt(x0i, x2i, 2);
        x1r = ia_add_flt(x1r, x3r);
        x1i = ia_add_flt(x1i, x3i);
        x3r = ia_msu_flt(x1r, x3r, 2);
        x3i = ia_msu_flt(x1i, x3i, 2);

        x0r = ia_add_flt(x0r, x1r);
        x0i = ia_add_flt(x0i, x1i);
        x1r = ia_msu_flt(x0r, x1r, 2);
        x1i = ia_msu_flt(x0i, x1i, 2);
        x2r = ia_sub_flt(x2r, x3i);
        x2i = ia_add_flt(x2i, (x3r));
        x3i = ia_mac_flt(x2r, x3i, 2);
        x3r = ia_msu_flt(x2i, x3r, 2);

        *data = x0r;
        *(data + 1) = x0i;
        data += (del << 1);

        *data = x2r;
        *(data + 1) = x2i;
        data += (del << 1);

        *data = x1r;
        *(data + 1) = x1i;
        data += (del << 1);

        *data = x3i;
        *(data + 1) = x3r;
        data += (del << 1);
      }
      data -= 2 * n_points;
      data += 2;
    }
    for (; j <= sec_loop_cnt * 2; j += nodespacing)
    {
      W1 = *(twiddles + j);
      W4 = *(twiddles + j + 257);
      W2 = *(twiddles + (j << 1) - 256);
      W5 = *(twiddles + (j << 1) + 1);
      W3 = *(twiddles + j + (j << 1) - 256);
      W6 = *(twiddles + j + (j << 1) + 1);

      // twiddles += nodespacing;
      for (k = in_loop_cnt; k != 0; k--)
      {
        FLOAT32 tmp;
        /*x0 is loaded later to avoid register crunch*/

        data += (del << 1);

        x1r = *data;
        x1i = *(data + 1);
        data += (del << 1);

        x2r = *data;
        x2i = *(data + 1);
        data += (del << 1);

        x3r = *data;
        x3i = *(data + 1);
        data -= 3 * (del << 1);

        tmp = ia_mac_flt(ia_mul_flt(x1r, W1), x1i, W4);
        x1i = ia_mac_flt(ia_negate_flt(ia_mul_flt(x1r, W4)), x1i, W1);
        x1r = tmp;

        tmp = ia_msu_flt(ia_mul_flt(x2r, W5), x2i, W2);
        x2i = ia_mac_flt(ia_mul_flt(x2r, W2), x2i, W5);
        x2r = tmp;

        tmp = ia_msu_flt(ia_mul_flt(x3r, W6), x3i, W3);
        x3i = ia_mac_flt(ia_mul_flt(x3r, W3), x3i, W6);
        x3r = tmp;

        x0r = (*data);
        x0i = (*(data + 1));

        x0r = ia_add_flt(x0r, x2r);
        x0i = ia_add_flt(x0i, x2i);
        x2r = ia_msu_flt(x0r, x2r, 2);
        x2i = ia_msu_flt(x0i, x2i, 2);
        x1r = ia_add_flt(x1r, x3r);
        x1i = ia_add_flt(x1i, x3i);
        x3r = ia_msu_flt(x1r, x3r, 2);
        x3i = ia_msu_flt(x1i, x3i, 2);

        x0r = ia_add_flt(x0r, x1r);
        x0i = ia_add_flt(x0i, x1i);
        x1r = ia_msu_flt(x0r, x1r, 2);
        x1i = ia_msu_flt(x0i, x1i, 2);
        x2r = ia_sub_flt(x2r, x3i);
        x2i = ia_add_flt(x2i, x3r);
        x3i = ia_mac_flt(x2r, x3i, 2);
        x3r = ia_msu_flt(x2i, x3r, 2);

        *data = x0r;
        *(data + 1) = x0i;
        data += (del << 1);

        *data = x2r;
        *(data + 1) = x2i;
        data += (del << 1);

        *data = x1r;
        *(data + 1) = x1i;
        data += (del << 1);

        *data = x3i;
        *(data + 1) = x3r;
        data += (del << 1);
      }
      data -= 2 * n_points;
      data += 2;
    }
    for (; j < nodespacing * del; j += nodespacing)
    {
      W1 = *(twiddles + j);
      W4 = *(twiddles + j + 257);
      W2 = *(twiddles + (j << 1) - 256);
      W5 = *(twiddles + (j << 1) + 1);
      W3 = *(twiddles + j + (j << 1) - 512);
      W6 = *(twiddles + j + (j << 1) - 512 + 257);

      // twiddles += nodespacing;
      for (k = in_loop_cnt; k != 0; k--)
      {
        FLOAT32 tmp;
        /*x0 is loaded later to avoid register crunch*/

        data += (del << 1);

        x1r = *data;
        x1i = *(data + 1);
        data += (del << 1);

        x2r = *data;
        x2i = *(data + 1);
        data += (del << 1);

        x3r = *data;
        x3i = *(data + 1);
        data -= 3 * (del << 1);

        tmp = ia_mac_flt(ia_mul_flt(x1r, W1), x1i, W4);
        x1i = ia_mac_flt(ia_negate_flt(ia_mul_flt(x1r, W4)), x1i, W1);
        x1r = tmp;

        tmp = ia_msu_flt(ia_mul_flt(x2r, W5), x2i, W2);
        x2i = ia_mac_flt(ia_mul_flt(x2r, W2), x2i, W5);
        x2r = tmp;

        tmp = ia_msu_flt(ia_negate_flt(ia_mul_flt(x3r, W3)), x3i, W6);
        x3i = ia_mac_flt(ia_negate_flt(ia_mul_flt(x3r, W6)), x3i, W3);
        x3r = tmp;

        x0r = (*data);
        x0i = (*(data + 1));

        x0r = ia_add_flt(x0r, x2r);
        x0i = ia_add_flt(x0i, x2i);
        x2r = ia_msu_flt(x0r, x2r, 2);
        x2i = ia_msu_flt(x0i, x2i, 2);
        x1r = ia_add_flt(x1r, x3r);
        x1i = ia_sub_flt(x1i, x3i);
        x3r = ia_msu_flt(x1r, x3r, 2);
        x3i = ia_mac_flt(x1i, x3i, 2);

        x0r = ia_add_flt(x0r, x1r);
        x0i = ia_add_flt(x0i, x1i);
        x1r = ia_msu_flt(x0r, x1r, 2);
        x1i = ia_msu_flt(x0i, x1i, 2);
        x2r = ia_sub_flt(x2r, x3i);
        x2i = ia_add_flt(x2i, x3r);
        x3i = ia_mac_flt(x2r, x3i, 2);
        x3r = ia_msu_flt(x2i, x3r, 2);

        *data = x0r;
        *(data + 1) = x0i;
        data += (del << 1);

        *data = x2r;
        *(data + 1) = x2i;
        data += (del << 1);

        *data = x1r;
        *(data + 1) = x1i;
        data += (del << 1);

        *data = x3i;
        *(data + 1) = x3r;
        data += (del << 1);
      }
      data -= 2 * n_points;
      data += 2;
    }
    nodespacing >>= 2;
    del <<= 2;
    in_loop_cnt >>= 2;
  }

  if (not_power_4)
  {
    const FLOAT32 *twiddles = ptr_w;
    nodespacing <<= 1;

    for (j = del / 2; j != 0; j--)
    {
      FLOAT32 W1 = *twiddles;
      FLOAT32 W4 = *(twiddles + 257);
      FLOAT32 tmp;
      twiddles += nodespacing;

      x0r = *ptr_y;
      x0i = *(ptr_y + 1);
      ptr_y += (del << 1);

      x1r = *ptr_y;
      x1i = *(ptr_y + 1);

      tmp = ia_mac_flt(ia_mul_flt(x1r, W1), x1i, W4);
      x1i = ia_mac_flt(ia_negate_flt(ia_mul_flt(x1r, W4)), x1i, W1);
      x1r = tmp;

      *ptr_y = ia_sub_flt((x0r), (x1r));
      *(ptr_y + 1) = ia_sub_flt((x0i), (x1i));
      ptr_y -= (del << 1);

      *ptr_y = ia_add_flt((x0r), (x1r));
      *(ptr_y + 1) = ia_add_flt((x0i), (x1i));
      ptr_y += 2;
    }
    twiddles = ptr_w;
    for (j = del / 2; j != 0; j--)
    {
      FLOAT32 W1 = *twiddles;
      FLOAT32 W4 = *(twiddles + 257);
      FLOAT32 tmp;
      twiddles += nodespacing;

      x0r = *ptr_y;
      x0i = *(ptr_y + 1);
      ptr_y += (del << 1);

      x1r = *ptr_y;
      x1i = *(ptr_y + 1);

      tmp = ia_msu_flt(ia_mul_flt(x1r, W4), x1i, W1);
      x1i = ia_mac_flt(ia_mul_flt(x1r, W1), x1i, W4);
      x1r = tmp;

      *ptr_y = ia_sub_flt((x0r), (x1r));
      *(ptr_y + 1) = ia_sub_flt((x0i), (x1i));
      ptr_y -= (del << 1);

      *ptr_y = ia_add_flt((x0r), (x1r));
      *(ptr_y + 1) = ia_add_flt((x0i), (x1i));
      ptr_y += 2;
    }
  }
  for (i = 0; i < n_points; i++)
  {
    ptr_real[i] = y[2 * i];
    ptr_imag[i] = y[2 * i + 1];
  }
}

/**
 *  impeghd_mix_rad_ifft_3nx3
 *
 *  \brief Mix radian inverse fast fourier transform
 *
 *  \param [in,out]  ptr_real  Real buffer
 *  \param [in,out]  ptr_imag  Imaginary buffer
 *  \param [in]    n_points  Number of points
 *  \param [in]    ptr_scratch  Pointer to scratch buffer for intermediate processing
 *
 *
 *
 */
VOID impeghd_mix_rad_ifft_3nx3(FLOAT32 *ptr_real, FLOAT32 *ptr_imag, WORD32 n_points,
                               FLOAT32 *ptr_scratch)
{
  WORD32 i, j;
  WORD32 cnfac;
  WORD32 m_points = n_points;
  FLOAT32 *x, *y, *real_temp, *imag_temp;
  FLOAT32 *ptr_x, *ptr_y;
  ptr_x = x = ptr_scratch;
  ptr_scratch += 2 * n_points;
  ptr_y = y = ptr_scratch;
  ptr_scratch += 2 * n_points;
  real_temp = ptr_scratch;
  ptr_scratch += 2 * n_points;
  imag_temp = ptr_scratch;
  ptr_scratch += 2 * n_points;

  cnfac = 0;
  while (m_points % 3 == 0)
  {
    m_points /= 3;
    cnfac++;
  }

  for (i = 0; i < 3 * cnfac; i++)
  {
    for (j = 0; j < m_points; j++)
    {
      real_temp[j] = ptr_real[3 * j + i];
      imag_temp[j] = ptr_imag[3 * j + i];
    }

    impeghd_rad2_cplx_ifft(real_temp, imag_temp, m_points, ptr_scratch);

    for (j = 0; j < m_points; j++)
    {
      ptr_real[3 * j + i] = real_temp[j];
      ptr_imag[3 * j + i] = imag_temp[j];
    }
  }

  {
    FLOAT32 *w1r, *w1i;
    FLOAT32 tmp;
    w1r = (FLOAT32 *)ia_rad_3_fft_twiddle_re;
    w1i = (FLOAT32 *)ia_rad_3_fft_twiddle_im;

    for (i = 0; i < n_points; i += 3)
    {
      tmp = ia_mac_flt(ia_mul_flt(ptr_real[i], (*w1r)), ptr_imag[i], (*w1i));
      ptr_imag[i] = ia_mac_flt(ia_mul_flt(-ptr_real[i], (*w1i)), ptr_imag[i], (*w1r));
      ptr_real[i] = tmp;

      w1r++;
      w1i++;

      tmp = ia_mac_flt(ia_mul_flt(ptr_real[i + 1], (*w1r)), ptr_imag[i + 1], (*w1i));
      ptr_imag[i + 1] = ia_mac_flt(ia_mul_flt(-ptr_real[i + 1], (*w1i)), ptr_imag[i + 1], (*w1r));
      ptr_real[i + 1] = tmp;

      w1r++;
      w1i++;

      tmp = ia_mac_flt(ia_mul_flt(ptr_real[i + 2], (*w1r)), ptr_imag[i + 2], (*w1i));
      ptr_imag[i + 2] =
          ia_mac_flt(ia_mul_flt(ia_negate_flt(ptr_real[i + 2]), (*w1i)), ptr_imag[i + 2], (*w1r));
      ptr_real[i + 2] = tmp;

      w1r += 3 * (128 / m_points - 1) + 1;
      w1i += 3 * (128 / m_points - 1) + 1;
    }
  }

  for (i = 0; i < n_points; i++)
  {
    ptr_x[2 * i] = ptr_real[i];
    ptr_x[2 * i + 1] = ptr_imag[i];
  }

  for (i = 0; i < m_points; i++)
  {
    impeghd_ifft_3(ptr_x, ptr_y);

    ptr_x = ptr_x + 6;
    ptr_y = ptr_y + 6;
  }

  for (i = 0; i < m_points; i++)
  {
    ptr_real[i] = y[6 * i];
    ptr_imag[i] = y[6 * i + 1];
  }

  for (i = 0; i < m_points; i++)
  {
    ptr_real[m_points + i] = y[6 * i + 2];
    ptr_imag[m_points + i] = y[6 * i + 3];
  }

  for (i = 0; i < m_points; i++)
  {
    ptr_real[2 * m_points + i] = y[6 * i + 4];
    ptr_imag[2 * m_points + i] = y[6 * i + 5];
  }
}
/**
*  impeghd_mix_rad_ifft_tw_mult
*
*  \brief Mixed radix IFFT and Twiddle multiply
*
*  \param [in]  inp    Input buffer
*  \param [out] op    Output buffer
*  \param [in]  dim1  Dimension 1
*  \param [in]  dim2  Dimension 2
*  \param [in]  tw    Twiddle array
*
*
*
*/
static VOID impeghd_mix_rad_ifft_tw_mult(FLOAT32 *inp, FLOAT32 *op, WORD32 dim1, WORD32 dim2,
                                         const FLOAT32 *tw)
{
  FLOAT32 accu1, accu2;
  WORD32 i, j;
  WORD32 step_val = (dim2 - 1) << 1;
  for (i = 0; i < (dim2); i++)
  {
    op[0] = inp[0];
    op[1] = inp[1];
    op += 2;
    inp += 2;
  }

  for (j = 0; j < (dim1 - 1); j++)
  {
    op[0] = inp[0];
    op[1] = inp[1];
    inp += 2;
    op += 2;
    for (i = 0; i < (dim2 - 1); i++)
    {
      CPLX_MPY_IFFT(accu1, accu2, inp[2 * i + 0], inp[2 * i + 1], tw[2 * i + 1], tw[2 * i]);
      op[2 * i + 0] = accu1;
      op[2 * i + 1] = accu2;
    }
    inp += step_val;
    op += step_val;
    tw += (dim2 - 1) * 2;
  }
}
/**
 *  impeghd_mix_rad_ifft_3nx7
 *
 *  \brief Mix radian inverse fast fourier transform of 3nx7
 *
 *  \param [in,out]  inp        Input and Output buffer inplace
 *  \param [in]    len        Length
 *  \param [in]    ptr_scratch    Pointer to scratch buffer for intermediate
 * processing
 *
 *
 *
 */
VOID impeghd_mix_rad_ifft_3nx7(FLOAT32 *inp, WORD32 len, FLOAT32 *ptr_scratch)
{
  WORD32 i, j;
  WORD32 m_points = len / 7;
  FLOAT32 *ptr_real, *ptr_imag, *p_real_1, *p_imag_1, *p_scratch;
  ptr_real = ptr_scratch;
  ptr_scratch += SLPD_MAX_FFT_SIZE;
  ptr_imag = ptr_scratch;
  ptr_scratch += SLPD_MAX_FFT_SIZE;
  p_scratch = p_real_1 = ptr_scratch;
  ptr_scratch += SLPD_MAX_FFT_SIZE;
  p_imag_1 = ptr_scratch;
  ptr_scratch += SLPD_MAX_FFT_SIZE;

  for (i = 0; i < len; i++)
  {
    ptr_real[i] = inp[2 * i];
    ptr_imag[i] = inp[2 * i + 1];
  }

  for (i = 0; i < m_points; i++)
  {
    for (j = 0; j < 7; j++)
    {
      p_real_1[2 * j + 0] = inp[m_points * 2 * j + 2 * i];
      p_real_1[2 * j + 1] = inp[m_points * 2 * j + 2 * i + 1];
    }

    impeghd_ifft_7(p_real_1, ptr_scratch);

    for (j = 0; j < 7; j++)
    {
      inp[m_points * 2 * j + 2 * i + 0] = ptr_scratch[2 * j + 0];
      inp[m_points * 2 * j + 2 * i + 1] = ptr_scratch[2 * j + 1];
    }
  }

  if (m_points == 48)
    impeghd_mix_rad_ifft_tw_mult(inp, p_scratch, 7, m_points, ia_fft_mix_rad_twid_tbl_336);
  else
    impeghd_mix_rad_ifft_tw_mult(inp, p_scratch, 7, m_points, ia_fft_mix_rad_twid_tbl_168);

  for (i = 0; i < len; i++)
  {
    ptr_real[i] = p_scratch[2 * i];
    ptr_imag[i] = p_scratch[2 * i + 1];
  }

  for (i = 0; i < 7; i++)
  {
    impeghd_mix_rad_ifft_3nx3(ptr_real, ptr_imag, m_points, ptr_scratch);
    ptr_real += (m_points);
    ptr_imag += (m_points);
  }
  ptr_real -= 7 * m_points;
  ptr_imag -= 7 * m_points;
  for (j = 0; j < 7; j++)
  {
    for (i = 0; i < m_points; i++)
    {
      p_real_1[7 * i + j] = ptr_real[m_points * j + i];
      p_imag_1[7 * i + j] = ptr_imag[m_points * j + i];
    }
  }

  for (i = 0; i < len; i++)
  {
    inp[2 * i] = p_real_1[i];
    inp[2 * i + 1] = p_imag_1[i];
  }

  return;
}

/**
 *  impeghd_cplx_ifft_8
 *
 *  \brief 8-Point complex IFFT
 *
 *  \param [in,out] x_r Pointer to real part data buffer
 *  \param [in,out] x_i Pointer to imaginary part data buffer
 *
 *
 *
 */
VOID impeghd_cplx_ifft_8(FLOAT32 *x_r, FLOAT32 *x_i)
{

  FLOAT32 x_0, x_1, x_2, x_3;
  FLOAT32 x_4, x_5, x_6, x_7;
  FLOAT32 n00, n10, n20, n30, n01, n11, n21, n31;
  FLOAT32 xh0_0, xh1_0, xl0_0, xl1_0;
  FLOAT32 xh0_1, xh1_1, xl0_1, xl1_1;
  FLOAT32 sin_pi_4 = 0.7071067811f;

  // 1. 4 Point FFT
  x_0 = x_r[0];
  x_1 = x_i[0];
  x_2 = x_r[2];
  x_3 = x_i[2];
  x_4 = x_r[4];
  x_5 = x_i[4];
  x_6 = x_r[6];
  x_7 = x_i[6];

  xh0_0 = ia_add_flt(x_0, x_4);
  xh1_0 = ia_add_flt(x_1, x_5);
  xl0_0 = ia_sub_flt(x_0, x_4);
  xl1_0 = ia_sub_flt(x_1, x_5);
  xh0_1 = ia_add_flt(x_2, x_6);
  xh1_1 = ia_add_flt(x_3, x_7);
  xl0_1 = ia_sub_flt(x_2, x_6);
  xl1_1 = ia_sub_flt(x_3, x_7);

  n00 = ia_add_flt(xh0_0, xh0_1);
  n01 = ia_add_flt(xh1_0, xh1_1);
  n10 = ia_sub_flt(xl0_0, xl1_1);
  n11 = ia_add_flt(xl1_0, xl0_1);
  n20 = ia_sub_flt(xh0_0, xh0_1);
  n21 = ia_sub_flt(xh1_0, xh1_1);
  n30 = ia_add_flt(xl0_0, xl1_1);
  n31 = ia_sub_flt(xl1_0, xl0_1);

  // 2. 4 Point FFT
  x_0 = x_r[1];
  x_1 = x_i[1];
  x_2 = x_r[3];
  x_3 = x_i[3];
  x_4 = x_r[5];
  x_5 = x_i[5];
  x_6 = x_r[7];
  x_7 = x_i[7];

  xh0_0 = ia_add_flt(x_0, x_4);
  xh1_0 = ia_add_flt(x_1, x_5);
  xl0_0 = ia_sub_flt(x_0, x_4);
  xl1_0 = ia_sub_flt(x_1, x_5);
  xh0_1 = ia_add_flt(x_2, x_6);
  xh1_1 = ia_add_flt(x_3, x_7);
  xl0_1 = ia_sub_flt(x_2, x_6);
  xl1_1 = ia_sub_flt(x_3, x_7);

  x_0 = ia_add_flt(xh0_0, xh0_1);
  x_1 = ia_add_flt(xh1_0, xh1_1);
  x_2 = ia_sub_flt(xl0_0, xl1_1);
  x_3 = ia_add_flt(xl1_0, xl0_1);
  x_4 = ia_sub_flt(xh0_0, xh0_1);
  x_5 = ia_sub_flt(xh1_0, xh1_1);
  x_6 = ia_add_flt(xl0_0, xl1_1);
  x_7 = ia_sub_flt(xl1_0, xl0_1);

  xh0_0 = ia_mul_flt(ia_sub_flt(x_2, x_3), sin_pi_4);
  xh0_1 = ia_mul_flt(ia_add_flt(x_3, x_2), sin_pi_4);
  x_2 = xh0_0;
  x_3 = xh0_1;
  xh0_0 = ia_mul_flt(ia_add_flt(x_6, x_7), -sin_pi_4);
  xh0_1 = ia_mul_flt(ia_sub_flt(x_6, x_7), sin_pi_4);
  x_6 = xh0_0;
  x_7 = xh0_1;

  x_r[0] = ia_add_flt(n00, x_0);
  x_i[0] = ia_add_flt(n01, x_1);
  x_r[1] = ia_add_flt(n10, x_2);
  x_i[1] = ia_add_flt(n11, x_3);
  x_r[2] = ia_sub_flt(n21, x_4);
  x_i[2] = ia_add_flt(n20, x_5);
  x_r[3] = ia_add_flt(n30, x_6);
  x_i[3] = ia_add_flt(n31, x_7);
  x_r[4] = ia_sub_flt(n00, x_0);
  x_i[4] = ia_sub_flt(n01, x_1);
  x_r[5] = ia_sub_flt(n10, x_2);
  x_i[5] = ia_sub_flt(n11, x_3);
  x_r[6] = ia_add_flt(n21, x_4);
  x_i[6] = ia_sub_flt(n20, x_5);
  x_r[7] = ia_sub_flt(n30, x_6);
  x_i[7] = ia_sub_flt(n31, x_7);

  return;
}

/**
 *  impeghd_cplx_ifft_4
 *
 *  \brief 4-Point complex IFFT
 *
 *  \param [in,out] x_r Pointer to real part data buffer
 *  \param [in,out] x_i Pointer to imaginary part data buffer
 *
 *
 *
 */
VOID impeghd_cplx_ifft_4(FLOAT32 *x_r, FLOAT32 *x_i)
{

  FLOAT32 x_0, x_1, x_2, x_3;
  FLOAT32 x_4, x_5, x_6, x_7;
  FLOAT32 xh0_0, xh1_0, xl0_0, xl1_0;
  FLOAT32 xh0_1, xh1_1, xl0_1, xl1_1;

  // 4 Point FFT
  x_0 = x_r[0];
  x_1 = x_i[0];
  x_2 = x_r[2];
  x_3 = x_i[2];
  x_4 = x_r[4];
  x_5 = x_i[4];
  x_6 = x_r[6];
  x_7 = x_i[6];

  xh0_0 = ia_add_flt(x_0, x_4);
  xh1_0 = ia_add_flt(x_1, x_5);
  xl0_0 = ia_sub_flt(x_0, x_4);
  xl1_0 = ia_sub_flt(x_1, x_5);
  xh0_1 = ia_add_flt(x_2, x_6);
  xh1_1 = ia_add_flt(x_3, x_7);
  xl0_1 = ia_sub_flt(x_2, x_6);
  xl1_1 = ia_sub_flt(x_3, x_7);

  x_r[0] = ia_add_flt(xh0_0, xh0_1);
  x_i[0] = ia_add_flt(xh1_0, xh1_1);
  x_r[1] = ia_sub_flt(xl0_0, xl1_1);
  x_i[1] = ia_add_flt(xl1_0, xl0_1);
  x_r[2] = ia_sub_flt(xh0_0, xh0_1);
  x_i[2] = ia_sub_flt(xh1_0, xh1_1);
  x_r[3] = ia_add_flt(xl0_0, xl1_1);
  x_i[3] = ia_sub_flt(xl1_0, xl0_1);

  return;
}

/**
 *  impeghd_cplx_ifft_8k
 *
 *  \brief Generic radix-2 complex IFFT for sizes greater than 1024 upto 8192
 *
 *  \param [in,out] ptr_in_buf_real Pointer to real part data buffer
 *  \param [in,out] ptr_in_buf_imag Pointer to imaginary part data buffer
 *  \param [in]     ptr_scratch_buf Pointer to scratch buffer
 *  \param [in]     fft_len         IFFT length.
 *
 *
 *
 */
VOID impeghd_cplx_ifft_8k(FLOAT32 *ptr_in_buf_real, FLOAT32 *ptr_in_buf_imag,
                          FLOAT32 *ptr_scratch_buf, WORD32 fft_len)
{
  /*
   * This algorithm works well for all IFFTs of size greater than 1024 and
   * fft length should be a power of 2
   */
  FLOAT32 *ptr_data_r;
  FLOAT32 *ptr_data_i;
  FLOAT32 *ptr_fft_interim_buf = &ptr_scratch_buf[2 * fft_len];
  WORD32 i, j;
  if (fft_len > 1024)
  {
    WORD32 dim2 = fft_len >> 10;
    WORD32 dim1 = fft_len / dim2;
    WORD32 fac = 1;
    switch (dim2)
    {
    case 8:
      fac = 2;
      break;
    case 4:
      fac = 4;
      break;
    case 2:
      fac = 8;
      break;
    case 1:
      fac = 16;
      break;
    }

    for (i = 0; i < dim2; i++)
    {
      ptr_data_r = &ptr_scratch_buf[(2 * i + 0) * dim1];
      ptr_data_i = &ptr_scratch_buf[(2 * i + 1) * dim1];
      for (j = 0; j < dim1; j++)
      {
        ptr_data_r[j] = ptr_in_buf_real[dim2 * j + i];
        ptr_data_i[j] = ptr_in_buf_imag[dim2 * j + i];
      }
      impeghd_rad2_cplx_ifft(ptr_data_r, ptr_data_i, dim1, ptr_fft_interim_buf);
    }
    ptr_data_r = &ptr_scratch_buf[0];
    ptr_data_i = &ptr_scratch_buf[0];
    for (i = 0; i < dim1; i++)
    {
      FLOAT32 *ptr_cos_val = (FLOAT32 *)&ia_mixed_rad_twiddle_cos[i * dim2 * fac];
      FLOAT32 *ptr_sin_val = (FLOAT32 *)&ia_mixed_rad_twiddle_sin[i * dim2 * fac];
      for (j = 0; j < dim2; j++)
      {
        FLOAT32 real = ptr_data_r[(2 * j + 0) * dim1 + i];
        FLOAT32 imag = ptr_data_i[(2 * j + 1) * dim1 + i];
        FLOAT32 cos_val = ptr_cos_val[j * fac];
        FLOAT32 sin_val = ptr_sin_val[j * fac];
        FLOAT32 temp_real = (FLOAT32)(real * cos_val - imag * sin_val);
        FLOAT32 temp_imag = (FLOAT32)(real * sin_val + imag * cos_val);
        ptr_fft_interim_buf[(2 * i + 0) * dim2 + j] = temp_real;
        ptr_fft_interim_buf[(2 * i + 1) * dim2 + j] = temp_imag;
      }
    }
    if (fft_len == 8192)
    {
      for (i = 0; i < dim1; i++)
      {
        ptr_data_r = &ptr_fft_interim_buf[(2 * i + 0) * dim2];
        ptr_data_i = &ptr_fft_interim_buf[(2 * i + 1) * dim2];
        impeghd_cplx_ifft_8(ptr_data_r, ptr_data_i);
      }
    }
    else if (fft_len == 4096)
    {
      for (i = 0; i < dim1; i++)
      {
        ptr_data_r = &ptr_fft_interim_buf[(2 * i + 0) * dim2];
        ptr_data_i = &ptr_fft_interim_buf[(2 * i + 1) * dim2];
        impeghd_cplx_ifft_4(ptr_data_r, ptr_data_i);
      }
    }
    else if (fft_len == 2048)
    {
      for (i = 0; i < dim1; i++)
      {
        FLOAT32 x0_r, x0_i, x1_r, x1_i;
        ptr_data_r = &ptr_fft_interim_buf[(2 * i + 0) * dim2];
        ptr_data_i = &ptr_fft_interim_buf[(2 * i + 1) * dim2];
        x0_r = ptr_data_r[0] + ptr_data_r[1];
        x1_r = ptr_data_r[0] - ptr_data_r[1];
        x0_i = ptr_data_i[0] + ptr_data_i[1];
        x1_i = ptr_data_i[0] - ptr_data_i[1];
        ptr_data_r[0] = x0_r;
        ptr_data_r[1] = x1_r;
        ptr_data_i[0] = x0_i;
        ptr_data_i[1] = x1_i;
      }
    }
    ptr_data_r = &ptr_fft_interim_buf[0];
    ptr_data_i = &ptr_fft_interim_buf[0];
    for (i = 0; i < dim1; i++)
    {
      for (j = 0; j < dim2; j++)
      {
        ptr_in_buf_real[j * dim1 + i] = ptr_data_r[(2 * i + 0) * dim2 + j];
        ptr_in_buf_imag[j * dim1 + i] = ptr_data_i[(2 * i + 1) * dim2 + j];
      }
    }
  }
  else
  {
    impeghd_rad2_cplx_ifft(ptr_in_buf_real, ptr_in_buf_imag, fft_len, ptr_scratch_buf);
  }
}
/** @} */ /* End of FFT */