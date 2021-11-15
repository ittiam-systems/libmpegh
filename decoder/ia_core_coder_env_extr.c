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

#include <math.h>
#include <string.h>

#include <impeghd_type_def.h>

#include "ia_core_coder_constants.h"
#include <ia_core_coder_basic_op.h>
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_env_extr.h"
#include "ia_core_coder_intrinsics.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_cnt_leading_zeroes
 *
 *  \brief Utility function to obtain leading zeroes information
 *
 *  \param [in] a Input operand
 *
 *  \return WORD32 Leading zeros value
 *
 */
static WORD32 ia_core_coder_cnt_leading_zeroes(WORD32 inp)
{
  WORD32 num_zereos = 0;

  while (inp)
  {
    if (inp & VAL_2_POW_30)
      num_zereos++;
    else
      break;
    inp = inp << 1;
  }
  return num_zereos;
}

/**
 *  ia_core_coder_huffman_decode
 *
 *  \brief Huffman decoding function
 *
 *  \param [in]  next_word   Next word in bit buffer sturcture.
 *  \param [out] h_index     Pointer to huffman index.
 *  \param [out] len         Pointer to huffman code word length.
 *  \param [in]  input_table Pointer to ROM tables needed for decoding
 *  \param [in]  idx_table   Pointer to index ROM table
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_huffman_decode(WORD32 next_word, WORD16 *h_index, WORD16 *len,
                                  const UWORD16 *input_table, const UWORD32 *idx_table)
{

  WORD32 drc_offset = 0, offset_found = 0;
  WORD32 len_end, lead_zeros, max_len, length;

  UWORD32 mask = 0x80000000, code_word, loc = 0, loc_1 = 0;

  len_end = max_len = input_table[0];
  mask = mask - (1 << (31 - max_len));
  mask = mask << 1;
  loc = (UWORD32)(next_word & mask);
  lead_zeros = ia_core_coder_cnt_leading_zeroes(loc);

  do
  {
    drc_offset = (idx_table[lead_zeros] >> 20) & 0xff;
    length = input_table[drc_offset + 1] & 0x1f;
    code_word = idx_table[lead_zeros] & 0xfffff;
    loc_1 = loc >> (32 - length);
    if (loc_1 > code_word)
    {
      len_end = len_end + ((idx_table[lead_zeros] >> 28) & 0xf);
      lead_zeros = len_end;
    }
    else
    {
      drc_offset = drc_offset - (code_word - loc_1);
      offset_found = 1;
    }
  } while (!offset_found);
  *h_index = input_table[drc_offset + 1] >> 5;
  *len = (WORD16)length;
}
/** @} */ /* End of CoreDecProc */