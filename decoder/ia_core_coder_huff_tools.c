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

#include <impeghd_type_def.h>
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_hufftab
 *
 *  \brief Initialize huffman code book and sort codewords by length
 *
 *  \param [in/out]    ptr_huff_code_book      Huffman code book
 * structure
 *  \param [in]      ptr_huff_code_word      Huffman code word
 * structure
 *  \param [in]      code_book_tbl        code book table pointer
 *  \param [in]      index            code book index
 *  \param [in]      dim              dimension
 *  \param [in]      lav              Largest
 * Absolute
 * Value
 *  \param [in]      lav_incr_esc        LAV escape
 *  \param [in]      sign_code_book        specify code books as signed
 * or
 * unsigned
 *  \param [in]      max_code_word_len      maximum code word length
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_hufftab(ia_huff_code_book_struct *ptr_huff_code_book,
                           const ia_huff_code_word_struct *ptr_huff_code_word,
                           const WORD16 *code_book_tbl, const WORD32 *index, WORD32 dim,
                           WORD32 lav, WORD32 lav_incr_esc, WORD32 sign_code_book,
                           UWORD8 max_code_word_len)
{
  WORD32 i, num;

  if (sign_code_book)
  {
    ptr_huff_code_book->huff_mode = (lav << 1) + 1;
    ptr_huff_code_book->off = lav;
  }
  else
  {
    ptr_huff_code_book->huff_mode = lav + 1;
    ptr_huff_code_book->off = 0;
  }
  num = 1;
  for (i = 0; i < dim; i++)
    num = num * ptr_huff_code_book->huff_mode;

  ptr_huff_code_book->code_book_tbl = code_book_tbl;
  ptr_huff_code_book->dim = dim;
  ptr_huff_code_book->idx_tbl = index;
  ptr_huff_code_book->lav = lav;
  ptr_huff_code_book->lav_incr_esc = lav_incr_esc;
  ptr_huff_code_book->max_code_word_len = max_code_word_len;
  ptr_huff_code_book->num = num;
  ptr_huff_code_book->pstr_huff_code_word = ptr_huff_code_word;
  ptr_huff_code_book->sign_code_book = sign_code_book;
}

/**
 *  ia_core_coder_huff_codeword
 *
 *  \brief Read Huffman codeword from bitstream
 *
 *  \param [in/out]    ptr_huff_code_word  Huffman code word structure
 *  \param [in]      it_bit_buff      bit stream buffer
 *
 *  \return WORD32
 *
 */
WORD32
ia_core_coder_huff_codeword(
    const ia_huff_code_word_struct *ptr_huff_code_word, /*UWORD16 data_present,*/
    ia_bit_buf_struct *it_bit_buff)

{
  WORD32 cw_len, cw_len_tmp;
  UWORD32 code_word = 0;

  cw_len = ptr_huff_code_word->len;
  code_word = ia_core_coder_read_bits_buf(it_bit_buff, cw_len);

  while (code_word != ptr_huff_code_word->code_word)
  {
    ptr_huff_code_word++;
    cw_len_tmp = ptr_huff_code_word->len - cw_len;
    if (cw_len_tmp < 0)
    {
      break;
    }

    cw_len += cw_len_tmp;
    code_word <<= cw_len_tmp;

    code_word |= (UWORD32)ia_core_coder_read_bits_buf(it_bit_buff, cw_len_tmp);
  }
  return (ptr_huff_code_word->index);
}
/** @} */ /* End of CoreDecProc */