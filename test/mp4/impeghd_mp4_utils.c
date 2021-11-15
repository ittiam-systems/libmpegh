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

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <impeghd_type_def.h>
#include "impeghd_mp4_utils.h"
#include "impeghd_mp4_defs.h"

/**
 * @defgroup MP4Parser MP4 Parser Utility
 * @ingroup  MP4Parser
 * @brief MP4 Parser Utility
 *
 * @{
 */

/**
*  impeghd_mp4_error_hdl
*
*  \brief Function for printing to both log file and console.
*         will exit if given an exit code
*
*  \param dfp
*  \param format
*  \param ...
*
*  \return VOID
*
*/
VOID impeghd_mp4_error_hdl(FILE *dfp, pWORD8 format, ...)
{
  char str[IT_STR_LEN];
  va_list ap;
  va_start(ap, format);
  vsprintf(str, (pCHAR8)format, ap);
  va_end(ap);

  if (dfp)
    fprintf(dfp, "%s", str);
}

/**
*  impeghd_mp4_rev16
*
*  \brief Change big endian to small endian and vice-versa.
*         Input is 2 bytes
*
*  \param l
*
*  \return WORD32
*
*/
UWORD16 impeghd_mp4_rev16(UWORD16 l)
{
#ifdef ITTIAM_BIG_ENDIAN
  return (l);
#else
  return (((l & 0xff) << 8) + ((l & 0xff00) >> 8));
#endif
}

/**
*  impeghd_mp4_rev24
*
*  \brief Change big endian to small endian and vice-versa.
*         Input is 4 bytes
*
*  \param l
*
*  \return WORD32
*
*/
UWORD32 impeghd_mp4_rev24(UWORD32 l)
{
#ifdef ITTIAM_BIG_ENDIAN
  return (l);
#else
  return ((((l & 0xff) << 24) + ((l & 0xff00) << 8) + ((l & 0xff0000) >> 8) +
           ((l & 0xff000000) >> 24)) >>
          8);
#endif
}

/**
*  impeghd_mp4_rev32
*
*  \brief Change big endian to small endian and vice-versa.
*         Input is 4 bytes
*
*  \param l
*
*  \return WORD32
*
*/
UWORD32 impeghd_mp4_rev32(UWORD32 l)
{
#ifdef ITTIAM_BIG_ENDIAN
  return (l);
#else
  return (((l & 0xff) << 24) + ((l & 0xff00) << 8) + ((l & 0xff0000) >> 8) +
          ((l & 0xff000000) >> 24));
#endif
}

/**
*  impeghd_mp4_rev64
*
*  \brief Change big endian to small endian and vice-versa.
*         Input is 8 bytes
*
*  \param l
*
*  \return WORD32
*
*/
UWORD32 impeghd_mp4_rev64(UWORD32 l) { return impeghd_mp4_rev32(l); }

/**
*  impeghd_mp4_add_mem_node
*
*  \brief Function to add allocated pointers to a linked list (m) which is of type 'ia_mp4_mem_node' in a view to free them at a stretch at a later stage
*
*  \param [in/out] m   Pointer to memory node pointer
*  \param [in]     ptr Pointer to node to be added to list
*
*  \return VOID
*
*/
static VOID impeghd_mp4_add_mem_node(ia_mp4_mem_node **m, pVOID ptr)
{
  ia_mp4_mem_node *mem_temp;

  mem_temp = (ia_mp4_mem_node *)impeghd_mp4_malloc_wrapper(sizeof(ia_mp4_mem_node));
  mem_temp->ptr = ptr;
  mem_temp->next = *m;
  *m = mem_temp;
}

/**
*  impeghd_mp4_mem_node_malloc
*
*  \brief Allocates the memory requested for node, and returns a void
*         pointer to it
*
*  \param m
*  \param size
*
*  \return pVOID
*
*/
pVOID impeghd_mp4_mem_node_malloc(ia_mp4_mem_node **m, WORD32 size)
{
  pVOID ptr = impeghd_mp4_malloc_wrapper(size);
  impeghd_mp4_add_mem_node(m, ptr);
  return ptr;
}

/**
*  impeghd_mp4_mem_node_calloc
*
*  \brief Allocates the memory requested, and returns a void
*         pointer to it
*
*  \param m
*  \param count
*  \param size
*
*  \return pVOID
*
*/
pVOID impeghd_mp4_mem_node_calloc(ia_mp4_mem_node **m, WORD32 count, WORD32 size)
{
  pVOID ptr = impeghd_mp4_calloc_wrapper(count, size);
  impeghd_mp4_add_mem_node(m, ptr);
  return ptr;
}

/**
*  impeghd_mp4_free_mem_node
*
*  \brief Deletes the node from linked list, and frees the ptr
*
*  \param ptr
*  \param m
*
*  \return VOID
*
*/
VOID impeghd_mp4_free_mem_node(pVOID ptr, ia_mp4_mem_node **m)
{
  /* search for ptr in linked list */
  ia_mp4_mem_node *t = NULL, *temp;
  temp = *m;
  if (!ptr)
    return;

  while (temp)
  {
    if (temp->ptr == ptr)
      break;
    temp = temp->next;
  }

  if (temp)
  {
    t = temp->next;
    temp->ptr = temp->next->ptr;
    temp->next = temp->next->next;
    impeghd_mp4_free_wrapper((pVOID)t);
  }
  impeghd_mp4_free_wrapper((pVOID)ptr);
}

/**
*  impeghd_mp4_free_all_nodes
*
*  \brief Frees all the memory that was once allocated using
*         impeghd_mp4_mem_node_malloc/calloc(), whose pointers
*         are stored in ia_mp4_mem_node->ptr.
*
*  \param m
*
*  \return WORD32
*
*/
VOID impeghd_mp4_free_all_nodes(ia_mp4_mem_node **m)
{
  ia_mp4_mem_node *mem_temp;
  if (*m == NULL)
    return;
  mem_temp = (*m)->next;
  do
  {
    impeghd_mp4_free_wrapper((pVOID)((*m)->ptr));
    impeghd_mp4_free_wrapper((pVOID)(*m));
    *m = mem_temp;
    if (mem_temp)
      mem_temp = mem_temp->next;
  } while (*m);
  return;
}

/**
*  impeghd_mp4_malloc_wrapper
*
*  \brief Allocates the memory requested, and returns a void pointer to it.
*
*  \param size
*
*  \return pVOID
*
*/
pVOID impeghd_mp4_malloc_wrapper(WORD32 size)
{
  pVOID ptr = malloc(size);
  return ptr;
}

/**
*  impeghd_mp4_calloc_wrapper
*
*  \brief Allocates the memory requested for count of object,
*        and returns a void pointer to it
*
*  \param cnt
*  \param size
*
*  \return pVOID
*
*/
pVOID impeghd_mp4_calloc_wrapper(WORD32 cnt, WORD32 size)
{
  pVOID ptr = calloc(cnt, size);
  return ptr;
}

/**
*  impeghd_mp4_free_wrapper
*
*  \brief Frees the memory allocated to the void pointer
*
*  \param ptr
*
*  \return VOID
*
*/
VOID impeghd_mp4_free_wrapper(pVOID ptr) { free(ptr); }

/**
*  impeghd_mp4_find_str_file
*
*  \brief Read given str from file
*
*  \param pfp
*  \param buf
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_find_str_file(VOID **pfp, pWORD8 buf)
{
  WORD8 tbuf[IT_STR_LEN];
  WORD32 buflen = (WORD32)strlen((pCHAR8)buf);
  WORD32 tpos = 0, ret, count = 0;

  if (buflen > IT_STR_LEN)
    return IT_ERROR;

  ret = impeghd_mp4_fread(tbuf, 1, buflen, *pfp);
  if (ret < buflen)
    return IT_ERROR;

  ret = 1;
  while (memcmp(buf, &tbuf[tpos], buflen) && ret == 1)
  {
    if (tpos + buflen == IT_STR_LEN - 1)
    {
      memmove(tbuf, &tbuf[tpos], buflen);
      tpos = 0;
    }
    ret = impeghd_mp4_fread(&tbuf[tpos + buflen], 1, 1, *pfp);
    tpos++;
    count++;
  }

  if (ret != 1)
    return IT_ERROR;

  tbuf[tpos + buflen - 1] = '\0'; /* just for printing */
  /*impeghd_mp4_error_hdl(IT_OK,NULL,"\n%s, %x",&tbuf[tpos],count); */
  return count;
}

/**
*  impeghd_mp4_sub64_64
*
*  \brief Function to substract two number of type it_uint64
*
*  \param [out] ans Pointer to subtracted output
*  \param [in]  a1  Pointer to input1
*  \param [in]  b1  Pointer to input2
*
*  \return VOID
*
*/
static VOID impeghd_mp4_sub64_64(it_uint64 *ans, it_uint64 *a1, it_uint64 *b1)
{
  it_uint64 b, a;

  b.lsw = b1->lsw;
  b.msw = b1->msw;

  a.lsw = a1->lsw;
  a.msw = a1->msw;

  ans->lsw = ans->msw = 0;

  if (a.lsw < b.lsw)
  {
    ans->lsw = (0xffffffff - (b.lsw - a.lsw)) + 1;
    a.msw--;
  }
  else
  {
    ans->lsw = a.lsw - b.lsw;
  }
  ans->msw = a.msw - b.msw;

  return;
}

/**
*  impeghd_mp4_multi_1k
*
*  \brief Multiplies 1k
*
*  \param a
*  \param b
*
*  \return VOID
*
*/
VOID impeghd_mp4_multi_1k(it_uint64 *a, it_uint64 b)
{
  it_uint64 a1, a2;

  a->msw = b.msw << 10;
  a->msw |= b.lsw >> 22;
  a->lsw = b.lsw << 10;

  a1.msw = b.msw << 4;
  a1.msw |= b.lsw >> 28;
  a1.lsw = b.lsw << 4;

  a2.msw = b.msw << 3;
  a2.msw |= b.lsw >> 29;
  a2.lsw = b.lsw << 3;

  impeghd_mp4_sub64_64(a, a, &a1);
  impeghd_mp4_sub64_64(a, a, &a2);
}

/**
*  impeghd_mp4_div64
*
*  \brief Divides two number of type it_uint64
*         Assumes dividend.msw < divisor
*
*  \param ans
*  \param divident
*  \param divisor
*
*  \return WORD32
*
*/
UWORD32 impeghd_mp4_div64(pUWORD32 ans, it_uint64 dividend, UWORD32 divisor)
{
  UWORD8 i;
  UWORD32 msw, lsw;
  *ans = 0;

  msw = dividend.msw;
  lsw = dividend.lsw;
  if ((divisor > dividend.lsw && dividend.msw == 0) || divisor == 0)
    return lsw;

  for (i = 1; i <= 33; i++)
  {
    if (msw < divisor)
    {
      *ans <<= 1;
    }
    else
    {
      *ans <<= 1;
      *ans |= 1;
      msw -= divisor;
    }
    msw = (msw << 1);
    msw |= (lsw >> 31);
    lsw = lsw << 1;
  }
  return (msw >> 1);
}

/**
*  impeghd_mp4_add64
*
*  \brief Adds two number of type it_uint64
*
*  \param ans
*  \param a
*  \param b
*
*  \return VOID
*
*/
VOID impeghd_mp4_add64(it_uint64 *ans, it_uint64 a, UWORD32 b)
{
  UWORD8 msbita, msbitb;

  msbita = a.lsw >> 31;
  msbitb = b >> 31;

  ans->msw = a.msw;

  if ((msbita == 1) && (msbitb == 1))
  {
    ans->msw += 1;
    ans->lsw = (a.lsw & 0x7fffffff) + (b & 0x7fffffff);
  }
  else
  {
    ans->lsw = a.lsw + b;
    if ((msbita == 1 || msbitb == 1) && (!ans->lsw >> 31))
    {
      ans->msw += 1;
    }
  }
}

/** @} */ /* End of MP4Parser */