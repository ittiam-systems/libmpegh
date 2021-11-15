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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <impeghd_type_def.h>
#include "impeghd_mp4_file.h"
#include "impeghd_mp4_utils.h"

/**
 * @defgroup MP4Parser MP4 Parser Utility
 * @ingroup  MP4Parser
 * @brief MP4 Parser Utility
 *
 * @{
 */

/**
*  impeghd_mp4_fopen
*
*  \brief function for opening file/buffer
*
*  \param file_name
*  \param with_file
*  \param size
*
*  \return it_avi_file_ctxt *
*
*/
it_avi_file_ctxt *impeghd_mp4_fopen(pUWORD8 file_name, UWORD8 with_file, WORD32 size)
{
  it_avi_file_ctxt *it_file =
      (it_avi_file_ctxt *)impeghd_mp4_malloc_wrapper(sizeof(it_avi_file_ctxt));
  it_file->with_file = with_file;
  if (it_file->with_file)
  {
    it_file->fp = fopen((void *)file_name, "rb");
    if (it_file->fp)
      return it_file;
    else
    {
      impeghd_mp4_free_wrapper((pVOID)it_file);
      return NULL;
    }
  }
  else
  {
    it_file->buf = (pWORD8)file_name;
    it_file->pos = (pWORD8)file_name;
    it_file->size = size;
    if (it_file->buf)
      return it_file;
    else
    {
      impeghd_mp4_free_wrapper(it_file);
      return NULL;
    }
  }
}

/**
*  impeghd_mp4_fread
*
*  \brief function for opening file/buffer
*
*  \param buffer
*  \param size
*  \param count
*  \param itf
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_fread(pVOID buffer, WORD32 size, WORD32 count, it_avi_file_ctxt *itf)
{
  if (itf->with_file)
  {
    return (WORD32)fread(buffer, size, count, itf->fp);
  }
  else
  {
    WORD32 avail = itf->size - (WORD32)(itf->pos - itf->buf);
    WORD32 toread = size * count;
    WORD32 allowed = (toread < avail) ? toread : avail;

    if (allowed < 0)
      return IT_ERROR;
    memcpy(buffer, itf->pos, allowed);
    itf->pos += allowed;
    return allowed / size;
  }
}

/**
*  impeghd_mp4_fseek
*
*  \brief function for seeking in file/buffer
*
*  \param itf
*  \param offset
*  \param origin
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_fseek(it_avi_file_ctxt *itf, WORD32 offset, WORD32 origin)
{
  if (itf->with_file)
  {
    if (!((offset == 0) && (origin == SEEK_CUR)))
      return fseek(itf->fp, offset, origin);
    else
      return (0);
  }
  else
  {
    switch (origin)
    {
    case SEEK_CUR:
      itf->pos += offset;
      break;
    case SEEK_END:
      itf->pos = itf->buf + itf->size + offset;
      break;
    case SEEK_SET:
      itf->pos = itf->buf + offset;
      break;
    }
    return 0;
  }
}

/**
*  impeghd_mp4_fclose
*
*  \brief function for closing file/buffer
*
*  \param itf
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_fclose(it_avi_file_ctxt *itf)
{
  if (itf->with_file)
  {
    if (fclose(itf->fp))
      return IT_ERROR;
    impeghd_mp4_free_wrapper((pVOID)itf);
    return IT_ERROR;
  }
  else
  {
    itf->buf = NULL;
    itf->pos = NULL;
    itf->size = 0;
    return 0;
  }
}

/**
*  impeghd_mp4_ftell
*
*  \brief function for geting current position in file/buffer
*
*  \param itf
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_ftell(it_avi_file_ctxt *itf)
{
  if (itf->with_file)
    return (WORD32)ftell(itf->fp);
  else
    return (WORD32)(itf->pos - itf->buf);
}

/**
*  impeghd_mp4_feof
*
*  \brief function to check if end of file is reached
*
*  \param itf
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_feof(it_avi_file_ctxt *itf)
{
  if (itf->with_file)
    return feof(itf->fp);
  else
    return !(itf->pos - (itf->buf + itf->size));
}

/**
*  impeghd_mp4_fread_buf
*
*  \brief function to read into buffer
*
*  \param buffer
*  \param size
*  \param count
*  \param itf
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_fread_buf(WORD32 **buffer, WORD32 size, WORD32 count, it_avi_file_ctxt *itf)
{
  return impeghd_mp4_fread(*buffer, size, count, itf);
}

WORD32 impeghd_mp4_find_mdat(it_avi_file_ctxt *itf, WORD32 *offset)
{
  WORD8 buf_size[4];
  WORD8 buf_test = 0;
  WORD32 bytes_test;
  WORD32 size;
  WORD32 ret = 0x777;
  UWORD32 *data_size = (UWORD32 *)&buf_size[0];
  while (!feof(itf->fp))
  {
    impeghd_mp4_fread(&buf_test, 1, 1, itf);
    if (buf_test == 'm')
    {
      impeghd_mp4_fread(&buf_test, 1, 1, itf);
      if (buf_test == 'd')
      {
        impeghd_mp4_fread(&buf_test, 1, 1, itf);
        if (buf_test == 'a')
          impeghd_mp4_fread(&buf_test, 1, 1, itf);
        if (buf_test == 't')
        {
          impeghd_mp4_fseek(itf, -8, SEEK_CUR);
          bytes_test = impeghd_mp4_fread(buf_size, 1, 4, itf);
          if (bytes_test < 4)
            return IT_ERROR;
          size = impeghd_mp4_rev32(*data_size);
          impeghd_mp4_fread(buf_size, 1, 4, itf);
          *offset = ftell(itf->fp);
          return (size - 8);
        }
      }
      if (buf_test == 'm')
        impeghd_mp4_fseek(itf, -1, SEEK_CUR);
    }
  }
  ret = 0;
  return ret;
}

WORD32 impeghd_mp4_read_mdat(it_avi_file_ctxt *itf, pUWORD8 buffer, WORD32 buf_size, WORD32 *size,
                             UWORD32 *length, WORD32 *offset)
{
  if (*size >= 0)
  {
    buf_size = (*size < buf_size) ? *size : buf_size;
    *length = (UWORD32)fread(buffer, 1, buf_size, itf->fp);
    *size -= *length;
    *offset = ftell(itf->fp);
  }
  return 0;
}

/** @} */ /* End of MP4Parser */