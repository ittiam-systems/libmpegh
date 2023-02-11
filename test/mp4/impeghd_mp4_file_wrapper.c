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
#include <impeghd_intrinsics_flt.h>
#include "impeghd_mp4_parser.h"
#include "impeghd_mp4_file_wrapper.h"
#include "impeghd_mp4_utils.h"

/**
 * @defgroup MP4Parser MP4 Parser Utility
 * @ingroup  MP4Parser
 * @brief MP4 Parser Utility
 *
 * @{
 */

WORD32 g_mhm1_tag;
WORD32 g_dash_tag;

/**
*  impeghd_mp4_fw_open
*
*  \brief Wrapper function to open file
*
*  \param [in] file_name Filename
*
*  \return ia_file_wrapper *
*
*/
ia_file_wrapper *impeghd_mp4_fw_open(WORD8 file_name[])
{

  ia_file_wrapper *transport =
      (ia_file_wrapper *)impeghd_mp4_calloc_wrapper(1, sizeof(ia_file_wrapper));

  transport->is_mp4_file = 0;
  transport->is_mp4_mhm1 = 0;
  transport->input_file = NULL;
  g_mhm1_tag = 0;
  g_dash_tag = 0;

  transport->header_given = 0;
  transport->file_cntxt = 0;
  transport->mp4_cntxt = 0;
  transport->interim_buffer = 0;
  transport->avail_buffer = 0;

  if ((transport->file_cntxt = impeghd_mp4_fopen((pUWORD8)file_name, 1, 0)) == NULL)
  {
    transport->input_file = fopen((pCHAR8)file_name, "rb");
    if (!transport->input_file)
    {
      free(transport);
      return 0;
    }
    else
      return transport;
  }

  if ((transport->mp4_cntxt = impeghd_mp4_parser_init(transport->file_cntxt)) == NULL)
  {
    transport->input_file = fopen((pCHAR8)file_name, "rb");
    if (!transport->input_file)
    {
      free(transport);
      return 0;
    }
    else
      return transport;
  }
  transport->is_mp4_file = 1;
  transport->is_mp4_mhm1 = g_mhm1_tag;
  transport->is_mp4_dash = g_dash_tag;
  transport->interim_buffer = impeghd_mp4_malloc_wrapper(IN_BUF_SIZE);
  if (transport->interim_buffer == NULL)
  {
    return 0;
  }
  return transport;
}

/**
*  impeghd_mp4_fw_read
*
*  \brief Wrapper function to read file
*
*  \param [in] transport File wrapper
*  \param [out] buffer   Buffer pointer
*  \param [in] buf_size  Buffer size
*  \param [out]length    Filled length
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_fw_read(ia_file_wrapper *transport, pUWORD8 buffer, WORD32 buf_size,
                           UWORD32 *length)
{
  if (!transport->is_mp4_file)
    *length = (UWORD32)fread(buffer, 1, buf_size, transport->input_file);
  else
  {
    WORD32 error_code;
    if (transport->header_given == 0)
    {
      transport->header_cntxt.header = (void *)buffer;
      error_code = impeghd_mp4_get_audio_header(transport->mp4_cntxt, &transport->header_cntxt);
      transport->header_given = 1;
      if (error_code != 0)
      {
        *length = 0;
        return 1;
      }
      *length = transport->header_cntxt.header_length;
    }
    else
    {
      WORD32 err_test;
      err_test = impeghd_mp4_stsz_valid(transport->mp4_cntxt);
      if (err_test == 2)
      {
        *length = 0;
        if (transport->is_mp4_file && transport->is_execution == 1)
        {
          err_test = impeghd_mp4_get_datamp4(
            transport->mp4_cntxt, &(transport->offset_dash),
            (pUWORD8)buffer, buf_size,
            (pUWORD32)length, &(transport->size_dash), &(transport->loc));
          if (err_test)
          {
            return err_test;
          }
        }
        return IT_DASH;
      }

      *length = 0;
      while ((WORD32)*length < buf_size)
      {
        if (transport->avail_buffer == 0)
        {
          transport->frame_cntxt.frame = (void *)transport->interim_buffer;
          error_code = impeghd_mp4_get_audio(transport->mp4_cntxt, &transport->frame_cntxt);
          if (error_code != 0)
          {
            if (*length == 0)
              return 1;
            else
              break;
          }
          transport->avail_buffer += transport->frame_cntxt.frame_length;
        }

        else
        {
          WORD32 size_to_cpy = ia_min_int((WORD32)(buf_size - *length), transport->avail_buffer);
          memcpy(buffer + (*length), transport->interim_buffer, size_to_cpy);
          memmove(transport->interim_buffer, ((char *)transport->interim_buffer) + size_to_cpy,
                  (transport->avail_buffer - size_to_cpy));
          *length += size_to_cpy;
          transport->avail_buffer -= size_to_cpy;
        }
      }
    }
  }

  return 0;
}

/**
*  impeghd_mp4_fw_close
*
*  \brief Wrapper function to close file
*
*  \param [in] transport File wrapper
*
*  \return WORD32
*
*/
UWORD32 impeghd_mp4_fw_close(ia_file_wrapper *transport)
{
  if (transport == 0)
    return 0;

  if (transport->mp4_cntxt)
  {
    impeghd_mp4_parser_close(transport->mp4_cntxt);
  }
  if (transport->file_cntxt)
  {
    impeghd_mp4_fclose(transport->file_cntxt);
  }
  // else
  if (transport->input_file)
  {
    fclose(transport->input_file);
  }

  if (transport->interim_buffer)
  {
    impeghd_mp4_free_wrapper((pVOID)(transport->interim_buffer));
  }
  if (transport)
  {
    impeghd_mp4_free_wrapper((pVOID)transport);
  }
  return 0;
}

/** @} */ /* End of MP4Parser */