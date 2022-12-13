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
#include <string.h>
#include <impeghd_type_def.h>
#include "impeghd_error_handler.h"
#include "impeghd_error_codes.h"

/**
 * @defgroup SampleApp Sample test bench
 * @ingroup  SampleApp
 * @brief Sample test bench
 *
 * @{
 */

/*****************************************************************************/
/* Global memory constants                                                   */
/*****************************************************************************/
/*****************************************************************************/
/* Ittiam mpeghd ErrorCode Definitions                               */
/*****************************************************************************/
/*****************************************************************************/
/* Class 0: API Errors
 */
/*****************************************************************************/
/* Fatal Errors */
pWORD8 impeghd_ppb_api_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "NULL Pointer: Memory Allocation Error"};
/*****************************************************************************/
/* Class 1: Configuration Errors
 */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 impeghd_ppb_config_non_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Invalid pcm word size value", (pWORD8) "Invalid target loudness value",
    (pWORD8) "Invalid effect type", (pWORD8) "Invalid cicp index", (pWORD8) "Invalid preset id"};
/* Fatal Errors */
pWORD8 impeghd_ppb_config_fatal[IA_MAX_ERROR_SUB_CODE] = {(pWORD8) ""};
/*****************************************************************************/
/* Class 2: Initialization Errors
 */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 impeghd_ppb_init_non_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Insufficient input bytes",
    (pWORD8) "Insufficient element interaction bytes",
    (pWORD8) "Insufficient local setup information bytes",
    (pWORD8) "Insufficient binaural renderer impulse response bytes",
    (pWORD8) "Invalid resample ratio",
    (pWORD8) "Insufficient mae bytes",
};

pWORD8 impeghd_ppb_drc_init_non_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Insufficient bits in bitbuffer",
    (pWORD8) "Unexpected error",
    (pWORD8) "Parameter error",
    (pWORD8) "Unsupported channel groups",
    (pWORD8) "DRC coefficients exceeded maximum value",
    (pWORD8) "Selection process Initialization failed",
    (pWORD8) "Unsupported ducking sequence",
    (pWORD8) "DRC instructions exceeded maximum value",
};

pWORD8 impeghd_ppb_hoa_init_non_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "HOA renderer Initialization failed",
    (pWORD8) "Invalid HOA order",
    (pWORD8) "Invalid quantization",
    (pWORD8) "Invalid framesize",
    (pWORD8) "Invalid matrix",
    (pWORD8) "Invalid transport channel",
    (pWORD8) "Invalid subband config",
    (pWORD8) "Invalid bitsize",
    (pWORD8) "Matrix mismatch",
    (pWORD8) "Invalid space position type",
    (pWORD8) "Spherical wave model not implemented",
    (pWORD8) "Space position Initialization failed",
};

/* Fatal Errors */
pWORD8 impeghd_ppb_init_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Decoder initialization failed",
    (pWORD8) "No. of channels in stream greater than max channels defined",
    (pWORD8) "Sample rate is not supported",
    (pWORD8) "Zero Receiver Compensation Delay not valid for multisignal groups",
    (pWORD8) "Invalid fill bytes",
    (pWORD8) "3D audio config data not found",
    (pWORD8) "Invalid SBR framelength index",
    (pWORD8) "Invalid CICP speaker index",
    (pWORD8) "Wrong window sequence",
    (pWORD8) "Unsupported number of ASI groups",
    (pWORD8) "Unsupported framelength",
    (pWORD8) "Unsupported ASI switch groups",
    (pWORD8) "Unsupported ASI group presets",
    (pWORD8) "MHAS syncword mismatch",
    (pWORD8) "Number of config extensions exceeded Maximum",
    (pWORD8) "Number of speakers exceeded Maximum",
    (pWORD8) "Initialization fatal error",
    (pWORD8) "Config extension length exceeded",
    (pWORD8) "Unexpected error",
    (pWORD8) "Number of earcons exceeded Maximum",
    (pWORD8) "Number of ASI description blocks exceeded Maximum",
    (pWORD8) "Number of ASI description languages exceeded Maximum",
    (pWORD8) "Number of CICP layouts exceeded Maximum",
    (pWORD8) "Unsupported number of pcm signals",
    (pWORD8) "Unsupported ILD index",
    (pWORD8) "Persist memory pointer is NULL",
    (pWORD8) "Number of signal groups exceeded maximum",
    (pWORD8) "Unsupported MPEG-H Profile",
    (pWORD8) "Invalid configuration parameters",
    (pWORD8) "Channel mask 1 not supported for any LFE channel",
    (pWORD8) "Unsupported lpd mode",
    (pWORD8) "Unsupported elevation angle index",
    (pWORD8) "Unsupported azimuth angle index",
    (pWORD8) "Unsupported igf all zero",
    (pWORD8) "Invalid fac data present flag",
    (pWORD8) "Invalid short fac flag",
    (pWORD8) "Invalid mct entries",
    (pWORD8) "Invalid channel pair index",
    (pWORD8) "Invalid downmix config type",
    (pWORD8) "Invalid downmix type",
    (pWORD8) "Invalid downmix id",
    (pWORD8) "Invalid speaker distance",
    (pWORD8) "Unsupported cicp speaker index",
    (pWORD8) "Unsupported extension element config",
    (pWORD8) "Audio preroll parsing failed",
    (pWORD8) "Invalid delta time",
    (pWORD8) "ASI parse failed",
    (pWORD8) "ASI preset group definition config failed",
    (pWORD8) "ASI preset group definition extension config failed",
    (pWORD8) "Invalid group id",
    (pWORD8) "Complex prediction not supported",
    (pWORD8) "Invalid preset group number of conditions",
    (pWORD8) "Invalid configuration for LC profile",
    (pWORD8) "Invalid number of downmix id group preset extension",
    (pWORD8) "Invalid number of wired outputs",
    (pWORD8) "Invalid number of shifted channel",
    (pWORD8) "Invalid max sfb",
    (pWORD8) "Invalid ASI parameter",
    (pWORD8) "Invalid config for number of channels",
    (pWORD8) "Unsupported signal group type",
};

pWORD8 impeghd_ppb_drc_init_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Exceeded channel group count",
    (pWORD8) "Invalid gain set index",
    (pWORD8) "Unsupported method definition",
    (pWORD8) "Unexpected error",
    (pWORD8) "Sample rate is not supported",
    (pWORD8) "Framesize is not supported",
    (pWORD8) "Channel count is not supported",
    (pWORD8) "Invalid delta tmin value",
    (pWORD8) "Unsupported delay mode",
    (pWORD8) "Unsupported delay samples",
    (pWORD8) "Unsupported subband domain mode",
    (pWORD8) "Unsupported number of subbands",
    (pWORD8) "Invalid drc parameter for LC profile",
};
pWORD8 impeghd_ppb_hoa_init_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Invalid vector index",
    (pWORD8) "HOA renderer matrix initialization failed",
    (pWORD8) "HOA Render initialization failed",
    (pWORD8) "Invalid framelength indicator",
    (pWORD8) "Invalid matrix selected",
    (pWORD8) "Invalid number of channels",
    (pWORD8) "Invalid active directional signal ids",
    (pWORD8) "Invalid minimum amb order",
    (pWORD8) "Invalid vector length",
    (pWORD8) "Invalid diff order",
    (pWORD8) "Invalid no of vq element bits",
    (pWORD8) "Invalid hoa independency flag",
    (pWORD8) "Invalid hoa nbitsq",
    (pWORD8) "Invalid hoa amb coef trans state",
    (pWORD8) "Invalid hoa amb coef index",
    (pWORD8) "Invalid vvec index",
    (pWORD8) "Invalid sparse order",
    (pWORD8) "NFC not allowed",
    (pWORD8) "Invalid configuration for LC profile",
};
pWORD8 impeghd_ppb_binaural_init_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Binaural renderer initialization failed",
    (pWORD8) "Binaural renderer unsupported diffuse length or direct length",
};

pWORD8 impeghd_ppb_fc_init_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Invalid parameter",
    (pWORD8) "Format converter initialization failed",
    (pWORD8) "Invalid channel number",
    (pWORD8) "Invalid CICP Index",
    (pWORD8) "Invalid precision level",
    (pWORD8) "Invalid gain table size",
    (pWORD8) "Unsupported speaker layout",
    (pWORD8) "Compact template not found",
    (pWORD8) "Invalid number of compact configurations",
    (pWORD8) "Invalid number of equalizers",
    (pWORD8) "Invalid equalizer configuration",
};

pWORD8 impeghd_ppb_oam_init_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Unsupported number of OAM objects",
};

pWORD8 impeghd_ppb_ei_init_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Unsupported interaction mode",
    (pWORD8) "Unsupported number of groups",
    (pWORD8) "Invalid id",
    (pWORD8) "Invalid change position",
    (pWORD8) "Invalid change gain",
};

/*****************************************************************************/
/* Class 3: Execution Errors
 */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 impeghd_ppb_exe_non_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Insufficient input bytes ", (pWORD8) "Insufficient scene displacement bytes",
    (pWORD8) "Insufficient HOA bytes", (pWORD8) "Insufficient meta data bytes",
};
/* Fatal Errors */

// Core coder
pWORD8 impeghd_ppb_exe_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Exceeded maximum downmix matrices per id",
    (pWORD8) "Unsupported window length",
    (pWORD8) "Unsupported window shape",
    (pWORD8) "Invalid TNS sfb",
    (pWORD8) "sfb info NULL",
    (pWORD8) "Unsupported number of channels",
    (pWORD8) "Invalid stereo config",
    (pWORD8) "Unsupported element index",
    (pWORD8) "Invalid window shape combination",
    (pWORD8) "LPD dec handle NULL",
    (pWORD8) "Invalid pitch lag",
    (pWORD8) "Invalid fd factor",
    (pWORD8) "Invalid pitch",
    (pWORD8) "Exceeded maximum downmix matrix elements",
    (pWORD8) "Exceeded maximum multichannel bands",
    (pWORD8) "Coefficients Pointer NULL",
    (pWORD8) "Invalid preset id",
    (pWORD8) "No suitable track found",
    (pWORD8) "MCT process failed",
    (pWORD8) "Invalid channel configuration",
    (pWORD8) "Unsupported MHAS packet type",
    (pWORD8) "Unsupported number of prerolls",
    (pWORD8) "Decode frame error",
    (pWORD8) "MDP process failed",
    (pWORD8) "Arthmetic decode failed",
    (pWORD8) "Invalid Sampling frequency for LPD mode",
    (pWORD8) "Exceeded maximum scale factor band index",
    (pWORD8) "Invalid start band value",
    (pWORD8) "Assigned group ids exceeded maximum",
    (pWORD8) "Invalid group index",
    (pWORD8) "Invalid fac length",
    (pWORD8) "Invalid max sfb value",
    (pWORD8) "Invalid configuration parameters",
    (pWORD8) "TD Config handle NULL",
    (pWORD8) "Uni DRC coefficients pointer NULL",
    (pWORD8) "Invalid scale factor",
    (pWORD8) "sfb exceeded maximum value",
    (pWORD8) "Domain switcher process failed",
};

// DRC
pWORD8 impeghd_ppb_drc_exe_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Invalid drc instructions index", (pWORD8) "DRC process failed",
    (pWORD8) "DRC unsupported subband domain mode",
};

// HOA
pWORD8 impeghd_ppb_hoa_exe_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "HOA spatial process failed", (pWORD8) "HOA scratch pointer NULL",
    (pWORD8) "Invalid hoa coded gain",
};

// Binaural
pWORD8 impeghd_ppb_binaural_exe_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Unsupported number of speakers", (pWORD8) "Unsupported number of channels",
    (pWORD8) "Unsupported data format",        (pWORD8) "Filter parameter computation failed",
    (pWORD8) "Unsupported number of taps",     (pWORD8) "Unsupported filter length",
};

// OAM
pWORD8 impeghd_ppb_oam_exe_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Object metadata decoding is unsupported", (pWORD8) "Framelength is unsupported",
};

// MAE preprocessor
pWORD8 impeghd_ppb_mae_exe_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "MAE divergence error",
    (pWORD8) "Invalid OAM index",
    (pWORD8) "MAE interactivity violation error",
    (pWORD8) "No diffusion error",
    (pWORD8) "Unsupported elevation range",
    (pWORD8) "Unsupported azimuth angle",
    (pWORD8) "Unsupported group preset",
    (pWORD8) "ASI and interaction data number of groups differ",
    (pWORD8) "Switch group off",
    (pWORD8) "On off interaction failed",
};

/*****************************************************************************/
/* error info structure                                                      */
/*****************************************************************************/
/* The Module's Error Info Structure */
ia_error_info_struct impeghd_error_info = {
    /* The Module Name  */
    (pWORD8) "Ittiam mpegh_dec",
    {/* The Class Names  */
     (pWORD8) "API", (pWORD8) "Configuration", (pWORD8) "Initialization", (pWORD8) "Execution",
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "",
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "MPEGHD"},
    {/* The Message Pointers  */
     {{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL}},
     {{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL}}}};

/*****************************************************************************/
/*                                                                           */
/*  Function name : impeghd_error_handler_init                     */
/*                                                                           */
/*  Description   : Initialize the error struct with string pointers         */
/*                                                                           */
/*  Inputs        : none                                                     */
/*                                                                           */
/*  Globals       : ia_error_info_struct impeghd_error_info        */
/*                  pWORD8 impeghd_ppb_api_non_fatal               */
/*                  pWORD8 impeghd_ppb_api_fatal                   */
/*                  pWORD8 impeghd_ppb_config_non_fatal            */
/*                  pWORD8 impeghd_ppb_config_fatal                */
/*                  pWORD8 impeghd_ppb_init_non_fatal              */
/*                  pWORD8 impeghd_ppb_init_fatal                  */
/*                  pWORD8 impeghd_ppb_exe_non_fatal               */
/*                  pWORD8 impeghd_ppb_exe_fatal                   */
/*                                                                           */
/*  Processing    : Init the struct with error string pointers               */
/*                                                                           */
/*  Outputs       : none                                                     */
/*                                                                           */
/*  Returns       : none                                                     */
/*                                                                           */
/*  Issues        : none                                                     */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

VOID impeghd_error_handler_init()
{
  /* The Message Pointers  */
  impeghd_error_info.ppppb_error_msg_pointers[1][0][0] = impeghd_ppb_api_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[0][1][0] = impeghd_ppb_config_non_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][1][0] = impeghd_ppb_config_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[0][2][0] = impeghd_ppb_init_non_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[0][2][1] = impeghd_ppb_drc_init_non_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[0][2][2] = impeghd_ppb_hoa_init_non_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][2][0] = impeghd_ppb_init_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][2][1] = impeghd_ppb_drc_init_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][2][2] = impeghd_ppb_hoa_init_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][2][3] = impeghd_ppb_binaural_init_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][2][4] = impeghd_ppb_fc_init_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][2][5] = impeghd_ppb_oam_init_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][2][6] = impeghd_ppb_ei_init_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[0][3][0] = impeghd_ppb_exe_non_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][3][0] = impeghd_ppb_exe_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][3][1] = impeghd_ppb_drc_exe_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][3][2] = impeghd_ppb_hoa_exe_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][3][3] = impeghd_ppb_binaural_exe_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][3][5] = impeghd_ppb_oam_exe_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][3][6] = impeghd_ppb_mae_exe_fatal;
}

/*****************************************************************************/
/* ia_testbench ErrorCode Definitions                                        */
/*****************************************************************************/
/*****************************************************************************/
/* Class 0: Memory & File Manager Errors
 */
/*****************************************************************************/
/* Non Fatal Errors */
/* Fatal Errors */
pWORD8 impeghd_ppb_ia_testbench_mem_file_man_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "File Open Failed", (pWORD8) "Wave header writing Failed"};

/*****************************************************************************/
/* error info structure                                                      */
/*****************************************************************************/
/* The Module's Error Info Structure */
ia_error_info_struct impeghd_ia_testbench_error_info = {
    /* The Module Name  */
    (pWORD8) "ia_testbench",
    {/* The Class Names  */
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "Memory & File Manager",
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "",
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) ""},
    {/* The Message Pointers  */
     {{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL}},
     {{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL},
      {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
       NULL}}}};

/*****************************************************************************/
/*                                                                           */
/*  Function name : ia_testbench_error_handler_init                          */
/*                                                                           */
/*  Description   : Initialize the error struct with string pointers         */
/*                                                                           */
/*  Inputs        : none                                                     */
/*                                                                           */
/*  Globals       : ia_error_info_struct impeghd_ia_testbench_error_info */
/*                  pWORD8 impeghd_ppb_ia_testbench_mem_file_man_fatal */
/*                                                                           */
/*  Processing    : Init the struct with error string pointers               */
/*                                                                           */
/*  Outputs       : none                                                     */
/*                                                                           */
/*  Returns       : none                                                     */
/*                                                                           */
/*  Issues        : none                                                     */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

VOID ia_testbench_error_handler_init()
{
  /* The Message Pointers  */
  impeghd_ia_testbench_error_info.ppppb_error_msg_pointers[1][4][0] =
      impeghd_ppb_ia_testbench_mem_file_man_fatal;
}

/*****************************************************************************/
/*                                                                           */
/*  Function name : impeghd_error_handler */
/*                                                                           */
/*  Description   : Called Prints the status error code from the err_info    */
/*                                                                           */
/*  Inputs        : ia_error_info_struct *p_mod_err_info (Error info struct) */
/*                  WORD8 *pb_context (Context of error)                     */
/*                  IA_ERRORCODE code (Error code)                           */
/*                                                                           */
/*  Globals       : none                                                     */
/*                                                                           */
/*  Processing    : whenever any module calls the errorhandler,  it  informs */
/*                  it about the module for which it is called and a context */
/*                  in which it was  called  in addition to  the  error_code */
/*                  the message is displayed  based  on the  module's  error */
/*                  message  array  that maps to  the error_code the context */
/*                  gives specific info in which the error occured  e.g. for */
/*                  testbench   module,  memory  allocator   can   call  the */
/*                  errorhandler   for  memory  inavailability  in   various */
/*                  contexts like input_buf or output_buf e.g.  for  mp3_enc */
/*                  module, there can be various instances running.  context */
/*                  can be used to  identify  the  particular  instance  the */
/*                  error handler is being called for                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : IA_ERRORCODE error_value  (Error value)                  */
/*                                                                           */
/*  Issues        : none                                                     */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

IA_ERRORCODE impeghd_error_handler(ia_error_info_struct *p_mod_err_info, WORD8 *pb_context,
                                   IA_ERRORCODE code)
{
  if (code == IA_MPEGH_DEC_NO_ERROR)
  {
    return IA_MPEGH_DEC_NO_ERROR;
  }

  {
    WORD is_fatal = (((UWORD)code & 0x8000) >> 15);
    WORD err_class = (((UWORD)code & 0x7800) >> 11);
    WORD mod_name = (((UWORD)code & 0x0700) >> 8);
    WORD err_sub_code = (((UWORD)code & 0x00FF));

    printf("\n");
    if (!is_fatal)
    {
      printf("non ");
    }
    printf("fatal error: ");

    if (p_mod_err_info->pb_module_name != NULL)
    {
      printf("%s  ", p_mod_err_info->pb_module_name);
    }

    if (p_mod_err_info->pb_module_name != NULL &&
        strcmp((const char *)p_mod_err_info->pb_module_name, "ia_testbench"))
    {
      switch (mod_name)
      {
      case 0:
        printf("core coder ");
        break;
      case 1:
        printf("DRC ");
        break;
      case 2:
        printf("HOA ");
        break;
      case 3:
        printf("Binaural Renderer ");
        break;
      case 4:
        printf("Format Converter ");
        break;
      case 5:
        printf("OAM ");
        break;
      case 6:
        printf("MAE Preprocessor ");
        break;
      default:
        break;
      }
      printf("module :");
    }

    if (p_mod_err_info->ppb_class_names[err_class] != NULL)
    {
      printf("%s: ", p_mod_err_info->ppb_class_names[err_class]);
    }
    if (pb_context != NULL)
    {
      printf("%s: ", pb_context);
    }
    if (err_sub_code >= IA_MAX_ERROR_SUB_CODE ||
        p_mod_err_info->ppppb_error_msg_pointers[is_fatal][err_class][mod_name][err_sub_code] ==
            NULL)
    {
      printf("error unlisted\n");
    }
    else
    {
      printf(
          "%s\n",
          p_mod_err_info->ppppb_error_msg_pointers[is_fatal][err_class][mod_name][err_sub_code]);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of SampleApp */
