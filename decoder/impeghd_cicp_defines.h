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

#ifndef IMPEGHD_CICP_DEFINES_H
#define IMPEGHD_CICP_DEFINES_H

#define USED_CHANNEL 37
#define CICP2GEOMETRY_MAX_LOUDSPEAKERS (32)

#define CICP2GEOMETRY_LOUDSPEAKER_KNOWN (1)
#define CICP2GEOMETRY_LOUDSPEAKER_UNKNOWN (0)
#define CICP2GEOMETRY_LOUDSPEAKER_INVALID (-1)

#define CH_EMPTY -1
#define CH_M_L030 0
#define CH_M_R030 1
#define CH_M_000 2
#define CH_LFE1 3
#define CH_M_L110 4
#define CH_M_R110 5
#define CH_M_L022 6
#define CH_M_R022 7
#define CH_M_L135 8
#define CH_M_R135 9
#define CH_M_180 10
#define CH_M_L090 13
#define CH_M_R090 14
#define CH_M_L060 15
#define CH_M_R060 16
#define CH_U_L030 17
#define CH_U_R030 18
#define CH_U_000 19
#define CH_U_L135 20
#define CH_U_R135 21
#define CH_U_180 22
#define CH_U_L090 23
#define CH_U_R090 24
#define CH_T_000 25
#define CH_LFE2 26
#define CH_L_L045 27
#define CH_L_R045 28
#define CH_L_000 29
#define CH_U_L110 30
#define CH_U_R110 31
#define CH_U_L045 32
#define CH_U_R045 33
#define CH_M_L045 34
#define CH_M_R045 35
#define CH_LFE3 36
#define CH_M_LSCR 37
#define CH_M_RSCR 38
#define CH_M_LSCH 39
#define CH_M_RSCH 40
#define CH_M_L150 41
#define CH_M_R150 42

#define CICP_LS_IDX_0 (0)
#define CICP_LS_IDX_1 (1)
#define CICP_LS_IDX_2 (2)
#define CICP_LS_IDX_3 (3)
#define CICP_LS_IDX_4 (4)
#define CICP_LS_IDX_5 (5)
#define CICP_LS_IDX_6 (6)
#define CICP_LS_IDX_7 (7)
#define CICP_LS_IDX_8 (8)
#define CICP_LS_IDX_9 (9)
#define CICP_LS_IDX_10 (10)
#define CICP_LS_IDX_13 (13)
#define CICP_LS_IDX_14 (14)
#define CICP_LS_IDX_15 (15)
#define CICP_LS_IDX_16 (16)
#define CICP_LS_IDX_17 (17)
#define CICP_LS_IDX_18 (18)
#define CICP_LS_IDX_19 (19)
#define CICP_LS_IDX_20 (20)
#define CICP_LS_IDX_21 (21)
#define CICP_LS_IDX_22 (22)
#define CICP_LS_IDX_23 (23)
#define CICP_LS_IDX_24 (24)
#define CICP_LS_IDX_25 (25)
#define CICP_LS_IDX_26 (26)
#define CICP_LS_IDX_27 (27)
#define CICP_LS_IDX_28 (28)
#define CICP_LS_IDX_29 (29)
#define CICP_LS_IDX_30 (30)
#define CICP_LS_IDX_31 (31)
#define CICP_LS_IDX_32 (32)
#define CICP_LS_IDX_33 (33)
#define CICP_LS_IDX_34 (34)
#define CICP_LS_IDX_35 (35)
#define CICP_LS_IDX_36 (36)
#define CICP_LS_IDX_37 (37)
#define CICP_LS_IDX_38 (38)
#define CICP_LS_IDX_39 (39)
#define CICP_LS_IDX_40 (40)
#define CICP_LS_IDX_41 (41)
#define CICP_LS_IDX_42 (42)

#define CICP_MAX_CH (CH_M_R150 + 1)
#define NUM_LS_CFGS (20)

#endif /*IMPEGHD_CICP_DEFINES_H*/