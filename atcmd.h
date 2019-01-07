/****************************************************************************
 * external/services/atcmd/atcmd.h
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __SERVICES_ATCMD_ATCMD_H
#define __SERVICES_ATCMD_ATCMD_H

#include <stddef.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)         (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef MIN
#  define MIN(a, b)             ((a) < (b) ? (a) : (b))
#endif

#define ATCMD_ACK_OK            "\r\nOK\r\n"
#define ATCMD_ACK_ERROR         "\r\nERROR\r\n"

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*atcmd_handler_t)(int fd, const char *cmd, char *param);

struct atcmd_table_s
{
  const char            *cmd;
  atcmd_handler_t       handler;
  int                   idx;
};

struct atcmd_table_ext_s
{
  const char            *cmd;
  atcmd_handler_t       handler;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern ssize_t atcmd_safe_write(int fd, const void *buf, size_t len);

extern void atcmd_cclk_handler(int fd, const char *cmd, char *param);
extern void atcmd_coap_handler(int fd, const char *cmd, char *param);
extern void atcmd_files_handler(int fd, const char *cmd, char *param);
extern void atcmd_http_handler(int fd, const char *cmd, char *param);
extern void atcmd_ifc_handler(int fd, const char *cmd, char *param);
extern void atcmd_ipr_handler(int fd, const char *cmd, char *param);
extern void atcmd_trb_handler(int fd, const char *cmd, char *param);
extern void atcmd_tso_handler(int fd, const char *cmd, char *param);
extern void atcmd_nping_handler(int fd, const char *cmd, char *param);
extern void atcmd_nping6_handler(int fd, const char *cmd, char *param);
extern void atcmd_ssl_handler(int fd, const char *cmd, char *param);

#endif /* __SERVICES_ATCMD_ATCMD_H */
