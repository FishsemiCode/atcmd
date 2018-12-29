/****************************************************************************
 * external/services/atcmd/atcmd_socket.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "atcmd.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct atcmd_tsocr_s
{
  int type;
  int protocol;
  int listen_port;
  int receive_ctl;
};

struct atcmd_tsost_s
{
  int  sockfd;
  char *remote_addr;
  int  remote_port;
  int  len;
  char *data;
};

struct atcmd_tsorf_s
{
  int  sockfd;
  int  req_len;
};

struct atcmd_tsocl_s
{
  int  sockfd;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern bool lib_isbasedigit(int ch, int base, int *value);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void atcmd_tsocr_handler(int fd, const char *cmd, char *param);
static void atcmd_tsost_handler(int fd, const char *cmd, char *param);
static void atcmd_tsorf_handler(int fd, const char *cmd, char *param);
static void atcmd_tsocl_handler(int fd, const char *cmd, char *param);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct atcmd_table_ext_s g_atcmd_socket[] =
{
  {"AT+TSOCR",  atcmd_tsocr_handler},
  {"AT+TSOST",  atcmd_tsost_handler},
  {"AT+TSORF",  atcmd_tsorf_handler},
  {"AT+TSOCL",  atcmd_tsocl_handler},
};

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

static int atcmd_tsocr_parser(char *str, struct atcmd_tsocr_s *tsocr)
{
  char *clr;
  int tmp;

  if (*str == '?')
    {
      return 1;
    }
  else if (*str != '=')
    {
      return -EINVAL;
    }

  /* Check '?' */

  str++;
  if (*str == '\0' || *str == '?')
    {
      return 1;
    }

  /* Get type */

  clr = strchr(str, ',');
  if (!clr)
    {
      return -EINVAL;
    }
  *clr = '\0';

  if (strcmp(str, "DGRAM") == 0)
    {
      tsocr->type = SOCK_DGRAM;
    }
  else if (strcmp(str, "RAW") == 0)
    {
      tsocr->type = SOCK_RAW;
    }
  else
    {
      return -EINVAL;
    }

  /* Get protocol */

  str = clr + 1;
  tmp = atoi(str);
  if (tmp < 0 || tmp > 255)
    {
      return -EINVAL;
    }

  tsocr->protocol = tmp;

  str = strstr(str, ",");
  if (!str)
    {
      return -EINVAL;
    }

  /* Get listen port */

  str++;
  tmp = atoi(str);
  if (tmp < 0 || tmp > 65536)
    {
      return -EINVAL;
    }

  tsocr->listen_port = tmp;

  str = strstr(str, ",");
  if (!str)
    {
      tsocr->receive_ctl = 1;
      return 0;
    }

  /* Get receive control */

  str++;
  tmp = atoi(str);
  if (tmp != 0 && tmp != 1)
    {
      return -EINVAL;
    }

  tsocr->receive_ctl = tmp;

  return 0;
}

static void atcmd_tsocr_handler(int fd, const char *cmd, char *param)
{
  struct atcmd_tsocr_s tsocr;
  struct sockaddr_in local;
  int ret, sockfd;

  ret = atcmd_tsocr_parser(param, &tsocr);
  if (ret)
    {
      dprintf(fd, "\r\n%s=<type>,<protocol>,<listen port>,[,<receive control>]\r\n", cmd);
      dprintf(fd, "\r\n%s=<DGRAM|RAW>,<0-255>,<0-65536>,[,<0|1>]\r\n", cmd);
      goto out;
    }

  sockfd = socket(AF_INET, tsocr.type, tsocr.protocol);
  if (sockfd < 0)
    {
      goto out;
    }

  local.sin_family      = AF_INET;
  local.sin_port        = HTONS(tsocr.listen_port);
  local.sin_addr.s_addr = HTONL(INADDR_ANY);
  ret = bind(sockfd, (struct sockaddr *)&local, sizeof(struct sockaddr_in));
  if (ret < 0)
    {
      goto out;
    }

  dprintf(fd, "\r\n%s:%d\r\n", cmd, sockfd);
out:
  dprintf(fd, "\r\n%s\r\n", ret >= 0 ? "OK" : "ERROR");
}

static char atcmd_tsost_cvt_to_char(char *str)
{
  int v1, v2;

  lib_isbasedigit(str[0], 16, &v1);
  lib_isbasedigit(str[1], 16, &v2);

  return v1 * 16 + v2;
}

static int atcmd_tsost_parser(char *str, struct atcmd_tsost_s *tsost)
{
  int tmp;

  if (*str == '?')
    {
      return 1;
    }
  else if (*str != '=')
    {
      return -EINVAL;
    }

  /* Check ? */

  str++;
  if (*str == '\0' || *str == '?')
    {
      return 1;
    }

  /* Get sockfd */

  tmp = atoi(str);
  if (tmp < 0)
    {
      return -EINVAL;
    }

  tsost->sockfd = tmp;

  str = strstr(str, ",");
  if (!str)
    {
      return -EINVAL;
    }

  /* Get remote addr */

  str++;
  tsost->remote_addr = str;
  str = strchr(str, ',');
  if (!str)
    {
      return -EINVAL;
    }
  *str = '\0';

  /* Get remote port */

  str++;
  tmp = atoi(str);
  if (tmp < 0 || tmp > 65535)
    {
      return -EINVAL;
    }

  tsost->remote_port = tmp;

  str = strstr(str, ",");
  if (!str)
    {
      return -EINVAL;
    }

  /* Get data len */

  str++;
  tmp = atoi(str);
  if (tmp < 1 || tmp > 1024)
    {
      return -EINVAL;
    }

  tsost->len = tmp;

  str = strstr(str, ",");
  if (!str)
    {
      return -EINVAL;
    }

  /* Get data */

  str++;
  if (*str == '"')
    {
      str++;
      tsost->data = str;

      str += tsost->len;
      if (*str != '"')
        {
          return -EINVAL;
        }
      *str = '\0';
    }
  else
    {
      int i;

      if (strlen(str) / 2 != tsost->len)
        {
          return -EINVAL;
        }

      tsost->data = str;

      for (i = 0; i < tsost->len; i++, str+=2)
        {
          tsost->data[i] = atcmd_tsost_cvt_to_char(str);
        }
    }

  return 0;
}

static void atcmd_tsost_handler(int fd, const char *cmd, char *param)
{
  struct atcmd_tsost_s tsost;
  struct sockaddr_in remote;
  uint32_t address;
  int ret;

  ret = atcmd_tsost_parser(param, &tsost);
  if (ret)
    {
      dprintf(fd, "\r\n%s=<socket>,<remote_addr>,<remote_port>,<length>,<data>\r\n", cmd);
      dprintf(fd, "\r\n%s=<returned by +TSOCR>,<ip>,<0-65536>,<1-1024>,<data>\r\n", cmd);
      goto out;
    }

  ret = inet_pton(AF_INET, tsost.remote_addr, &address);
  if (ret < 0)
    {
      goto out;
    }

  remote.sin_family      = AF_INET;
  remote.sin_port        = HTONS(tsost.remote_port);
  remote.sin_addr.s_addr = (in_addr_t)address;
  ret = sendto(tsost.sockfd, tsost.data, tsost.len, 0,
               (struct sockaddr *)&remote, sizeof(struct sockaddr_in));
  if (ret < 0)
    {
      goto out;
    }

  dprintf(fd, "\r\n%s:%d,%d\r\n", cmd, tsost.sockfd, ret);
out:
  dprintf(fd, "\r\n%s\r\n", ret >= 0 ? "OK" : "ERROR");
}

static int atcmd_tsorf_parser(char *str, struct atcmd_tsorf_s *tsorf)
{
  int tmp;

  if (*str == '?')
    {
      return 1;
    }
  else if (*str != '=')
    {
      return -EINVAL;
    }

  /* Check ? */

  str++;
  if (*str == '\0' || *str == '?')
    {
      return 1;
    }

  /* Get sockfd */

  tmp = atoi(str);
  if (tmp < 0)
    {
      return -EINVAL;
    }

  tsorf->sockfd = tmp;

  str = strstr(str, ",");
  if (!str)
    {
      return -EINVAL;
    }

  /* Get request length */

  str++;
  tmp = atoi(str);
  if (tmp < 0 || tmp > 65535)
    {
      return -EINVAL;
    }

  tsorf->req_len = tmp;

  return 0;
}

static void atcmd_tsorf_handler(int fd, const char *cmd, char *param)
{
  struct atcmd_tsorf_s tsorf;
  struct sockaddr_in remote;
  socklen_t remote_len;
  in_addr_t tmpaddr;
  char *req_buf;
  int ret, i;

  ret = atcmd_tsorf_parser(param, &tsorf);
  if (ret)
    {
      dprintf(fd, "\r\n%s=<socket>,<req_length>\r\n", cmd);
      dprintf(fd, "\r\n%s=<returned by +TSOCR>,<1-65536>\r\n", cmd);
      goto out;
    }

  req_buf = malloc(tsorf.req_len);
  if (!req_buf)
    {
      ret = -ENOMEM;
      goto out;
    }

  remote_len = sizeof(struct sockaddr_in);
  ret = recvfrom(tsorf.sockfd, req_buf, tsorf.req_len, 0,
               (struct sockaddr *)&remote, &remote_len);
  if (ret < 0)
    {
      goto out;
    }

  tmpaddr = ntohl(remote.sin_addr.s_addr);
  dprintf(fd, "\r\n%s:%d,%d.%d.%d.%d,%d,%d,",
              cmd, tsorf.sockfd,
              tmpaddr >> 24, (tmpaddr >> 16) & 0xff,
              (tmpaddr >> 8) & 0xff, tmpaddr & 0xff,
              ntohs(remote.sin_port),
              ret);

  for (i = 0; i < ret; i++)
    {
      dprintf(fd, "%02x", req_buf[i]);
    }

  dprintf(fd, ",0\r\n");

out:
  dprintf(fd, "\r\n%s\r\n", ret >= 0 ? "OK" : "ERROR");
}

static int atcmd_tsocl_parser(char *str, struct atcmd_tsocl_s *tsocl)
{
  int tmp;

  if (*str == '?')
    {
      return 1;
    }
  else if (*str != '=')
    {
      return -EINVAL;
    }

  /* Check ? */

  str++;
  if (*str == '\0' || *str == '?')
    {
      return 1;
    }

  /* Get sockfd */

  tmp = atoi(str);
  if (tmp < 0)
    {
      return -EINVAL;
    }

  tsocl->sockfd = tmp;

  return 0;
}

static void atcmd_tsocl_handler(int fd, const char *cmd, char *param)
{
  struct atcmd_tsocl_s tsocl;
  int ret;

  ret = atcmd_tsocl_parser(param, &tsocl);
  if (ret)
    {
      dprintf(fd, "\r\n%s=<socket>\r\n", cmd);
      dprintf(fd, "\r\n%s=<returned by +TSOCR>\r\n", cmd);
      goto out;
    }

  ret = close(tsocl.sockfd);
  if (ret < 0)
    {
      goto out;
    }

out:
  dprintf(fd, "\r\n%s\r\n", ret >= 0 ? "OK" : "ERROR");
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

void atcmd_tso_handler(int fd, const char *cmd, char *param)
{
  int i, prefix;

  for (i = 0; i < ARRAY_SIZE(g_atcmd_socket); i++)
    {
      prefix = strlen(g_atcmd_socket[i].cmd);
      if (strncasecmp(param, g_atcmd_socket[i].cmd, prefix) == 0)
        {
          g_atcmd_socket[i].handler(fd, &g_atcmd_socket[i].cmd[2], param + prefix);
          break;
        }
    }

  if (i == ARRAY_SIZE(g_atcmd_socket))
    {
      dprintf(fd, "\r\nERROR\r\n");
    }
}
