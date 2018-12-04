/****************************************************************************
 * external/services/atcmd/atcmd_nping6.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *  used to endorse or promote products derived from this software
 *  without specific prior written permission.
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
#include <unistd.h>

#include <arpa/inet.h>
#include <netutils/icmpv6_ping.h>

#include "atcmd.h"

#ifdef CONFIG_NETUTILS_PING6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ATCMD_NPING6_NPINGS         10    /* Default number of pings */
#define ATCMD_NPING6_POLL_DELAY     1000  /* 1 second in milliseconds */
#define ATCMD_NPING6_DATA_SIZE      56    /* Default ping data size */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct atcmd_nping6_priv_s
{
  int fd;
  int total;
};

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

static int atcmd_nping6_parase(struct ping6_info_s *info, char *param)
{
  char *str;
  int tmp;

  str = param + strlen("at+nping6");

  if (*str == '?')
    {
      return 1;
    }
  else if (*str != '=')
    {
      return -EINVAL;
    }

  /* Check ? & " */

  str++;
  if (*str == '\0')
    {
      return -EINVAL;
    }
  else if (*str == '?')
    {
      return 1;
    }
  else if (*str != '"')
    {
      return -EINVAL;
    }

  /* Get hostname */

  str++;
  info->hostname = str;

  str = strchr(str, '"');
  if (!str)
    {
      return -EINVAL;
    }
  *str = '\0';

  /* Get count */

  str++;
  if (*str == '\0')
    {
      return 0;
    }
  else if (*str != ',')
    {
      return -EINVAL;
    }

  str++;
  tmp = atoi(str);
  if (tmp < 1 || tmp > 255)
    {
      return -EINVAL;
    }

  info->count = tmp;

  str = strstr(str, ",");
  if (!str)
    {
      return 0;
    }

  /* Get datalen */

  str++;
  tmp = atoi(str);
  if (tmp < 1 || tmp > 1460)
    {
      return -EINVAL;
    }

  info->datalen = tmp;

  str = strstr(str, ",");
  if (!str)
    {
      return 0;
    }

  /* Get interval */

  str++;
  tmp = atoi(str);
  if (tmp < 1000 || tmp > 10000)
    {
      return -EINVAL;
    }

  info->delay = tmp;

  str = strstr(str, ",");
  if (!str)
    {
      return 0;
    }

  /* Get timeout */

  str++;
  tmp = atoi(str);
  if (tmp < 1000 || tmp > 60000)
    {
      return -EINVAL;
    }

  info->timeout = tmp;

  str = strstr(str, ",");
  if (!str)
    {
      return 0;
    }

  return -EINVAL;
}

static void atcmd_nping6_result(FAR const struct ping6_result_s *result)
{
  struct atcmd_nping6_priv_s *priv = result->info->priv;
  char strbuffer[INET6_ADDRSTRLEN];

  switch (result->code)
    {
      case ICMPv6_W_TIMEOUT:
        inet_ntop(AF_INET6, result->dest.s6_addr16, strbuffer, INET6_ADDRSTRLEN);
        dprintf(priv->fd, "\r\n+NPING6:%s,%d,%d,timout\r\n",
                strbuffer,result->seqno, result->extra);
        break;

      case ICMPv6_I_ROUNDTRIP:
        inet_ntop(AF_INET6, result->dest.s6_addr16, strbuffer, INET6_ADDRSTRLEN);
        dprintf(priv->fd, "\r\n+NPING6:%s,%d,%d\r\n",
                strbuffer,result->seqno, result->extra);
        priv->total += result->extra;
        break;

      case ICMPv6_E_HOSTIP:
      case ICMPv6_E_MEMORY:
      case ICMPv6_E_SOCKET:
      case ICMPv6_E_SENDTO:
      case ICMPv6_E_SENDSMALL:
      case ICMPv6_E_POLL:
      case ICMPv6_E_RECVFROM:
      case ICMPv6_E_RECVSMALL:
      case ICMPv6_I_FINISH:
        dprintf(priv->fd, "\r\n+NPING6STATE:%d,%u,%u,%d\r\n",
                result->nrequests ? 0 : result->code,
                result->nrequests, result->nreplies,
                result->nreplies ? priv->total / result->nreplies : 0);
        break;
    }
}
#endif

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

void atcmd_nping6_handler(int fd, const char *cmd, char *param)
{
#ifdef CONFIG_NETUTILS_PING6
  struct atcmd_nping6_priv_s priv;
  struct ping6_info_s info;
  int ret;

  /* Init default values */

  info.count    = ATCMD_NPING6_NPINGS;
  info.datalen  = ATCMD_NPING6_DATA_SIZE;
  info.delay    = ATCMD_NPING6_POLL_DELAY;
  info.timeout  = ATCMD_NPING6_POLL_DELAY;
  info.callback = atcmd_nping6_result;

  priv.fd    = fd;
  priv.total = 0;
  info.priv  = &priv;

  /* Parse user param */

  ret = atcmd_nping6_parase(&info, param);
  if (ret == 0)
    {
      icmp6_ping(&info);
    }
  else
    {
      dprintf(fd, "\r\n+NPING6:\"ipaddr\",<1-255>,<1-1460>,<1000-10000>,<1000-60000>\r\n");

      if (ret >= 0)
        {
          dprintf(fd, "\r\nOK\r\n");
        }
      else
        {
          dprintf(fd, "\r\nERROR\r\n");
        }
    }
#else
  dprintf(fd, "\r\n+NPING6: don't support\r\n");
#endif
}
