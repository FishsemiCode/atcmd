/****************************************************************************
 * external/services/atcmd/atcmd_nping.c
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

#include <netutils/icmp_ping.h>

#include "atcmd.h"

#ifdef CONFIG_NETUTILS_PING

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ATCMD_NPING_NPINGS          10    /* Default number of pings */
#define ATCMD_NPING_POLL_DELAY      1000  /* 1 second in milliseconds */
#define ATCMD_NPING_DATA_SIZE       56    /* Default ping data size */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct atcmd_nping_priv_s
{
  int fd;
  int total;
};

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

static int atcmd_nping_parase(struct ping_info_s *info, char *param)
{
  char *str;

  str = param + strlen("at+nping");

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
  info->count = atoi(str);
  if (info->count < 1 || info->count > 255)
    {
      return -EINVAL;
    }

  str = strstr(str, ",");
  if (!str)
    {
      return 0;
    }

  /* Get datalen */

  str++;
  info->datalen = atoi(str);
  if (info->datalen < 1 || info->datalen > 1460)
    {
      return -EINVAL;
    }

  str = strstr(str, ",");
  if (!str)
    {
      return 0;
    }

  /* Get interval */

  str++;
  info->delay = atoi(str);
  if (info->delay < 1000 || info->delay > 10000)
    {
      return -EINVAL;
    }

  str = strstr(str, ",");
  if (!str)
    {
      return 0;
    }

  /* Get timeout */

  str++;
  info->timeout = atoi(str);
  if (info->timeout < 1000 || info->timeout > 60000)
    {
      return -EINVAL;
    }

  str = strstr(str, ",");
  if (!str)
    {
      return 0;
    }

  return -EINVAL;
}

static void atcmd_nping_result(FAR const struct ping_result_s *result)
{
  struct atcmd_nping_priv_s *priv = result->info->priv;

  switch (result->code)
    {
      case ICMP_W_TIMEOUT:
        dprintf(priv->fd, "\r\n+NPING:%u.%u.%u,%u,%d,%d,timout\r\n",
               (result->dest.s_addr      ) & 0xff,
               (result->dest.s_addr >> 8 ) & 0xff,
               (result->dest.s_addr >> 16) & 0xff,
               (result->dest.s_addr >> 24) & 0xff,
               result->seqno, result->extra);
        break;

      case ICMP_I_ROUNDTRIP:
        dprintf(priv->fd, "\r\n+NPING:%u.%u.%u,%u,%d,%d\r\n",
               (result->dest.s_addr      ) & 0xff,
               (result->dest.s_addr >> 8 ) & 0xff,
               (result->dest.s_addr >> 16) & 0xff,
               (result->dest.s_addr >> 24) & 0xff,
               result->seqno, result->extra);
        priv->total += result->extra;
        break;

      case ICMP_E_HOSTIP:
      case ICMP_E_MEMORY:
      case ICMP_E_SOCKET:
      case ICMP_E_SENDTO:
      case ICMP_E_SENDSMALL:
      case ICMP_E_POLL:
      case ICMP_E_RECVFROM:
      case ICMP_E_RECVSMALL:
      case ICMP_I_FINISH:
        dprintf(priv->fd, "\r\n+NPINGSTATE:%d,%u,%u,%d\r\n",
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

void atcmd_nping_handler(int fd, const char *cmd, char *param)
{
#ifdef CONFIG_NETUTILS_PING
  struct atcmd_nping_priv_s priv;
  struct ping_info_s info;
  int ret;

  /* Init default values */

  info.count    = ATCMD_NPING_NPINGS;
  info.datalen  = ATCMD_NPING_DATA_SIZE;
  info.delay    = ATCMD_NPING_POLL_DELAY;
  info.timeout  = ATCMD_NPING_POLL_DELAY;
  info.callback = atcmd_nping_result;

  priv.fd    = fd;
  priv.total = 0;
  info.priv  = &priv;

  /* Parse user param */

  ret = atcmd_nping_parase(&info, param);
  if (ret == 0)
    {
      icmp_ping(&info);
    }
  else
    {
      dprintf(fd, "\r\n+NPING:\"ipaddr\",<1-255>,<1-1460>,<1000-10000>,<1000-60000>\r\n");

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
  dprintf(fd, "\r\n+NPING: don't support\r\n");
#endif
}
