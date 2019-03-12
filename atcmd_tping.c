/****************************************************************************
 * external/services/atcmd/atcmd_tping.c
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

#define ATCMD_TPING_NPINGS          1     /* Default number of pings */
#define ATCMD_TPING_POLL_DELAY      1000  /* 1 second in milliseconds */
#define ATCMD_TPING_DATA_SIZE       8     /* Default ping data size */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct atcmd_tping_priv_s
{
  int fd;
};

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

static int atcmd_tping_parase(struct ping_info_s *info, char *param)
{
  char *str;
  int tmp;

  str = param + strlen("at+tping");

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

  /* Get hostname */

  info->hostname = str;

  str = strchr(str, ',');
  if (!str)
    {
      return 0;
    }
  *str = '\0';

  /* Get datalen */

  str++;
  tmp = atoi(str);
  if (tmp < 8 || tmp > 1460)
    {
      return -EINVAL;
    }

  info->datalen = tmp;

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

  return 0;
}

static void atcmd_tping_result(FAR const struct ping_result_s *result)
{
  struct atcmd_tping_priv_s *priv = result->info->priv;

  switch (result->code)
    {
      case ICMP_I_ROUNDTRIP:
        dprintf(priv->fd, "\r\n+TPING:%u.%u.%u,%u,%d,%d\r\n",
               (result->dest.s_addr      ) & 0xff,
               (result->dest.s_addr >> 8 ) & 0xff,
               (result->dest.s_addr >> 16) & 0xff,
               (result->dest.s_addr >> 24) & 0xff,
               result->seqno, result->extra);
        break;

      case ICMP_I_FINISH:
        if (result->nreplies == 0)
          dprintf(priv->fd, "\r\n+TPINGERR:%d\r\n", result->code);
        break;
    }
}
#endif

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

void atcmd_tping_handler(int fd, const char *cmd, char *param)
{
#ifdef CONFIG_NETUTILS_PING
  struct atcmd_tping_priv_s priv;
  struct ping_info_s info;
  int ret;

  /* Init default values */

  info.count    = ATCMD_TPING_NPINGS;
  info.datalen  = ATCMD_TPING_DATA_SIZE;
  info.delay    = ATCMD_TPING_POLL_DELAY;
  info.timeout  = ATCMD_TPING_POLL_DELAY;
  info.callback = atcmd_tping_result;

  priv.fd    = fd;
  info.priv  = &priv;

  /* Parse user param */

  ret = atcmd_tping_parase(&info, param);
  if (ret == 0)
    {
      icmp_ping(&info);
    }
  else
    {
      dprintf(fd, "\r\n+TPING=ipaddr,<8-1460>,<1000-60000>\r\n");

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
  dprintf(fd, "\r\n+TPING: don't support\r\n");
#endif
}
