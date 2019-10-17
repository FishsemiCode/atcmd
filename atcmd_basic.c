/****************************************************************************
 * external/services/atcmd/atcmd_basic.c
 * AT cmd basic services
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

#include <nuttx/power/pm.h>
#include <nuttx/environ.h>
#include <nuttx/misc/misc_rpmsg.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/boardctl.h>
#include <sys/ioctl.h>
#include <time.h>

#include "atcmd.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct atcmd_env_s
{
  const char *name;
  const char *value;
  bool       flash_write;
};

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

static int atcmd_cclk_parser(char *str, time_t *time)
{
  struct tm tmtime;
  int tmp;

  str += strlen("at+cclk");

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
      return 0;
    }

  if (*str++ != '"')
    {
      return -EINVAL;
    }

  /* Get year */

  tmp = strtoul(str, &str, 0);
  if (tmp < 0)
    {
      return -EINVAL;
    }

  tmtime.tm_year = tmp - 1900;

  if (*str++ != '/')
    {
      return -EINVAL;
    }

  /* Get month */

  tmp = strtoul(str, &str, 0);
  if (tmp < 1 || tmp > 12)
    {
      return -EINVAL;
    }

  tmtime.tm_mon = tmp - 1;

  if (*str++ != '/')
    {
      return -EINVAL;
    }

  /* Get mday */

  tmp = strtoul(str, &str, 0);
  if (tmp < 1 || tmp > 31)
    {
      return -EINVAL;
    }

  tmtime.tm_mday = tmp;

  if (*str++ != ',')
    {
      return -EINVAL;
    }

  /* Get hours */

  tmp = strtoul(str, &str, 0);
  if (tmp < 0 || tmp > 23)
    {
      return -EINVAL;
    }

  tmtime.tm_hour = tmp;

  if (*str++ != ':')
    {
      return -EINVAL;
    }

  /* Get minutes */

  tmp = strtoul(str, &str, 0);
  if (tmp < 0 || tmp > 59)
    {
      return -EINVAL;
    }

  tmtime.tm_min = tmp;

  if (*str++ != ':')
    {
      return -EINVAL;
    }

  /* Get second */

  tmp = strtoul(str, &str, 0);
  if (tmp < 0 || tmp > 59)
    {
      return -EINVAL;
    }

  tmtime.tm_sec = tmp;

  *time = mktime(&tmtime);

  return 2;
}

void atcmd_cclk_handler(int fd, const char *cmd, char *param)
{
  time_t time;
  int ret;

  ret = atcmd_cclk_parser(param, &time);
  if (ret < 1)
    {
     // dprintf(fd, "\r\n+CCLK=yy/mm/dd,hh:mm:ss\r\n");
      goto out;
    }
  else if (ret == 1)
    {
      struct timespec tp = {0};
      struct tm tmtime;

      clock_gettime(CLOCK_REALTIME, &tp);

      gmtime_r(&tp.tv_sec, &tmtime);
      tmtime.tm_year += 1900;

      dprintf(fd, "\r\n+CCLK=\"%d/%d/%d,%d:%d:%d\"\r\n",
              tmtime.tm_year, tmtime.tm_mon,
              tmtime.tm_mday, tmtime.tm_hour,
              tmtime.tm_min, tmtime.tm_sec);
    }
  else
    {
      struct timespec tp;

      tp.tv_sec  = time;
      tp.tv_nsec = 0;
      clock_settime(CLOCK_REALTIME, &tp);
    }

out:
  dprintf(fd, "\r\n%s\r\n", ret >= 0 ? "OK" : "ERROR");
}

static int atcmd_env_parser(char *str, struct atcmd_env_s *env)
{
  char *ptr;

  str += strlen("at+penv");

  if (*str != ':')
    {
      return -EINVAL;
    }

  env->name = ++str;

  ptr = strchr(str, '=');
  if (!ptr)
    {
      return 1;
    }

  *ptr = 0;
  str  = ++ptr;
  env->value = str;

  ptr = strchr(str, '!');
  if (!ptr)
    {
      env->flash_write = 0;
    }
  else
    {
      env->flash_write = 1;
      *ptr = 0;
    }

  return 2;
}

void atcmd_env_handler(int fd, const char *cmd, char *param)
{
  struct atcmd_env_s env;
  int ret;

  ret = atcmd_env_parser(param, &env);
  if (ret < 0)
    {
      dprintf(fd, "\r\n+PENV:envname=envvalue!\r\n");
    }
  else if (ret == 1)
    {
      env.value = getenv_global(env.name);
      if (env.value)
        {
          dprintf(fd, "\r\n+PENV:%s=%s\r\n",env.name, env.value);
        }
      else
        {
          ret = -EINVAL;
        }
    }
  else if (ret == 2)
    {
      setenv_global(env.name, env.value, 1);
      if (env.flash_write)
        {
#ifdef CONFIG_MISC_RPMSG
          int fd_misc;

          fd_misc = open("/dev/misc", 0);
          if (fd_misc >= 0)
            {
              struct misc_remote_infowrite_s info =
                {
                  .name  = env.name,
                  .value = (uint8_t *)env.value,
                  .len   = strlen(env.value),
                };

              ret = ioctl(fd_misc, MISC_REMOTE_INFOWRITE, (unsigned long)&info);
              close(fd_misc);
            }
          else
#endif
            {
              ret = -EINVAL;
            }
        }
    }

  dprintf(fd, "\r\n%s\r\n", ret >= 0 ? "OK" : "ERROR");
}

void atcmd_ifc_handler(int fd, const char *cmd, char *param)
{
  pm_activity(0, 10);
  atcmd_safe_write(fd, ATCMD_ACK_OK, strlen(ATCMD_ACK_OK));
}

void atcmd_ipr_handler(int fd, const char *cmd, char *param)
{
  atcmd_safe_write(fd, ATCMD_ACK_OK, strlen(ATCMD_ACK_OK));
}

void atcmd_trb_handler(int fd, const char *cmd, char *param)
{
  boardctl(BOARDIOC_RESET, 0);
}
