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
#include <termios.h>
#include <time.h>

#include "atcmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

# define putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))
#define ATCMD_DMCFG_FILE_PATH "/data/dmcfg"

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

void atcmd_flush_handler(int fd, const char *cmd, char *param)
{
#ifdef CONFIG_MISC_RPMSG
  char *str = param;
  const char *path;
  int ret = -EINVAL;
  int fd_misc;

  str += strlen("at+pflush");
  if (*str != '=')
    {
      goto out;
    }

  path = ++str;

  fd_misc = open("/dev/misc", 0);
  if (fd_misc >= 0)
    {
      struct misc_remote_ramflush_s flush =
        {
          .fpath = path,
        };

      ret = ioctl(fd_misc, MISC_REMOTE_RAMFLUSH, (unsigned long)&flush);
      close(fd_misc);
    }

out:
  dprintf(fd, "\r\n%s\r\n", ret >= 0 ? "OK" : "ERROR");
#else
  dprintf(fd, "\r\n+PFLUSH NOT SUPPORT\r\n");
#endif
}

void atcmd_ifc_handler(int fd, const char *cmd, char *param)
{
  pm_activity(0, 10);
  atcmd_safe_write(fd, ATCMD_ACK_OK, strlen(ATCMD_ACK_OK));
}

static int atcmd_ipr_parser(char *str, int *rate)
{
  int tmp;

  str += strlen("at+ipr");

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
      return 2;
    }

  /* Get rate */

  tmp = strtoul(str, &str, 0);
  if (tmp < 0)
    {
      return -EINVAL;
    }

  *rate = tmp;

  return 0;
}

void atcmd_ipr_handler(int fd, const char *cmd, char *param)
{
  struct termios term;
  int rate;
  int ret;
  int ft;

  ret = atcmd_ipr_parser(param, &rate);
  if (ret < 0)
    {
      goto out;
    }
  else if (ret == 1)
    {
      dprintf(fd, "\r\nAT+IPR=rate\r\n");
      goto out;
    }

  ft = open("/dev/ttyS0", 0);
  if (ft < 0)
    {
      ret = -EINVAL;
      goto out;
    }

  dprintf(fd, "\r\nOK\r\n");  /* response ok before set reg by old speed. */
  usleep(10000);              /* 4ms at least for response ok successfully. */

  ioctl(ft, TCGETS, (unsigned long)&term);
  if (ret == 2)
    {
      close(ft);
      dprintf(fd, "\r\n+IPR=%d\r\n", term.c_speed);
      goto out;
    }

  if (rate <= 9600)
    {
      /* Set uart0 RX to clk 32K */

      putreg32(0x08012f, 0xb2010084);
      putreg32(0x101240, 0xb2010010);
    }
  else
    {
      /* Set uart0 RX to pll0 */

      putreg32(0x08012d, 0xb2010084);
      putreg32(0x103240, 0xb2010010);
    }
  term.c_speed = rate;
  ioctl(ft, TCSETS, (unsigned long)&term);

  close(ft);
  return;
out:
  dprintf(fd, "\r\nERROR\r\n");
}

void atcmd_trb_handler(int fd, const char *cmd, char *param)
{
  boardctl(BOARDIOC_RESET, 0);
}

static int atcmd_pmset_parser(char *str, int *set)
{
  int tmp;

  str += strlen("at+pmset");

  if (*str != '=')
    {
      return -EINVAL;
    }

  str++;
  if (*str == '\0' || *str == '?')
    {
      return 2;
    }

  tmp = strtoul(str, &str, 0);
  if ((tmp < 0) || (tmp > 1))
    {
      return -EINVAL;
    }

  *set = tmp;

  return 0;
}

/*
 at+pmset=<set>
 <set> 1  pmstay idle
       0  relax idle
*/

void atcmd_pmset_handler(int fd, const char *cmd, char *param)
{
  int  set = 0;
  int ret = 0;

  ret = atcmd_pmset_parser(param, &set);
  if (ret < 0)
    {
      goto out;
    }

  if (set == 1)
    {
      pm_stay(PM_IDLE_DOMAIN, PM_IDLE);
    }
  else
    {
      pm_relax(PM_IDLE_DOMAIN, PM_IDLE);
    }

out:
    dprintf(fd, "\r\n%s\r\n", ret >= 0 ? "OK" : "ERROR");
}

static int atcmd_dmcfg_parser(char *str, int *set)
{
  int tmp;

  str += strlen("at+dmcfg");

  if (*str == '?')
    {
      *set = 2;
      return 0;
    }

  if (*str != '=')
    {
      return -EINVAL;
    }

  str++;
  if (*str == '\0' || *str == '?')
    {
      return -EINVAL;
    }

  tmp = strtoul(str, &str, 0);
  if ((tmp < 0) || (tmp > 1))
    {
      return -EINVAL;
    }

  *set = tmp;

  return 0;
}

/*
 at+dmcfg=<set>
 <set> 0  disable DM auto reg
       1  enable DM auto reg
*/

void atcmd_dmcfg_handler(int fd, const char *cmd, char *param)
{
  int  set = 0;
  int ret = 0;

  ret = atcmd_dmcfg_parser(param, &set);
  if (ret < 0)
    {
      goto out;
    }

  if (set == 0)
    {
      int fd;
      fd = open(ATCMD_DMCFG_FILE_PATH, O_RDWR | O_CREAT);
      if (fd < 0)
        {
          syslog(LOG_INFO, "open dmcfg error\n");
          ret = -1;
          goto out;
        }
      close(fd);
    }
  else if (set == 1)
    {
      unlink(ATCMD_DMCFG_FILE_PATH);
    }
  else if (set == 2)
    {
      int fd1;
      fd1 = open(ATCMD_DMCFG_FILE_PATH, O_RDONLY);
      if (fd1 < 0)
        {
          dprintf(fd, "\r\n+DMCFG:1\r\n");
        }
      else
        {
          dprintf(fd, "\r\n+DMCFG:0\r\n");
          close(fd1);
        }
    }
  else
    {
      ret = -1;
    }

out:
    dprintf(fd, "\r\n%s\r\n", ret >= 0 ? "OK" : "ERROR");
}
