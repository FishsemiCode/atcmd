/****************************************************************************
 * external/services/atcmd/atcmd_pwr.c
 * AT cmd pwr services
 *
 *   Copyright (C) 2021 Fishsemi Inc. All rights reserved.
 *
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
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>

#include "atcmd.h"

#include "nuttx/power/pm.h"

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

void atcmd_pwr_handler(int fd, const char *cmd, char *param)
{
  char *str = param;
  uint8_t ret = true;
  const char *str_cfun = "at+cfun=0\r\n";
  int fd0;
  uint8_t n;

  fd0 = open("/dev/ttyAT", O_WRONLY);
  if (fd0 < 0)
    {
      printf("Open ttyAT failed...\n");
    }

  n = write(fd0, str_cfun, strlen(str_cfun));
  if(n != strlen(str_cfun))
  {
    printf("The number of bytes is less than specified...\n");
    ret = false;
    close(fd0);
    goto out;
  }
  close(fd0);

  str += strlen("at+pwr");
  if (*str != '=')
    {
      ret = false;
      goto out;
    }

  str++;
  if (strcmp(str,"psm") == 0)
    {
      pm_stay(PM_IDLE_DOMAIN, PM_STANDBY);
      if (pm_staycount(PM_IDLE_DOMAIN, PM_IDLE))
        {
          pm_relax(PM_IDLE_DOMAIN, PM_IDLE);
        }
      if (pm_staycount(PM_IDLE_DOMAIN, PM_NORMAL))
        {
          pm_relax(PM_IDLE_DOMAIN, PM_NORMAL);
        }
    }
  else if(strcmp(str,"ds") == 0)
    {
      pm_stay(PM_IDLE_DOMAIN, PM_SLEEP);
      if (pm_staycount(PM_IDLE_DOMAIN, PM_IDLE))
        {
          pm_relax(PM_IDLE_DOMAIN, PM_IDLE);
        }
      if (pm_staycount(PM_IDLE_DOMAIN, PM_NORMAL))
        {
          pm_relax(PM_IDLE_DOMAIN, PM_NORMAL);
        }
      if (pm_staycount(PM_IDLE_DOMAIN, PM_STANDBY))
        {
          pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
        }
    }
  else
    {
      ret = false;
    }

out:
  dprintf(fd, "\r\n%s\r\n", ret >0 ? "OK" : "ERROR");
}
