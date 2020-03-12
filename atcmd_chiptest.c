/****************************************************************************
 * external/services/atcmd/atcmd_chiptest.c
 * AT cmd files services
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
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

#include "atcmd.h"

/****************************************************************************
 * Macro definition
 ****************************************************************************/

#define PTEST_SPI_FILE_CONTENT "chip test spi"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void atcmd_ptest_spi_handler(int fd, const char *cmd, char *param);
static void atcmd_ptest_base_handler(int fd, const char *cmd, char *param);
static int atcmd_ptest_write_file(char *path, char *content, int len);
static int atcmd_ptest_read_file(char *path, char *content, int len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct atcmd_table_ext_s g_atcmd_ptest[] =
{
  {"AT+PTESTSPI",    atcmd_ptest_spi_handler},
  {"AT+PTEST",       atcmd_ptest_base_handler},
};

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

static void atcmd_ptest_spi_handler(int fd, const char *cmd, char *param)
{
  FILE *fp = NULL;
  int ret = -EINVAL;
  char read_str[sizeof(PTEST_SPI_FILE_CONTENT)] = {'\0'};

  if ('=' != *param++)
    {
      goto end;
    }

  if ('/' != *param)
    {
      goto end;
    }

  if (NULL != (fp = fopen(param, "r")))
    {
      fclose(fp);
      dprintf(fd, "\r\n%s:ERROR(file exist)\r\n", cmd);
      return;
    }

  ret = atcmd_ptest_write_file(param, PTEST_SPI_FILE_CONTENT, strlen(PTEST_SPI_FILE_CONTENT));
  if (ret < 0)
    {
      goto end;
    }

  ret = atcmd_ptest_read_file(param, read_str, strlen(PTEST_SPI_FILE_CONTENT));
  unlink(param);
  if (ret < 0)
    {
      goto end;
    }

  ret = memcmp(read_str, PTEST_SPI_FILE_CONTENT, strlen(PTEST_SPI_FILE_CONTENT));

end:
  if (0 == ret)
    {
      dprintf(fd, "\r\n%s:OK\r\n", cmd);
    }
  else
    {
      dprintf(fd, "\r\n%s:ERROR(%d)\r\n", cmd, ret);
    }
}

static void atcmd_ptest_base_handler(int fd, const char *cmd, char *param)
{
  int i;

  dprintf(fd, "\r\n+PTEST Support List:\r\n");

  for (i = 0; i < ARRAY_SIZE(g_atcmd_ptest); i++)
    {
      dprintf(fd, "%s\r\n", g_atcmd_ptest[i].cmd);
    }
}

static int atcmd_ptest_write_file(char *path, char *content, int len)
{
  int ret;
  int written = 0;
  FILE *fp;

  if (NULL == (fp = fopen(path, "w")))
    {
      ret = -errno;
      goto end;
    }

  while (len > 0)
    {
      ret = fwrite(content, 1, len, fp);
      if (ret <= 0)
        {
          break;
        }
      len -= ret;
      written += ret;
      content += ret;
    }

  ret = written ? written : ret;
  if (ret <= 0)
    {
      goto end;
    }

  if (0 != fflush(fp))
    {
      ret = -errno;
      goto end;
    }

end:
  if (fp)
    {
      fclose(fp);
      if (ret <= 0)
        {
          unlink(path);
        }
    }

  return ret;
}

static int atcmd_ptest_read_file(char *path, char *content, int len)
{
  int ret;
  FILE *fp;

  if (NULL == (fp = fopen(path, "r")))
    {
      ret = -errno;
      goto end;
    }

  ret = fread(content, 1, len, fp);
  if (ret != len)
    {
      ret = -errno;
      goto end;
    }

end:
  if (fp)
    {
      fclose(fp);
    }

  return ret;
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

void atcmd_ptest_handler(int fd, const char *cmd, char *param)
{
  int i, prefix;

  for (i = 0; i < ARRAY_SIZE(g_atcmd_ptest); i++)
    {
      prefix = strlen(g_atcmd_ptest[i].cmd);

      if (0 == strncasecmp(param, g_atcmd_ptest[i].cmd, prefix))
        {
          g_atcmd_ptest[i].handler(fd, &g_atcmd_ptest[i].cmd[2], param + prefix);
          break;
        }
    }

  if (i == ARRAY_SIZE(g_atcmd_ptest))
    {
      dprintf(fd, ATCMD_ACK_ERROR);
    }
}
