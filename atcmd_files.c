/****************************************************************************
 * external/services/atcmd/atcmd_files.c
 * AT cmd files services
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
#include <netutils/base64.h>

#include "atcmd.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void atcmd_file_open_handler(int fd, const char *cmd, char *param);
static void atcmd_file_close_handler(int fd, const char *cmd, char *param);
static void atcmd_file_read_handler(int fd, const char *cmd, char *param);
static void atcmd_file_write_handler(int fd, const char *cmd, char *param);
static void atcmd_file_flush_handler(int fd, const char *cmd, char *param);
static void atcmd_file_seek_handler(int fd, const char *cmd, char *param);
static void atcmd_file_unlink_handler(int fd, const char *cmd, char *param);
static void atcmd_file_base_handler(int fd, const char *cmd, char *param);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct atcmd_table_ext_s g_atcmd_file[] =
{
  {"AT+PFUNLINK", atcmd_file_unlink_handler},
  {"AT+PFCLOSE",  atcmd_file_close_handler},
  {"AT+PFFLUSH",  atcmd_file_flush_handler},
  {"AT+PFSEEK",   atcmd_file_seek_handler},
  {"AT+PFWRITE",  atcmd_file_write_handler},
  {"AT+PFOPEN",   atcmd_file_open_handler},
  {"AT+PFREAD",   atcmd_file_read_handler},
  {"AT+PF",       atcmd_file_base_handler},
};

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

static void atcmd_file_open_handler(int fd, const char *cmd, char *param)
{
  int ret = -EINVAL;
  FILE *fp = NULL;
  char *type;

  if (*param++ != '=')
    {
      goto err;
    }

  type = strchr(param, ',');
  if (!type)
    {
      goto err;
    }

  *type++ = '\0';

  fp = fopen(param, type);
  if (!fp)
    {
      ret = -errno;
    }
  else
    {
      ret = 0;
    }

err:
  dprintf(fd, "\r\n%s:%#p,%d\r\n", cmd, fp, ret);
}

static void atcmd_file_close_handler(int fd, const char *cmd, char *param)
{
  int ret = -EINVAL;
  FILE *fp = NULL;

  if (*param++ != '=')
    {
      goto err;
    }

  fp = (FILE *)(uintptr_t)strtoul(param, NULL, 0);
  if (!fp)
    {
      goto err;
    }

  ret = fclose(fp);
  if (ret)
    {
      ret = -errno;
    }

err:
  dprintf(fd, "\r\n%s:%#p,%d\r\n", cmd, fp, ret);
}

static void atcmd_file_read_handler(int fd, const char *cmd, char *param)
{
  int ret = -EINVAL;
  void *tmp = NULL;
  void *buf = NULL;
  FILE *fp = NULL;
  size_t len;

  if (*param++ != '=')
    {
      goto err;
    }

  fp = (FILE *)(uintptr_t)strtoul(param, &param, 0);
  if (!fp)
    {
      goto err;
    }

  if (*param++ != ',')
    {
      goto err;
    }

  len = base64_decode_length(strtoul(param, NULL, 0));
  buf = malloc(len);
  if (!buf)
    {
      ret = -ENOMEM;
      goto err;
    }

  ret = fread(buf, 1, len, fp);
  if (ret < 0)
    {
      ret = -errno;
      goto err;
    }

  tmp = base64_encode(buf, ret, NULL, &len);
  if (!tmp)
    {
      ret = -ENOMEM;
    }

err:
  if (ret < 0)
    {
      dprintf(fd, "\r\n%s:%#p,%d\r\n", cmd, fp, ret);
    }
  else
    {
      dprintf(fd, "\r\n%s:%#p,%d", cmd, fp, len);
      if (len)
        {
          dprintf(fd, ",");
          atcmd_safe_write(fd, tmp, len);
        }
      dprintf(fd, "\r\n");
    }

  if (buf) free(buf);
  if (tmp) free(tmp);
}

static void atcmd_file_write_handler(int fd, const char *cmd, char *param)
{
  int ret = -EINVAL;
  FILE *fp = NULL;
  size_t len;

  if (*param++ != '=')
    {
      goto err;
    }

  fp = (FILE *)(uintptr_t)strtoul(param, &param, 0);
  if (!fp)
    {
      goto err;
    }

  if (*param++ != ',')
    {
      goto err;
    }

  base64_decode(param, strlen(param), param, &len);

  ret = fwrite(param, 1, len, fp);
  if (ret >= 0)
    {
      ret = base64_encode_length(ret);
    }
  else
    {
      ret = -errno;
    }

err:
  dprintf(fd, "\r\n%s:%#p,%d\r\n", cmd, fp, ret);
}

static void atcmd_file_flush_handler(int fd, const char *cmd, char *param)
{
  int ret = -EINVAL;
  FILE *fp = NULL;

  if (*param++ != '=')
    {
      goto err;
    }

  fp = (FILE *)(uintptr_t)strtoul(param, NULL, 0);
  if (!fp)
    {
      goto err;
    }

  ret = fflush(fp);
  if (ret)
    {
      ret = -errno;
    }

err:
  dprintf(fd, "\r\n%s:%#p,%d\r\n", cmd, fp, ret);
}

static void atcmd_file_seek_handler(int fd, const char *cmd, char *param)
{
  off_t offset = -EINVAL;
  FILE *fp = NULL;
  int where;

  if (*param++ != '=')
    {
      goto err;
    }

  fp = (FILE *)(uintptr_t)strtoul(param, &param, 0);
  if (!fp)
    {
      goto err;
    }

  if (*param++ != ',')
    {
      goto err;
    }

  offset = strtol(param, &param, 0);

  if (*param++ != ',')
    {
      goto err;
    }

  where = strtol(param, NULL, 0);

  offset = fseek(fp, offset, where);
  if (offset < 0)
    {
      offset = -errno;
    }

err:
  dprintf(fd, "\r\n%s:%p,%d\r\n", cmd, fp, offset);
}

static void atcmd_file_unlink_handler(int fd, const char *cmd, char *param)
{
  int ret = -EINVAL;

  if (*param++ != '=')
    {
      goto err;
    }

  ret = unlink(param);
  if (ret)
    {
      ret = -errno;
    }

err:
  dprintf(fd, "\r\n%s:%d\r\n", cmd, ret);
}

static void atcmd_file_base_handler(int fd, const char *cmd, char *param)
{
  int i;

  dprintf(fd, "\r\n+PF Support List:\r\n");

  for (i = 0; i < ARRAY_SIZE(g_atcmd_file); i++)
    {
      dprintf(fd, "%s\r\n", g_atcmd_file[i].cmd);
    }
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

void atcmd_files_handler(int fd, const char *cmd, char *param)
{
  int i, prefix;

  for (i = 0; i < ARRAY_SIZE(g_atcmd_file); i++)
    {
      prefix = strlen(g_atcmd_file[i].cmd);
      if (strncasecmp(param, g_atcmd_file[i].cmd, prefix) == 0)
        {
          g_atcmd_file[i].handler(fd, &g_atcmd_file[i].cmd[2], param + prefix);
          break;
        }
    }

  if (i == ARRAY_SIZE(g_atcmd_file))
    {
      dprintf(fd, "\r\nERROR\r\n");
    }
}
