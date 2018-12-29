/****************************************************************************
 * external/services/atcmd/atcmd.c
 * AT cmd services
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

#include <nuttx/ascii.h>
#include <nuttx/serial/pty.h>

#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <string.h>
#include <strings.h>

#include "atcmd.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define ATCMD_BUFMAX            1024

#define ATCMD_UART_SERIAL        0
#define ATCMD_UART_GPS           1
#define ATCMD_UART_MODEM         2
#define ATCMD_UART_APP           3
#define ATCMD_NUARTS             4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct atcmd_uart_s
{
  int  fd;
  char buf[ATCMD_BUFMAX];
  int  len;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void atcmd_remote_handler(int fd, const char *cmd, char *param);

static int atcmd_serial_handler(struct atcmd_uart_s *serial);
static int atcmd_response_handler(int outfd, struct atcmd_uart_s *uart);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct atcmd_uart_s g_uarts[ATCMD_NUARTS];

static const char *g_names[ATCMD_NUARTS] =
{
  "/dev/ttyS0",     // 0, ATCMD_UART_SERIAL
  "/dev/ttyGPS",    // 1, ATCMD_UART_GPS
  "/dev/ttyAT",     // 2, ATCMD_UART_MODEM
  "/dev/pty0",      // 3, ATCMD_UART_APP
};

static const struct atcmd_table_s g_atcmd[] =
{
  {"AT+NPING6", atcmd_nping6_handler,   ATCMD_UART_SERIAL},
  {"AT+NPING",  atcmd_nping_handler,    ATCMD_UART_SERIAL},
  {"AT+PCOAP",  atcmd_coap_handler,     ATCMD_UART_SERIAL},
  {"AT+PHTTP",  atcmd_http_handler,     ATCMD_UART_SERIAL},
  {"AT+PGNSS",  atcmd_remote_handler,   ATCMD_UART_GPS},
  {"AT+PSSL",   atcmd_ssl_handler,      ATCMD_UART_SERIAL},
  {"AT+TRB",    atcmd_trb_handler,      ATCMD_UART_SERIAL},
  {"AT+TSO",    atcmd_tso_handler,      ATCMD_UART_SERIAL},
  {"AT+IFC",    atcmd_ifc_handler,      ATCMD_UART_SERIAL},
  {"AT+IPR",    atcmd_ipr_handler,      ATCMD_UART_SERIAL},
  {"AT+PF",     atcmd_files_handler,    ATCMD_UART_SERIAL},
  {"AT+X",      atcmd_remote_handler,   ATCMD_UART_APP},
  {"AT",        atcmd_remote_handler,   ATCMD_UART_MODEM},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void atcmd_remote_handler(int fd, const char *cmd, char *param)
{
  int len = strlen(param);

  if (cmd)
    {
      param[len++] = ASCII_SUB;
    }
  else
    {
      param[len++] = '\r';
    }

  atcmd_safe_write(fd, param, len);
}

static int atcmd_serial_handler(struct atcmd_uart_s *serial)
{
  char *pbuf, *end;
  int i;

  pbuf = serial->buf;
  while (1)
    {
      if ((end = strchr(pbuf, '\r')) != NULL)
        {
          /* Process AT cmd */

          end[0] = '\0';

          pbuf = strcasestr(pbuf, "at");
          if (!pbuf)
            {
              pbuf = end + 1;
              continue;
            }

          /* Cmd handle */

          for (i = 0; i < ARRAY_SIZE(g_atcmd); i++)
            {
              if (strncasecmp(pbuf, g_atcmd[i].cmd, strlen(g_atcmd[i].cmd)) == 0)
                {
                  g_atcmd[i].handler(g_uarts[g_atcmd[i].idx].fd, NULL, pbuf);
                  break;
                }
            }

          /* Unknown cmd handle */
          syslog(LOG_INFO, "ap rx buf:%s\n", pbuf);

          if (i == ARRAY_SIZE(g_atcmd))
            {
              atcmd_remote_handler(g_uarts[ATCMD_UART_MODEM].fd, NULL, pbuf);
            }

          pbuf = end + 1;
        }
      else if ((end = strchr(pbuf, ASCII_SUB)) != NULL)
        {
          /* Process bit stream transfer with '^z' */

          end[0] = '\0';
          atcmd_remote_handler(g_uarts[ATCMD_UART_MODEM].fd, pbuf, pbuf);

          pbuf = end + 1;
        }
      else
        {
          break;
        }
    }

  return pbuf - serial->buf;
}

static int atcmd_response_handler(int fd, struct atcmd_uart_s *uart)
{
  int len;
  char *buf;

  buf = strrchr(uart->buf, '\n');
  if (buf == NULL)
    {
      return 0;
    }

  len = buf - uart->buf + 1;
  atcmd_safe_write(fd, uart->buf, len);

  return len;
}

static int atcmd_daemon(int argc, char *argv[])
{
  struct pollfd fds[ATCMD_NUARTS];
  int i, ret = -EINVAL;

  pty_register(0);

  memset(fds, 0, sizeof(fds));

  for (i = 0; i < ATCMD_NUARTS; i++)
    {
      g_uarts[i].fd = open(g_names[i], O_RDWR);
      if (g_uarts[i].fd < 0)
        {
          goto errout;
        }

      fds[i].fd     = g_uarts[i].fd;
      fds[i].events = POLLIN;
    }

  while (1)
    {
      ret = poll(fds, ATCMD_NUARTS, -1);
      if (ret <= 0)
        {
          continue;
        }

      for (i = 0; i < ATCMD_NUARTS; i++)
        {
          if (fds[i].revents == POLLIN)
            {
              struct atcmd_uart_s *uart = &g_uarts[i];
              int len;

              uart->len +=
                  read(uart->fd, uart->buf + uart->len, ATCMD_BUFMAX - uart->len - 1);

              uart->buf[uart->len] = '\0';

              if (i == ATCMD_UART_SERIAL)
                {
                  len = atcmd_serial_handler(uart);
                }
              else
                {
                  len = atcmd_response_handler(g_uarts[ATCMD_UART_SERIAL].fd, uart);
                }

              memmove(uart->buf, uart->buf + len, uart->len - len);
              uart->len -= len;

              if (uart->len + 1 == ATCMD_BUFMAX)
                {
                  uart->len = 0;
                  _err("%s: idx %d, ERROR response msg: [%s]\n",
                          __func__, i, uart->buf);
                }
            }
        }
    }

errout:
  for (i = 0; i < ATCMD_NUARTS; i++)
    {
      if (g_uarts[i].fd > 0)
        {
          close(g_uarts[i].fd);
        }
    }

  return ret;
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int atcmd_main(int argc, char *argv[])
#endif
{
  int ret;

  ret = task_create(argv[0],
          CONFIG_SERVICES_ATCMD_PRIORITY,
          CONFIG_SERVICES_ATCMD_STACKSIZE,
          atcmd_daemon,
          argv + 1);

  return ret > 0 ? 0 : ret;
}
