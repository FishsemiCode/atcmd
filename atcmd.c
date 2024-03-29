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

#include <ctype.h>
#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>

#include "atcmd.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifdef CONFIG_SERVICES_SOFTSIM
#define ATCMD_BUFMAX            3072
#else
#define ATCMD_BUFMAX            1024
#endif

#define ATCMD_UART_SERIAL        0
#define ATCMD_UART_GPS           1
#define ATCMD_UART_MODEM         2
#define ATCMD_UART_APP           3
#define ATCMD_UART_S2            4
#define ATCMD_UART_S3            5
#define ATCMD_UART_SP            6

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
#if defined(CONFIG_SERVICES_ATCMD_CHIP_TEST) && (CONFIG_16550_UART2_BAUD == 9600)
static int atcmd_serial_chiptest_handler(struct atcmd_uart_s *serial);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_names[] =
{
  "/dev/ttyS0",     // 0, ATCMD_UART_SERIAL
  "/dev/ttyGPS",    // 1, ATCMD_UART_GPS
  "/dev/ttyAT",     // 2, ATCMD_UART_MODEM
  "/dev/pty0",      // 3, ATCMD_UART_APP

#if defined(CONFIG_SERVICES_ATCMD_CHIP_TEST) && (CONFIG_16550_UART2_BAUD == 9600)
  "/dev/ttyS2",     // 4, ATCMD_UART_S2
  "/dev/ttyS3",     // 5, ATCMD_UART_S3
#endif

#ifdef CONFIG_SOFTSIM_ON_CHIP_SP
  "/dev/ttyAT2",    // 6, ATCMD_UART_SP
#endif
};

#define ATCMD_NUARTS ARRAY_SIZE(g_names)

static struct atcmd_uart_s g_uarts[ATCMD_NUARTS];

static const struct atcmd_table_s g_atcmd[] =
{
  {"AT+PFLUSH", atcmd_flush_handler,   ATCMD_UART_SERIAL},
  {"AT+NPING6", atcmd_nping6_handler,   ATCMD_UART_SERIAL},
  {"AT+NPING",  atcmd_nping_handler,    ATCMD_UART_SERIAL},
  {"AT+TPING",  atcmd_tping_handler,    ATCMD_UART_SERIAL},
  {"AT+PCOAP",  atcmd_coap_handler,     ATCMD_UART_SERIAL},
  {"AT+PHTTP",  atcmd_http_handler,     ATCMD_UART_SERIAL},
  {"AT+PGNSS",  atcmd_remote_handler,   ATCMD_UART_GPS},
  {"AT+PON"  ,  atcmd_pon_handler,      ATCMD_UART_SERIAL},
  {"AT+PWR"  ,  atcmd_pwr_handler,      ATCMD_UART_SERIAL},
#ifdef CONFIG_SERVICES_ATCMD_CHIP_TEST
  {"AT+PTEST",  atcmd_ptest_handler,    ATCMD_UART_SERIAL},
#endif
  {"AT+CCLK",   atcmd_cclk_handler,     ATCMD_UART_SERIAL},
#ifdef CONFIG_SERVICES_SOFTSIM
  {"AT+ESIM",   atcmd_esim_handler,     ATCMD_UART_SERIAL},
#elif defined(CONFIG_SOFTSIM_ON_CHIP_SP)
  {"AT+ESIM",   atcmd_remote_handler,   ATCMD_UART_SP},
#endif
  {"AT+GPIO",   atcmd_gpio_handler,     ATCMD_UART_SERIAL},
  {"AT+PENV",   atcmd_env_handler,      ATCMD_UART_SERIAL},
  {"AT+PSSL",   atcmd_ssl_handler,      ATCMD_UART_SERIAL},
  {"AT+TRB",    atcmd_trb_handler,      ATCMD_UART_SERIAL},
  {"AT+TSO",    atcmd_tso_handler,      ATCMD_UART_SERIAL},
  {"AT+IFC",    atcmd_ifc_handler,      ATCMD_UART_SERIAL},
  {"AT+IPR",    atcmd_ipr_handler,      ATCMD_UART_SERIAL},
  {"AT+PF",     atcmd_files_handler,    ATCMD_UART_SERIAL},
  {"AT+PMSET",  atcmd_pmset_handler,    ATCMD_UART_SERIAL},
  {"AT"CONFIG_SERVICES_ATCMD_APP_PREFIX,   atcmd_remote_handler,   ATCMD_UART_APP},
  {"AT+DMCFG",  atcmd_dmcfg_handler,    ATCMD_UART_SERIAL},
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

          syslog(LOG_INFO, "atcmd rx buf:%s\n", pbuf);

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

static int atcmd_check_error_char(char *buf, int len)
{
  int i;

  for (i = 0; i < len; i++)
    {
      if (!isprint(buf[i]) && !isspace(buf[i]))
        {
          return 1;
        }
    }

  return 0;
}

#if defined(CONFIG_SERVICES_ATCMD_CHIP_TEST) && (CONFIG_16550_UART2_BAUD == 9600)
static int atcmd_serial_chiptest_handler(struct atcmd_uart_s *serial)
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

          atcmd_safe_write(serial->fd, "\r\nOK\r\n", 6);

          pbuf = end + 1;
        }
      else
        {
          break;
        }
    }

  return pbuf - serial->buf;
}
#endif

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

#ifdef BUILD_MODULE
int main(int argc, char *argv[])
#else
int atcmd_main(int argc, char *argv[])
#endif
{
  struct pollfd fds[ATCMD_NUARTS];
  int fdx[ATCMD_NUARTS];
  int i, ret = -EINVAL;
  int nfds = 0;

  pty_register(0);

  memset(fds, 0, sizeof(fds));

  for (i = 0; i < ATCMD_NUARTS; i++)
    {
      if (!g_names[i])
        {
          continue;
        }

      g_uarts[i].fd = open(g_names[i], O_RDWR);
      if (g_uarts[i].fd < 0)
        {
          goto errout;
        }

      fds[nfds].fd     = g_uarts[i].fd;
      fds[nfds].events = POLLIN;
      fdx[nfds]        = i;
      nfds++;
    }

  unlockpt(g_uarts[ATCMD_UART_APP].fd);

  while (1)
    {
      ret = poll(fds, nfds, -1);
      if (ret <= 0)
        {
          continue;
        }

      for (i = 0; i < nfds; i++)
        {
          if (fds[i].revents == POLLIN)
            {
              int idx = fdx[i];
              struct atcmd_uart_s *uart = &g_uarts[idx];
              int len;

              uart->len +=
                  read(uart->fd, uart->buf + uart->len, ATCMD_BUFMAX - uart->len - 1);

              uart->buf[uart->len] = '\0';

              if (atcmd_check_error_char(uart->buf, uart->len))
                {
                  uart->len = 0;
                  break;
                }

              if (idx == ATCMD_UART_SERIAL)
                {
                  len = atcmd_serial_handler(uart);
                }
#if defined(CONFIG_SERVICES_ATCMD_CHIP_TEST) && (CONFIG_16550_UART2_BAUD == 9600)
              else if (idx == ATCMD_UART_S2 || idx == ATCMD_UART_S3)
                {
                  len = atcmd_serial_chiptest_handler(uart);
                }
#endif
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
                          __func__, idx, uart->buf);
                }
            }
        }
    }

errout:
  for (i = 0; i < nfds; i++)
    {
      int idx = fdx[i];

      if (g_uarts[idx].fd > 0)
        {
          close(g_uarts[idx].fd);
        }
    }

  return ret;
}
