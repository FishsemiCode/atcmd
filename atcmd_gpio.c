/****************************************************************************
 * external/services/atcmd/atcmd_gpio.c
 * AT cmd gpio services
 *
 *   Copyright (C) 2020 fishsemi Inc. All rights reserved.
 *   Author: fishsemi
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

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

#include "atcmd.h"

#include <nuttx/pinctrl/pinctrl.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ATCMD_GPIO_READ         (1)
#define ATCMD_GPIO_WRITE        (0)
#define ATCMD_GPIO_NAME_LENGTH  (16)

#define ATCMD_GPIO_NOPULL       (0)
#define ATCMD_GPIO_PULLUP       (1)
#define ATCMD_GPIO_PULLDOWN     (2)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpio_port_register(int port, enum gpio_pintype_e type);
static int gpio_port_read(int port, bool *read_val);
static int gpio_port_write(int port, bool write_val);
static int gpio_port_unregister(int port,  enum gpio_pintype_e type);

/****************************************************************************
 * Private variables
 ****************************************************************************/

static uint8_t g_state;

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

static int gpio_port_unregister(int port,  enum gpio_pintype_e type)
{
  char dev_name[ATCMD_GPIO_NAME_LENGTH];

  if (type == GPIO_INPUT_PIN)
    {
      snprintf(dev_name, ATCMD_GPIO_NAME_LENGTH, "/dev/gpin%u", (unsigned int)port);
    }
  else if (type == GPIO_OUTPUT_PIN)
    {
      snprintf(dev_name, ATCMD_GPIO_NAME_LENGTH, "/dev/gpout%u", (unsigned int)port);
    }

  return unregister_driver(dev_name);
}

static int gpio_port_register(int port, enum gpio_pintype_e type)
{
  char dev_name[ATCMD_GPIO_NAME_LENGTH];
  struct stat buf;
  int ret;

  /* Set mux pin to GPIO */

  PINCTRL_SELGPIO(g_pinctrl[0], port);
  if (type == GPIO_INPUT_PIN)
    {
      snprintf(dev_name, ATCMD_GPIO_NAME_LENGTH, "/dev/gpin%u", (unsigned int)port);
      IOEXP_SETDIRECTION(g_ioe[0], port, IOEXPANDER_DIRECTION_IN);
    }
  else if (type == GPIO_OUTPUT_PIN)
    {
      snprintf(dev_name, ATCMD_GPIO_NAME_LENGTH, "/dev/gpout%u", (unsigned int)port);
      IOEXP_SETDIRECTION(g_ioe[0], port, IOEXPANDER_DIRECTION_OUT);
    }

  ret = stat(dev_name, &buf);
  if (ret == 0)
    {
      return ret;
    }

  return gpio_lower_half(g_ioe[0], (unsigned int)port, type, port);
}

static int gpio_port_read(int port, bool *read_val)
{
  int fd, ret;
  char dev_name[ATCMD_GPIO_NAME_LENGTH];

  snprintf(dev_name, ATCMD_GPIO_NAME_LENGTH, "/dev/gpin%u", (unsigned int)port);

  /* Set specify GPIO state */

  if (g_state == ATCMD_GPIO_PULLUP)
    PINCTRL_SETDT(g_pinctrl[0], port, BIAS_PULLUP);
  else if(g_state == ATCMD_GPIO_PULLDOWN)
    PINCTRL_SETDT(g_pinctrl[0], port, BIAS_PULLDOWN);
  else
    PINCTRL_SETDT(g_pinctrl[0], port, BIAS_DISABLE);

  fd = open(dev_name, O_RDONLY);
  if (fd < 0)
    {
      printf("open %s failed\n", dev_name);
      return -ENXIO;
    }

  ret = ioctl(fd, GPIOC_READ, (unsigned long)read_val);
  if (ret)
    {
      printf("%s:P%u failed\n", __func__, (unsigned int)port);
    }

  close(fd);
  return ret;
}

static int gpio_port_write(int port, bool write_val)
{
  int fd, ret;
  char dev_name[ATCMD_GPIO_NAME_LENGTH];

  snprintf(dev_name, ATCMD_GPIO_NAME_LENGTH, "/dev/gpout%u", (unsigned int)port);

  fd = open(dev_name, O_RDONLY);
  if (fd < 0)
    {
      printf("open %s failed\n", dev_name);
      return -ENXIO;
    }

  ret = ioctl(fd, GPIOC_WRITE, (unsigned long)write_val);
  if (ret)
    {
      printf("%s:P%u failed\n", __func__, (unsigned int)port);
    }

  close(fd);
  return ret;
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

void atcmd_gpio_handler(int fd, const char *cmd, char *param)
{
  char *str = param;
  char *oldstr;
  uint8_t level = 0;
  uint32_t gpio;
  uint8_t dir;
  int ret = -EINVAL;
  bool gpio_val = false;

  str += strlen("at+gpio");
  if (*str != '=')
    {
      goto out;
    }

  if (!(strchr(str, '?')) && !(strchr(str, ',')))
    {
      goto out;
    }

  oldstr = ++str;

  if ((str = strchr(str, ',')) != NULL)
    {
      dir = ATCMD_GPIO_WRITE;

      /* get gpio pin level */

      level = atoi(++str);
      if (level == 1 || level == 0)
        {
          str--;
          *str = '\0';
        }
      else
        goto out;
    }

  str = oldstr;

  if ((str = strchr(str, '?')) != NULL)
    {
      dir = ATCMD_GPIO_READ;
      str++;
      if (strcmp(str,"pullup") == 0)
        {
          g_state = ATCMD_GPIO_PULLUP;
        }
      else if(strcmp(str,"pulldown") == 0)
        {
          g_state = ATCMD_GPIO_PULLDOWN;
        }
      else if(strcmp(str,"") == 0 || strcmp(str,"nopull") ==0)
        {
          g_state = ATCMD_GPIO_NOPULL;
        }
      else
        {
          goto out;
        }
      *str = '\0';
    }

  /* get gpio number */

  gpio = atoi(oldstr);
  if (gpio >= 45 || gpio < 0)
    {
      goto out;
    }

  if (dir == ATCMD_GPIO_READ)
    {
      ret = gpio_port_register(gpio, GPIO_INPUT_PIN);
      ret |= gpio_port_read(gpio, &gpio_val);
      dprintf(fd, "\r\ngpio%d read val = %d \r\n", gpio, gpio_val);
      ret = gpio_port_unregister(gpio, GPIO_INPUT_PIN);
    }
  else
    {
      ret = gpio_port_register(gpio, GPIO_OUTPUT_PIN);
      gpio_val = (bool)level;
      ret |= gpio_port_write(gpio, gpio_val);
      dprintf(fd, "\r\ngpio%d write = %d \r\n", gpio, gpio_val);
      ret = gpio_port_unregister(gpio, GPIO_OUTPUT_PIN);
    }

out:
  dprintf(fd, "\r\n%s\r\n", ret >=0 ? "OK" : "ERROR");
}

