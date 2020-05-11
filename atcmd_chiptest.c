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

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mount.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <nuttx/pinctrl/pinctrl.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/i2c/tca6424a.h>

#include "atcmd.h"

/****************************************************************************
 * Macro definition
 ****************************************************************************/

#define PTEST_SPI_FILE_CONTENT "chip test spi"

#define GPIO_DS_LEVEL0         0
#define GPIO_DS_LEVEL1         1
#define GPIO_DS_LEVEL2         2
#define GPIO_DS_LEVEL3         3

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct atcmd_ptest_param_s
{
  int para1;
  int para2;
  int para3;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void atcmd_ptest_spi_handler(int fd, const char *cmd, char *param);
static void atcmd_ptest_i2c_handler(int fd, const char *cmd, char *param);
static void atcmd_ptest_pin_handler(int fd, const char *cmd, char *param);
static void atcmd_ptest_do_handler(int fd, const char *cmd, char *param);
static void atcmd_ptest_base_handler(int fd, const char *cmd, char *param);

static int atcmd_ptest_write_file(char *path, char *content, int len);
static int atcmd_ptest_read_file(char *path, char *content, int len);
static int atcmd_ptest_parse_param(struct atcmd_ptest_param_s *param, char *str);
static int atcmd_ptest_parse_int(int *out, char *str);
static int atcmd_ptest_pin_proc(int gpio, int i2cbus, int port);
static int atcmd_ptest_i2c_proc(int i2cbus);
static int atcmd_ptest_do_proc(int i2cbus, int port, int expect);
static int atcmd_ptest_muxpin_selgpio(int gpio);
static int atcmd_ptest_muxpin_drvtype(int gpio, enum pinctrl_drivertype_e type);
static int atcmd_ptest_muxpin_drvstrength(int gpio, int level);
static int atcmd_ptest_gpio_initialize(int gpio, int level,
                                       enum pinctrl_drivertype_e drvtype,
                                       enum gpio_pintype_e pintype);
static int atcmd_ptest_gpio_write(int gpio, int val);
static int atcmd_ptest_expio_initialize(int i2cbus, int port,
                                        enum tca6424a_direction_e dir);
static int atcmd_ptest_expio_querydir(int i2cbus, int port);
static int atcmd_ptest_expio_read(int i2cbus, int port);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct atcmd_table_ext_s g_atcmd_ptest[] =
{
  {"AT+PTESTSPI",    atcmd_ptest_spi_handler},
  {"AT+PTESTIIC",    atcmd_ptest_i2c_handler},
  {"AT+PTESTPIN",    atcmd_ptest_pin_handler},
  {"AT+PTESTDO",     atcmd_ptest_do_handler},
  {"AT+PTEST",       atcmd_ptest_base_handler},
};

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

static void atcmd_ptest_spi_handler(int fd, const char *cmd, char *param)
{
  char read_str[sizeof(PTEST_SPI_FILE_CONTENT)] = {'\0'};
  const char *target = "/ctpersist";
  int ret = -EINVAL;
  FILE *fp = NULL;

  if ('=' != *param++)
    {
      goto end;
    }

  if ('/' != *param)
    {
      goto end;
    }

  ret = mount("/dev/ctdata", target, "littlefs", 0, "autoformat");
  if (ret)
    {
      ret = 100;
      goto end;
    }

  if (NULL != (fp = fopen(param, "r")))
    {
      fclose(fp);
      umount(target);
      dprintf(fd, "\r\nERROR(file exist)\r\n");
      return;
    }

  ret = atcmd_ptest_write_file(param, PTEST_SPI_FILE_CONTENT, strlen(PTEST_SPI_FILE_CONTENT));
  if (ret < 0)
    {
      goto endp;
    }

  ret = atcmd_ptest_read_file(param, read_str, strlen(PTEST_SPI_FILE_CONTENT));
  unlink(param);
  if (ret < 0)
    {
      goto endp;
    }

  ret = memcmp(read_str, PTEST_SPI_FILE_CONTENT, strlen(PTEST_SPI_FILE_CONTENT));

endp:
  umount(target);
end:
  if (0 == ret)
    {
      dprintf(fd, "\r\nOK\r\n");
    }
  else
    {
      dprintf(fd, "\r\nERROR(%d)\r\n", ret);
    }
}

static void atcmd_ptest_i2c_handler(int fd, const char *cmd, char *param)
{
  int i2cbus, ret;

  ret = atcmd_ptest_parse_int(&i2cbus, param);
  if (!ret)
    {
      ret = atcmd_ptest_i2c_proc(i2cbus);
    }
  else if (ret > 0)
    {
      dprintf(fd, "\r\n%s=<I2C BUS>\r\n", cmd);
      return;
    }

  if (!ret)
    {
      dprintf(fd, "\r\nOK\r\n");
    }
  else
    {
      dprintf(fd, "\r\nERROR(%d)\r\n", ret);
    }
}

static void atcmd_ptest_pin_handler(int fd, const char *cmd, char *param)
{
  struct atcmd_ptest_param_s paraset;
  int ret;

  ret = atcmd_ptest_parse_param(&paraset, param);
  if (!ret)
    {
      ret = atcmd_ptest_pin_proc(paraset.para1, paraset.para2, paraset.para3);
    }
  else if (ret > 0)
    {
      dprintf(fd, "\r\n%s=<GPIO>,<I2C BUS>,<EXPANDER PORT>\r\n", cmd);
      return;
    }

  if (!ret)
    {
      dprintf(fd, "\r\nOK\r\n");
    }
  else
    {
      dprintf(fd, "\r\nERROR(%d)\r\n", ret);
    }
}

static void atcmd_ptest_do_handler(int fd, const char *cmd, char *param)
{
  struct atcmd_ptest_param_s paraset;
  int ret;

  ret = atcmd_ptest_parse_param(&paraset, param);
  if (!ret)
    {
      ret = atcmd_ptest_do_proc(paraset.para1, paraset.para2, paraset.para3);
    }
  else if (ret > 0)
    {
      dprintf(fd, "\r\n%s=<I2C BUS>,<EXPANDER PORT>,<EXPECT VALUE>\r\n", cmd);
      return;
    }

  if (!ret)
    {
      dprintf(fd, "\r\nOK\r\n");
    }
  else
    {
      dprintf(fd, "\r\nERROR(%d)\r\n", ret);
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

static int atcmd_ptest_parse_param(struct atcmd_ptest_param_s *param, char *str)
{
  if ('?' == *str)
    {
      return 1;
    }
  else if ('=' != *str)
    {
      return -EINVAL;
    }

  str++;
  if ('\0' == *str)
    {
      return -EINVAL;
    }
  else if ('?' == *str)
    {
      return 1;
    }

  param->para1 = atoi(str);
  str = strstr(str, ",");
  if (!str)
    {
      return -EINVAL;
    }
  param->para2 = atoi(++str);
  str = strstr(str, ",");
  if (!str)
    {
      return -EINVAL;
    }
  param->para3 = atoi(++str);
  return 0;
}

static int atcmd_ptest_parse_int(int *out, char *str)
{
  if ('?' == *str)
    {
      return 1;
    }
  else if ('=' != *str)
    {
      return -EINVAL;
    }

  str++;
  if ('\0' == *str)
    {
      return -EINVAL;
    }
  else if ('?' == *str)
    {
      return 1;
    }

  *out = atoi(str);
  return 0;
}

static int atcmd_ptest_pin_proc(int gpio, int i2cbus, int port)
{
  int ret, i = 0;

  ret = atcmd_ptest_expio_initialize(i2cbus, port, TCA_IO_IN);
  if (ret)
    return 100;
  ret = atcmd_ptest_gpio_initialize(gpio, GPIO_DS_LEVEL0,
                                    BIAS_DISABLE, GPIO_OUTPUT_PIN);
  if (ret)
    return 101;

  do
    {
      if (atcmd_ptest_gpio_write(gpio, i & 0x1))
        return 102;

      usleep(10000);
      ret = atcmd_ptest_expio_read(i2cbus, port);
      if (ret < 0)
        return 103;

      if (ret != (i & 0x1))
        break;
    } while (++i < 3);

  return (i >= 3) ? 0 : 104;
}

static int atcmd_ptest_i2c_proc(int i2cbus)
{
  int ret;

  ret = atcmd_ptest_expio_initialize(i2cbus, 0, TCA_IO_IN);
  if (ret)
    return 100;
  ret = atcmd_ptest_expio_querydir(i2cbus, 0);
  if (ret < 0)
    return 101;

  return (ret == TCA_IO_IN) ? 0 : 102;
}

static int atcmd_ptest_do_proc(int i2cbus, int port, int expect)
{
  int ret;

  if (expect && expect != 1)
    return -EINVAL;

  ret = atcmd_ptest_expio_initialize(i2cbus, port, TCA_IO_IN);
  if (ret)
    return 100;

  usleep(10000);
  ret = atcmd_ptest_expio_read(i2cbus, port);
  if (ret < 0)
    return 101;

  return (ret == expect) ? 0 : 102;
}

static int atcmd_ptest_muxpin_selgpio(int gpio)
{
  int fd, ret;

  fd = open("/dev/pinctrl0", 0);
  if (fd < 0)
    {
      return -ENXIO;
    }

  ret = ioctl(fd, PINCTRLC_SELGPIO, (unsigned long)gpio);

  close(fd);
  return ret;
}

static int atcmd_ptest_muxpin_drvtype(int gpio, enum pinctrl_drivertype_e type)
{
  struct pinctrl_iotrans_s trans;
  int fd, ret;

  fd = open("/dev/pinctrl0", 0);
  if (fd < 0)
    {
      return -ENXIO;
    }

  trans.pin = gpio;
  trans.para.type = type;

  ret = ioctl(fd, PINCTRLC_SETDT, (unsigned long)&trans);

  close(fd);
  return ret;
}

static int atcmd_ptest_muxpin_drvstrength(int gpio, int level)
{
  struct pinctrl_iotrans_s trans;
  int fd, ret;

  fd = open("/dev/pinctrl0", 0);
  if (fd < 0)
    {
      return -ENXIO;
    }

  trans.pin = gpio;
  trans.para.level = level;

  ret = ioctl(fd, PINCTRLC_SETDS, (unsigned long)&trans);

  close(fd);
  return ret;
}

static int atcmd_ptest_gpio_initialize(int gpio, int level,
                                       enum pinctrl_drivertype_e drvtype,
                                       enum gpio_pintype_e pintype)
{
  int ret = 0;

  if (gpio >= 4)
    {
      ret = atcmd_ptest_muxpin_selgpio(gpio);
      ret |= atcmd_ptest_muxpin_drvstrength(gpio, level);
      ret |= atcmd_ptest_muxpin_drvtype(gpio, drvtype);
    }

  if (!ret)
    {
      ret = gpio_lower_half(g_ioe[0], gpio, pintype, gpio);
    }

  return ret;
}

static int atcmd_ptest_gpio_write(int gpio, int val)
{
  char dev_name[16];
  int fd, ret;

  snprintf(dev_name, 16, "/dev/gpout%u", gpio);
  fd = open(dev_name, 0);
  if (fd < 0)
    {
      return -ENXIO;
    }

  ret = ioctl(fd, GPIOC_WRITE, (unsigned long)val);

  close(fd);
  return ret;
}

static int atcmd_ptest_expio_initialize(int i2cbus, int port,
                                        enum tca6424a_direction_e dir)
{
  struct tca6424a_iotrans_s trans;
  char dev_name[20];
  int fd, ret;

  snprintf(dev_name, 20, "/dev/tca6424a%u", i2cbus);
  fd = open(dev_name, 0);
  if (fd < 0)
    {
      return -ENXIO;
    }

  trans.pin = port;
  trans.para.type = dir;

  ret = ioctl(fd, TCA6424A_SETTYPE, (unsigned long)&trans);

  close(fd);
  return ret;
}

static int atcmd_ptest_expio_querydir(int i2cbus, int port)
{
  struct tca6424a_iotrans_s trans;
  char dev_name[20];
  int fd, ret;

  snprintf(dev_name, 20, "/dev/tca6424a%u", i2cbus);
  fd = open(dev_name, 0);
  if (fd < 0)
    {
      return -ENXIO;
    }

  trans.pin = port;

  ret = ioctl(fd, TCA6424A_READTYPE, (unsigned long)&trans);

  close(fd);
  return ret ? -EIO : trans.para.type;
}

static int atcmd_ptest_expio_read(int i2cbus, int port)
{
  struct tca6424a_iotrans_s trans;
  char dev_name[20];
  int fd, ret;

  snprintf(dev_name, 20, "/dev/tca6424a%u", i2cbus);
  fd = open(dev_name, 0);
  if (fd < 0)
    {
      return -ENXIO;
    }

  trans.pin = port;

  ret = ioctl(fd, TCA6424A_READPIN, (unsigned long)&trans);

  close(fd);
  return ret ? -EIO : trans.para.state;
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
