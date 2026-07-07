/*
 * Copyright (c) 2013-2015, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <libudev.h>
#include "edbg.h"
#include "dbg.h"

/*- Variables ---------------------------------------------------------------*/
static int debugger_fd = -1;
static uint8_t hid_buffer[DBG_MAX_EP_SIZE + 1];
static int report_size = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int dbg_enumerate(debugger_t *debuggers, int size)
{
  struct udev *udev;
  struct udev_enumerate *enumerate;
  struct udev_list_entry *devices, *dev_list_entry;
  struct udev_device *dev, *parent;
  int rsize = 0;

  udev = udev_new();
  check(udev, "unable to create udev object");

  enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "hidraw");
  udev_enumerate_scan_devices(enumerate);
  devices = udev_enumerate_get_list_entry(enumerate);

  udev_list_entry_foreach(dev_list_entry, devices)
  {
    const char *path;

    path = udev_list_entry_get_name(dev_list_entry);
    dev = udev_device_new_from_syspath(udev, path);

    parent = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");

    if (NULL == parent)
      continue;

    if (rsize < size)
    {
      const char *serial = udev_device_get_sysattr_value(parent, "serial");
      const char *manufacturer = udev_device_get_sysattr_value(parent, "manufacturer");
      const char *product = udev_device_get_sysattr_value(parent, "product");

      debuggers[rsize].path = strdup(udev_device_get_devnode(dev));
      debuggers[rsize].serial = serial ? strdup(serial) : "<unknown>";
      debuggers[rsize].manufacturer = manufacturer ? strdup(manufacturer) : "<unknown>";
      debuggers[rsize].product = product ? strdup(product) : "<unknown>";
      debuggers[rsize].vid = strtol(udev_device_get_sysattr_value(parent, "idVendor"), NULL, 16);
      debuggers[rsize].pid = strtol(udev_device_get_sysattr_value(parent, "idProduct"), NULL, 16);

      if (strstr(debuggers[rsize].product, "CMSIS-DAP"))
        rsize++;
    }

    udev_device_unref(parent);
  }

  udev_enumerate_unref(enumerate);
  udev_unref(udev);

  return rsize;
}

//-----------------------------------------------------------------------------
static int parse_hid_report_desc(uint8_t *data, int size)
{
  uint32_t count = 0;
  uint32_t input = 0;
  uint32_t output = 0;

  // This is a very primitive parser, but CMSIS-DAP descriptors are pretty uniform
  for (int i = 0; i < size; )
  {
    int prefix = data[i++];
    int bTag = (prefix >> 4) & 0x0f;
    int bType = (prefix >> 2) & 0x03;
    int bSize = prefix & 0x03;

    bSize = (3 == bSize) ? 4 : bSize;

    if (1 == bType && 9 == bTag)
    {
      count = 0;

      for (int j = 0; j < bSize; j++)
        count |= (data[i + j] << (j * 8));
    }
    else if (0 == bType && 8 == bTag)
      input = count;
    else if (0 == bType && 9 == bTag)
      output = count;

    i += bSize;
  }

  if (input != output)
    error_exit("input and output report sizes do not match");

  if (64 != input && 512 != input && 1024 != input)
    error_exit("detected report size (%d) is not 64, 512 or 1024", input);

  return input;
}

//-----------------------------------------------------------------------------
void dbg_open(debugger_t *debugger)
{
  struct hidraw_report_descriptor rpt_desc;
  struct hidraw_devinfo info;
  int desc_size, res;

  debugger_fd = open(debugger->path, O_RDWR);

  if (debugger_fd < 0)
    error_exit("unable to open device %s: %s",  debugger->path, strerror(errno));

  // Exclusive advisory lock on the probe: two concurrent sessions (a second
  // edbg, or an oc mux/power command mid-flash) interleave commands on the
  // same USB pipe and corrupt both the protocol stream and the target flash.
  // flock() is held for the life of this process and released by the kernel
  // on any exit, so it cannot go stale. Grace period lets a quick command
  // (mux set, identify) finish before we give up.
  // EDBG_LOCK_INHERITED=1: our parent (oc) already holds the lock and keeps
  // it for our whole lifetime (mux/power + flash as one atomic sequence);
  // taking it here would deadlock against our own caller.
  for (int tries = 0; (NULL == getenv("EDBG_LOCK_INHERITED")) &&
      flock(debugger_fd, LOCK_EX | LOCK_NB) < 0; tries++)
  {
    if (tries >= 20)
    {
      // Not error_exit(): its cleanup tries to send a disconnect command to
      // the device we never initialized (report_size is still 0).
      fprintf(stderr, "Error: probe %s is in use by another session "
          "(edbg or oc); refusing to interleave commands\n", debugger->path);
      close(debugger_fd);
      exit(1);
    }
    usleep(100 * 1000);
  }

  memset(&rpt_desc, 0, sizeof(rpt_desc));
  memset(&info, 0, sizeof(info));

  res = ioctl(debugger_fd, HIDIOCGRDESCSIZE, &desc_size);
  if (res < 0)
    perror_exit("debugger ioctl()");

  rpt_desc.size = desc_size;
  res = ioctl(debugger_fd, HIDIOCGRDESC, &rpt_desc);
  if (res < 0)
    perror_exit("debugger ioctl()");

  report_size = parse_hid_report_desc(rpt_desc.value, rpt_desc.size);

  // Drain any stale input reports (e.g. responses to pipelined requests from a
  // session that exited mid-window), so the first command of this session does
  // not consume a leftover response.
  int flags = fcntl(debugger_fd, F_GETFL, 0);
  fcntl(debugger_fd, F_SETFL, flags | O_NONBLOCK);
  while (read(debugger_fd, hid_buffer, sizeof(hid_buffer)) > 0);
  fcntl(debugger_fd, F_SETFL, flags);
}

//-----------------------------------------------------------------------------
void dbg_close(void)
{
  if (debugger_fd)
    close(debugger_fd);
}

//-----------------------------------------------------------------------------
int dbg_get_report_size(void)
{
  return report_size;
}

//-----------------------------------------------------------------------------
int dbg_dap_cmd(uint8_t *data, int resp_size, int req_size)
{
  uint8_t cmd = data[0];
  int res;

  memset(hid_buffer, 0xff, report_size + 1);

  hid_buffer[0] = 0x00; // Report ID
  memcpy(&hid_buffer[1], data, req_size);

  res = write(debugger_fd, hid_buffer, report_size + 1);
  if (res < 0)
    perror_exit("debugger write()");

  res = read(debugger_fd, hid_buffer, report_size + 1);
  if (res < 0)
    perror_exit("debugger read()");

  check(res, "empty response received");

  check(hid_buffer[0] == cmd, "invalid response received");

  res--;
  memcpy(data, &hid_buffer[1], (resp_size < res) ? resp_size : res);

  return res;
}

//-----------------------------------------------------------------------------
void dbg_dap_cmd_submit(uint8_t *data, int req_size)
{
  int res;

  memset(hid_buffer, 0xff, report_size + 1);

  hid_buffer[0] = 0x00; // Report ID
  memcpy(&hid_buffer[1], data, req_size);

  res = write(debugger_fd, hid_buffer, report_size + 1);
  if (res < 0)
    perror_exit("debugger write()");
}

//-----------------------------------------------------------------------------
// Like dbg_dap_cmd_reap but returns -1 on a mismatched/empty response instead
// of exiting -- lets idempotent bulk transfers drain and retry transients.
int dbg_dap_cmd_reap_try(uint8_t cmd, uint8_t *data, int resp_size)
{
  int res;

  res = read(debugger_fd, hid_buffer, report_size + 1);
  if (res <= 0)
    return -1;

  if (hid_buffer[0] != cmd)
    return -1;

  res--;
  memcpy(data, &hid_buffer[1], (resp_size < res) ? resp_size : res);

  return res;
}

//-----------------------------------------------------------------------------
int dbg_dap_cmd_reap(uint8_t cmd, uint8_t *data, int resp_size)
{
  int res;

  res = read(debugger_fd, hid_buffer, report_size + 1);
  if (res < 0)
    perror_exit("debugger read()");

  check(res, "empty response received");

  check(hid_buffer[0] == cmd, "invalid response received");

  res--;
  memcpy(data, &hid_buffer[1], (resp_size < res) ? resp_size : res);

  return res;
}

