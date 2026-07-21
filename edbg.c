/*
 * Copyright (c) 2013-2019, Alex Taradov <alex@taradov.com>
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
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifdef _WIN32
#include <windows.h>
#endif
#include "target.h"
#include "edbg.h"
#include "dap.h"
#include "dbg.h"

/*- Definitions -------------------------------------------------------------*/
#define MAX_DEBUGGERS     20

// Long-only options start past the ASCII range so they never collide with a
// short flag letter in the getopt switch.
#define OPT_DFU           0x100
#define OPT_RTT           0x101
#define OPT_RTT_ADDR      0x102
#define OPT_RTT_SCAN      0x103
#define OPT_RTT_SCAN_LEN  0x104
#define OPT_MEMDUMP       0x105
#define OPT_FREEZE        0x106
#define OPT_V2            0x107

#ifndef O_BINARY
#define O_BINARY 0
#endif

/*- Constants ---------------------------------------------------------------*/
static const struct option long_options[] =
{
  { "help",      no_argument,        0, 'h' },
  { "verbose",   no_argument,        0, 'b' },
  { "erase",     no_argument,        0, 'e' },
  { "program",   no_argument,        0, 'p' },
  { "verify",    no_argument,        0, 'v' },
  { "lock",      no_argument,        0, 'k' },
  { "unlock",    no_argument,        0, 'u' },
  { "read",      no_argument,        0, 'r' },
  { "file",      required_argument,  0, 'f' },
  { "target",    required_argument,  0, 't' },
  { "list",      no_argument,        0, 'l' },
  { "serial",    required_argument,  0, 's' },
  { "clock",     required_argument,  0, 'c' },
  { "offset",    required_argument,  0, 'o' },
  { "size",      required_argument,  0, 'z' },
  { "fuse",      required_argument,  0, 'F' },
  { "identify",  no_argument,  0, 'i' },
  { "dfu",       no_argument,  0, OPT_DFU },
  { "rtt",       no_argument,        0, OPT_RTT },
  { "rtt-addr",  required_argument,  0, OPT_RTT_ADDR },
  { "rtt-scan",  required_argument,  0, OPT_RTT_SCAN },
  { "rtt-scan-len", required_argument, 0, OPT_RTT_SCAN_LEN },
  { "memdump",   no_argument,        0, OPT_MEMDUMP },
  { "v2",        no_argument,        0, OPT_V2 },
  { "freeze",    no_argument,        0, OPT_FREEZE },
  { 0, 0, 0, 0 }
};

static const char *short_options = "hbepvkurf:t:ls:c:o:z:F:i";

/*- Variables ---------------------------------------------------------------*/
static char *g_serial = NULL;
bool dbg_use_v2 = false;   // --v2: talk to the frog CMSIS-DAP v2 bulk interface
static bool g_list = false;
static bool g_dfu = false;
static bool g_rtt = false;
static uint32_t g_rtt_addr = 0;
static uint32_t g_rtt_scan = 0;      // scan-range base (0 = use the target driver's ram_start)
static uint32_t g_rtt_scan_len = 0;  // scan-range length in bytes (0 = use the driver's ram_size)
static bool g_memdump = false;       // --memdump: snapshot [offset, offset+size) of target memory
static bool g_freeze = false;        // --freeze: halt the core across the dump, then resume ASAP
static char *g_target = NULL;
static bool g_verbose = false;
static long g_clock = 16000000;
bool g_clock_explicit = false;   // set when the user passes -c; targets may pick a safe default otherwise
static debugger_t *g_debugger = NULL;

static target_options_t g_target_options =
{
  .erase        = false,
  .program      = false,
  .verify       = false,
  .lock         = false,
  .unlock       = false,
  .read         = false,
  .name         = NULL,
  .offset       = -1,
  .size         = -1,
  .fuse_cmd     = NULL,
  .identify      = false,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void verbose(char *fmt, ...)
{
  va_list args;

  if (g_verbose)
  {
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    fflush(stdout);
  }
}

//-----------------------------------------------------------------------------
void message(char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);

  fflush(stdout);
}

//-----------------------------------------------------------------------------
void warning(char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  fprintf(stderr, "Warning: ");
  vfprintf(stderr, fmt, args);
  fprintf(stderr, "\n");
  va_end(args);
}

//-----------------------------------------------------------------------------
static void disconnect_debugger(void)
{
  if (!g_debugger)
    return;

  g_debugger = NULL;

  dap_reset_target_hw(1);
  dap_led(0, 0);
  dap_disconnect();
  dbg_close();
}

//-----------------------------------------------------------------------------
void check(bool cond, char *fmt, ...)
{
  static bool check_failed = false;

  if (check_failed)
    return;

  if (!cond)
  {
    va_list args;

    check_failed = true;

    disconnect_debugger();

    va_start(args, fmt);
    fprintf(stderr, "Error: ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    va_end(args);

    exit(1);
  }
}

//-----------------------------------------------------------------------------
// Top-level flash retry. When armed (during the program/verify phase), a
// failure that would normally be fatal instead unwinds to main, which
// reconnects and re-runs the differential flash -- the robust way to ride out a
// multi-second line-noise burst on a marginal cable that outlasts the
// in-transfer retries (a fresh connect after the burst is far more reliable
// than mid-stream re-sync).
jmp_buf g_flash_retry_jmp;
int g_flash_retries_left = 0;

//-----------------------------------------------------------------------------
void error_exit(char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);

  if (g_flash_retries_left > 0)
  {
    fprintf(stderr, "\nWarning (will reconnect and retry): ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    va_end(args);
    longjmp(g_flash_retry_jmp, 1);
  }

  disconnect_debugger();

  fprintf(stderr, "Error: ");
  vfprintf(stderr, fmt, args);
  fprintf(stderr, "\n");
  va_end(args);

  exit(1);
}

//-----------------------------------------------------------------------------
void perror_exit(char *text)
{
  disconnect_debugger();
  perror(text);
  exit(1);
}

//-----------------------------------------------------------------------------
int round_up(int value, int multiple)
{
  return ((value + multiple - 1) / multiple) * multiple;
}

//-----------------------------------------------------------------------------
void sleep_ms(int ms)
{
#ifdef _WIN32
  Sleep(ms);
#else
  struct timespec ts;

  ts.tv_sec = ms / 1000;
  ts.tv_nsec = (ms % 1000) * 1000000;

  nanosleep(&ts, NULL);
#endif
}

//-----------------------------------------------------------------------------
void *buf_alloc(int size)
{
  void *buf;

  if (NULL == (buf = malloc(size)))
    error_exit("out of memory");

  memset(buf, 0, size);

  return buf;
}

//-----------------------------------------------------------------------------
void buf_free(void *buf)
{
  free(buf);
}

//-----------------------------------------------------------------------------
int load_file(char *name, uint8_t *data, int size)
{
  struct stat stat;
  int fd, rsize;

  check(NULL != name, "input file name is not specified");

  fd = open(name, O_RDONLY | O_BINARY);

  if (fd < 0)
    perror_exit("open()");

  fstat(fd, &stat);

  if (stat.st_size < size)
    size = stat.st_size;

  rsize = read(fd, data, size);

  if (rsize < 0)
    perror_exit("read()");

  check(rsize == size, "cannot fully read file");

  close(fd);

  return rsize;
}

//-----------------------------------------------------------------------------
void save_file(char *name, uint8_t *data, int size)
{
  int fd, rsize;

  check(NULL != name, "output file name is not specified");

  fd = open(name, O_WRONLY | O_TRUNC | O_CREAT | O_BINARY, 0644);

  if (fd < 0)
    perror_exit("open()");

  rsize = write(fd, data, size);

  if (rsize < 0)
    perror_exit("write()");

  check(rsize == size, "error writing the file");

  close(fd);
}

//-----------------------------------------------------------------------------
uint8_t *mem_find(uint8_t *haystack, int haystack_size, uint8_t *needle, int needle_size)
{
  if (haystack_size == 0 || needle_size == 0 || haystack_size < needle_size)
    return NULL;

  for (int i = 0; i < (haystack_size - needle_size); i++)
  {
    if (memcmp(haystack + i, needle, needle_size) == 0)
      return haystack + i;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
static void print_debugger_info(debugger_t *debugger)
{
  uint8_t buf[256];
  char str[512] = "Debugger: ";
  int size;

  size = dap_info(DAP_INFO_VENDOR, buf, sizeof(buf));
  strcat(str, size ? (char *)buf : debugger->manufacturer);
  strcat(str, " ");

  size = dap_info(DAP_INFO_PRODUCT, buf, sizeof(buf));
  strcat(str, size ? (char *)buf : debugger->product);
  strcat(str, " ");

  size = dap_info(DAP_INFO_SER_NUM, buf, sizeof(buf));
  strcat(str, size ? (char *)buf : debugger->serial);
  strcat(str, " ");

  size = dap_info(DAP_INFO_FW_VER, buf, sizeof(buf));
  strcat(str, (char *)buf);
  strcat(str, " ");

  size = dap_info(DAP_INFO_CAPABILITIES, buf, sizeof(buf));
  check(size == 1, "incorrect DAP_INFO_CAPABILITIES size");

  strcat(str, "(");

  if (buf[0] & DAP_CAP_SWD)
    strcat(str, "S");

  if (buf[0] & DAP_CAP_JTAG)
    strcat(str, "J");

  strcat(str, ")\n");

  verbose(str);

  check(buf[0] & DAP_CAP_SWD, "SWD support required");
}

//-----------------------------------------------------------------------------
static void print_clock_freq(int freq)
{
  float value = freq;
  char *unit;

  if (value < 1.0e6)
  {
    value /= 1.0e3;
    unit = "kHz";
  }
  else
  {
    value /= 1.0e6;
    unit = "MHz";
  }

  verbose("Clock frequency: %.1f %s\n", value, unit);
}

//-----------------------------------------------------------------------------
static void reconnect_debugger(void)
{
  dap_disconnect();
  dap_connect(DAP_INTERFACE_SWD);
  // 8 idle cycles between transfers: with pipelined packets the probe otherwise
  // drives requests back-to-back with zero idle bits, and a target that misses
  // a start bit ignores the (now misframed) request silently -- reads as ack=7
  // line-dead bursts under sustained window>1 traffic.
  //
  // On an electrically noisy SWD line, sustained writes storm (ack=7) well
  // below the read clock ceiling: the write->ACK bus turnaround is where a
  // corrupted ACK bit desyncs the DP. Two knobs buy margin without dropping
  // the clock -- SWD turnaround cycles (EDBG_SWD_TURNAROUND, 0..3 => 1..4
  // cycles) and inter-transfer idle cycles (EDBG_IDLE_CYCLES). Defaults keep
  // the fast-path behavior (turnaround 1 cycle, idle 8).
  // NOTE: SWD turnaround (cfg bits 0..1) must MATCH the target DP's DLCR; a
  // host-only bump misaligns every ACK. cfg bit 2 ("always data phase") does
  // NOT need target coordination -- it keeps the probe clocking a full data
  // phase after a non-OK ACK, so a corrupted ACK on a noisy line resyncs
  // instead of cascading into a wedge. EDBG_SWD_CFG sets the raw cfg byte.
  char *e_idle = getenv("EDBG_IDLE_CYCLES");
  char *e_cfg  = getenv("EDBG_SWD_CFG");
  int idle = e_idle ? atoi(e_idle) : 8;
  int cfg  = e_cfg  ? atoi(e_cfg)  : 0;
  if (idle < 0 || idle > 255) idle = 8;
  if (cfg < 0 || cfg > 7) cfg = 0;
  dap_transfer_configure((uint8_t)idle, 32768, 128);
  dap_swd_configure(cfg);
  dap_swj_clock(g_clock);
  dap_led(0, 1);
}

//-----------------------------------------------------------------------------
static void print_help(char *name)
{
  message("CMSIS-DAP SWD programmer. Built " __DATE__ " " __TIME__ ".\n\n");

  if (g_target)
  {
    target_ops_t *target_ops = target_get_ops(g_target);

    if (target_ops->help)
      message(target_ops->help);
    else
      message("Specified target does not have a help text.\n");
  }
  else
  {
    message("Usage: %s [options]\n", name);
    message(
      "Options:\n"
      "  -h, --help                 print this help message and exit\n"
      "  -b, --verbose              print verbose messages\n"
      "  -e, --erase                perform a chip erase before programming\n"
      "  -p, --program              program the chip\n"
      "  -v, --verify               verify memory\n"
      "  -k, --lock                 lock the chip (set security bit)\n"
      "  -u, --unlock               unlock the chip (forces chip erase in most cases)\n"
      "  -r, --read                 read the whole content of the chip flash\n"
      "  -f, --file <file>          binary file to be programmed or verified; also read output file name\n"
      "  -t, --target <name>        specify a target type (use '-t list' for a list of supported target types)\n"
      "  -l, --list                 list all available debuggers\n"
      "  -s, --serial <number>      use a debugger with a specified serial number or index in the list\n"
      "  -c, --clock <freq>         interface clock frequency in kHz (default 16000)\n"
      "  -o, --offset <offset>      offset for the operation\n"
      "  -z, --size <size>          size for the operation\n"
      "  -F, --fuse <options>       operations on the fuses (use '-F help' for details)\n"
      "  -i, --identify             print core id of target\n"
      "      --dfu                  send the Windborne frog DFU trigger, then exit: the frog\n"
      "                             resets into the SAM-BA bootloader for a bossac reflash\n"
      "      --rtt                  bridge the target's SEGGER RTT ring buffers to stdio while\n"
      "                             the core runs (no halt); needs -t. With no --rtt-addr it\n"
      "                             scans the target SRAM for the 'SEGGER RTT' control block\n"
      "      --rtt-addr <addr>      address of the _SEGGER_RTT control block (from the ELF/.map);\n"
      "                             the fast path -- skips the RAM scan\n"
      "      --rtt-scan <addr>      base of the RAM range to scan (default: target driver's SRAM)\n"
      "      --rtt-scan-len <n>     length of the RAM range to scan in bytes\n"
      "      --memdump              snapshot target memory [<-o offset>, +<-z size>) over SWD\n"
      "                             and write it raw to -f <file> (needs -t, -o, -z). Attaches\n"
      "                             WITHOUT resetting the target. Without --freeze the read is\n"
      "                             non-intrusive (core keeps running) but NOT a coherent\n"
      "                             snapshot -- multi-word structs can tear; use --freeze.\n"
      "      --freeze               (with --memdump) halt the core, dump, resume ASAP; prints\n"
      "                             the freeze window (halt->resume, ms) to stderr. If the core\n"
      "                             is already halted (e.g. a gdb session), it is dumped as-is\n"
      "                             and NOT resumed. Do NOT --freeze large regions -- the target\n"
      "                             watchdog is not fed while halted (ballpark: 1KB ~1-3ms,\n"
      "                             16KB ~15-40ms, 256KB SRAM ~0.3-0.7s; USB-round-trip-bound).\n"
    );
  }

  exit(0);
}

//-----------------------------------------------------------------------------
static void print_fuse_help(void)
{
  message(
    "Fuse operations format: <actions><section>,<index/range>,<value>\n"
    "  <actions>     - any combination of 'r' (read), 'w' (write), 'v' (verify)\n"
    "  <section>     - index of the fuse section, may be omitted if device has only\n"
    "                  one section; use '-h -t <target>' for more information\n"
    "  <index/range> - index of the fuse, or a range of fuses (limits separated by ':')\n"
    "                  specify ':' to read all fuses\n"
    "                  specify '*' to read and write values from a file\n"
    "  <value>       - fuses value or file name for write and verify operations\n"
    "                  immediate values must be 32 bits or less\n"
    "\n"
    "Multiple operations may be specified in the same command.\n"
    "They must be separated with a ';'.\n"
    "\n"
    "Exact fuse bits locations and values are target-dependent.\n"
    "\n"
    "Examples:\n"
    "  -F w,1,1             -- set fuse bit 1\n"
    "  -F w,8:7,0           -- clear fuse bits 8 and 7\n"
    "  -F v,31:0,0x12345678 -- verify that fuse bits 31-0 are equal to 0x12345678\n"
    "  -F wv,5,1            -- set and verify fuse bit 5\n"
    "  -F r1,:,             -- read all fuses in a section 1\n"
    "  -F wv,*,fuses.bin    -- write and verify all fuses from a file\n"
    "  -F w0,1,1;w1,5,0     -- set fuse bit 1 in section 0 and\n"
    "                          clear fuse bit 5 in section 1\n"
  );

  exit(0);
}

//-----------------------------------------------------------------------------
static void parse_command_line(int argc, char **argv)
{
  int option_index = 0;
  int c;
  bool help = false;

  while ((c = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1)
  {
    switch (c)
    {
      case 'h': help = true; break;
      case 'e': g_target_options.erase = true; break;
      case 'p': g_target_options.program = true; break;
      case 'v': g_target_options.verify = true; break;
      case 'k': g_target_options.lock = true; break;
      case 'u': g_target_options.unlock = true; break;
      case 'r': g_target_options.read = true; break;
      case 'f': g_target_options.name = optarg; break;
      case 't': g_target = optarg; break;
      case 'l': g_list = true; break;
      case 's': g_serial = optarg; break;
      case OPT_V2: dbg_use_v2 = true; break;
      case 'c': g_clock = strtoul(optarg, NULL, 0) * 1000; g_clock_explicit = true; break;
      case 'b': g_verbose = true; break;
      case 'o': g_target_options.offset = (uint32_t)strtoul(optarg, NULL, 0); break;
      case 'z': g_target_options.size = (uint32_t)strtoul(optarg, NULL, 0); break;
      case 'F': g_target_options.fuse_cmd = optarg; break;
      case 'i': g_target_options.identify = true; break;
      case OPT_DFU: g_dfu = true; break;
      case OPT_RTT: g_rtt = true; break;
      case OPT_RTT_ADDR: g_rtt_addr = (uint32_t)strtoul(optarg, NULL, 0); break;
      case OPT_RTT_SCAN: g_rtt_scan = (uint32_t)strtoul(optarg, NULL, 0); break;
      case OPT_RTT_SCAN_LEN: g_rtt_scan_len = (uint32_t)strtoul(optarg, NULL, 0); break;
      case OPT_MEMDUMP: g_memdump = true; break;
      case OPT_FREEZE: g_freeze = true; break;
      default: exit(1); break;
    }
  }

  if (help)
    print_help(argv[0]);

  if (g_target_options.fuse_cmd && 0 == strcmp(g_target_options.fuse_cmd, "help"))
    print_fuse_help();

  check(optind >= argc, "malformed command line, use '-h' for more information");
}

//-----------------------------------------------------------------------------
// Resolve which attached debugger to use, honoring -s (serial or list index) and
// falling back to the sole debugger when only one is present. Exits on ambiguity.
static int select_debugger(debugger_t *debuggers, int n_debuggers)
{
  int debugger = -1;

  if (g_serial)
  {
    char *end = NULL;
    int index = strtoul(g_serial, &end, 10);

    if (index < n_debuggers && end[0] == 0)
    {
      debugger = index;
    }
    else
    {
      for (int i = 0; i < n_debuggers; i++)
      {
        if (0 == strcmp(debuggers[i].serial, g_serial))
        {
          debugger = i;
          break;
        }
      }
    }

    if (-1 == debugger)
      error_exit("unable to find a debugger with a specified serial number");
  }

  if (0 == n_debuggers)
    error_exit("no debuggers found");
  else if (1 == n_debuggers)
    debugger = 0;
  else if (n_debuggers > 1 && -1 == debugger)
    error_exit("more than one debugger found, please specify a serial number");

  return debugger;
}

//-----------------------------------------------------------------------------
// SEGGER control-block layout (matches boards/lib/rtt/rtt.c):
//   0 char acID[16]  16 i32 MaxUp  20 i32 MaxDown  24 RING up[0]  48 RING down[0]
//   RING(24): 0 sName*  4 pBuffer*  8 SizeOfBuffer  12 WrOff  16 RdOff  20 Flags
enum { RTT_UP = 24, RTT_DOWN = 48, RTT_R_PBUF = 4, RTT_R_SIZE = 8, RTT_R_WR = 12, RTT_R_RD = 16 };

// The 16-byte acID at the head of the control block. SEGGER stores the literal
// "SEGGER RTT" then 6 NULs. (SEGGER's own runtime writes the 'S' last so a debugger
// can't latch a half-initialized block during startup -- but our target ring uses a
// STATICALLY initialized .acID = "SEGGER RTT" that the C runtime copies into .bss/.data
// before main(), so by the time we scan the full string is always present. A plain
// literal match is therefore correct; no reversed/partial handling is needed.)
static const char RTT_ID[16] = "SEGGER RTT";

//-----------------------------------------------------------------------------
// Validate that `cb` looks like a real control block, not a stale/embedded copy of
// the id string. Reads the header and checks MaxUp/MaxDown are sane and the up/down
// ring buffers point inside [ram_start, ram_end) with WrOff/RdOff < SizeOfBuffer.
// ram_start/ram_end bound the sanity check on pointers; pass 0/0 to skip the RAM-range
// part (still checks MaxUp/Down and offsets). Returns true if it passes.
static bool rtt_validate_cb(uint32_t cb, uint32_t ram_start, uint32_t ram_end)
{
  int32_t max_up   = (int32_t)dap_read_word(cb + 16);
  int32_t max_down = (int32_t)dap_read_word(cb + 20);

  // A statically-defined block has a small fixed channel count. probe-rs/OpenOCD reject
  // > 255; be tighter -- a real ring here has a handful of channels, and a bogus hit
  // (id bytes appearing in data) almost always yields wild counts.
  if (max_up < 1 || max_up > 16 || max_down < 0 || max_down > 16)
    return false;

  // Up channel 0 must have a buffer inside RAM with a non-zero, in-range size and
  // read/write offsets within it. (Down channel 0 may legitimately be absent if
  // max_down is 0, so only check it when present.)
  uint32_t up_buf  = dap_read_word(cb + RTT_UP + RTT_R_PBUF);
  uint32_t up_size = dap_read_word(cb + RTT_UP + RTT_R_SIZE);
  uint32_t up_wr   = dap_read_word(cb + RTT_UP + RTT_R_WR);
  uint32_t up_rd   = dap_read_word(cb + RTT_UP + RTT_R_RD);

  if (0 == up_size || up_size > 0x00100000u)   // 1 MB ceiling: no sane RTT ring is bigger
    return false;
  if (up_wr >= up_size || up_rd >= up_size)
    return false;
  if (ram_end > ram_start)
  {
    if (up_buf < ram_start || up_buf >= ram_end)
      return false;
    if ((uint64_t)up_buf + up_size > ram_end)
      return false;
  }

  if (max_down > 0)
  {
    uint32_t dn_buf  = dap_read_word(cb + RTT_DOWN + RTT_R_PBUF);
    uint32_t dn_size = dap_read_word(cb + RTT_DOWN + RTT_R_SIZE);
    if (0 == dn_size || dn_size > 0x00100000u)
      return false;
    if (ram_end > ram_start && (dn_buf < ram_start || (uint64_t)dn_buf + dn_size > ram_end))
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
// Scan [start, start+len) of target RAM for the SEGGER RTT control block and return
// its address. Reads the range in overlapping chunks (the 16-byte id can straddle a
// chunk boundary) and byte-scans each for RTT_ID, validating every hit so id bytes
// that happen to appear in ring data aren't mistaken for the block. error_exits if
// nothing valid is found. The id is 4-byte aligned in practice (it's the first field
// of an aligned struct) but we scan every byte to be safe against odd linker layouts.
static uint32_t rtt_scan(uint32_t start, uint32_t len, uint32_t ram_start, uint32_t ram_end)
{
  enum { CHUNK = 8192, OVERLAP = 16 };   // OVERLAP >= sizeof(RTT_ID) so a straddling id is seen
  static uint8_t buf[CHUNK];

  // dap_block_read needs word-aligned addr+size; round the window inward/outward to words.
  start &= ~3u;
  len = (len + 3u) & ~3u;

  fprintf(stderr, "RTT scan: 0x%08x .. 0x%08x (%u KB) for the control block ...\n",
      (unsigned)start, (unsigned)(start + len), (unsigned)(len / 1024));

  uint32_t addr = start;
  uint32_t end = start + len;

  while (addr < end)
  {
    uint32_t want = end - addr;
    if (want > CHUNK)
      want = CHUNK;
    want &= ~3u;
    if (0 == want)
      break;

    dap_block_read(addr, buf, (int)want);

    // A candidate can start anywhere the full 16-byte id still fits in this chunk.
    if (want >= sizeof(RTT_ID))
    {
      for (uint32_t i = 0; i + sizeof(RTT_ID) <= want; i++)
      {
        if (buf[i] != RTT_ID[0])
          continue;
        if (0 != memcmp(&buf[i], RTT_ID, sizeof(RTT_ID)))
          continue;

        uint32_t cb = addr + i;
        if (rtt_validate_cb(cb, ram_start, ram_end))
        {
          fprintf(stderr, "RTT scan: found control block at 0x%08x\n", (unsigned)cb);
          return cb;
        }
        // id matched but validation failed -- a stale/partial block; keep scanning.
        fprintf(stderr, "RTT scan: id at 0x%08x failed validation, continuing\n",
            (unsigned)cb);
      }
    }

    // Step by chunk minus the overlap so an id spanning the boundary is caught next pass.
    if (want < OVERLAP)
      break;
    addr += want - OVERLAP;
  }

  error_exit("RTT scan found no control block in 0x%08x..0x%08x; give --rtt-addr or "
      "widen --rtt-scan/--rtt-scan-len", (unsigned)start, (unsigned)end);
  return 0;   // unreachable
}

//-----------------------------------------------------------------------------
// RTT-over-SWD bridge: pump a target's SEGGER-layout ring buffers <-> stdio while
// the core keeps running (background MEM-AP access, no halt). Wrap this in a PTY
// (socat/a tiny script) and tacoma -- or any RTT viewer -- uses the byte stream as
// a serial port, unchanged. On any SWD fault the dap layer error_exits; a
// supervisor respawns us, which reconnects: that's the re-attach-after-reset /
// marginal-cable recovery, kept out here instead of complicating the loop.
static void cmd_rtt(target_ops_t *ops)
{
  enum { UP = RTT_UP, DOWN = RTT_DOWN, R_PBUF = RTT_R_PBUF, R_SIZE = RTT_R_SIZE,
         R_WR = RTT_R_WR, R_RD = RTT_R_RD };

  // Scan range for auto-detect: explicit --rtt-scan[/-len] wins, else the target
  // driver's SRAM base+size. The range also bounds the pointer-sanity check.
  uint32_t scan_base = g_rtt_scan ? g_rtt_scan : ops->ram_start;
  uint32_t scan_len  = g_rtt_scan_len ? g_rtt_scan_len : ops->ram_size;
  uint32_t ram_start = ops->ram_start ? ops->ram_start : scan_base;
  uint32_t ram_end   = ram_start ? ram_start + (ops->ram_size ? ops->ram_size : scan_len) : 0;

  uint32_t cb = g_rtt_addr;

  if (0 == cb)
  {
    // No explicit address: scan target RAM for the "SEGGER RTT" id.
    if (0 == scan_base || 0 == scan_len)
      error_exit("--rtt with no --rtt-addr needs a scan range: the '%s' target exposes "
          "no SRAM map, so pass --rtt-scan <addr> --rtt-scan-len <bytes> (or --rtt-addr)",
          g_target);
    cb = rtt_scan(scan_base, scan_len, ram_start, ram_end);
  }
  else
  {
    // Validate the id so a wrong address fails loudly instead of streaming garbage.
    uint8_t id[16];
    dap_read_block(cb, id, sizeof(id));
    if (0 != memcmp(id, "SEGGER RTT", 10))
      error_exit("no RTT control block at 0x%08x (id mismatch)", (unsigned)cb);
  }

  uint32_t up_buf  = dap_read_word(cb + UP + R_PBUF);
  uint32_t up_size = dap_read_word(cb + UP + R_SIZE);
  uint32_t dn_buf  = dap_read_word(cb + DOWN + R_PBUF);
  uint32_t dn_size = dap_read_word(cb + DOWN + R_SIZE);

  fprintf(stderr, "RTT bridge: up %u B / down %u B at 0x%08x\n",
      (unsigned)up_size, (unsigned)dn_size, (unsigned)cb);

  fcntl(0, F_SETFL, fcntl(0, F_GETFL, 0) | O_NONBLOCK);   // non-blocking stdin

  uint8_t scratch[256];

  for (;;)
  {
    // up: target -> host (stdout). Snapshot WrOff, drain to it.
    uint32_t wr = dap_read_word(cb + UP + R_WR);
    uint32_t rd = dap_read_word(cb + UP + R_RD);

    while (rd != wr)
    {
      uint32_t run = (wr > rd ? wr : up_size) - rd;   // contiguous run to wr or the wrap
      if (run > sizeof(scratch))
        run = sizeof(scratch);

      // dap_read_block is word-granular: read the covering aligned span, copy bytes.
      uint32_t a = up_buf + rd;
      uint32_t s = a & ~3u;
      uint32_t e = (a + run + 3u) & ~3u;
      uint8_t tmp[e - s];

      dap_read_block(s, tmp, (int)(e - s));

      if (write(STDOUT_FILENO, tmp + (a - s), run) < 0)
        return;                                       // stdout closed

      rd += run;
      if (rd >= up_size)
        rd -= up_size;
      dap_write_word(cb + UP + R_RD, rd);
    }

    // down: host (stdin) -> target
    int n = (int)read(STDIN_FILENO, scratch, sizeof(scratch));
    if (0 == n)
      return;                                         // stdin EOF
    if (n > 0)
    {
      uint32_t dwr = dap_read_word(cb + DOWN + R_WR);
      uint32_t drd = dap_read_word(cb + DOWN + R_RD);

      for (int i = 0; i < n; i++)
      {
        uint32_t nx = dwr + 1;
        if (nx >= dn_size)
          nx = 0;
        if (nx == drd)
          break;                                      // ring full -> drop (host resyncs on CRC)
        dap_write_byte(dn_buf + dwr, scratch[i]);
        dwr = nx;
      }
      dap_write_word(cb + DOWN + R_WR, dwr);           // publish after the data
    }

    usleep(1000);   // 1 ms poll -- pokes are latency-tolerant, keeps SWD load low
  }
}

//-----------------------------------------------------------------------------
// ARMv7-M/ARMv8-M debug halting-control registers, used by --freeze. Same values
// the stm32u5 driver writes for its gentle halt (DHCSR at 0xe000edf0):
//   HALT   = DBGKEY | C_DEBUGEN | C_HALT  = 0xa05f0003
//   RESUME = DBGKEY (clears C_DEBUGEN | C_HALT, no C_MASKINTS) = 0xa05f0000
// S_HALT (bit 17 of DHCSR read) reports whether the core is currently halted.
#define MD_DHCSR          0xe000edf0
#define MD_DHCSR_HALT     0xa05f0003u
#define MD_DHCSR_RESUME   0xa05f0000u
#define MD_DHCSR_S_HALT   (1u << 17)
#define MD_ABORT_CLR      0x1e   // STKCMP|STKERR|WDERR|ORUNERR clear (no DAPABORT)

//-----------------------------------------------------------------------------
// Wall-clock milliseconds, for measuring the freeze window.
static double now_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

//-----------------------------------------------------------------------------
// memdump: snapshot [offset, offset+size) of target memory over SWD and write it
// raw to -f <file>. The link is already connected (reconnect_debugger in main);
// we deliberately never reset or select() the target -- a plain MEM-AP attach, so
// the running app is undisturbed in live mode.
//
// --freeze halts the core across the block read and resumes it ASAP, giving a
// coherent snapshot (no torn multi-word reads) at the cost of a short stall. The
// S_HALT rule: if the core is ALREADY halted when we attach (a gdb session stopped
// it), we dump as-is and never resume -- resuming a core someone else deliberately
// stopped would be wrong. Live mode (no --freeze) skips all DHCSR writes: the
// reads are non-intrusive while the core runs, but the result is NOT a coherent
// snapshot (multi-word structs can tear mid-read), so we warn.
//
// dap_block_read wants a word-aligned addr+size; we widen [addr, addr+len) outward
// to word boundaries for the transfer, then slice back to exactly `len` on output.
static void cmd_memdump(void)
{
  check(NULL != g_target_options.name, "--memdump needs an output file (-f)");
  check(g_target_options.offset >= 0, "--memdump needs a start address (-o/--offset)");
  check(g_target_options.size > 0, "--memdump needs a size (-z/--size)");

  uint32_t addr = (uint32_t)g_target_options.offset;
  uint32_t len  = (uint32_t)g_target_options.size;

  // Widen outward to word boundaries for the block read; slice back to `len`.
  uint32_t a0 = addr & ~3u;
  uint32_t a1 = (addr + len + 3u) & ~3u;
  uint32_t span = a1 - a0;

  uint8_t *buf = buf_alloc((int)span);

  bool we_halted = false;

  if (g_freeze)
  {
    uint32_t dhcsr_pre = dap_read_word(MD_DHCSR);

    if (dhcsr_pre & MD_DHCSR_S_HALT)
    {
      // Someone else (a gdb session) stopped the core. Dump as-is, never resume.
      fprintf(stderr, "core already halted -- dumped without resume\n");
    }
    else
    {
      dap_write_word(MD_DHCSR, MD_DHCSR_HALT);
      we_halted = true;
    }
  }
  else
  {
    fprintf(stderr, "WARNING: live dump (no --freeze) is NOT a coherent snapshot -- "
        "multi-word structs can tear mid-read; use --freeze for consistency\n");
  }

  double t_halt = now_ms();

  // The fast path: 1 KB TAR-boundary segmentation + retry + link re-sync.
  dap_block_read(a0, buf, (int)span);

  if (we_halted)
  {
    // Resume ASAP -- one word write, no C_MASKINTS.
    dap_write_word(MD_DHCSR, MD_DHCSR_RESUME);
    double window = now_ms() - t_halt;
    fprintf(stderr, "freeze window (halt->resume): %.1f ms (%u bytes)\n",
        window, (unsigned)len);
  }

  // Clear any sticky DP error left by the sequence (no DAPABORT: nothing in flight).
  dap_write_abort(MD_ABORT_CLR);

  // Slice the word-aligned span back to exactly [addr, addr+len).
  save_file(g_target_options.name, buf + (addr - a0), (int)len);

  fprintf(stderr, "memdump: wrote %u bytes from 0x%08x to %s\n",
      (unsigned)len, (unsigned)addr, g_target_options.name);

  buf_free(buf);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  debugger_t debuggers[MAX_DEBUGGERS];
  int n_debuggers = 0;
  int debugger = -1;
  target_ops_t *target_ops;

  parse_command_line(argc, argv);

  if (!(g_target_options.erase || g_target_options.program || g_target_options.verify ||
      g_target_options.lock || g_target_options.read || g_target_options.fuse_cmd ||
      g_list || g_dfu || g_rtt || g_memdump || g_target))
    error_exit("no actions specified");

  if (g_target_options.read && (g_target_options.erase || g_target_options.program ||
      g_target_options.verify || g_target_options.lock))
    error_exit("mutually exclusive actions specified");

  n_debuggers = dbg_enumerate(debuggers, MAX_DEBUGGERS);

  if (g_list)
  {
    message("Attached debuggers:\n");
    for (int i = 0; i < n_debuggers; i++)
      message("  %d: %s - %s %s\n", i, debuggers[i].serial, debuggers[i].manufacturer, debuggers[i].product);
    return 0;
  }

  // DFU trigger is a pure vendor HID command -- no target, clock, or SWD connect.
  // Open the frog, fire it, and exit; the frog resets into SAM-BA immediately.
  if (g_dfu)
  {
    debugger = select_debugger(debuggers, n_debuggers);
    g_debugger = &debuggers[debugger];

    dbg_open(g_debugger);
    message("Sending DFU trigger to %s -- it will reset into the SAM-BA "
            "bootloader for a bossac reflash.\n", g_debugger->serial);
    dap_dfu(g_debugger->serial);
    return 0;
  }

  if (NULL == g_target)
    error_exit("no target type specified (use '-t' option)");

  if (0 == strcmp(g_target, "list"))
  {
    target_list();
    return 0;
  }

  target_ops = target_get_ops(g_target);

  // A target may prefer a safe clock when the user gave no -c (e.g. stm32u5 needs
  // 8 MHz on a long AC-coupled cable). Apply it before the first connect so every
  // step -- identify, select, program -- runs at it.
  if (!g_clock_explicit && target_ops->default_clock)
    g_clock = target_ops->default_clock;

  debugger = select_debugger(debuggers, n_debuggers);
  g_debugger = &debuggers[debugger];

  dbg_open(g_debugger);

  print_debugger_info(g_debugger);
  print_clock_freq(g_clock);

  reconnect_debugger();

  // RTT bridges target RAM rings over the LIVE link -- connect (done) but never
  // select()/halt the core; the target keeps running.
  if (g_rtt)
  {
    cmd_rtt(target_ops);
    return 0;
  }

  // memdump attaches over the LIVE link -- connect (done) but never reset or
  // select() the target. In live mode the core keeps running; --freeze halts it
  // just for the block read and resumes ASAP (see cmd_memdump). Return directly
  // (like cmd_rtt): the normal exit path's disconnect_debugger() pulses nRESET,
  // which would reset the very target we are dumping non-intrusively.
  if (g_memdump)
  {
    cmd_memdump();
    dap_disconnect();
    dbg_close();
    return 0;
  }

  // Flashing does its select() inside the retry block below (so a transient
  // connect failure on a marginal cable retries with the rest); other ops
  // select once here.
  if (!(g_target_options.program || g_target_options.verify))
    target_ops->select(&g_target_options);

  if (g_target_options.unlock)
  {
    verbose("Unlocking...");
    target_ops->unlock();
    verbose(" done.\n");
  }

  // When flashing, identify runs inside the retry region below so the same
  // reconnect + clock step-down that protects the program phase also covers the
  // Core ID read on a marginal cable. Standalone identify runs here.
  if (g_target_options.identify && !(g_target_options.program || g_target_options.verify))
  {
    verbose("Indentifying... \n");
    target_ops->identify();
    verbose("done.\n");
  }

  if (g_target_options.erase)
  {
    verbose("Erasing...");
    target_ops->erase();
    verbose(" done.\n");
  }

  if (g_target_options.program || g_target_options.verify)
  {
    char *fr = getenv("EDBG_FLASH_RETRIES");
    // Default 10 gives enough retries for the *0.75/retry step-down to walk the
    // 8 MHz start clock all the way to the 1 MHz floor (8 -> 6 -> 4.5 -> 3.4 ->
    // 2.5 -> 1.9 -> 1.4 -> 1.1 -> 1.0). EDBG_FLASH_RETRIES overrides.
    volatile int flash_max = fr ? atoi(fr) : 10;
    if (flash_max < 1)
      flash_max = 1;
    volatile int attempt = 0;

    if (setjmp(g_flash_retry_jmp) != 0)
    {
      // A prior program/verify unwound here. Line-noise bursts on a marginal
      // cable last from ms to seconds, so wait progressively longer for the
      // burst to subside before reconnecting fresh and re-running. The
      // differential flash reprograms only the pages still wrong, so each retry
      // is cheap and convergent -- reruns make monotonic progress toward done.
      attempt++;
      g_flash_retries_left = (attempt + 1 < flash_max);

      // Auto clock step-down for a worse-than-usual cable. The default 8 MHz
      // clears most long cables first-try; the first retry re-tries the start
      // clock (a transient noise burst recovers at the same speed via the wait +
      // differential rerun). If it KEEPS failing -- a genuinely worse/noisier
      // cable, not a burst -- drop the clock *0.75 every retry thereafter, down
      // to a 1 MHz floor, reached within the default retry budget
      // (8 -> 6 -> 4.5 -> 3.4 -> 2.5 -> 1.9 -> 1.4 -> 1.1 -> 1.0 MHz). Stepping only
      // continues after a FAILED attempt, so a good cable succeeds high and
      // never drops; the higher clocks are always tried first, so going low can
      // only help. 1 MHz clears every long cable measured to date (resistive
      // wires read cleanly well below it -- a pure AC-coupled cable would just
      // fail the sub-3 MHz tries harmlessly after the good clocks were tried).
      // The differential flash keeps pages already written, so each lower-clock
      // rerun only reprograms what's still wrong -- convergent, not a full redo.
      // EDBG_MIN_CLOCK (kHz) overrides the floor.
      if (attempt >= 2)
      {
        char *mc = getenv("EDBG_MIN_CLOCK");
        long floor_hz = mc ? atol(mc) * 1000 : 1000000;
        if (floor_hz < 100000)
          floor_hz = 100000;

        long stepped = (long)g_clock * 3 / 4;
        if (stepped < floor_hz)
          stepped = floor_hz;
        if (stepped < g_clock)
          g_clock = stepped;
      }

      int wait_ms = 250 * attempt;
      if (wait_ms > 3000)
        wait_ms = 3000;
      warning("flash interrupted; waiting %dms, then reconnect at %ld kHz + differential re-run [attempt %d/%d]\n",
          wait_ms, g_clock / 1000, attempt + 1, flash_max);
      sleep_ms(wait_ms);
      reconnect_debugger();
    }

    g_flash_retries_left = (attempt + 1 < flash_max);

    // Inside the retry region: a select failure now reconnects and re-runs too.
    target_ops->select(&g_target_options);

    if (g_target_options.identify)
    {
      verbose("Indentifying... \n");
      target_ops->identify();
      verbose("done.\n");
    }

    if (g_target_options.program)
    {
      verbose("Programming...");
      target_ops->program();
      verbose(" done.\n");
    }

    if (g_target_options.verify)
    {
      verbose("Verification...");
      target_ops->verify();
      verbose(" done.\n");
    }

    g_flash_retries_left = 0;
  }

  if (g_target_options.lock)
  {
    verbose("Locking...");
    target_ops->lock();
    verbose(" done.\n");
  }

  if (g_target_options.read)
  {
    verbose("Reading...");
    target_ops->read();
    verbose(" done.\n");
  }

  if (g_target_options.fuse_cmd)
  {
    verbose("Fuses:\n");
    target_fuse_commands(target_ops, g_target_options.fuse_cmd);
    verbose("done.\n");
  }

  target_ops->deselect();

  disconnect_debugger();

  return 0;
}

