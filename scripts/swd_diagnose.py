#!/usr/bin/env python3
"""SWD link diagnoser for CMSIS-DAP probes (frogs) -- raw hidraw, no edbg.

Discriminates between the known long-wire failure theories in one run:

  T-RESET  nRESET not wired through the rig/mux  -> reset-catch silently no-ops
  T-SLEEP  target app in WFI sleep, DBGMCU_CR clear -> AP access fails while asleep
  T-BATCH  multi-op DAP_Transfer packets fail where single-op packets work
           (back-to-back wire ops; 2021 probe fw wedges SWDIO on one garbage ACK)
  T-CFG    edbg's TransferConfigure(8, 32768, 128) itself changes behavior
  T-PWRUP  first AP access races the CTRL/STAT power-up handshake
  T-WEDGE  after any failure, whether the link stays dead until DAP_Connect
           (the 2021-firmware SWDIO-direction wedge signature)

Usage: swd_diagnose.py <probe-serial> [clk_hz=1000000]
Point the frog's mux at the target FIRST (oc -s <SN> -m <N>).
"""
import os, glob, struct, sys, time

SERIAL = sys.argv[1] if len(sys.argv) > 1 else sys.exit(__doc__)
CLK = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000

node = None
for h in glob.glob('/sys/class/hidraw/hidraw*'):
    try:
        if SERIAL in open(h + '/device/uevent').read():
            node = '/dev/' + os.path.basename(h)
    except OSError:
        pass
if not node:
    sys.exit(f'no hidraw device matches {SERIAL!r} -- frog plugged in? permissions?')
fd = os.open(node, os.O_RDWR)
R = 1024

def cmd(p):
    os.write(fd, bytes([0]) + bytes(p) + bytes(R - len(p)))
    return os.read(fd, R + 1)

def W(x):
    return list(struct.pack('<I', x))

def connect(configure=None):
    """DAP_Connect + clock + optional TransferConfigure + line reset. True if DPIDR reads."""
    cmd([0x02, 0x01])
    cmd([0x11] + W(CLK))
    if configure:
        idle, retry, match = configure
        cmd([0x04, idle] + list(struct.pack('<H', retry)) + list(struct.pack('<H', match)))
    cmd([0x12, 136] + [0xff] * 7 + [0x9e, 0xe7] + [0xff] * 7 + [0x00])
    r = cmd([0x05, 0x00, 0x01, 0x02])
    return (r[2] & 7) == 1, (struct.unpack('<I', r[3:7])[0] if (r[2] & 7) == 1 else 0)

def powerup_spaced():
    """ABORT / SELECT / CTRL-STAT as separate single-op packets (the known-good raw shape)."""
    a = cmd([0x05, 0x00, 0x01, 0x00] + W(0x1e))[2] & 7
    s = cmd([0x05, 0x00, 0x01, 0x08] + W(0))[2] & 7
    c = cmd([0x05, 0x00, 0x01, 0x04] + W(0x50000f00))[2] & 7
    return a == s == c == 1

def dp_read(reg):
    r = cmd([0x05, 0x00, 0x01, reg | 0x02])
    return r[2] & 7, (struct.unpack('<I', r[3:7])[0] if (r[2] & 7) == 1 else 0)

def ap_setup_spaced():
    return (cmd([0x05, 0x00, 0x01, 0x01] + W(0x23000052))[2] & 7) == 1  # CSW word autoinc

def mem_read_spaced(addr):
    t = cmd([0x05, 0x00, 0x01, 0x05] + W(addr))[2] & 7
    r = cmd([0x05, 0x00, 0x01, 0x0f])
    return (t == 1 and (r[2] & 7) == 1), (struct.unpack('<I', r[3:7])[0] if (r[2] & 7) == 1 else 0)

def mem_write_spaced(addr, val):
    t = cmd([0x05, 0x00, 0x01, 0x05] + W(addr))[2] & 7
    d = cmd([0x05, 0x00, 0x01, 0x0d] + W(val))[2] & 7
    return t == 1 and d == 1

def mem_write_batched(addr, val):
    """CSW+TAR+DRW as ONE 3-op DAP_Transfer packet (the edbg wire shape). -> count, status"""
    b = cmd([0x05, 0x00, 0x03,
             0x01] + W(0x23000052) + [0x05] + W(addr) + [0x0d] + W(val))
    return b[1], b[2]

def dpidr_alive():
    r = cmd([0x05, 0x00, 0x01, 0x02])
    return (r[2] & 7) == 1

def fresh(configure=None):
    ok, _ = connect(configure)
    return ok and powerup_spaced()

DHCSR = 0xe000edf0
S_HALT, S_SLEEP = 1 << 17, 1 << 18

print(f'== swd_diagnose {SERIAL} @ {CLK / 1e6:g} MHz on {node} ==')

# ---- probe info --------------------------------------------------------------
r = cmd([0x00, 0x04])  # DAP_Info: firmware version string
fwver = bytes(r[3:3 + r[2]]).decode(errors='replace').rstrip('\x00') if r[2] else '(none)'
print(f'probe FW version string: {fwver!r}')

# ---- T-RESET: is nRESET wired? -----------------------------------------------
cmd([0x02, 0x01])
pins_idle = cmd([0x10, 0x00, 0x00, 0, 0, 0, 0])[1]
cmd([0x10, 0x00, 0x80, 0, 0, 0, 0])          # assert nRESET (value 0, select bit7)
pins_lo = cmd([0x10, 0x00, 0x00, 0, 0, 0, 0])[1]
cmd([0x10, 0x80, 0x80, 0, 0, 0, 0])          # release
pins_hi = cmd([0x10, 0x00, 0x00, 0, 0, 0, 0])[1]
wired = not (pins_lo & 0x80) and bool(pins_hi & 0x80)
print(f'T-RESET  pins idle=0x{pins_idle:02x} asserted=0x{pins_lo:02x} released=0x{pins_hi:02x}'
      f'  -> nRESET wired: {"YES" if wired else "NO -- reset-catch is a NO-OP on this port"}')

# ---- reachability ------------------------------------------------------------
okc = 0
dpid = 0
for i in range(20):
    ok, v = connect()
    if ok:
        okc += 1
        dpid = v
print(f'DPIDR reachability: {okc}/20 fresh connects  (DPIDR=0x{dpid:08x})')
if okc == 0:
    sys.exit('target unreachable even raw -- power/mux/cable problem, stop here')

# ---- T-SLEEP: core state sampling --------------------------------------------
if fresh() and ap_setup_spaced():
    ok, dhcsr0 = mem_read_spaced(DHCSR)
    halted = bool(dhcsr0 & S_HALT)
    sleep_n = halt_n = fail_n = 0
    for i in range(100):
        ok, v = mem_read_spaced(DHCSR)
        if not ok:
            fail_n += 1
            fresh() and ap_setup_spaced()
            continue
        sleep_n += bool(v & S_SLEEP)
        halt_n += bool(v & S_HALT)
    ok, dbgmcu = mem_read_spaced(0xe0044004)  # DBGMCU_CR (STM32U5)
    print(f'T-SLEEP  DHCSR first=0x{dhcsr0:08x}  100 samples: S_SLEEP={sleep_n} S_HALT={halt_n} '
          f'read-fails={fail_n}  DBGMCU_CR=0x{dbgmcu:08x}'
          + ('  << core sleeps with DBG_LP clear!' if sleep_n > 50 and not (dbgmcu & 7) else ''))
else:
    print('T-SLEEP  skipped (could not set up AP access)')

# ---- T-BATCH / T-PWRUP / T-CFG matrix -----------------------------------------
def trial(name, reps, setup_cfg, delay, batched):
    fails = wedged = 0
    detail = ''
    for i in range(reps):
        if not fresh(setup_cfg):
            fails += 1
            detail += 'C'
            continue
        if delay:
            time.sleep(delay)
        if batched:
            cnt, st = mem_write_batched(DHCSR, 0xa05f0003)
            ok = (cnt == 3 and st == 1)
            if not ok:
                detail += f'[{cnt}/{st}]'
        else:
            ok = mem_write_spaced(DHCSR, 0xa05f0003)
            if not ok:
                detail += '[s]'
        if ok:
            detail += '.'
        else:
            fails += 1
            if not dpidr_alive():
                wedged += 1
                detail += 'X'   # X = link dead after failure until next DAP_Connect (T-WEDGE)
    print(f'{name:52s} fail {fails}/{reps} wedged {wedged}  {detail}')
    return fails

EDBG_CFG = (8, 32768, 128)
print('--- write-DHCSR matrix (fresh connect+powerup per rep) ---')
f_single = trial('single-op writes, default cfg, no delay', 8, None, 0, False)
f_batch = trial('BATCHED 3-op write, default cfg, no delay', 8, None, 0, True)
f_batch_d = trial('BATCHED 3-op write, default cfg, 10ms after pwrup', 8, None, 0.010, True)
f_cfg = trial('BATCHED 3-op write, edbg cfg (8,32768,128)', 8, EDBG_CFG, 0, True)
f_single_c = trial('single-op writes, edbg cfg (8,32768,128)', 8, EDBG_CFG, 0, False)

# ---- verdict ------------------------------------------------------------------
print('--- verdict hints ---')
if f_single == 0 and f_batch >= 6:
    print('=> T-BATCH CONFIRMED: multi-op packets fail where spaced single ops work')
    if f_batch_d < f_batch:
        print('   (delay helps -> T-PWRUP contributes)')
if f_single == 0 and f_batch == 0 and f_cfg >= 6:
    print('=> T-CFG CONFIRMED: TransferConfigure values are the discriminator')
if f_single >= 6:
    print('=> even spaced single ops fail -> look at T-SLEEP / T-RESET / target state above')
if f_single == f_batch == f_cfg == 0:
    print('=> everything passes raw at this clock: the failure lives in an edbg-specific step')
os.close(fd)
