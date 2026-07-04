# RAM stub sources

`lz4stub.c` — the streaming ring-loader stub (STM32U5). The host streams a
single LZ4 block into an SRAM ring; this stub decodes it through a 64 KB
decoded-window ring and burst-programs each page, self-driving the flash
erase/program sequence. It is the default write path.

Build (produces the `lz4stub_bin.h` the driver bakes in):

    arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -O2 -ffreestanding -nostdlib \
      -fno-builtin -Wl,-Ttext=0x20004400 -Wl,-e,_start -o lz4stub.elf lz4stub.c
    arm-none-eabi-objcopy -O binary lz4stub.elf lz4stub.bin
    xxd -i lz4stub.bin > ../lz4stub_bin.h
    # append: unsigned int lz4stub_entry = 0x<addr of _start from nm>;

Constraints (each bit us at least once):
  - The params struct address in `lz4stub.c` must match `RINGPARAMS` in
    `target_st_stm32u5.c`, and must sit >= 1 KB above the stub's `__bss_end`
    (check `arm-none-eabi-nm`) so growing state can't collide with it.
  - `-Ttext` must match `RINGSTUB_LOAD`.
  - `_start` is not necessarily the first byte of the .bin — always export
    `lz4stub_entry` from `nm` and kick PC to it.

The raw burst-write stub and the CRC scan stub are small hand-assembled byte
arrays embedded directly in the driver (see `wstub_*` / `crc_stub_*`).
