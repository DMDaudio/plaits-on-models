#!/bin/bash
# Build the Plaits bench for the Model:Samples/Cycles ColdFire (MCF54415, no FPU).
# usage: ./build.sh [extra CXXFLAGS...]   ->  build-hf/bench.elf, build-hf/bench.bin
set -e
cd "$(dirname "$0")"
ER=../../repos/eurorack
mkdir -p build-hf
CXX=m68k-elf-g++
FLAGS=(-mcpu=5475 -mhard-float -O2 -DTEST -ffreestanding -fno-exceptions -fno-rtti -fno-threadsafe-statics
       -fno-use-cxa-atexit -ffunction-sections -fdata-sections -I../shim -include cstdio -I"$ER" -w "$@")
SRCS=$(cd "$ER" && ls plaits/dsp/*.cc plaits/dsp/*/*.cc plaits/resources.cc stmlib/dsp/units.cc stmlib/utils/random.cc 2>/dev/null)
OBJS=()
for s in $SRCS; do
  o=build-hf/$(echo "$s" | tr / _ | sed 's/\.cc$/.o/')
  $CXX "${FLAGS[@]}" -c "$ER/$s" -o "$o" &
  OBJS+=("$o")
done
$CXX "${FLAGS[@]}" -c bench.cc -o build-hf/bench.o &
$CXX "${FLAGS[@]}" -c ../shim/shim.cc -o build-hf/shim.o &
m68k-elf-gcc -mcpu=5475 -mhard-float -c start.S -o build-hf/start.o &
wait
cd build-hf
$CXX -mcpu=5475 -mhard-float -nostdlib -T ../link.ld -Wl,--gc-sections -Wl,-Map=bench.map \
  start.o bench.o shim.o $(for o in "${OBJS[@]}"; do basename "$o"; done) -lgcc -o bench.elf
m68k-elf-objcopy -O binary bench.elf bench.bin
m68k-elf-size bench.elf
