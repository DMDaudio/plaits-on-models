#!/bin/bash
# Build the Plaits bench for the Model:Samples/Cycles ColdFire (MCF54415, no FPU).
# usage: ./build.sh [extra CXXFLAGS...]   ->  build/bench.elf, build/bench.bin
set -e
cd "$(dirname "$0")"
ER=../../repos/eurorack
mkdir -p build
CXX=m68k-elf-g++
FLAGS=(-mcpu=54418 -O2 -DTEST -ffreestanding -fno-exceptions -fno-rtti -fno-threadsafe-statics
       -fno-use-cxa-atexit -ffunction-sections -fdata-sections -I../shim -include cstdio -I"$ER" -w "$@")
SRCS=$(cd "$ER" && ls plaits/dsp/*.cc plaits/dsp/*/*.cc plaits/resources.cc stmlib/dsp/units.cc stmlib/utils/random.cc 2>/dev/null)
OBJS=()
for s in $SRCS; do
  o=build/$(echo "$s" | tr / _ | sed 's/\.cc$/.o/')
  $CXX "${FLAGS[@]}" -c "$ER/$s" -o "$o" &
  OBJS+=("$o")
done
$CXX "${FLAGS[@]}" -c bench.cc -o build/bench.o &
$CXX "${FLAGS[@]}" -c ../shim/shim.cc -o build/shim.o &
m68k-elf-gcc -mcpu=54418 -c start.S -o build/start.o &
wait
cd build
$CXX -mcpu=54418 -nostdlib -T ../link.ld -Wl,--gc-sections -Wl,-Map=bench.map \
  start.o bench.o shim.o $(for o in "${OBJS[@]}"; do basename "$o"; done) -lgcc -o bench.elf
m68k-elf-objcopy -O binary bench.elf bench.bin
m68k-elf-size bench.elf
