# Verification

Everything here compares the fixed-point engines in `../engines/` with Plaits' own source, compiled for a PC.
You need a clone of pichenettes/eurorack (commit `08460a6`, with its stmlib submodule) and a C/C++ compiler.

## References

Each `*_ref.cc` renders one unchanged Plaits engine in 32-sample blocks and writes OUT and AUX as doubles.
Build each twice: as is (float Plaits) and with `dbl.h` force-included, which compiles `float` as `double`.

```sh
ER=path/to/eurorack
SRCS="$ER/plaits/resources.cc $ER/stmlib/dsp/units.cc $ER/stmlib/utils/random.cc"
c++ -O2 -DTEST -w -I$ER chord_ref.cc $ER/plaits/dsp/engine/chord_engine.cc $ER/plaits/dsp/chords/chord_bank.cc $SRCS -o chord_ref
c++ -O2 -DTEST -w -include dbl.h -I$ER chord_ref.cc $ER/plaits/dsp/engine/chord_engine.cc $ER/plaits/dsp/chords/chord_bank.cc $SRCS -o chord_ref_d
# same for swarm_ref.cc (engine/swarm_engine.cc), wt_ref.cc (engine/wavetable_engine.cc)
# and va_ref.cc (engine/virtual_analog_engine.cc)
```

## Comparisons

```sh
python3 compare.py                 # CHORDS, 8,464 cases
python3 compare.py --wavetable     # WAVETABLE, 8,736 cases
python3 compare.py --va            # VA, 8,736 cases
python3 swarm_compare.py           # SWARM, 2,100 cases, by band levels
```

Add `--quick` for a smaller sweep. They need numpy.

## Unit tests

`ss_test.cc` (organ voice), `wt_test.cc` (wavetable voice), `genv_test.cc` (SWARM grain envelope) and `va_test.cc`
(VA's two oscillators, with `va_ours.c`) compare one building block at a time. `ss_ours.c` exposes the static functions of `macro.c` to them. Build with the C file
compiled as C and the test as C++, for example:

```sh
cc -c -O1 -w -I../engines -DMACRO_SEL=MACRO_CHORD -DMACRO_MASK=0x100 ss_ours.c -o ss_ours.o
c++ -O1 -w -DTEST -include dbl.h -I$ER -I../engines wt_test.cc $ER/plaits/resources.cc ss_ours.o -o wt_test
```

`ss_ours.c` includes `macro.c` by relative name: add `-I../engines`.

## Feasibility

`feasibility/` builds unchanged Plaits for the ColdFire (software float, and an FPU variant as a proxy for a
fixed-point port) and counts instructions per block in Unicorn. Results in `softfloat.txt` and `fpu-estimate.txt`.
The scripts expect the eurorack clone at `../../repos/eurorack`; edit `ER=` in `build.sh` otherwise.
