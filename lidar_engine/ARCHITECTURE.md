# LiDAR Engine — Architecture & Interview Notes

A real-time 3D LiDAR scanning engine for Raspberry Pi 5 (PREEMPT_RT). This is
the refactor of the original single-file `lidar_engine_v19.c` (1336 lines) into
focused modules, with the correctness bugs fixed.

## One-paragraph summary (for "explain this project")

A single multi-threaded C process drives a pan/tilt LiDAR platform. Each thread
is pinned to a specific CPU core and given a SCHED_FIFO priority that matches how
time-critical it is: stepper pulses must never wait for I2C or the network, so
motor control gets the isolated core at priority 99. Threads share state through
C11 atomics (single words, lock-free) and protect the two shared I2C buses and
the network client with mutexes. Memory is locked with `mlockall` so the RT paths
never page-fault. A hardware watchdog reboots the board if the process hangs.

## Module map

| File | Responsibility | Why it's separate |
|------|----------------|-------------------|
| `config.h` | All tunables: pins, calibration, scan geometry, RT layout | One place to retune hardware without touching logic |
| `types.h` | Shared structs + the global state object `G` | Single source of truth for the data model |
| `globals.c` | The one definition of `G` | Avoids multiple-definition link errors |
| `rt_utils.{c,h}` | Monotonic time, syscall-free busy-wait, core-pinning | RT primitives reused by every thread |
| `mathlut.{c,h}` | sin/cos lookup table | Keeps libm out of the per-point hot loop |
| `i2c.{c,h}` | I2C transport (combined write-reg/read transaction) | Isolates the bus layer from device logic |
| `encoder.{c,h}` | AS5600 read + filter + polling thread | One device, one module |
| `imu.{c,h}` | MPU6050 complementary filter + polling thread | One device, one module |
| `motor.{c,h}` | Trapezoidal stepper profile + closed-loop correction | The hard-RT control logic |
| `lidar.{c,h}` | UART setup, packet parse, point transform, color, UDP | The data-producing core |
| `scanner.{c,h}` | Sweep orchestration (pitch / grid) | Policy on top of `lidar` + `motor` |
| `thermal.{c,h}` | Temp sensors, fan PWM, emergency shutdown | Safety subsystem |
| `network.{c,h}` | UDP command server | Control-plane |
| `watchdog.{c,h}` | Hardware watchdog kick + disarm | System-level fault recovery |
| `main.c` | Init, calibration, thread spawn, signal handling | Composition root |

## Threading & priority model

```
Core 0 (non-RT) : network (cmd)   | thermal (fans)  | watchdog
Core 1 (sensor) : encoder P80     | imu P50
Core 2 (scan)   : scanner P90  -> drives lidar capture
Core 3 (motor)  : motor_control_loop P99  (main thread, isolated core)
```

The rule behind the numbers: a higher-priority job must never be blocked waiting
on a lower-priority one's resource. Motor stepping (P99) only touches GPIO and
atomics — never a mutex — so it can't be blocked by the encoder thread holding an
I2C lock. The scanner (P90) does hold the net mutex briefly to send UDP, but it
never blocks the motor core.

## Inter-thread communication

- **Atomics for single values.** Positions, target angles, filtered encoder
  angles, and all the run/scan/emergency flags are `_Atomic`. A reader always
  sees a complete value; no lock needed.
- **Mutexes for multi-step shared resources.** The two I2C buses and the network
  client struct are guarded, because a "transaction" spans several operations and
  must be atomic as a unit.

## Data flow (one scan step)

1. `scanner` sets a motor target and waits (bounded) until the step count matches.
2. After a settle delay, it reads the **measured** encoder angle.
3. `lidar_capture_angle` reads the UART for ~250 ms, parses `0xAA55` packets,
   range- and flying-pixel-filters each return, and transforms it to world XYZ
   using the platform angle.
4. Points are packed into ~1400-byte UDP datagrams and streamed to the client.

## Bugs fixed in this refactor

1. **I2C address bug (correctness).** The old `i2c_read16_batch` hardcoded the
   AS5600 address (0x36) inside the `I2C_RDWR` messages. Because `I2C_RDWR` uses
   the per-message address and ignores the `I2C_SLAVE` setting, every IMU read
   was actually addressed to the encoder. IMU *init* worked (it used `write()`,
   which respects `I2C_SLAVE`) but all IMU *reads* returned garbage. `i2c_read16`
   now takes an explicit device address; encoder passes `AS5600_ADDR`, IMU passes
   `MPU6050_ADDR`.

2. **No signal handler (deployment).** A `systemctl stop` sends SIGTERM, which
   killed the old process with the watchdog still armed and GPIO/I2C open — so a
   clean stop could trigger a watchdog reboot. `main.c` now traps SIGINT/SIGTERM,
   flips `G.running`, and unwinds cleanly (threads join, watchdog disarms via the
   magic `V` close, fds close).

3. **`target_angle` data race.** It was a plain `float` written by the scanner /
   network threads and read by the motor loop. Now `_Atomic float`.

4. **BMP280 returned uncalibrated temperature.** The old `read_bmp280` divided the
   raw ADC by a constant. Now the dig_T1..T3 trim is read once at init and the
   datasheet compensation formula is applied.

5. **~150 lines of duplicated scan code.** The pitch and roll capture blocks were
   near-identical copies differing only in the transform equations. Collapsed into
   one `lidar_capture_angle` + `sweep_axis`, with the axis-specific transform
   behind a `ScanAxis` enum.

### Known tradeoff left in place
`motor_correct` can block the motor loop for up to
`MAX_CORRECTION_STEPS * CORRECTION_PULSE_US` (~8 ms). It only runs when **both**
axes are idle, so it can't disturb an active sweep — left as-is, flagged in the
code as the spot to convert to one-step-per-iteration if correction-while-moving
is ever needed.

## Will NEON intrinsics help?

Short answer: **no meaningful gain for this workload**, and being able to explain
*why* is the stronger interview answer than "yes, SIMD is faster."

**Where the time actually goes.** Per platform angle the engine spends ~50–80 ms
settling for vibration and ~250 ms in the capture window waiting on UART data,
plus I2C transactions. The arithmetic per LiDAR point is a handful of
multiply-adds plus two LUT lookups and a branchy color ramp. That math is a tiny
fraction of a percent of the cycle. The system is **I/O- and mechanically-bound**,
not compute-bound — so vectorizing the transform optimizes the part that was
never the bottleneck. Amdahl's law kills it.

**Structural mismatch too.** Points arrive a few at a time as serial packets are
parsed, not in large contiguous batches. To use NEON's 4-wide float lanes well
you'd first buffer points, then transform in blocks of 4+ — adding latency and
complexity to save microseconds. The color ramp is a `switch` on the hue sextant;
NEON dislikes branches, so you'd have to rewrite it branchlessly with `vbslq`,
for a payoff that rounds to zero here.

**When NEON *would* pay off** (worth saying, shows you know the tool):
- A genuinely compute-bound DSP inner loop — e.g. audio filtering, where you
  stream large blocks of samples through the same arithmetic. That's the textbook
  NEON win.
- If this engine grew an **on-Pi point-cloud stage** (statistical outlier
  removal, voxel downsampling over thousands of accumulated points), batch
  distance math and transforms over big arrays would vectorize cleanly. That's
  the place to reach for NEON in *this* project — not the per-packet hot loop.

**The honest framing:** "I'd profile before optimizing. Here the profile points
at mechanical settle time and serial I/O, so the highest-value work is reducing
settle time or overlapping capture with motion — not SIMD. NEON is the right tool
for a compute-bound loop; this loop isn't one."

## Build

```bash
sudo apt install liblgpio-dev
make
sudo ./lidar_engine     # needs RT priority + GPIO/watchdog access
```
