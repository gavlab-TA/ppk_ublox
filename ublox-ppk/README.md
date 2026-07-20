# ublox-ppk

Two small programs for cm-level **post-processed kinematic (PPK)** positioning
from a u-blox **ZED-F9P**, corrected against a **NOAA CORS** reference station.
No live corrections, no base of your own, no website service.

1. **`ubx_logger`** — configures the F9P for raw observables and streams them to a
   `.ubx` file in the field.
2. **`ppk_solve`** — takes that log plus a **folder** of CORS RINEX and produces a
   cm-level **ECEF / LLA** track. Thin wrapper around the demo5 RTKLIB engine
   (LAMBDA ambiguity resolution, cycle-slip handling, tropo model).

```
ublox-ppk/
  logger/src/ubx_logger.cpp        # POSIX serial, no deps
  ppk/src/ppk_solve.cpp            # RTKLIB wrapper
  ppk/src/rinex_folder.h           # resolve a RINEX folder against a session
  ppk/config/f9p_kinematic.conf    # tuned options
  CMakeLists.txt
```

## Prerequisites (Ubuntu / Raspberry Pi OS)

```bash
sudo apt update && sudo apt install -y git cmake build-essential zlib1g-dev
sudo usermod -aG dialout $USER      # serial access without sudo — then log out/in

pip install hatanaka          # provides crx2rnx, for Hatanaka-compressed base RINEX
```

`ppk_solve` un-gzips base RINEX itself via zlib; Hatanaka (`.YYd`/`.crx`) needs
`crx2rnx` on `PATH`, which the `hatanaka` package installs.

Ubuntu and Raspberry Pi OS are both Debian-based and the code is plain POSIX, so
the same steps work on each. RTKLIB is built with `-fsigned-char` so it behaves
correctly on ARM (Raspberry Pi), where `char` is unsigned by default.

## Build

```bash
cmake -B build -S .        # ppk fetches + builds RTKLIB demo5 (needs network + git)
cmake --build build -j
```

Binaries land in `build/logger/ubx_logger` and `build/ppk/ppk_solve` (the `.conf`
is copied next to `ppk_solve`).

## 1. Log in the field

```bash
# F9P over USB (CDC-ACM), 5 Hz, until Ctrl+C
./ubx_logger --port /dev/ttyACM0 --out rover.ubx --rate 5

# Over UART1 instead (use a high baud):
./ubx_logger --port /dev/ttyUSB0 --port-target uart1 --baud 460800 --rate 5
```

It enables `UBX-RXM-RAWX` + `UBX-RXM-SFRBX` (RAM layer), **turns NMEA off on
every port**, sets the measurement rate, and prints a live epoch/byte counter so
you can confirm raw data is actually flowing before you drive off. If `RAWX
epochs = 0`, check the `--port-target`, that firmware is HPG, and that the
antenna has sky view.

**Watch that the RAWX counter keeps climbing, not just that it is non-zero.**
If it freezes while `SFRBX` and the byte count keep rising, the receiver is
dropping RXM-RAWX and the log is worthless — the logger now says so after 5 s,
and reports the shortfall (`RAWX epochs 14 of ~22800 expected — 99% MISSING`) at
exit. Neither the receiver's fix nor the byte counter looks wrong when this
happens; see "Why NMEA is off" below.

### Why NMEA is off, and why on every port

The F9P ships with NMEA enabled on all ports, and `CFG-RATE-MEAS` is **global** —
so asking for 5 Hz raw data on USB also asks for 5 Hz NMEA on ports nobody is
reading. The default NMEA set at 5 Hz is ~3.8 kB/s; UART1's factory-default
38400 baud carries 3.84 kB/s. It backs up permanently, and the transmit buffers
it holds come from a pool **shared with USB**. The receiver then cannot allocate
for the tail of each epoch's output and reports `txbuf alloc` once a second,
forever. RXM-RAWX is last in the burst and the largest message (~2 kB/epoch), so
it is dropped every single time.

Measured on a 76-minute F9P session logged off `/dev/ttyACM0`, before this fix:

| | |
|---|---|
| NMEA | 79% of 22 MB logged |
| `$GPGSV` (early in the burst) | 4.29 per epoch — arrived |
| `$GLGSV` (later in the burst) | 0.02 per epoch — dropped, though GLONASS was tracked hard |
| **RXM-RAWX** | **14 epochs of ~22800 — stopped dead after 2.8 s** |

The receiver held a 12-satellite fix and decoded nav subframes continuously for
all 76 minutes. Nothing looked broken. The drive was unpostprocessable.

This is why disabling NMEA on the logged port alone is not enough: on USB the
port has no bandwidth limit of its own, and the port that starves it is UART1.

Notes: for a vehicle, 5–10 Hz is plenty; prefer USB for high-rate multi-GNSS.
A good multi-band antenna with a ground plane matters more than the rate.

**Raspberry Pi serial:** over USB the F9P is `/dev/ttyACM0` exactly as on Ubuntu —
simplest and most reliable, use it if you can. If you instead wire the F9P to the
Pi's GPIO UART, use `/dev/serial0` and first: (1) disable the serial login console
(`sudo raspi-config` → Interface → Serial → login shell **off**, hardware serial
**on**), and (2) map the good PL011 UART to the header by adding
`dtoverlay=disable-bt` to `/boot/firmware/config.txt` — the default mini-UART
(`ttyS0`) drops bytes at the high baud you need for raw multi-GNSS. On a fresh Pi,
build with `-j$(nproc)` (or `-j2` on a Zero/low-RAM board to avoid OOM).

## 2. Download the CORS base for your session

Everything is public HTTPS on the NOAA CORS S3 bucket (`ssss` = 4-char station id,
`DDD` = day-of-year, `YY` = 2-digit year, `h` = UTC hour letter a=00…x=23):

```bash
YMD=2026/196; SS=ala1; YY=26; H=t      # example: Auburn (ALA1), one UTC hour
BASE=https://noaa-cors-pds.s3.amazonaws.com

# hourly base obs (Hatanaka .YYd) — lowest latency (~1 h after the hour)
# (ppk_solve --base-dir reads .gz and Hatanaka directly; no need to decompress)
curl -O $BASE/rinex/$YMD/$SS/${SS}${YMD##*/}${H}.${YY}d.gz
# broadcast nav (optional; SFRBX from the rover already gives nav)
curl -O $BASE/rinex/$YMD/brdc${YMD##*/}0.${YY}n.gz
# published ARP coordinates (use these, NOT the RINEX header approx)
# NOTE: coord_NN is the reference-frame REALIZATION, not a year -- coord_20 is
# ITRF2020 and is current. There is no coord_26; $YY here would 404.
curl -O $BASE/coord/coord_20/${SS}_20.coord.txt
```

The `.coord.txt` is the only place the ARP position is published; it is not on the
data-download page and not derivable from the RINEX. For ALA1 it gives:

```
| ITRF2020 POSITION (EPOCH 2020.0)                                            |
|     X =    421642.670 m     latitude    =  32 35 55.88736 N                 |
|     Y =  -5362176.551 m     longitude   = 085 30 14.13658 W                 |
|     Z =   3416663.333 m     ellipsoid height =  184.100   m                 |
| ITRF2020 VELOCITY                                                           |
|     VX =  -0.0134 m/yr      VY =  0.0024 m/yr      VZ = -0.0007 m/yr        |
| NAD_83 (2011) POSITION (EPOCH 2010.0)                                       |
|     X =    421643.531 m  Y =  -5362178.063 m  Z =   3416663.494 m           |
```

Propagate the ITRF position to your observation epoch with the velocity —
`X + VX·(epoch − 2020.0)`, etc. For 2026.53 that is 6.53 years, which moves X by
−8.75 cm and gives the `--base-ecef 421642.5824 -5362176.5353 3416663.3284` used
throughout this README.

The NAD83(2011) block is **exactly** what the RINEX header's `APPROX POSITION`
contains (verified to 0.0 mm against ALA1's own file) — so the header is not an
approximation at all, it is simply the other datum.

No need to decompress: `--base-dir` reads `.gz` and Hatanaka in place. Drop the
whole download — several days, obs and nav together — into one folder and pass it.
Grab the **nav** too — the F9P's own SFRBX already carries it, so it costs
nothing, but having the CORS `brdc*` in the folder is a fine fallback.

The **daily** file (`…0.YYd`) only exists after the UTC day closes; use the
**hourly** files above for same-session turnaround. Daily ALA1 files are 30 s;
the hourly ones are worth checking if you want a higher base rate.

Station selection: Auburn has **ALA1** (Auburn Univ, ALDOT). Note it is on the
**campus**, which is ~19 km from the NCAT test track — a medium baseline, not the
near-zero one the name suggests, and long enough that residual ionosphere limits
ambiguity resolution with `pos1-ionoopt=brdc`. For Laguna Seca, pick the nearest
1-Hz station from https://geodesy.noaa.gov/CORS_Map/ (aim < ~30 km) and confirm
its data-availability calendar covers your dates *before* you travel.

## 3. Solve

Point `--base-dir` at the folder you downloaded and let it pick what it needs:

```bash
./ppk_solve \
  --rover rover.ubx \
  --base-dir ./rinex \
  --conf  f9p_kinematic.conf \
  --base-ecef 421642.5824 -5362176.5353 3416663.3284 \
  --out ppk.csv
```

This **solves** for position; it does not adjust the receiver's own fix. RTKLIB
forms double-differenced carrier-phase observations between rover and base,
resolves the integer ambiguities, and runs its own Kalman filter. Only raw
observables go in — the receiver's reported positions are never an input, which
is why the output can be compared against them as an independent check.

- `--rover` accepts `.ubx` (auto-converted via RTKLIB `convrnx`) or a RINEX `.obs`.
- `--base-dir` takes a folder of CORS RINEX exactly as it arrives: daily and/or
  hourly, short (`ssssDDDf.YYt`) or long (RINEX 3) names, gzipped and/or
  Hatanaka-compressed, several days and both obs and `brdc*` nav mixed together.
  The rover's own GPS time span selects what is read; everything else is ignored.
  Use `--station SSSS` if the folder holds more than one station.
- `--base-ecef` (or `--base-llh LAT LON H`) comes from the CORS `.coord` file.
  If omitted, the base RINEX header `APPROX POSITION` is used.

### Output

Two CSVs, both solved by the same engine from the same observables:

| file | what it is |
|---|---|
| `ppk_positions.csv` | the corrected track — differential, ambiguities resolved |
| `raw_positions.csv` | the **uncorrected** track — the same observables solved single-point, with no base |

The pair isolates exactly what the corrections bought, and it works for any
receiver: a u-blox log contains only observables and no computed position of its
own, so there would otherwise be nothing to compare against. `--no-raw` skips it.

`raw_positions.csv` is plain SPP on broadcast ephemeris/ionosphere — metre-level
(0.79 m median horizontal here). It is **not** a stand-in for what a receiver
reports in real time: a receiver applying WAAS/SBAS is using corrections that are
not in the observable log at all. So use `raw_positions.csv` to answer "what did
PPK buy me over the raw observables", not "how good was my receiver's own fix".

Both are plain CSV — one header row, then one row per epoch. Nothing to skip, no
`%` preamble. `pd.read_csv(path)` and MATLAB's `readtable(path)` both take it
with no arguments (the old `.pos` needed `FileType='text'` and still fought you).

```
gps_week,gps_tow_s,utc,lat_deg,lon_deg,height_m,ecef_x_m,ecef_y_m,ecef_z_m,q,quality,n_sats,sd_e_m,sd_n_m,sd_u_m,age_s,ar_ratio
2427,172479.000,2026/07/13 23:54:21.000,32.597634904,-85.307425043,163.3108,440034.8352,-5360754.3028,3416537.8962,1,fix,12,0.0301,0.0296,0.1079,9.00,3.3
```

| column | meaning |
|---|---|
| `gps_week`, `gps_tow_s` | GPS time — what you join other GNSS data on |
| `utc` | same instant as UTC, for joining against vehicle logs |
| `lat_deg`, `lon_deg`, `height_m` | geodetic; height is **ellipsoidal**, not MSL |
| `ecef_*_m` | the same position in ECEF |
| `q` / `quality` | 1/`fix`, 2/`float`, 5/`single` — **filter on `quality == "fix"`** |
| `n_sats` | satellites used in the solution |
| `sd_e_m`, `sd_n_m`, `sd_u_m` | 1σ in local **ENU** (rotated from the ECEF covariance) |
| `age_s` | rover-to-base time difference |
| `ar_ratio` | ambiguity-resolution ratio; higher is a more confident fix |

The formal `sd` reads roughly 1.5–2× optimistic — measured scatter was ~1.9 cm
where `sd` claimed ~1 cm. Treat it as a relative quality indicator, not an error bar.

RTKLIB's native `.pos` files (with the long `%` header) are still written to
`<workdir>/` for debugging. Passing a non-`.csv` `--out` puts the `.pos` at that
path instead and writes a `.csv` beside it.

### Day rollovers

A session that crosses UTC midnight needs two daily base files. `ppk_solve`
selects both and merges them into one base stream automatically — you do not
have to pre-concatenate anything:

```
[rover] GPS span 2026/07/13 23:43:56 -> 2026/07/14 03:41:27
[base-dir] station ala1, 2 obs file(s) cover the session:
           ala11940.26d.gz  (2026/194) -> ./ppk_work/base_2026194_00.obs
           ala11950.26d.gz  (2026/195) -> ./ppk_work/base_2026195_01.obs
[base-dir] session spans 2 RINEX files -- merging into one base stream (day rollover).
```

This is not merely cosmetic: handing RTKLIB two base files directly does *not*
work, because `readobsnav()` gives each input file its own receiver index and
would silently treat the second day as a third receiver. The selected files are
staged under one wildcard-matchable name instead, which `readrnxt()` expands
under a single index.

### When the next file does not exist yet

Overlapping the session is not the same as covering it, and the usual reason is
timing: **CORS publishes a daily file only once the UTC day closes**, so a
session that runs past midnight is downloaded with its own last hours missing.
An hour can also be absent from the middle of a download.

Corrections cannot be invented for those epochs. `ppk_solve` clamps the solve to
the first contiguous run of base coverage, **ends the CSV where the corrections
end**, and says so — rather than emitting single-point epochs for the uncovered
tail, which look like solutions and are not:

```
[base-dir] ***** MISSING CORRECTIONS *****
           The base folder does not cover the whole session.
           session : 2026/07/13 23:43:56 -> 2026/07/14 03:41:27  (14251 s)
           base    : 2026/07/13 00:00:00 -> 2026/07/14 00:00:00
           MISSING 13287 s at the END   (2026/07/14 00:00:00 -> 2026/07/14 03:41:27) -- not solved.
           Solving 964 s of 14251 (7%).
...
*** MISSING CORRECTIONS -- ppk_positions.csv ends early ***
    no base for the last  13287 s of the session
    raw_positions.csv still spans the whole session (it needs no base).
```

Both ends are handled — a missing first day truncates the *start* — as is a hole
in the middle, where the stretch after the hole is reported but not used (the
filter would have to re-converge across it anyway; fill the gap and re-run to get
it). `raw_positions.csv` is left at full span, since a single-point solve needs
no base.

The split is exact. On the session above, with both days present it solves 3907
epochs; with day 195 removed, 277; with day 194 removed, 3630. 277 + 3630 = 3907,
with no epoch solved twice and none lost at the boundary — the truncated runs
reproduce the fully-covered solve epoch-for-epoch.

A fully-covered session is unaffected: the window is left unset and the solve is
identical to before.

**Coverage is read from the filename**, which is what selection already trusts.
A file whose *contents* are short of what its name declares will still be
selected; that is a base-station outage, not a missing file, and RTKLIB reports
it as age-of-differential growing.

**Selection is by GPS time, never by the recording host's clock.** On the data
this was built against, the recording machine's clock was ~1.7 days off; trusting
it would have quietly selected the wrong day's base file and produced a solve
that fails for reasons that look like anything but a wrong file.

**Datum:** the track comes out in the frame of the base coordinates you supply.
Use the CORS published ITRF2020 or NAD83(2011) ARP position and know which frame
your answer is in (they differ by ~1–2 m in North America). Two traps:

- A NOAA CORS RINEX header's `APPROX POSITION` is the **NAD83(2011)** value. It
  is the default only because it is always present — it is 1.81 m from ITRF2020
  for ALA1. If you are comparing against a receiver's own WGS84/ITRF solution,
  pass `--base-ecef` with the ITRF position **propagated to the observation
  epoch** using the velocities in the `.coord` file (~9 cm over 6.5 years here).
- Without an antenna file the base's phase-centre offset (~6 cm for ALA1's
  LEIAR10) is not modelled. Add `igs20.atx` and set the antenna types in the
  conf if you need that last few cm of *absolute* accuracy.

## Validate the wrapper once

Confirm `ppk_solve` reproduces the reference engine on the same inputs:

```bash
# build demo5's rnx2rtkp separately, then:
rnx2rtkp -k f9p_kinematic.conf -o ref.pos rover.ubx.obs ala11960.26o brdc1960.26n
```

Compare `ref.pos` and `ppk.pos` — they should match to the last digit. After that
you can trust the wrapper and drive the whole pipeline from your own code.

## Why the fix rate is a config setting, not a GNSS problem

**`pos1-dynamics` is the single biggest lever, and it is a cliff.** It predicts
each epoch from the last with a constant-acceleration model. Across a multi-second
gap on a vehicle that is *turning*, that prediction is simply wrong — the filter
never validates a fix, and the symptom (a noisy float solution) looks nothing like
the cause.

Controlled test on the NCAT truck: one continuous 1669 s block, same satellites,
same geometry, same 19 km baseline, same 30 s base. Only the epoch spacing was
changed, by decimating the observables:

| rover epoch spacing | `dynamics=on` | `dynamics=off` |
|---|---|---|
| 1 s (all epochs) | 96.8% | **98.0%** |
| uniform 4 s | 98.8% | — |
| uniform 10 s | **0.0%** | **97.6%** |
| uniform 30 s | **0.0%** | 57.1% |
| the real bursty gap pattern | **5.2%** | **93.2%** |

Three things fall out of that:

- **It is not the data rate.** Throwing away 75% of the observables *uniformly*
  (1 Hz → 4 s) costs nothing at all. What kills it is long gaps, not sparse data.
- **`dynamics=on` does not degrade gracefully.** It is fine to 4 s and *zero* by
  10 s. And it never pays for itself: even on continuous 1 Hz data it is slightly
  worse than off. Hence `off` is now the shipped default. Turning it on is a
  tuning exercise (`stats-prnaccelh/v` matched to the vehicle), not a free win.
- **It rescued the existing data.** The full session went from 52% → **82.5%** of
  epochs, and 28% → **75% of elapsed time**, with no new field work. The fixes are
  genuine, not false: stationary scatter *improved* to 1.6 cm horizontal rms, and
  the spread against the real-time track is unchanged (0.54 m median, 1.01 m p95).

`ppk_solve` prints the measured epoch spacing every run and warns if
`dynamics=on` meets data it cannot handle.

## Read the fix rate by time, not by epoch

With an irregular observable stream, "% of epochs fixed" flatters the result —
fixes cluster exactly where epochs are dense, so a per-epoch tally over-weights
them. On this session:

| state | epochs | % epochs | % of elapsed time |
|---|---|---|---|
| fixed | 3222 | 82.5% | **75.1%** |
| float | 685 | 17.5% | 24.9% |

**Use `quality == "fix"` and treat everything else as missing** — do not
interpolate through float and do not average it. With `dynamics=off` the float
solution is at least sane (0.55 m median, 2.2 m worst); with `dynamics=on` it was
1.6 m median with excursions to **1.9 km**, which is what a filter that is
permanently re-converging produces.
