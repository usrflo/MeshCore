# Quiet-Dwell TX Gate

A core-only, **always-on** transmit gate that lets a recent interferer clear before a packet is sent, so TX (including resends and forwards) lands in a genuinely quiet slot. No runtime configuration, no per-device tuning, no changes to `CommonCLI` or the examples.

## What it achieves

LoRa's hardware CAD is a *preamble detector*: it sees LoRa preambles but is blind to other energy — mid-packet traffic, collisions, and non-LoRa interferers slip past. On a shared channel this produces back-to-back collisions where two nodes each see the channel as "free" and transmit into each other. The quiet-dwell gate closes that gap:

- **Fewer collisions** — TX is deferred until the channel has been *observed* quiet for a dwell window, not merely instantaneously free.
- **Robustness against hidden / non-LoRa energy** — detection is RSSI-margin-based, so it catches interferers CAD misses.
- **No configuration burden** — one fixed margin drives both mechanisms; nothing for the user (or companion app) to set.

## How it works

Two protections share one **SF-scaled** margin above the calibrated noise floor (18 dB at SF7, −1 dB per SF above 7, floored at 12 dB — see `getDwellRssiMargin()`). The margin band stays above the noise floor, so **sub-noise-floor weak LoRa is deliberately not caught** — handling those marginal signals is left to CAD, which is disabled here (see *Scope* below).

1. **Quiet-dwell gate** (`Dispatcher::loop` / `checkSend`)
   Every `DWELL_SAMPLE_INTERVAL` (50 ms), while the radio is in RX, `isChannelNoisy()` probes live channel energy (cheap RSSI read, **no CAD — RX stays open**, excludes an in-progress legit RX). If energy exceeds the margin, the timestamp `last_channel_noisy_ms` is refreshed.
   In `checkSend`, even when the channel is instantaneously free, a TX is deferred if it was noisy within the last **airtime-scaled** dwell window (`getQuietDwellMs()` = `airtime/2`, clamped to 150–600 ms; e.g. ~150 ms at SF7/BW125, ~600 ms at SF12/BW125). The TX is rescheduled `dwell - since` ms out so it lands once the interferer has cleared. Because a *persistently* loud channel would refresh `last_channel_noisy_ms` forever, the **cumulative** deferral of one continuous busy streak is capped by `getCADFailMaxDuration()` (default 4 s): past that the node TXes despite recent noise rather than starving.

2. **Instantaneous LBT** (`RadioLibWrapper::isChannelActive`)
   The clear-channel check uses the same SF-scaled margin over the noise floor instead of the (configurable) interference threshold. Because it is modulation-blind, it also catches mid-packet and non-LoRa energy. Instantaneous LBT and the dwell gate thus share one definition of "channel occupied."

### Prerequisite

A stable noise floor is **required** — the margin is noise-floor-relative. The noise-floor ratchet drift fix (`fix/noise-floor-ratchet`, median filter) must be merged in alongside this branch; without it the margin drifts into pure noise and produces mass false-busies.

### Scope (weak signals)

CAD is the only mechanism that could detect sub-noise-floor LoRa (it has preamble processing gain). It is **disabled** on this branch: empirically CAD caused notable packet loss here, so weak-signal handling is set aside for separate investigation. Consequence: the dwell gate reliably avoids collisions with **strong / mid-band / non-LoRa / mid-packet** energy, but cannot see a marginal/hidden node whose signal sits below the noise floor — the dwell window is the only probabilistic backstop against it.

## Configuration

There is none at runtime. All knobs are compile-time `#define`s with defaults, overridable via the build:

| Define                    | Default | Meaning                                                                                       |
|---------------------------|---------|-----------------------------------------------------------------------------------------------|
| `DWELL_AIRTIME_LEN`       | 64      | payload length (bytes) used to estimate the dwell airtime                                     |
| `DWELL_AIRTIME_DIVISOR`   | 2       | dwell ≈ `airtime / DIVISOR`, before clamping                                                  |
| `DWELL_MS_MIN`            | 150     | min dwell (ms) — fast/low-SF nets                                                             |
| `DWELL_MS_MAX`            | 600     | max dwell (ms) — slow/high-SF nets; bounds added TX latency                                   |
| `DWELL_SAMPLE_INTERVAL`   | 50      | ms between quiet-dwell RSSI probes                                                            |
| `DWELL_RSSI_MARGIN_BASE`  | 18      | dB above noise floor at SF7                                                                   |
| `DWELL_RSSI_MARGIN_STEP`  | 1       | dB subtracted per SF above 7                                                                  |
| `DWELL_RSSI_MARGIN_MIN`   | 12      | floor for the margin at high SF                                                               |

The dwell gate is active whenever `getQuietDwellMs() > 0` (always, given the clamps). The base `isChannelNoisy()` returns `false`, so mock/simulated radios never trip the gate — only real `RadioLibWrapper` hardware does.

> Note: the configurable interference threshold (`_threshold`) is decoupled from both mechanisms and is now dead storage on this branch; the SF-scaled margin replaces it.

## Files

- `src/Dispatcher.cpp` / `src/Dispatcher.h` — dwell sampling in `loop()`, TX deferral in `checkSend()`, `getQuietDwellMs()`.
- `src/helpers/radiolib/RadioLibWrappers.cpp` / `.h` — `isChannelNoisy()`, fixed-margin `isChannelActive()`.

Branch: `feature/quiet-dwell` (dev + 2 commits). Self-contained and core-only — merges cleanly into any feature branch.
