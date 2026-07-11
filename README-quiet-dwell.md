# Quiet-Dwell TX Gate

A core-only, **always-on** transmit gate that lets a recent interferer clear before a packet is sent, so TX (including resends and forwards) lands in a genuinely quiet slot. No runtime configuration, no per-device tuning, no changes to `CommonCLI` or the examples.

## What it achieves

LoRa's hardware CAD is a *preamble detector*: it sees LoRa preambles but is blind to other energy — mid-packet traffic, collisions, and non-LoRa interferers slip past. On a shared channel this produces back-to-back collisions where two nodes each see the channel as "free" and transmit into each other. The quiet-dwell gate closes that gap:

- **Fewer collisions** — TX is deferred until the channel has been *observed* quiet for a dwell window, not merely instantaneously free.
- **Robustness against hidden / non-LoRa energy** — detection is RSSI-margin-based, so it catches interferers CAD misses.
- **No configuration burden** — one fixed margin drives both mechanisms; nothing for the user (or companion app) to set.

## How it works

Two protections share a single fixed margin (`DWELL_RSSI_MARGIN = 15 dB` above the calibrated noise floor):

1. **Quiet-dwell gate** (`Dispatcher::loop` / `checkSend`)
   Every `DWELL_SAMPLE_INTERVAL` (50 ms), while the radio is in RX, `isChannelNoisy()` probes live channel energy (cheap RSSI read, **no CAD — RX stays open**, excludes an in-progress legit RX). If energy exceeds the margin, the timestamp `last_channel_noisy_ms` is refreshed.
   In `checkSend`, even when the channel is instantaneously free, a TX is deferred if it was noisy within the last dwell window (`QUIET_DWELL_MS_DEFAULT = 300 ms`). The TX is rescheduled `dwell - since` ms out so it lands once the interferer has cleared. `getCADFailMaxDuration()` caps the deferral so a persistently loud channel cannot starve the node.

2. **Instantaneous LBT** (`RadioLibWrapper::isChannelActive`)
   The clear-channel check now uses the same fixed margin over the noise floor instead of the (configurable) interference threshold. Because it is modulation-blind, it also catches mid-packet and non-LoRa energy. Instantaneous LBT and the dwell gate thus share one definition of "channel occupied."

## Configuration

There is none at runtime. All three knobs are compile-time `#define`s with defaults, overridable via the build:

| Define                  | Default | Meaning                                                              |
|-------------------------|---------|----------------------------------------------------------------------|
| `QUIET_DWELL_MS_DEFAULT`| 300     | ms the channel must be quiet before a TX is allowed                  |
| `DWELL_SAMPLE_INTERVAL` | 50      | ms between quiet-dwell RSSI probes                                   |
| `DWELL_RSSI_MARGIN`      | 15      | dB above the calibrated noise floor at which the channel counts as "noisy" |

The dwell gate is active whenever `getQuietDwellMs() > 0`. The base `isChannelNoisy()` returns `false`, so mock/simulated radios never trip the gate — only real `RadioLibWrapper` hardware does.

> Note: the configurable interference threshold (`_threshold`) is decoupled from both mechanisms and is now dead storage on this branch; the fixed margin replaces it.

## Files

- `src/Dispatcher.cpp` / `src/Dispatcher.h` — dwell sampling in `loop()`, TX deferral in `checkSend()`, `getQuietDwellMs()`.
- `src/helpers/radiolib/RadioLibWrappers.cpp` / `.h` — `isChannelNoisy()`, fixed-margin `isChannelActive()`.

Branch: `feature/quiet-dwell` (dev + 2 commits). Self-contained and core-only — merges cleanly into any feature branch.
