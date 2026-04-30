# Repeated Sending — Test Documentation

This document records simulation tests that compare two independent implementations of
MeshCore's repeated-sending (retry) mechanism across an identical 3-hop marginal chain.

| Branch | Strategy | Retry actor |
|--------|----------|-------------|
| [`repeated-sending-2`](https://github.com/usrflo/MeshCore/tree/repeated-sending-2) | Distributed hop-by-hop retry | Every node (Companion + each Repeater) retries if next hop is not overheard |
| [`halo-direct-path-retries`](https://github.com/usrflo/MeshCore/tree/halo-direct-path-retries) | Repeater-only slot-based retry | Each repeater retries up to 15×; Companion does not retry |

Both implementations use the same topology, behaviour file, seeds, and simulation
duration, so results are directly comparable.

---

## Implementation A — `repeated-sending-2` (Distributed hop-by-hop retry)

The **same cancellation mechanism runs at every node** — Companion and Repeaters alike.
Every node that forwards a direct packet schedules a retry via `getDirectRetransmitDelay()`;
it cancels that retry as soon as it overhears the *next* hop forwarding the same packet
(same payload hash).  This creates a cascading, hop-local safety net along the full path.

The key difference between node types is the **retry delay**:

| Node type | `getDirectRetransmitDelay()` | Effect |
|-----------|------------------------------|--------|
| Companion (Alice) | `path_len × 300 ms` (e.g. 900 ms for 3 hops) | Waits long enough for the entire chain to propagate before firing TX2; retries end-to-end |
| Repeater (R1/R2/R3) | `random(0, 5 × airtime × direct_tx_delay_factor)` ≈ 0–33 ms | Retries the individual hop quickly if the next hop does not echo within the window |

The **cancellation logic** runs uniformly in `Mesh::onRecvPacket()` for all nodes:
- When a node has a pending retry for a packet (`sending_attempts > 0`) and overhears
  any downstream node forwarding that same packet, it removes the retry from the outbound queue.
- This fires regardless of `path_len` (including `path_len == 0`, i.e. the final hop to the destination).

---

## Implementation B — `halo-direct-path-retries` (Repeater-side retry)

Each **Repeater** independently retries forwarding a direct packet if it does not hear
an echo from its next hop within a computed window.  The Companion firmware does **not**
override `allowDirectRetry()`, so Alice herself never retries — only the intermediate
hops do.

Key parameters (default prefs in `examples/simple_repeater/MyMesh.cpp`):

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `direct_retry_attempts` | 15 | Max retries per repeater per packet |
| `direct_retry_base_ms` | 200 ms | Minimum echo-wait window |
| Echo-wait formula | `base + (bits × 4 / kbps)` | Scales with packet size and radio rate |
| Attempt delay | `base + attempt_idx × 100 ms` | Linear back-off: 200 ms, 300 ms, 400 ms … |
| `direct_retry_recent_enabled` | 1 | Uses recent-prefix cache for next-hop lookup |
| `direct_retry_snr_margin_db` | 2.5 dB | Min SNR margin to allow retry for a given hop |

The retry slot table (`MAX_DIRECT_RETRY_SLOTS = 6`) is managed in `src/Mesh.cpp` via
`armDirectRetryOnSendComplete()` / `cancelDirectRetryOnEcho()` / `clearDirectRetrySlot()`.

---

## Test Setup (identical for both implementations)

### Topology — `examples/topologies/retry_showcase.yaml`

Seven nodes, three marginal forward hops, reliable return path:

```
                         Observer
                     (not on path, R2 neighbor)
                          │
                       12 dB ±1σ
                          │
Alice ─15dB±2σ─ R1 ─-3dB±4σ─► R2 ─-3dB±4σ─► R3 ─-3dB±4σ─► Bob ─15dB±2σ─ Sidecar
       ◄─15dB±2σ─    ◄─10dB±2σ──    ◄─10dB±2σ──    ◄─10dB±2σ──           (not on path)
```

**Radio parameters:** 910.525 MHz · SF7 · BW 62.5 kHz · CR 5 · 20 dBm TX  
**SNR threshold (SF7):** −7.5 dB

| Link direction | SNR at 20 dBm | σ | P(success/packet) |
|----------------|--------------|---|-------------------|
| Alice → R1 | +15.0 dB | 2.0 | ~100 % |
| R1 → R2 | −3.0 dB | 4.0 | **~87 %** (marginal) |
| R2 → R3 | −3.0 dB | 4.0 | **~87 %** (marginal) |
| R3 → Bob | −3.0 dB | 4.0 | **~87 %** (marginal) |
| Return path (all hops) | +10 to +15 dB | 1–2 | ~100 % |

**Node roles:**

| Node | Firmware | Role |
|------|----------|------|
| Alice | Companion | DM sender |
| R1, R2, R3 | Repeater | Forwarding chain |
| Bob | Companion | DM receiver |
| Observer | Repeater | R2 neighbor, not on path |
| Sidecar | Repeater | Bob neighbor, not on path |

### Behavior — `examples/behaviors/retry_stress.yaml`

Alice sends **20 DMs to Bob**, one per 10 s, starting after a 90 s warmup
(allows route/path discovery to complete before the first message).

```
startup_s:            90
session_interval_s:   10
message_count:        20
session_message_count: 1
```

### Theoretical Predictions

| Metric | Calculation | Expected |
|--------|-------------|---------|
| P(delivered \| 1 attempt, 3 hops) | 0.87³ | ~66 % |
| P(delivered \| 3 sender attempts) | 1 − (1 − 0.66)³ | **~96 %** |
| P(delivered \| 15 per-hop repeater retries) | (1 − 0.13¹⁵)³ | **~100 %** (theoretical) |

Without any retry the expected ACK rate on this topology is **~66 %**.

---

## Test Execution

```bash
for seed in 1234 5678 9012 3456 7890 2345 6789 4321 8765 1111; do
  result=$(cargo run -q -- run \
    examples/topologies/retry_showcase.yaml \
    examples/behaviors/retry_stress.yaml   \
    --duration 350s --seed $seed --metrics-output json 2>/dev/null \
  | python3 -c "
import sys, json
d = json.load(sys.stdin)['counters']
print(d.get('mcsim.dm.delivered',0),
      d.get('mcsim.dm.acked',0),
      d.get('mcsim.packet.tx_direct',0))")
  echo "seed=$seed  del/ack/tx: $result"
done
```

---

## Results — Implementation A (`repeated-sending-2`, Sender-side retry)

*Alice retries up to 3× with `path_len × 300 ms` delay; early cancellation on relay echo.*

| Seed | Delivered | Acked | TX (direct) |
|------|----------:|------:|------------:|
| 1234 | 20 / 20 | 20 / 20 | 187 |
| 5678 | 20 / 20 | 18 / 20 | 198 |
| 9012 | 20 / 20 | 20 / 20 | 181 |
| 3456 | 20 / 20 | 20 / 20 | 191 |
| 7890 | 19 / 20 | 19 / 20 | 187 |
| 2345 | 18 / 20 | 18 / 20 | 175 |
| 6789 | 18 / 20 | 18 / 20 | 173 |
| 4321 | 19 / 20 | 19 / 20 | 187 |
| 8765 | 20 / 20 | 20 / 20 | 194 |
| 1111 | 18 / 20 | 18 / 20 | 182 |
| **Total** | **192 / 200** | **190 / 200** | **1855** |
| **Rate** | **96.0 %** | **95.0 %** | **~9.3 TX/msg** |

---

## Results — Implementation B (`halo-direct-path-retries`, Repeater-only slot-based retry)

*Each repeater retries forwarding up to 15× if no echo is heard; Alice does not retry.*

| Seed | Delivered | Acked | TX (direct) |
|------|----------:|------:|------------:|
| 1234 | 15 / 20 | 15 / 20 | 342 |
| 5678 | 16 / 20 | 15 / 20 | 436 |
| 9012 | 19 / 20 | 18 / 20 | 509 |
| 3456 | 16 / 20 | 16 / 20 | 443 |
| 7890 | 18 / 20 | 16 / 20 | 459 |
| 2345 | 16 / 20 | 15 / 20 | 370 |
| 6789 | 14 / 20 | 13 / 20 | 410 |
| 4321 | 17 / 20 | 17 / 20 | 428 |
| 8765 | 15 / 20 | 15 / 20 | 378 |
| 1111 | 13 / 20 | 12 / 20 | 430 |
| **Total** | **159 / 200** | **152 / 200** | **4205** |
| **Rate** | **79.5 %** | **76.0 %** | **~26.4 TX/msg** |

---

## Comparison

| Metric | Implementation A (`repeated-sending-2`) | Implementation B (`halo-direct-path-retries`) | Baseline (no retry) |
|--------|----------------------------------------|-----------------------------------------------|---------------------|
| Delivery rate | **96.0 %** | 79.5 % | ~66 % |
| ACK rate | **95.0 %** | 76.0 % | ~66 % |
| TX per message | **~9.3** | ~26.4 | ~3 |
| Matches theory | Yes (predicted 96 %) | No (predicted ~100 %, got 79.5 %) | — |
| Retry actor | Every node (Companion + Repeaters) | Repeaters only (Companion does not retry) | — |
| Retry delay | Companion: `path_len × 300 ms`; Repeaters: short hop-local window | Per-hop: 200–400 ms linear back-off (15 attempts) | — |
| Cancellation trigger | Next-hop echo (uniform for all nodes) | Next-hop echo (slot-based, per repeater) | — |

### Why Implementation B underperforms

1. **Companion does not retry.**  In `halo-direct-path-retries`, `Mesh::allowDirectRetry()`
   returns `false` by default, and the Companion firmware does not override it.  Alice
   sends each DM exactly once.  If the full chain fails on all three of Alice's attempts,
   there is no end-to-end safety net — unlike Implementation A where Alice retries the
   entire path.

2. **Channel congestion from high per-hop retry counts.**  With up to 15 retries per
   repeater at 200–400 ms intervals, each message can generate on average ~26 direct
   transmissions.  On the three marginal hops sharing a single channel, the resulting
   retry storms cause additional collisions that reduce rather than improve delivery —
   a self-defeating effect.

3. **Theoretical prediction not reached.**  While 15 per-hop retries should yield
   $(1 - 0.13^{15})^3 \approx 100\,\%$ in isolation, the collision overhead prevents
   this in practice.

4. **Echo-wait window not tuned for path length.**  The 200 ms base is derived from
   local radio parameters only.  For a 3-hop chain, the repeater needs to wait long
   enough for its forwarded packet to traverse the remaining hops AND for the echo to
   travel back — which takes several hundred milliseconds more than 200 ms.

### Why Implementation A works well

- **Every node participates with appropriate timing.**  Repeaters retry the local hop
  quickly (short window ≈33 ms); Alice retries the end-to-end path with a long,
  path-length-aware delay (`path_len × 300 ms = 900 ms` for 3 hops).  Both layers
  reinforce each other without competing.
- **Uniform cancellation mechanism.**  The same `onRecvPacket()` code cancels retries
  at every node the moment the next hop's echo is heard.  No special coordination is
  required between Companion and Repeater firmware.
- **Bounded airtime.**  The hop-local repeater retries fire only within a short window
  and are quickly cancelled; Alice's retries fire only if the whole chain failed.
  Combined channel load is ~9.3 TX/msg — far less than Impl. B's ~26.4.
- **Bounded worst-case window.**  Alice: 3 attempts × ~1.2 s spacing = ~2.4 s, well
  inside the 10 s message interval.

### Recommended approach

For a multi-hop topology with marginal forward links and reliable return links,
**distributed hop-by-hop retry with path-length-aware Companion delay (Implementation A)**
is more effective and more channel-efficient than repeater-only slot-based retry.
Repeater-only retry (Impl. B) may suit topologies where the Companion cannot hear any
relay echo, but should use far fewer retries (e.g. 2–3) and longer back-off to avoid
congestion.

---

## Files

| File | Purpose |
|------|---------|
| `examples/topologies/retry_showcase.yaml` | 3-hop marginal topology used for both tests |
| `examples/behaviors/retry_stress.yaml` | 20-DM workload, 10 s interval, 90 s warmup |
