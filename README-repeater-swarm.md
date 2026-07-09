# Repeater Swarm Relay (experimental)

> Branch `feature/repeater-swarm-2` · example `simple_repeater` (Heltec tracker v2) · opt-in via
> `direct.swarm`. This is **not** part of upstream `dev` yet. CLI knobs live in
> [`docs/cli_commands.md`](docs/cli_commands.md) under *Routing → [Experimental]*.

## What it does

A repeater that **overhears** a DIRECT packet it is *not* the next-hop of can schedule a single
shortened retransmit that **imitates the addressed next-hop's forward**. If the legit forward then
happens on air, the relay is **cancelled**; if the addressed hop is silent (the next repeater is
off, deaf, or the link stalled), the relay **fires** and carries the packet forward.

The effect: every hop of a DIRECT path is backed up by neighbouring repeaters, **at most one extra
TX per stalled hop**, without flooding and without the sender-side resend that deafens the
half-duplex radio.

This document focuses on the two rescue paths that complete the design — **final-hop rescue** and
**ACK swarm** — and how they relate to the core cancel/re-arm logic.

## The moving parts

Overhearing repeater **R** hears a DIRECT packet `[A, B, …, dest]` where `A` (path position 0) is
the addressed next-hop and R is **not** A.

| Stage | What happens |
|---|---|
| **Gate** | R relays only if it can both *hear A's forward* (to cancel) and *reach the target* (to deliver). Positional: `A` must be a neighbour; the relay target (pos 1 after the trim) must be `dest` or a neighbour. SNR: R→A ≥ `swarm.snr_a`, R→target ≥ `swarm.snr_b` (skipped when target is `dest`). |
| **Target** | `trimPathFront(1)` — strip `A`, so the relay looks exactly like A's forward (`[B, …, dest]` or `[]` → direct to `dest`). |
| **Timing** | `yield (3·airtime)` clears A's legit forward window, then an SNR-to-target slot (best R→target link fires first), then jitter. |
| **LBT** | Before firing, `isResendChannelActive()` (RX-energy / receiving check, **no CAD**) so RX stays open to overhear a late cancel. |
| **Cancel** | On any overheard copy with the same payload hash, compare path counts (see table below). |
| **Re-arm** | After a cancel/fire, R may schedule a relay for the *next* hop, dedup'd per `(payload, hop)` so each hop is covered at most once. |

### Path-aware cancellation

`c` = path count of the overheard copy, `pc` = planned path count of the queued relay
(`c`-1 of the copy that scheduled it). `short_path` ≡ `pc ≤ 1` (the relay's target is the last
repeater before `dest`, i.e. the next copy is the `[]` delivery).

| overheard `c` vs queued `pc` | short path (`pc ≤ 1`) | long path (`pc ≥ 2`) |
|---|---|---|
| `c < pc` | impossible | **cancel** — downstream forward progressed past the relay's target |
| `c == pc`, within yield | **cancel** — A's forward reached `dest` (or another relay) | **keep** — that was A's legit forward; the next hop is still pending |
| `c == pc`, past yield | **cancel** — another neighbour's relay | **cancel** — another neighbour's relay |
| `c > pc` | ignore (the original / a longer-path copy) | ignore |

## Final-hop rescue (c == 1)

Earlier versions required `count ≥ 2` to schedule a relay, i.e. they only backed up hops where a
*repeater* was the target. The **last** hop — where the overheard copy is `[A, dest]` (count 1) and
the relay would deliver `[]` straight to `dest` — was excluded.

That exclusion drops the packet exactly when the **last** repeater fails:

```
MeshL1New ──[ND1_R1, BB1_R4]──► ND1_R1 ──[BB1_R4]──► BB1_R4 ──[]──► WL1_C2
                                                            
                       SB1_R2  (swarm amp; neighbour of ND1_R1 AND BB1_R4,
                                and sitting right next to WL1_C2)
```

- **ND1_R1 off** → hop 1 stalls, but the overheard send is `[ND1_R1, BB1_R4]` (c = 2 ≥ 2): SB1_R2
  relays `[BB1_R4]`. ✅ already worked.
- **BB1_R4 off** → hop 2's overheard copy is `[BB1_R4]` (c = 1): the old `count ≥ 2` gate released
  it, nobody delivered to WL1_C2. ❌ stalled.

The fix lowers the gate to `count ≥ 1`. SB1_R2 now schedules a `[]` relay delivering directly to
WL1_C2. Two paired edits keep it DUP-free:

- cancel scan: `short_path = (pc ≤ 1)` — a queued `[]` relay (pc = 0) cancels on the matching `[]`
  delivery;
- schedule: `short_path = (c ≤ 2)` — at c = 1 there is no pos-2 repeater to reach-check (pos 2 *is*
  `dest`), so the SNR-B check is skipped.

**Ideal case (all repeaters on), per packet:**

| t | overheard | SB1_R2 queued relay | action |
|---|---|---|---|
| t0 | `[ND1_R1, BB1_R4]` (c=2) | schedule `[BB1_R4]` (pc=1) | — |
| t1 | ND1_R1 fwd `[BB1_R4]` (c=1) | — | **cancel** pc=1 (c==pc, short); re-arm → schedule `[]` (pc=0) |
| t2 | BB1_R4 fwd `[]` (c=0) | — | **cancel** pc=0 (c==pc, short) |

Net: SB1_R2 fires **nothing** — two relay cycles, both cancelled on the legit forwards. No DUP.
With **BB1_R4 off**, the t2 cancel never comes; SB1_R2's yield expires and it fires `[]` → WL1_C2
gets the message. The pos-1 (A = BB1_R4) neighbour/SNR gate still applies, because SB1_R2 must be
able to *hear* BB1_R4's forward to cancel against it.

## ACK swarm (the return path)

A DIRECT ACK is itself a DIRECT packet carrying the **reverse** path, so the same machinery rescues
it. The ACK was previously excluded (`PAYLOAD_TYPE_ACK`) "to avoid ACK amplification" — a concern
from older, unbounded relay logic. Under the bounded logic here (one relay per payload·hop,
cancel-on-overheard via the ACK's `ack_crc` `isRetryMatch`, SNR gate, `wasSeen`) the amplification
is controlled, so the exclusion was removed.

How the ACK travels (see `BaseChatMesh::sendAckTo`, `MyMesh.cpp` ACK on TXT receipt):

- **`out_path` known** → `sendDirect(ack, out_path, …)`: the ACK runs back as a DIRECT packet over
  the reverse path. This is the case swarm rescues — with BB1_R4 off, SB1_R2 relays the ACK hop
  past BB1_R4 exactly as it relays a data packet.
- **`out_path` unknown** → flooded: the ACK reaches the sender on any path anyway, so swarm is not
  needed (and does not apply to floods).

Note `PAYLOAD_TYPE_MULTIPART` (multi-ACKs when `extraAckTransmitCount > 0`) was already
swarm-relayed; plain ACKs now behave the same.

## Self-in-path exclusion

A relay scheduled by a node that is **itself a later hop** of the overheard packet is structurally
un-cancellable: every cancelling copy (a shorter-path forward) is addressed *to* that node, so the
node consumes it via the normal next-hop forward path and it never reaches the cancel scan. The
gate therefore scans positions 1…count−1 and releases if R finds itself there. (Position 0 — the
addressed next-hop — is already excluded at the `onRecvPacket` entry, since that node forwards
normally.) This is what removed the late DUPs in an ideal, fully-meshed setup.

## Configuration (simple_repeater)

| CLI | default | meaning |
|---|---|---|
| `set direct.swarm on\|off` | `on` | master switch for neighbour-swarm relay (data **and** ACK) |
| `set swarm.snr_a <dB>` | `6` | min R→A SNR to hear the forward and cancel reliably (overload lever) |
| `set swarm.snr_b <dB>` | `6` | min R→target SNR for the relay to deliver (delivery lever) |

SNR values are signed dB (allow negative for marginal environments). Raising `snr_a` cuts redundant
fires; raising `snr_b` tightens delivery but, set too high, excludes useful helpers. The neighbour
table is populated passively from zero-hop repeater adverts, so give the fleet a boot window before
testing.

## Building

```
DISABLE_DEBUG=1 FIRMWARE_VERSION=v1.16.0-swarm2 \
  bash build.sh build-firmware heltec_tracker_v2_repeater
```

Only `simple_repeater` overrides the swarm virtuals (opts in). Other targets carry the
`direct_swarm_fwd` field inert (default off) and compile unchanged.

## Caveats

- Swarm relay covers hops that a repeater can actually **overhear and reach**. If no neighbour
  decodes the stalled hop, no relay logic can help — that is a topology gap (add a side-repeater).
- ACK swarm helps only when the recipient **direct**-ACKs via a known `out_path`. Unknown `out_path`
  ⇒ flood ACK, which already succeeds.
- The simulator-side wiring (`direct_swarm_fwd` model property + `swarm_relay_snr_a/b`) lights up
  automatically once `CommonCLI.h` carries the field; see the `mcsim` parent repo.
