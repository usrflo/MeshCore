#pragma once

#include <Mesh.h>
#include <RadioLib.h>
#include <helpers/WindowedPercent.h>

#define NUM_NOISE_FLOOR_SAMPLES  64   // RSSI samples reduced to a median per noise-floor calibration block

#define NOISE_FLOOR_MAX_RISE_DB  15   // block median jumping this far ABOVE the published floor is treated as
                                      // activity-contaminated and held, so the RSSI-margin LBT keeps a meaningful
                                      // (idle) reference while the channel is occupied
#define NOISE_FLOOR_SAMPLE_INTERVAL_MS  50   // min spacing between RSSI samples so a 64-sample block spans a real
                                             // ~3.2 s window, giving the median temporal interference rejection
                                             // instead of collapsing to a few ms of near-simultaneous readings
#define NOISE_FLOOR_MAX_HELD_BLOCKS  3   // after this many consecutive held blocks the median is accepted, so a
                                         // permanent floor rise can't keep it stuck low. Count-based: rides out load
                                         // bursts, a true rise releases in a few blocks.

#define QUIET_FLOOR_BLOCKS  64    // published noise-floor values retained for the quiet-floor percentile. A block
                                  // spans ~3.2 s+ of idle, so this ring covers the last several minutes
#define QUIET_FLOOR_MIN_BLOCKS  8 // ring fill required before the percentile is trusted (before that the busy
                                  // verdict falls back to the current noise floor, as before)
#define CHAN_BUSY_REF_MAX_DB  -100  // absolute cap on the busy-verdict reference floor: ambient noise this high is
                                    // interference, not a quiet channel the node merely adapted to. Keeps a
                                    // multi-hour jammer visible in the utilization even once the ring has filled
                                    // with contaminated medians

// RX-desync watchdog: the `state` variable is firmware-side truth. If the chip silently
// leaves RX (supply dip during TX, SPI glitch, front-end upset) the RAM copy still says
// STATE_RX, so recvRaw() never re-arms and the Dispatcher-side check (reading the same
// variable) stays quiet — the node goes deaf until reboot, and the Current-RSSI register
// freezes at the last energy seen (the "noise floor pinned high" symptom).
#define RX_DESYNC_CHECK_INTERVAL_MS  10000   // cadence of the chip-mode verification poll
#define RX_DESYNC_CONFIRM_TICKS      2       // consecutive bad polls before recovery starts (debounces one misread,
                                             // e.g. the first GetStatus that wakes the chip from warm sleep)
#define RX_DESYNC_FATAL_STREAK       5       // streak that survived sleep-level recovery — flag via ERR_EVENT_RX_DESYNC
#ifdef USE_CC310_HW_CRYPTO
#include <Adafruit_nRFCrypto.h>
#endif
struct PacketMillis {
  uint32_t preambleMillis;  // preamble-detect -> header-valid deadline
  uint32_t payloadMillis;   // header-valid   -> rx-done deadline
};

class RadioLibWrapper : public mesh::Radio {
protected:
  PhysicalLayer* _radio;
  mesh::MainBoard* _board;
  uint32_t n_recv, n_sent, n_recv_errors;
  uint32_t n_recv_errors_strong;   // failures whose SNR says they should have decoded (RX-quality window)
  uint16_t n_rx_desync_events;    // desync episodes detected (recovery was attempted for each)
  uint16_t n_rx_desync_fatals;    // episodes that survived sleep-level recovery (reboot needed)
  int16_t _noise_floor, _threshold;
  bool _cad_enabled;
  uint16_t _num_floor_samples;
  int16_t _floor_samples[NUM_NOISE_FLOOR_SAMPLES];
  bool _floor_block_ready;   // true once a full block has been reduced to a median (waits for trigger to restart)
  uint32_t _last_floor_sample_at;   // millis() of the last accepted RSSI sample (rate-limits block sampling)
  uint8_t _held_block_count;   // consecutive held blocks since the last published noise-floor value
  int16_t _quiet_floor_ring[QUIET_FLOOR_BLOCKS];   // recently published noise-floor medians
  uint8_t _quiet_floor_cnt;    // ring fill level (grows to QUIET_FLOOR_BLOCKS)
  uint8_t _quiet_floor_idx;    // next slot to overwrite
  int16_t _quiet_floor;        // P10 of the ring: the busy-verdict reference (see busyRefFloor())
  uint8_t _preamble_sf;
  uint32_t _last_rx_sync_check;   // millis() of the last chip-mode verification (RX-desync watchdog)
  uint8_t _rx_desync_streak;      // consecutive verifications that found the chip out of RX (0 = healthy)

  // windowed channel-health metrics (sampled in loop())
  WindowedPercent _busy_win;      // channel busy: own TX, mid-receive, or energy above floor + margin
  WindowedPercent _deaf_win;      // radio not in RX (listening) mode
  WindowedPercent _jam_win;       // ambient energy far above the QUIET floor (interference, not our traffic)
  WindowedCountedRatio<> _err_win;  // RX attempts with relevant CRC errors (~10 min window)
  uint32_t _last_metric_ms = 0;       // stamp of previous loop() metric sample
  uint32_t _last_rssi_ms = 0;         // rate limit for the RSSI busy poll
  uint32_t _last_recv_cnt = 0;        // previous packet counter (for deltas)
  uint32_t _last_strong_err_cnt = 0;  // previous SNR-relevant failure counter (for deltas)
  uint32_t _last_rxq_ev_ms = 0;       // millis() of the last RX-quality window event (staleness vs jam)
  bool _cur_busy = false;             // last busy verdict (held between RSSI polls)
  bool _cur_jam = false;              // last ambient-jam verdict (held between RSSI polls)
  bool _rx_snr_latched = false;       // any packet status latched: getLastSNR() is trustworthy

  void idle();
  void startRecv();
  // Reference floor for the channel-busy verdict: the quietest decile of recently
  // published noise floors, absolutely capped. Unlike the adapted _noise_floor
  // (which must follow a sustained interferer for LBT), this stays near the real
  // ambient so a Dauerstoerer keeps the utilization high instead of hiding under
  // its own adapted floor.
  int16_t busyRefFloor();
  float packetScoreInt(float snr, int sf, int packet_len);
  virtual bool isReceivingPacket() =0;
  virtual void doResetAGC();

public:
  RadioLibWrapper(PhysicalLayer& radio, mesh::MainBoard& board) : _radio(&radio), _board(&board), _preamble_sf(0) { n_recv = n_sent = n_recv_errors = n_recv_errors_strong = 0; }

  void begin() override;
  virtual void powerOff() { _radio->sleep(); }
  int recvRaw(uint8_t* bytes, int sz) override;
  uint32_t getEstAirtimeFor(int len_bytes) override;
  bool startSendRaw(const uint8_t* bytes, int len) override;
  bool isSendComplete() override;
  void onSendFinished() override;
  bool isInRecvMode() const override;
  bool isChannelActive();

  bool isReceiving() override {
    if (isReceivingPacket()) return true;

    return isChannelActive();
  }

  virtual void setParams(float freq, float bw, uint8_t sf, uint8_t cr) = 0;
  uint32_t getRngSeed();
  void setTxPower(int8_t dbm);

  virtual float getCurrentRSSI() =0;
  virtual uint8_t getSpreadingFactor() const { return LORA_SF; }
  static uint16_t preambleLengthForSF(uint8_t sf) { return sf <= 8 ? 32 : 16; }
  void updatePreamble(uint8_t sf) { _preamble_sf = sf; _radio->setPreambleLength(preambleLengthForSF(sf)); }
  PacketMillis calcMaxPacketMillis(uint8_t sf, float bw, uint8_t cr, uint8_t preambleSymbols);
  virtual int16_t performChannelScan();

  int getNoiseFloor() const override { return _noise_floor; }
  bool hasChannelHealth() override { return true; }
  uint8_t getChannelUtilizationPct() override { return _busy_win.pct(); }
  uint8_t getRxDeafnessPct() override { return _deaf_win.pct(); }
  void getRxQualityCounts(uint16_t& good, uint16_t& total) override {
    uint16_t ev, bad;
    _err_win.counts(ev, bad);
    total = ev;      // all reception attempts
    good = ev - bad; // ...of which decoded OK
  }
  bool getRxQualityPct(uint8_t& pct) override;   // defined in the .cpp: needs millis() for the jam-staleness policy
  void triggerNoiseFloorCalibrate(int threshold) override;
  void setCADEnabled(bool enable) override { _cad_enabled = enable; }
  void resetAGC() override;

  void loop() override;

  uint32_t getPacketsRecv() const { return n_recv; }
  uint32_t getPacketsRecvErrors() const { return n_recv_errors; }
  uint32_t getPacketsSent() const { return n_sent; }
  // Zeroing the counters without re-stamping the delta bases would underflow
  // the next loop() delta and inject a garbage spike into one ~10 min window
  // bucket, so clear the window and stamps together with the counters. All
  // three channel-health windows are cleared so a stats reset produces a
  // consistent all-metrics snapshot (the 5 s windows refill within seconds).
  void resetStats() {
    n_recv = n_sent = n_recv_errors = n_recv_errors_strong = 0;
    _last_recv_cnt = 0; _last_strong_err_cnt = 0;
    _busy_win.clear(); _deaf_win.clear(); _err_win.clear();
  }

  // RX-desync watchdog. verifyRxChipMode() reads the chip's real operating mode;
  // base assumes RX (no authoritative status register on every radio type).
  // Override in radio-specific wrappers that can check (SX126x GetStatus).
  virtual bool verifyRxChipMode() { return true; }
  bool isRxDamaged() const override { return _rx_desync_streak >= RX_DESYNC_FATAL_STREAK; }
  uint16_t getRxDesyncEvents() const { return n_rx_desync_events; }
  uint16_t getRxDesyncFatals() const { return n_rx_desync_fatals; }

  virtual float getLastRSSI() const override;
  virtual float getLastSNR() const override;

  float packetScore(float snr, int packet_len) override { return packetScoreInt(snr, 10, packet_len); }  // assume sf=10

  virtual bool setRxBoostedGainMode(bool) { return false; }
  virtual bool getRxBoostedGainMode() const { return false; }
  
  virtual bool configSideDetectors(const uint8_t sideDetSFs[], uint8_t num, float bw) { return false; }
};

/**
 * \brief  an RNG impl using the noise from the LoRa radio as entropy.
 *         NOTE: this is VERY SLOW!  Use only for things like creating new LocalIdentity
*/
class RadioNoiseListener : public mesh::RNG {
  PhysicalLayer* _radio;
public:
  RadioNoiseListener(PhysicalLayer& radio): _radio(&radio) { }

  void random(uint8_t* dest, size_t sz) override {
#ifdef USE_CC310_HW_CRYPTO
    nRFCrypto.Random.generate(dest, (uint16_t)sz);
    for (int i = 0; i < sz; i++) {
      dest[i] ^= _radio->randomByte() ^ (::random(0, 256) & 0xFF); // combine with Radio's entropy
    }
#else
    for (int i = 0; i < sz; i++) {
      dest[i] = _radio->randomByte() ^ (::random(0, 256) & 0xFF);
    }
#endif
  }
};
