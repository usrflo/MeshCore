#pragma once

#include <MeshCore.h>
#include <Identity.h>
#include <Packet.h>
#include <Utils.h>
#include <string.h>

// Quiet-dwell TX gate — always on, AIRTIME-SCALED (no runtime configuration).
// How long the channel must be quiet (energy below the dwell margin) before a TX is allowed,
// derived from the estimated packet air-time so fast (low-SF) nets get a short dwell and slow
// (high-SF) nets a longer one — see Dispatcher::getQuietDwellMs(). Both ends clamped.
#ifndef DWELL_AIRTIME_LEN
  #define DWELL_AIRTIME_LEN     64      // representative payload length (bytes) for the airtime estimate
#endif
#ifndef DWELL_AIRTIME_DIVISOR
  #define DWELL_AIRTIME_DIVISOR 2       // dwell ≈ airtime / DIVISOR
#endif
#ifndef DWELL_MS_MIN
  #define DWELL_MS_MIN          150     // floor (fast nets) — also ≥ ~3 dwell-RSSI samples at 50 ms
#endif
#ifndef DWELL_MS_MAX
  #define DWELL_MS_MAX          600     // ceiling (slow/high-SF nets) — bounds added TX latency
#endif

namespace mesh {

/**
 * \brief  Abstraction of local/volatile clock with Millisecond granularity.
*/
class MillisecondClock {
public:
  virtual unsigned long getMillis() = 0;
};

/**
 * \brief  Abstraction of this device's packet radio.
*/
class Radio {
public:
  virtual void begin() { }

  /**
   * \brief  polls for incoming raw packet.
   * \param  bytes  destination to store incoming raw packet.
   * \param  sz   maximum packet size allowed.
   * \returns 0 if no incoming data, otherwise length of complete packet received.
  */
  virtual int recvRaw(uint8_t* bytes, int sz) = 0;

  /**
   * \returns  estimated transmit air-time needed for packet of 'len_bytes', in milliseconds.
  */
  virtual uint32_t getEstAirtimeFor(int len_bytes) = 0;

  virtual float packetScore(float snr, int packet_len) = 0;

  /**
   * \brief  starts the raw packet send. (no wait)
   * \param  bytes   the raw packet data
   * \param  len  the length in bytes
   * \returns true if successfully started
  */
  virtual bool startSendRaw(const uint8_t* bytes, int len) = 0;

  /**
   * \returns true if the previous 'startSendRaw()' completed successfully.
  */
  virtual bool isSendComplete() = 0;

  /**
   * \brief  a hook for doing any necessary clean up after transmit.
  */
  virtual void onSendFinished() = 0;

  /**
   * \brief  do any processing needed on each loop cycle
   */
  virtual void loop() { }

  virtual int getNoiseFloor() const { return 0; }

  virtual void triggerNoiseFloorCalibrate(int threshold) { }

  virtual void setCADEnabled(bool enable) { }

  virtual void resetAGC() { }

  virtual bool isInRecvMode() const = 0;

  /**
   * \returns  true if the radio is currently mid-receive of a packet.
  */
  virtual bool isReceiving() { return false; }

  /**
   * \brief  Cheap, non-invasive live-energy probe for the quiet-dwell gate: is the channel
   *         currently carrying energy above a fixed margin over the noise floor (RSSI-margin,
   *         no CAD — RX stays open)? Default false (unknown / unsupported) so mock/simulated
   *         radios never trip the gate.
  */
  virtual bool isChannelNoisy() { return false; }

  virtual float getLastRSSI() const { return 0; }
  virtual float getLastSNR() const { return 0; }
};

/**
 * \brief  An abstraction for managing instances of Packets (eg. in a static pool),
 *        and for managing the outbound packet queue.
*/
class PacketManager {
public:
  virtual Packet* allocNew() = 0;
  virtual void free(Packet* packet) = 0;

  virtual void queueOutbound(Packet* packet, uint8_t priority, uint32_t scheduled_for) = 0;
  virtual Packet* getNextOutbound(uint32_t now) = 0;    // by priority
  virtual int getOutboundCount(uint32_t now) const = 0;
  virtual int getOutboundTotal() const = 0;
  virtual int getFreeCount() const = 0;
  virtual Packet* getOutboundByIdx(int i) = 0;
  virtual Packet* removeOutboundByIdx(int i) = 0;
  virtual void queueInbound(Packet* packet, uint32_t scheduled_for) = 0;
  virtual Packet* getNextInbound(uint32_t now) = 0;
};

typedef uint32_t  DispatcherAction;

#define ACTION_RELEASE           (0)
#define ACTION_MANUAL_HOLD       (1)
#define ACTION_RETRANSMIT(pri)   (((uint32_t)1 + (pri))<<24)
#define ACTION_RETRANSMIT_DELAYED(pri, _delay)  ((((uint32_t)1 + (pri))<<24) | (_delay))

#define ERR_EVENT_FULL              (1 << 0)
#define ERR_EVENT_CAD_TIMEOUT       (1 << 1)
#define ERR_EVENT_STARTRX_TIMEOUT   (1 << 2)

/**
 * \brief  The low-level task that manages detecting incoming Packets, and the queueing
 *      and scheduling of outbound Packets.
*/
class Dispatcher {
  Packet* outbound;  // current outbound packet
  unsigned long outbound_expiry, outbound_start, total_air_time, rx_air_time;
  unsigned long next_tx_time;
  unsigned long cad_busy_start;
  unsigned long radio_nonrx_start;
  unsigned long next_floor_calib_time, next_agc_reset_time;
  unsigned long last_channel_noisy_ms;    // 0 = channel has not been seen noisy since boot
  unsigned long next_dwell_sample_ms;     // throttles the isChannelNoisy() RSSI probe
  unsigned long dwell_starve_start_ms;    // start of the current continuous busy streak (0 = none); drives the dwell starvation cap
  bool  prev_isrecv_mode;
  uint32_t n_sent_flood, n_sent_direct;
  uint32_t n_recv_flood, n_recv_direct;
  unsigned long tx_budget_ms;
  unsigned long last_budget_update;
  unsigned long duty_cycle_window_ms;

  void processRecvPacket(Packet* pkt);
  void updateTxBudget();

protected:
  PacketManager* _mgr;
  Radio* _radio;
  MillisecondClock* _ms;
  uint16_t _err_flags;

  Dispatcher(Radio& radio, MillisecondClock& ms, PacketManager& mgr)
    : _radio(&radio), _ms(&ms), _mgr(&mgr)
  {
    outbound = NULL;
    total_air_time = rx_air_time = 0;
    next_tx_time = ms.getMillis();
    cad_busy_start = 0;
    next_floor_calib_time = next_agc_reset_time = 0;
    last_channel_noisy_ms = 0;
    next_dwell_sample_ms = 0;
    dwell_starve_start_ms = 0;
    _err_flags = 0;
    radio_nonrx_start = 0;
    prev_isrecv_mode = true;
    tx_budget_ms = 0;
    last_budget_update = 0;
    duty_cycle_window_ms = 3600000;
  }

  virtual DispatcherAction onRecvPacket(Packet* pkt) = 0;

  virtual void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) { }   // custom hook

  virtual void logRx(Packet* packet, int len, float score) { }   // hooks for custom logging
  virtual void logTx(Packet* packet, int len) { }
  virtual void logTxFail(Packet* packet, int len) { }
  virtual const char* getLogDateTime() { return ""; }

  virtual float getAirtimeBudgetFactor() const;
  virtual int calcRxDelay(float score, uint32_t air_time) const;
  virtual uint32_t getCADFailRetryDelay() const;
  virtual uint32_t getCADFailMaxDuration() const;
  virtual int getInterferenceThreshold() const { return 0; }    // disabled by default
  // quiet-dwell TX gate: dwell window scaled from estimated packet air-time (always on, no config).
  // See the DWELL_* defaults above. At SF7/BW125 (~120 ms airtime) this floors at DWELL_MS_MIN;
  // at SF12/BW125 (~3.9 s) it ceilings at DWELL_MS_MAX; mid-SF lands in between.
  virtual uint32_t getQuietDwellMs() const {
    uint32_t airtime = _radio->getEstAirtimeFor(DWELL_AIRTIME_LEN);
    uint32_t dwell = airtime / DWELL_AIRTIME_DIVISOR;
    if (dwell < DWELL_MS_MIN) dwell = DWELL_MS_MIN;
    if (dwell > DWELL_MS_MAX) dwell = DWELL_MS_MAX;
    return dwell;
  }
  virtual bool getCADEnabled() const { return false; }    // hardware CAD disabled by default
  virtual int getAGCResetInterval() const { return 0; }    // disabled by default
  virtual unsigned long getDutyCycleWindowMs() const { return 3600000; }

public:
  void begin();
  void loop();

  Packet* obtainNewPacket();
  void releasePacket(Packet* packet);
  void sendPacket(Packet* packet, uint8_t priority, uint32_t delay_millis=0);

  unsigned long getTotalAirTime() const { return total_air_time; }
  unsigned long getReceiveAirTime() const {return rx_air_time; }
  unsigned long getRemainingTxBudget() const { return tx_budget_ms; }
  uint32_t getNumSentFlood() const { return n_sent_flood; }
  uint32_t getNumSentDirect() const { return n_sent_direct; }
  uint32_t getNumRecvFlood() const { return n_recv_flood; }
  uint32_t getNumRecvDirect() const { return n_recv_direct; }
  void resetStats() {
    n_sent_flood = n_sent_direct = n_recv_flood = n_recv_direct = 0;
    _err_flags = 0;
  }

  // helper methods
  bool millisHasNowPassed(unsigned long timestamp) const;
  unsigned long futureMillis(int millis_from_now) const;

  bool tryParsePacket(Packet* pkt, const uint8_t* raw, int len);

private:
  void checkRecv();
  void checkSend();
};

}
