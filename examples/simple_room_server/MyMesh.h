#pragma once

#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <helpers/AdvertDataHelpers.h>
#include <helpers/TxtDataHelpers.h>
#include <helpers/CommonCLI.h>
#include <helpers/StatsFormatHelper.h>
#include <helpers/ClientACL.h>
#include <helpers/RegionMap.h>
#include <RTClib.h>
#include <target.h>

/* ------------------------------ Config -------------------------------- */

#ifndef FIRMWARE_BUILD_DATE
  #define FIRMWARE_BUILD_DATE   "6 Jun 2026"
#endif

#ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION   "v1.16.0"
#endif

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     250
#endif
#ifndef LORA_SF
  #define LORA_SF     10
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

#ifndef ADVERT_NAME
  #define  ADVERT_NAME   "Test BBS"
#endif
#ifndef ADVERT_LAT
  #define  ADVERT_LAT  0.0
#endif
#ifndef ADVERT_LON
  #define  ADVERT_LON  0.0
#endif

#ifndef ADMIN_PASSWORD
  #define  ADMIN_PASSWORD  "password"
#endif

#ifndef MAX_UNSYNCED_POSTS
  #define MAX_UNSYNCED_POSTS    32
#endif

#ifndef SERVER_RESPONSE_DELAY
  #define SERVER_RESPONSE_DELAY   300
#endif

#ifndef TXT_ACK_DELAY
  #define TXT_ACK_DELAY     200
#endif

#define FIRMWARE_ROLE "room_server"

#define PACKET_LOG_FILE  "/packet_log"

// Two-file append log for the recent-posts ring (bounded to MAX_UNSYNCED_POSTS).
// /posts_cur holds the newest batch (append-only, 0..N-1 records); when it fills
// it is rotated to /posts_old (delete the old /posts_old, rename /posts_cur -> it).
// The newest batch is never rewritten/truncated -> crash-safe; a rotate is just a
// remove()+rename() metadata op on SPIFFS/LittleFS (no data rewrite), so there is
// never a full-file write.
// PostRec gained a `seq` field -> record size changed (188 -> 192 B). The file
// names are versioned so a stale pre-seq file (different record size, reads as
// garbage) is never opened; it is simply left orphaned on flash.
#define POSTS_FILE_CUR   "/posts_cur"
#define POSTS_FILE_OLD   "/posts_old"
#define POSTS_CLIENTSEQ_FILE  "/s_client_seq"   // per-companion last_seq watermark (pub_key+last_seq records)

#define MAX_POST_TEXT_LEN    (160-9)

struct PostInfo {
  mesh::Identity author;
  uint32_t post_timestamp;   // by OUR clock
  uint32_t seq;              // monotonic post sequence (clock-independent sync watermark)
  char text[MAX_POST_TEXT_LEN+1];
};

// On-flash record for one post. Only the author's pub_key is stored: that is all
// matches() and pushPostToClient() ever use (Identity.h / MyMesh.cpp).
struct PostRec {
  uint32_t post_timestamp;
  uint32_t seq;
  uint8_t  author_pubkey[PUB_KEY_SIZE];
  char     text[MAX_POST_TEXT_LEN+1];
};

// Per-companion sync watermark (server-authoritative). Keyed by pub_key; persisted
// in POSTS_CLIENTSEQ_FILE so the watermark survives a room-server restart
// (independent of the clock). last_seq = highest confirmed-delivered post seq;
// pushed_seq = seq currently outstanding (awaiting ACK), transient (not persisted).
struct ClientSeq {
  uint8_t  pub_key[PUB_KEY_SIZE];
  uint32_t last_seq;
  uint32_t pushed_seq;
};

class MyMesh : public mesh::Mesh, public CommonCLICallbacks {
  FILESYSTEM* _fs;
  uint32_t last_millis;
  uint64_t uptime_millis;
  unsigned long next_local_advert, next_flood_advert;
  bool _logging;
  bool region_load_active;
  NodePrefs _prefs;
  TransportKeyStore key_store;
  RegionMap region_map, temp_map;
  ClientACL acl;
  CommonCLI _cli;
  unsigned long dirty_contacts_expiry;
  uint8_t reply_data[MAX_PACKET_PAYLOAD];
  unsigned long next_push;
  uint16_t _num_posted, _num_post_pushes;
  int next_client_idx;  // for round-robin polling
  int next_post_idx;
  uint16_t cur_count;   // valid records currently in POSTS_FILE_CUR (0..MAX_UNSYNCED_POSTS-1)
  uint32_t next_post_seq;                 // next monotonic post sequence number
  ClientSeq client_seqs[MAX_CLIENTS];     // per-companion seq watermarks
  int n_client_seqs;
  bool client_seq_dirty;                  // client_seqs changed -> lazy-flush pending
  unsigned long client_seq_write_at;      // when to flush client_seqs to flash
  unsigned long next_restart_notify;      // one-shot: when to send the post-restart re-login notice
  PostInfo posts[MAX_UNSYNCED_POSTS];   // cyclic queue
  CayenneLPP telemetry;
  RegionEntry* load_stack[8];
  RegionEntry* recv_pkt_region;
  TransportKey default_scope;
  unsigned long set_radio_at, revert_radio_at;
  float pending_freq;
  float pending_bw;
  uint8_t pending_sf;
  uint8_t pending_cr;
  int  matching_peer_indexes[MAX_CLIENTS];

  void addPost(ClientInfo* client, const char* postData);
  void appendPostToFlash(uint8_t slot);       // append newest post to POSTS_FILE_CUR
  void rotatePostFiles();                     // POSTS_FILE_CUR full -> rotate to POSTS_FILE_OLD (remove+rename)
  int  streamFileIntoRing(const char* fname); // stream a batch file into the RAM ring (returns records read)
  bool loadPostsFromFlash();                  // stream POSTS_FILE_OLD then _CUR into the ring (keeps newest N)
  uint32_t getClientLastSeq(const uint8_t* pubkey);  // lookup a companion's last_seq (0 if unknown)
  ClientSeq* ensureClientSeq(const uint8_t* pubkey); // find or create a companion's seq entry (evicts if full)
  void loadClientSeqs();                             // boot: read POSTS_CLIENTSEQ_FILE into client_seqs
  void saveClientSeqs();                             // lazy-flush client_seqs to POSTS_CLIENTSEQ_FILE
  void pushPostToClient(ClientInfo* client, PostInfo& post);
  void sendRestartNotice(ClientInfo* client);   // one-shot post-restart "please re-login" notice (server-only)
  uint8_t getUnsyncedCount(ClientInfo* client);
  bool processAck(const uint8_t *data);
  mesh::Packet* createSelfAdvert();
  File openAppend(const char* fname);
  File openWrite(const char* fname);    // open (create/truncate) for sequential write, like ClientACL::openWrite
  int handleRequest(ClientInfo* sender, uint32_t sender_timestamp, uint8_t* payload, size_t payload_len);

protected:
  float getAirtimeBudgetFactor() const override {
    return _prefs.airtime_factor;
  }

  void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override;
  void logRx(mesh::Packet* pkt, int len, float score) override;
  void logTx(mesh::Packet* pkt, int len) override;
  void logTxFail(mesh::Packet* pkt, int len) override;

  int calcRxDelay(float score, uint32_t air_time) const override;
  const char* getLogDateTime() override;
  uint32_t getRetransmitDelay(const mesh::Packet* packet) override;
  uint32_t getDirectRetransmitDelay(const mesh::Packet* packet) override;

  int getInterferenceThreshold() const override {
    return _prefs.interference_threshold;
  }
  bool getCADEnabled() const override {
    return _prefs.cad_enabled;
  }
  int getAGCResetInterval() const override {
    return ((int)_prefs.agc_reset_interval) * 4000;   // milliseconds
  }
  uint8_t getExtraAckTransmitCount() const override {
    return _prefs.multi_acks;
  }

  bool filterRecvFloodPacket(mesh::Packet* pkt) override;

  bool allowPacketForward(const mesh::Packet* packet) override;
  void onAnonDataRecv(mesh::Packet* packet, const uint8_t* secret, const mesh::Identity& sender, uint8_t* data, size_t len) override;
  int searchPeersByHash(const uint8_t* hash) override ;
  void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override;
  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override;
  bool onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override;
  void onAckRecv(mesh::Packet* packet, uint32_t ack_crc) override;

#if ENV_INCLUDE_GPS == 1
  void applyGpsPrefs() {
    sensors.setSettingValue("gps", _prefs.gps_enabled?"1":"0");
  }
#endif

  void sendFloodReply(mesh::Packet* packet, unsigned long delay_millis, uint8_t path_hash_size);

public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables);

  void begin(FILESYSTEM* fs);

  const char* getFirmwareVer() override { return FIRMWARE_VERSION; }
  const char* getBuildDate() override { return FIRMWARE_BUILD_DATE; }
  const char* getRole() override { return FIRMWARE_ROLE; }
  const char* getNodeName() { return _prefs.node_name; }
  NodePrefs* getNodePrefs() {
    return &_prefs;
  }

  void savePrefs() override {
    _cli.savePrefs(_fs);
  }

  void sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis, uint8_t path_hash_size);

  // CommonCLICallbacks
  void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) override;
  bool formatFileSystem() override;
  void sendSelfAdvertisement(int delay_millis, bool flood) override;
  void updateAdvertTimer() override;
  void updateFloodAdvertTimer() override;

  void setLoggingOn(bool enable) override { _logging = enable; }

  void eraseLogFile() override {
    _fs->remove(PACKET_LOG_FILE);
  }

  void dumpLogFile() override;
  void setTxPower(int8_t power_dbm) override;

  void formatNeighborsReply(char *reply) override {
    strcpy(reply, "not supported");
  }
  void formatStatsReply(char *reply) override;
  void formatRadioStatsReply(char *reply) override;
  void formatPacketStatsReply(char *reply) override;
  void startRegionsLoad() override;
  bool saveRegions() override;
  void onDefaultRegionChanged(const RegionEntry* r) override;

  mesh::LocalIdentity& getSelfId() override { return self_id; }

  static bool saveFilter(ClientInfo* client);

  void saveIdentity(const mesh::LocalIdentity& new_id) override;
  void clearStats() override;
  void handleCommand(uint32_t sender_timestamp, char* command, char* reply);
  void loop();
};
