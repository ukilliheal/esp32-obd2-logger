// ESP32 OBD2 bridge + logger (firmware v4)
//
// 1. Auto-connects to any listed reader, over and over until one answers:
//    Wi-Fi scan -> join a listed reader network -> check its TCP port, then
//    Bluetooth scan -> connect to a listed Bluetooth Classic reader (strongest first).
//    A reader whose PIN is "auto" is tried with 1234, 0000, 6789; the working PIN is saved.
// 2. Relays the reader to the USB serial port (115200 baud) for the Mac.
// 3. Logs standard OBD2 engine data to flash whenever a reader is connected and the Mac
//    isn't using it (e.g. running from a USB power bank in the car). Works on any OBD2
//    car: it asks the car which values it supports. 2 MB rotating log in 64 KB files.
//
// Who talks to the reader: the logger, unless the Mac is. The Mac helper sends #HOLD when
// a program connects and #RELEASE when it disconnects. Raw serial use without the helper
// also works: Mac data takes the reader over, and it goes back to the logger after 20 s of
// silence. Every handover first re-synchronises with the reader (a space aborts any
// command in progress; then input is discarded until the line goes quiet).
//
// Build: the sketch's partitions.csv gives 1.875 MB to firmware and 2 MB to the log.
//   arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app \
//     --build-property upload.maximum_size=1966080 --output-dir build .
//
// Serial protocol: status lines are "#..." ending in '\n'. ELM327 data never contains
// '#', so the Mac-side helper (esp32_bridge.py) separates them from reader data.
//
// Commands (one line each; reader list, mode and the logging switch are saved in flash):
//   #LIST                               show the reader list
//   #ADDWIFI ssid,password,host,port    add a Wi-Fi reader (host/port default 192.168.0.10:35000)
//   #ADDBT MAC-or-name,pin              add a Bluetooth Classic reader (pin default "auto")
//   #DEL n                              remove reader n
//   #MODE auto|wifi|bt|n                search all / only Wi-Fi / only Bluetooth / only reader n
//   #SCAN / #BTSCAN                     list visible Wi-Fi / Bluetooth OBD readers
//   #STATUS                             state, connected reader, logger, storage, uptime
//   #RESET                              default reader list, auto mode (logs are kept)
//   #HOLD / #RELEASE                    Mac takes / returns the reader (sent by the helper)
//   #TIME unix-seconds                  tell the board the real time (sent by the helper)
//   #LOG on|off                         enable/disable logging
//   #LOGLIST                            list log files
//   #LOGGET name                        send a log file as "#LOGDATA <base64>" lines
//   #LOGCLEAR                           delete all log files
// Commands sent while a blocking search step runs are handled when it finishes (<~40 s).
//
// Optional display: a 0.91" 128x32 SSD1306 I2C OLED (GND, VCC->3V3, SCL->GPIO22,
// SDA->GPIO21). Found at boot; without one the board works exactly as before.

#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <vector>
#include <algorithm>
#include "BluetoothSerial.h"
#include "BTAddress.h"
#include "esp_rom_sys.h"
#include "base64.h"
#include "SHA2Builder.h"

static const int LED_PIN = 2;
static const int MAX_READERS = 8;
static const uint8_t T_WIFI = 1, T_BT = 2;
static const char *DEF_HOST = "192.168.0.10";
static const uint16_t DEF_PORT = 35000;
static const char *BT_NAME = "ESP32-OBD-Bridge";
static const uint32_t ROUND_PAUSE_MS = 2000;   // pause between failed search rounds
static const uint32_t WIFI_JOIN_MS = 10000;    // max wait to join a reader's network
static const uint32_t BT_DISCOVER_MS = 6000;   // Bluetooth inquiry length
static const uint32_t MAC_IDLE_MS = 20000;     // raw-serial fallback: logger returns after this

// Name fragments used by common OBD readers (only these are named in scan output).
static const char *OBD_NAMES[] = {"obd", "elm", "vlink", "v-link", "obdlink", "veepeak",
                                  "konnwei", "vgate", "icar", "scan", "car"};

struct Reader {
  uint8_t type = 0;   // T_WIFI or T_BT
  String target;      // Wi-Fi: SSID      Bluetooth: MAC (AA:BB:..) or device name
  String secret;      // Wi-Fi: password  Bluetooth: PIN or "auto"
  String host;        // Wi-Fi only
  uint16_t port = 0;  // Wi-Fi only
};

Preferences prefs;
WiFiClient obd;
BluetoothSerial bt;
std::vector<Reader> readers;
String mode = "auto";
bool btReady = false;
bool logEnabled = true;

enum State { SEARCHING, LINKED };
State state = SEARCHING;
int active = -1;            // index of the linked reader
uint32_t nextSearch = 0;
uint32_t searchRound = 0;
uint32_t lastNotLinkedMsg = 0;
int tcpFails = 0;           // consecutive failed TCP connects to a Wi-Fi reader

String cmdLine;
bool inCmd = false;

// Who is using the reader.
enum Owner { OWNER_LOGGER, OWNER_TO_MAC, OWNER_MAC };
Owner owner = OWNER_LOGGER;
bool held = false;                  // #HOLD from the Mac helper
uint32_t lastMacData = 0;
std::vector<uint8_t> macPending;    // Mac bytes waiting for the handover to finish

// Re-synchronisation with the reader (see header comment).
bool syncing = false;
uint32_t syncStartMs = 0, syncLastByte = 0;
static const uint32_t SYNC_QUIET_MS = 150, SYNC_MIN_MS = 200, SYNC_MAX_MS = 1500;

// Real time: unix seconds at millis()==0, once the Mac has sent #TIME (0 = unknown).
uint32_t epochAtBoot = 0;

// --------------------------------------------------------------------- utilities

void oledSearchNote(const String &note);

void status(const String &msg) {
  Serial.print("#");
  Serial.print(msg);
  Serial.print("\n");
  if (msg.startsWith("SEARCH ")) oledSearchNote(msg.substring(7));
}

bool isMac(const String &s) {
  return s.length() == 17 && s.charAt(2) == ':' && s.charAt(5) == ':' && s.charAt(8) == ':';
}

bool isDigits(const String &s) {
  if (s.length() == 0) return false;
  for (size_t i = 0; i < s.length(); i++)
    if (!isdigit((unsigned char)s[i])) return false;
  return true;
}

bool looksLikeObd(String name) {
  name.toLowerCase();
  for (auto n : OBD_NAMES)
    if (name.indexOf(n) >= 0) return true;
  return false;
}

String describe(int i) {
  const Reader &r = readers[i];
  if (r.type == T_WIFI)
    return String(i) + " wifi " + r.target + " " + r.host + ":" + String(r.port) +
           " password=" + (r.secret.length() ? "set" : "none");
  return String(i) + " bt " + r.target + " pin=" + r.secret;
}

bool allowed(int i) {
  if (mode == "auto") return true;
  if (mode == "wifi") return readers[i].type == T_WIFI;
  if (mode == "bt") return readers[i].type == T_BT;
  return mode == String(i);
}

// Split "a,b,c" into exactly `max` fields (missing ones empty; the last keeps any commas).
std::vector<String> fields(const String &s, int max) {
  std::vector<String> out;
  int start = 0;
  for (int i = 0; i < max; i++) {
    int comma = (i == max - 1) ? -1 : s.indexOf(',', start);
    String f = comma < 0 ? s.substring(start) : s.substring(start, comma);
    f.trim();
    out.push_back(f);
    if (comma < 0) break;
    start = comma + 1;
  }
  while ((int)out.size() < max) out.push_back("");
  return out;
}

uint32_t crc32(const uint8_t *d, size_t n, uint32_t crc = 0) {  // zlib-compatible
  crc = ~crc;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

// ------------------------------------------------------------------ persistence
// Keys are short (NVS limit is 15 chars). 'ver' is written last, so a reset in the
// middle of a save is caught on the next boot (n/ver missing or entries incomplete).

// The reader list the board starts with (first boot, #RESET, or a settings-version change).
// Readers can also be added at run time without reflashing: #ADDWIFI / #ADDBT, or
// tools/esp32_bridge.py --add-wifi / --add-bt. Up to MAX_READERS entries.
void defaultReaders() {
  readers.clear();
  // Wi-Fi reader: target = the reader's network name, secret = its password ("" if open).
  // "WiFi_OBDII" at 192.168.0.10:35000 is what most ELM327 Wi-Fi clones ship with.
  Reader w;
  w.type = T_WIFI; w.target = "WiFi_OBDII"; w.secret = ""; w.host = DEF_HOST; w.port = DEF_PORT;
  readers.push_back(w);
  // Bluetooth Classic reader: target = its MAC address (find it with #BTSCAN) or its
  // device name, secret = its PIN, or "auto" to try 1234, 0000 and 6789. Example:
  //   Reader b;
  //   b.type = T_BT; b.target = "AA:BB:CC:DD:EE:FF"; b.secret = "auto";
  //   readers.push_back(b);
  mode = "auto";
}

void saveAll() {
  prefs.begin("obdbridge", false);
  for (size_t i = 0; i < readers.size(); i++) {
    String k = String(i);
    prefs.putUChar(("t" + k).c_str(), readers[i].type);
    prefs.putString(("a" + k).c_str(), readers[i].target);
    prefs.putString(("s" + k).c_str(), readers[i].secret);
    prefs.putString(("h" + k).c_str(), readers[i].host);
    prefs.putUShort(("p" + k).c_str(), readers[i].port);
  }
  for (int i = readers.size(); i < MAX_READERS; i++) {
    String k = String(i);
    for (const char *p : {"t", "a", "s", "h", "p"}) prefs.remove((String(p) + k).c_str());
  }
  prefs.putUChar("n", readers.size());
  prefs.putString("mode", mode);
  prefs.putBool("log", logEnabled);
  prefs.putUChar("ver", 4);
  prefs.end();
}

void loadAll() {
  prefs.begin("obdbridge", false);
  uint8_t ver = prefs.getUChar("ver", 0);
  if (ver != 4) {  // first boot of this firmware: drop old settings (keep the session counter)
    uint32_t sess = prefs.getUInt("sess", 0);
    prefs.clear();
    prefs.putUInt("sess", sess);
    prefs.end();
    defaultReaders();
    saveAll();
    status("SETTINGS defaults loaded");
    return;
  }
  mode = prefs.getString("mode", "auto");
  logEnabled = prefs.getBool("log", true);
  int n = std::min((int)prefs.getUChar("n", 0), MAX_READERS);
  readers.clear();
  for (int i = 0; i < n; i++) {
    String k = String(i);
    Reader r;
    r.type = prefs.getUChar(("t" + k).c_str(), 0);
    r.target = prefs.getString(("a" + k).c_str(), "");
    r.secret = prefs.getString(("s" + k).c_str(), "");
    r.host = prefs.getString(("h" + k).c_str(), DEF_HOST);
    r.port = prefs.getUShort(("p" + k).c_str(), DEF_PORT);
    if ((r.type == T_WIFI || r.type == T_BT) && r.target.length()) readers.push_back(r);
  }
  prefs.end();
  if ((int)readers.size() != n || readers.empty()) {
    status("SETTINGS reader list damaged or empty, restoring defaults");
    defaultReaders();
    saveAll();
  }
}

uint32_t nextSessionId() {
  prefs.begin("obdbridge", false);
  uint32_t s = prefs.getUInt("sess", 0) + 1;
  prefs.putUInt("sess", s);
  prefs.end();
  return s;
}

// =============================================================== logger: storage
//
// Files: /sSSSSSS_GGGGG.bin (session, segment), at most SEG_MAX bytes each, deleted
// oldest first when free space drops below RESERVE. Little-endian. Header (format 2):
//   "OBDL" u8 version=2 u32 session u16 segment u8 reader
//   u32 epoch_at_session_start (0 = real time unknown)  u32 session_start_millis
//   u8 protocol (ELM ATDPN digit: '1' J1850 PWM, '2' VPW, '3' ISO 9141, '6'-'9' CAN; '?')
//   u32 support[4]   (mode 01 support bitmaps for PIDs 01-20, 21-40, 41-60, 61-80)
//   u8 vin_hash_len (0 or 8) + first 8 bytes of SHA-256(VIN)  (the VIN itself is never stored)
//   u8 pid_count, then pid_count x (u8 pid, u8 every_n_cycles)
//   u8 dtc_len + stored trouble codes as text ("P0171 P0174")
// Records:
//   0xA5 u32 t_ms (since session start) u32 mask, then 2 bytes (A,B) per set mask bit in
//        pid-list order (single-byte PIDs have B=0)
//   0xA6 u32 t_ms u32 unix_seconds    time anchor (written when #TIME arrives)
//   0xA7 u32 t_ms u8 event            1=engine off/no data 2=engine running 3=Mac took over

static const size_t SEG_MAX = 65536;
static const size_t RESERVE = 2 * SEG_MAX;     // keep this much free; delete oldest beyond
static const uint32_t FLUSH_MS = 15000;

bool fsReady = false;
File logFile;
uint32_t sessionId = 0;
uint16_t segment = 0;
uint32_t sessionStart = 0;
std::vector<uint8_t> wbuf;   // records waiting to be written
uint32_t lastFlush = 0;
uint32_t recordsWritten = 0;
bool sessionOpen = false;
uint32_t lastReopenTry = 0;

// What the current car told us during logger start-up (kept for every segment header).
char carProtocol = '?';
uint32_t support[4] = {0, 0, 0, 0};
bool supportKnown[4] = {false, false, false, false};
uint8_t vinHash[8];
bool vinKnown = false;
String carDtcs;             // stored trouble codes (mode 03), space-separated
String carPending;          // pending trouble codes (mode 07): RAM only, not logged
bool dtcsChecked = false;   // the car has been asked for codes this session

// Standard mode 01 PIDs useful on most cars (bit i of the record mask = PIDS[i]).
// every=1: read each cycle; every=5: every fifth cycle (slow-changing values).
// RPM must stay first: engine-off detection reads it from the first slot.
struct PidDef { uint8_t pid; uint8_t bytes; uint8_t every; };
static const PidDef PIDS[32] = {
  {0x0C, 2, 1},  // engine rpm
  {0x0D, 1, 1},  // vehicle speed
  {0x04, 1, 1},  // calculated load
  {0x10, 2, 1},  // MAF airflow
  {0x0B, 1, 1},  // intake manifold pressure
  {0x11, 1, 1},  // throttle position
  {0x06, 1, 1},  // short-term fuel trim bank 1
  {0x08, 1, 1},  // short-term fuel trim bank 2
  {0x07, 1, 1},  // long-term fuel trim bank 1
  {0x09, 1, 1},  // long-term fuel trim bank 2
  {0x0E, 1, 1},  // timing advance
  {0x14, 2, 1},  // O2 bank 1 sensor 1 (narrowband)
  {0x18, 2, 1},  // O2 bank 2 sensor 1 (narrowband)
  {0x24, 2, 1},  // O2 sensor 1 wideband (lambda + voltage)
  {0x28, 2, 1},  // O2 sensor 5 wideband (usually bank 2 sensor 1)
  {0x44, 2, 1},  // commanded air-fuel equivalence ratio
  {0x49, 1, 1},  // accelerator pedal position D
  {0x05, 1, 5},  // coolant temperature
  {0x0F, 1, 5},  // intake air temperature
  {0x15, 2, 5},  // O2 bank 1 sensor 2 (after the catalyst)
  {0x23, 2, 5},  // fuel rail pressure (diesel / direct injection)
  {0x2C, 1, 5},  // commanded EGR
  {0x2F, 1, 5},  // fuel tank level
  {0x31, 2, 5},  // distance since codes cleared
  {0x33, 1, 5},  // barometric pressure
  {0x3C, 2, 5},  // catalyst temperature bank 1 sensor 1
  {0x42, 2, 5},  // control module (battery) voltage
  {0x46, 1, 5},  // ambient air temperature
  {0x5C, 1, 5},  // engine oil temperature
  {0x5E, 2, 5},  // engine fuel rate
  {0x61, 1, 5},  // driver's demand engine torque
  {0x62, 1, 5},  // actual engine torque
};
static const int NPIDS = 32;

void put32(std::vector<uint8_t> &v, uint32_t x) { for (int i = 0; i < 4; i++) v.push_back(x >> (8 * i)); }
void put16(std::vector<uint8_t> &v, uint16_t x) { v.push_back(x); v.push_back(x >> 8); }

std::vector<String> logFiles() {  // sorted oldest first (fixed-width names sort correctly)
  std::vector<String> names;
  if (!fsReady) return names;
  File dir = LittleFS.open("/");
  if (!dir) return names;
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    String n = f.name();
    if (n.endsWith(".bin")) names.push_back(n.startsWith("/") ? n.substring(1) : n);
    f.close();
  }
  dir.close();
  std::sort(names.begin(), names.end());
  return names;
}

String currentName() {
  char b[32];
  snprintf(b, sizeof(b), "s%06lu_%05u.bin", (unsigned long)sessionId, (unsigned)segment);
  return String(b);
}

void makeRoom() {
  auto names = logFiles();
  String cur = currentName();
  for (size_t i = 0; i < names.size(); i++) {
    if (LittleFS.totalBytes() - LittleFS.usedBytes() >= RESERVE) break;
    if (names[i] == cur) continue;
    LittleFS.remove("/" + names[i]);
    status("LOG rotated out " + names[i]);
  }
}

bool openSegment(uint8_t readerIdx);

void flushLog(bool force) {
  if (!logFile || wbuf.empty()) return;
  if (!force && wbuf.size() < 4096 && millis() - lastFlush < FLUSH_MS) return;
  size_t n = logFile.write(wbuf.data(), wbuf.size());
  logFile.flush();
  lastFlush = millis();
  if (n != wbuf.size()) {  // flash full or failing: start a fresh segment after making room
    status("ERR log write failed (" + String(n) + "/" + String(wbuf.size()) + " bytes)");
    wbuf.clear();
    logFile.close();
    segment++;
    openSegment(active < 0 ? 255 : active);
    return;
  }
  wbuf.clear();
}

void writeHeader(uint8_t readerIdx) {
  std::vector<uint8_t> h = {'O', 'B', 'D', 'L', 2};
  put32(h, sessionId);
  put16(h, segment);
  h.push_back(readerIdx);
  put32(h, epochAtBoot ? epochAtBoot + sessionStart / 1000 : 0);
  put32(h, sessionStart);
  h.push_back((uint8_t)carProtocol);
  for (int i = 0; i < 4; i++) put32(h, supportKnown[i] ? support[i] : 0);
  h.push_back(vinKnown ? 8 : 0);
  if (vinKnown) h.insert(h.end(), vinHash, vinHash + 8);
  h.push_back(NPIDS);
  for (auto &p : PIDS) { h.push_back(p.pid); h.push_back(p.every); }
  String d = carDtcs.substring(0, 200);
  h.push_back(d.length());
  for (size_t i = 0; i < d.length(); i++) h.push_back(d[i]);
  logFile.write(h.data(), h.size());
  logFile.flush();
}

bool openSegment(uint8_t readerIdx) {
  if (!fsReady) return false;
  if (logFile) { flushLog(true); logFile.close(); }
  wbuf.clear();
  makeRoom();
  logFile = LittleFS.open("/" + currentName(), "w");
  if (!logFile) { status("ERR could not create log file " + currentName()); return false; }
  writeHeader(readerIdx);
  return true;
}

bool startSession(uint8_t readerIdx) {
  if (!fsReady || !logEnabled) return false;
  sessionId = nextSessionId();
  segment = 0;
  sessionStart = millis();
  bool ok = openSegment(readerIdx);
  status("LOG session " + String(sessionId) + (ok ? " started" : " could not open a file") +
         ", protocol " + String(carProtocol) + ", VIN " + (vinKnown ? "hashed" : "not reported") +
         ", codes: " + (carDtcs.length() ? carDtcs : String("none")));
  return true;  // the session exists even if the file failed; appendRecord retries
}

void endSession() {
  if (logFile) { flushLog(true); logFile.close(); }
  sessionOpen = false;
}

void appendRecord(const std::vector<uint8_t> &rec) {
  if (!sessionOpen) return;
  if (!logFile) {  // an earlier open failed: retry now and then
    if (millis() - lastReopenTry < 10000) return;
    lastReopenTry = millis();
    segment++;
    if (!openSegment(active < 0 ? 255 : active)) return;
  }
  if (logFile.size() + wbuf.size() + rec.size() > SEG_MAX) {
    segment++;
    if (!openSegment(active < 0 ? 255 : active)) return;
  }
  wbuf.insert(wbuf.end(), rec.begin(), rec.end());
  recordsWritten++;
  flushLog(false);
}

void logEvent(uint8_t ev) {
  std::vector<uint8_t> r = {0xA7};
  put32(r, millis() - sessionStart);
  r.push_back(ev);
  appendRecord(r);
}

// ============================================================= logger: ELM polling

enum LogState { L_OFF, L_INIT, L_POLL, L_ENGINE_OFF, L_PAUSED };
LogState lstate = L_OFF;
const char *lstateName() {
  switch (lstate) {
    case L_OFF: return "off";
    case L_INIT: return "starting";
    case L_POLL: return "recording";
    case L_ENGINE_OFF: return "waiting-for-engine";
    case L_PAUSED: return "paused-for-mac";
  }
  return "?";
}

bool reqBusy = false;       // a request is outstanding
String reqText;             // the request outstanding / just completed ("" = none)
String resp;                // its response text
bool respGotBytes = false;  // anything at all came back for it
bool reqTimedOut = false;
String pendingAfterSync;            // response of a timed-out request, handled after resync
bool havePendingAfterSync = false;
uint32_t reqDeadline = 0;
uint32_t nextLogAction = 0;
int initStep = -1;
int silentTimeouts = 0;     // consecutive requests with no bytes at all from the reader

int pollIdx = -1;           // PIDS[] index being read; -1 = between cycles
uint32_t cycleNo = 0;
uint32_t cycleStart = 0;
uint32_t gotMask = 0;
uint8_t vals[NPIDS][2];
uint32_t seenMask = 0;      // values read at least once this session (vals[] keeps the latest)
int sentThisCycle = 0;
int zeroRpmCycles = 0;
int failCycles = 0;

void linkWrite(const uint8_t *d, size_t n);
void dropLink(const String &why);

void sendReq(const String &cmd, uint32_t timeoutMs) {
  reqText = cmd;
  resp = "";
  respGotBytes = false;
  reqTimedOut = false;
  reqBusy = true;
  reqDeadline = millis() + timeoutMs;
  String line = cmd + "\r";
  linkWrite((const uint8_t *)line.c_str(), line.length());
}

void beginSync() {
  syncing = true;
  syncStartMs = syncLastByte = millis();
  reqBusy = false;
  const uint8_t sp = ' ';  // aborts a busy ELM327; an idle one just ignores it
  linkWrite(&sp, 1);
}

// Reader bytes while the logger owns the link. The '>' prompt ends a response.
void loggerFeed(const uint8_t *d, size_t n) {
  if (!reqBusy) return;  // stray bytes
  respGotBytes = true;
  for (size_t i = 0; i < n; i++) {
    char c = d[i];
    if (c == '>') { reqBusy = false; return; }
    if (resp.length() < 600) resp += c;
  }
}

// Response -> lines of uppercase hex (CAN frame prefixes like "0:" removed; lines with
// text such as NO DATA / SEARCHING... / STOPPED / ATDPN echoes are dropped).
std::vector<String> hexLines(const String &r) {
  std::vector<String> out;
  String cur;
  bool bad = false;
  auto finish = [&]() {
    if (cur.length() && !bad) out.push_back(cur);
    cur = ""; bad = false;
  };
  for (size_t i = 0; i < r.length(); i++) {
    char c = toupper(r[i]);
    if (c == '\r' || c == '\n') finish();
    else if (c == ':') cur = "";
    else if (isxdigit(c)) cur += c;
    else if (c != ' ') bad = true;
  }
  finish();
  return out;
}

uint32_t hexAt(const String &h, int pos, int nbytes) {
  uint32_t v = 0;
  for (int b = 0; b < nbytes; b++) v = (v << 8) | strtoul(h.substring(pos + 2 * b, pos + 2 * b + 2).c_str(), nullptr, 16);
  return v;
}

// Mode 03 reply -> "P0171 P0174". CAN puts a code-count byte after 43 and may split a long
// reply over numbered frames; J1850/ISO send 3 codes per line with no count.
String parseDtcs(const String &r, bool can, const char *reply = "43") {
  auto lines = hexLines(r);
  std::vector<String> msgs;
  bool multiFrame = false;
  for (auto &l : lines) multiFrame |= (l.length() % 2) == 1;  // CAN byte-count line
  if (can && multiFrame) {
    String joined;
    for (auto &l : lines) if (l.length() % 2 == 0) joined += l;
    msgs.push_back(joined);
  } else {
    msgs = lines;
  }
  String out;
  for (auto &m : msgs) {
    if (!m.startsWith(reply)) continue;
    int at = 2, count = 99;
    if (can && m.length() >= 4) { count = hexAt(m, 2, 1); at = 4; }
    for (int i = 0; i < count && at + 4 <= (int)m.length(); i++, at += 4) {
      uint16_t v = hexAt(m, at, 2);
      if (v == 0) continue;
      static const char sys[] = {'P', 'C', 'B', 'U'};
      char code[6];
      snprintf(code, sizeof(code), "%c%X%03X", sys[v >> 14], (v >> 12) & 3, v & 0xFFF);
      if (out.indexOf(code) < 0) out += (out.length() ? " " : "") + String(code);
    }
  }
  return out;
}

// Mode 09 PID 02 reply -> 17-character VIN or "" (J1850/ISO line layout and CAN frames).
String parseVin(const String &r) {
  auto lines = hexLines(r);
  String chars;
  bool started = false;
  for (auto &l : lines) {
    if (l == "0902") continue;               // echoed request
    if (!started) { if (l.indexOf("4902") < 0) continue; started = true; }
    if (l.length() % 2) continue;            // CAN byte-count line
    for (int i = 0; i + 2 <= (int)l.length(); i += 2) {
      char c = (char)hexAt(l, i, 1);
      // VIN characters: 0-9, A-Z except I, O, Q (so the 0x49 'I' mode byte drops out)
      if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z' && c != 'I' && c != 'O' && c != 'Q'))
        chars += c;
    }
  }
  return chars.length() >= 17 ? chars.substring(chars.length() - 17) : String("");
}

// Pick up anything useful from any start-up response. Some clones answer one command
// late, so a reply can arrive while a later request is outstanding.
void harvest(const String &r) {
  auto lines = hexLines(r);
  for (auto &l : lines) {
    for (int k = 0; k < 4; k++) {            // 4100 / 4120 / 4140 / 4160 support bitmaps
      char key[5];
      snprintf(key, sizeof(key), "41%02X", 0x20 * k);
      int p = l.indexOf(key);
      if (p >= 0 && p % 2 == 0 && (int)l.length() >= p + 12) {
        uint32_t v = hexAt(l, p + 4, 4);
        support[k] = supportKnown[k] ? (support[k] | v) : v;  // several ECUs: union
        supportKnown[k] = true;
      }
    }
    // ATDPN reply: "A6" (auto-detected) or "6"
    if ((l.length() == 1 || (l.length() == 2 && l[0] == 'A')) && l[l.length() - 1] >= '1')
      carProtocol = l[l.length() - 1];
  }
  if (r.indexOf("49") >= 0) {
    String vin = parseVin(r);
    if (vin.length() == 17 && !vinKnown) {
      SHA256Builder sha;
      uint8_t full[32];
      sha.begin();
      sha.add((const uint8_t *)vin.c_str(), 17);
      sha.calculate();
      sha.getBytes(full);
      memcpy(vinHash, full, 8);
      vinKnown = true;
    }
  }
  bool can = carProtocol >= '6' && carProtocol <= '9';
  for (auto &l : lines)
    if (l.startsWith("43")) { carDtcs = parseDtcs(r, can); break; }
  for (auto &l : lines)
    if (l.startsWith("47")) { carPending = parseDtcs(r, can, "47"); break; }
}

// Reply to a 03 (stored) or 07 (pending) request. Codes are parsed by harvest(); a car
// with no codes often answers NO DATA instead of an empty list.
void dtcReply(const String &r, bool stored) {
  harvest(r);
  if (r.indexOf("NO DATA") >= 0 && r.indexOf(stored ? "43" : "47") < 0) (stored ? carDtcs : carPending) = "";
  dtcsChecked = true;
}

bool pidSupported(uint8_t pid) {
  int range = (pid - 1) / 32;            // 01-20 -> 0, 21-40 -> 1, ...
  if (pid == 0 || range > 3 || !supportKnown[range]) return false;
  int bit = 31 - ((pid - 1) % 32);
  return (support[range] >> bit) & 1;
}

int supportedCount() {
  int n = 0;
  for (auto &p : PIDS) n += pidSupported(p.pid);
  return n;
}

void loggerReset() {
  if (sessionOpen) endSession();
  lstate = L_OFF;
  reqBusy = false;
  reqText = "";
  havePendingAfterSync = false;
}

// (Re)start the logger: resync, then the start-up sequence. A session that is already
// open (e.g. the Mac borrowed the reader) carries on.
void loggerStart() {
  if (!logEnabled || !fsReady || state != LINKED) { lstate = L_OFF; return; }
  lstate = L_INIT;
  initStep = -1;
  reqText = "";
  havePendingAfterSync = false;
  silentTimeouts = 0;
  if (!sessionOpen) {
    carProtocol = '?';
    for (int i = 0; i < 4; i++) { support[i] = 0; supportKnown[i] = false; }
    vinKnown = false;
    carDtcs = "";
    carPending = "";
    dtcsChecked = false;
    seenMask = 0;
  }
  beginSync();
  nextLogAction = millis();
}

// While recording, ask for stored and pending codes again every few minutes so a code
// that sets during a drive shows up on the display (and in the next segment header).
static const uint32_t DTC_RECHECK_MS = 300000;
uint32_t lastDtcCheck = 0;
int dtcStep = 0;             // 1 = waiting for the 03 reply, 2 = waiting for the 07 reply

struct InitDef { const char *cmd; uint16_t timeoutMs; };
static const InitDef INIT[] = {
  {"ATZ", 3000}, {"ATE0", 1500}, {"ATL0", 1500}, {"ATS0", 1500}, {"ATH0", 1500},
  {"ATSP0", 1500}, {"0100", 13000}, {"ATDPN", 1500}, {"0120", 4000}, {"0140", 4000},
  {"0160", 4000}, {"0902", 5000}, {"03", 6000}, {"07", 6000},
};
static const int N_INIT = sizeof(INIT) / sizeof(INIT[0]);

bool initStepNeeded(int s) {
  if (s == 8) return supportKnown[0] && (support[0] & 1);   // PID 20 = "21-40 supported"
  if (s == 9) return supportKnown[1] && (support[1] & 1);
  if (s == 10) return supportKnown[2] && (support[2] & 1);
  return true;
}

void engineOff(const String &why) {
  if (sessionOpen) logEvent(1);
  lstate = L_ENGINE_OFF;
  reqText = "";
  nextLogAction = millis() + 30000;
  status("LOG waiting for the engine (" + why + ")");
}

void initHandle(const String &r) {
  const char *cmd = initStep >= 0 ? INIT[initStep].cmd : "";
  if (!strcmp(cmd, "03") || !strcmp(cmd, "07")) dtcReply(r, cmd[1] == '3');
  else harvest(r);
  do { initStep++; } while (initStep < N_INIT && !initStepNeeded(initStep));
  if (initStep < N_INIT) { sendReq(INIT[initStep].cmd, INIT[initStep].timeoutMs); return; }

  // Start-up finished.
  if (!supportKnown[0]) { engineOff("car did not answer 0100"); return; }
  if (!sessionOpen) sessionOpen = startSession(active);
  lstate = L_POLL;
  pollIdx = -1;
  dtcStep = 0;
  lastDtcCheck = millis();       // start-up just read them
  cycleNo = 0;
  zeroRpmCycles = 0;
  failCycles = 0;
  nextLogAction = millis();
  int n = supportedCount();
  status("LOG recording " + String(n) + " of " + String(NPIDS) + " values");
  if (n == 0) engineOff("car supports none of the logged values");
}

bool pidDueThisCycle(int i) {
  return pidSupported(PIDS[i].pid) && (cycleNo % PIDS[i].every == 0);
}

// Store every "41 <pid> data" in a response under the PID it names (tolerates replies
// that arrive one request late).
void storeAnswers(const String &r) {
  for (auto &l : hexLines(r)) {
    for (int pos = 0; pos + 4 <= (int)l.length(); pos += 2) {
      if (l[pos] != '4' || l[pos + 1] != '1') continue;
      uint8_t pid = hexAt(l, pos + 2, 1);
      for (int i = 0; i < NPIDS; i++) {
        if (PIDS[i].pid != pid || !pidDueThisCycle(i) || (gotMask >> i & 1)) continue;
        int need = PIDS[i].bytes;
        if ((int)l.length() < pos + 4 + 2 * need) break;
        vals[i][0] = hexAt(l, pos + 4, 1);
        vals[i][1] = need == 2 ? hexAt(l, pos + 6, 1) : 0;
        gotMask |= 1u << i;
        pos += 2 + 2 * need;  // skip the data bytes
        break;
      }
    }
  }
}

void finishCycle() {
  if (gotMask) {
    std::vector<uint8_t> rec = {0xA5};
    put32(rec, cycleStart - sessionStart);
    put32(rec, gotMask);
    for (int i = 0; i < NPIDS; i++)
      if (gotMask >> i & 1) { rec.push_back(vals[i][0]); rec.push_back(vals[i][1]); }
    appendRecord(rec);
    seenMask |= gotMask;
    failCycles = 0;
    if (gotMask & 1) {  // RPM is PIDS[0]
      bool rpmZero = vals[0][0] == 0 && vals[0][1] == 0;
      if (rpmZero) {
        if (++zeroRpmCycles == 5) logEvent(1);
      } else {
        if (zeroRpmCycles >= 5) logEvent(2);
        zeroRpmCycles = 0;
      }
    }
  } else if (sentThisCycle > 0 && ++failCycles >= 3) {
    engineOff("car stopped answering");
    return;
  }
  cycleNo++;
  pollIdx = -1;
  // Ignition on but engine off: one sample every 10 s instead of continuous polling.
  nextLogAction = millis() + (zeroRpmCycles >= 5 ? 10000 : 0);
}

void pollHandle(const String &r) {
  if (dtcStep == 1) { dtcReply(r, true); dtcStep = 2; sendReq("07", 6000); return; }
  if (dtcStep == 2) { dtcReply(r, false); dtcStep = 0; lastDtcCheck = millis(); }
  else if (pollIdx == -1 && millis() - lastDtcCheck >= DTC_RECHECK_MS) {
    dtcStep = 1;
    sendReq("03", 6000);
    return;
  }
  if (pollIdx == -1) {           // starting a new cycle
    cycleStart = millis();
    gotMask = 0;
    sentThisCycle = 0;
  } else {
    storeAnswers(r);
  }
  do { pollIdx++; } while (pollIdx < NPIDS && (!pidDueThisCycle(pollIdx) || (gotMask >> pollIdx & 1)));
  if (pollIdx < NPIDS) {
    char cmd[6];
    snprintf(cmd, sizeof(cmd), "01%02X", PIDS[pollIdx].pid);
    sendReq(cmd, 1500);
    sentThisCycle++;
    return;
  }
  finishCycle();
}

void loggerStep() {
  if (lstate == L_OFF || lstate == L_PAUSED || state != LINKED || syncing) return;
  if (reqBusy) {
    if ((int32_t)(millis() - reqDeadline) < 0) return;
    reqBusy = false;  // timed out
    reqTimedOut = true;
  }
  bool completed = reqText.length() > 0;
  if (completed && reqTimedOut) {
    // Nothing at all came back several times in a row: the reader is gone.
    if (!respGotBytes) {
      if (++silentTimeouts >= 4) { dropLink("reader stopped responding"); return; }
    } else {
      silentTimeouts = 0;
    }
    // The reader may still be working on it: resync before the next request, then
    // carry on with whatever did arrive.
    pendingAfterSync = resp;
    havePendingAfterSync = true;
    reqTimedOut = false;
    reqText = "";
    beginSync();
    return;
  }
  if (completed) silentTimeouts = 0;
  if (!completed && !havePendingAfterSync && (int32_t)(millis() - nextLogAction) < 0) return;
  String r = havePendingAfterSync ? pendingAfterSync : resp;
  havePendingAfterSync = false;
  reqText = "";

  if (lstate == L_INIT) initHandle(r);
  else if (lstate == L_ENGINE_OFF) loggerStart();   // retry the whole start-up
  else if (lstate == L_POLL) pollHandle(r);
}

// --------------------------------------------------------------------- display
// 0.91" 128x32 SSD1306 OLED on I2C. Four text rows of 21 characters (5x7 font); RPM is
// drawn double size. One 128-byte page goes out per loop pass, so redrawing never holds
// up reader traffic; search steps (which block anyway) redraw at once.

static const int OLED_SDA = 21, OLED_SCL = 22;
static const uint32_t OLED_REFRESH_MS = 500;
uint8_t oledAddr = 0;          // 0 = no display found
uint8_t fb[4][128];            // frame buffer, one row per 8-pixel page
int oledPage = 4;              // next page to send; 4 = frame complete
uint32_t nextOledDraw = 0;
String searchNote;             // latest search step, shown while searching
int linkRssi = -127;           // Bluetooth: RSSI when the reader was found
uint32_t logUsedAt = 0;
size_t logUsed = 0, logTotal = 0;  // cached: usedBytes() walks the whole file system

// Classic 5x7 font, ASCII 32..126, 5 column bytes per character (bit 0 = top).
static const uint8_t FONT[] = {
  0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00, 0x14,0x7F,0x14,0x7F,0x14,
  0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62, 0x36,0x49,0x55,0x22,0x50, 0x00,0x05,0x03,0x00,0x00,
  0x00,0x1C,0x22,0x41,0x00, 0x00,0x41,0x22,0x1C,0x00, 0x08,0x2A,0x1C,0x2A,0x08, 0x08,0x08,0x3E,0x08,0x08,
  0x00,0x50,0x30,0x00,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00, 0x20,0x10,0x08,0x04,0x02,
  0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00, 0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31,
  0x18,0x14,0x12,0x7F,0x10, 0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
  0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E, 0x00,0x36,0x36,0x00,0x00, 0x00,0x56,0x36,0x00,0x00,
  0x08,0x14,0x22,0x41,0x00, 0x14,0x14,0x14,0x14,0x14, 0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x51,0x09,0x06,
  0x32,0x49,0x79,0x41,0x3E, 0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
  0x7F,0x41,0x41,0x22,0x1C, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01, 0x3E,0x41,0x49,0x49,0x7A,
  0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00, 0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41,
  0x7F,0x40,0x40,0x40,0x40, 0x7F,0x02,0x0C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
  0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46, 0x46,0x49,0x49,0x49,0x31,
  0x01,0x01,0x7F,0x01,0x01, 0x3F,0x40,0x40,0x40,0x3F, 0x1F,0x20,0x40,0x20,0x1F, 0x3F,0x40,0x38,0x40,0x3F,
  0x63,0x14,0x08,0x14,0x63, 0x07,0x08,0x70,0x08,0x07, 0x61,0x51,0x49,0x45,0x43, 0x00,0x7F,0x41,0x41,0x00,
  0x02,0x04,0x08,0x10,0x20, 0x00,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04, 0x40,0x40,0x40,0x40,0x40,
  0x00,0x01,0x02,0x04,0x00, 0x20,0x54,0x54,0x54,0x78, 0x7F,0x48,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x20,
  0x38,0x44,0x44,0x48,0x7F, 0x38,0x54,0x54,0x54,0x18, 0x08,0x7E,0x09,0x01,0x02, 0x0C,0x52,0x52,0x52,0x3E,
  0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00, 0x20,0x40,0x44,0x3D,0x00, 0x7F,0x10,0x28,0x44,0x00,
  0x00,0x41,0x7F,0x40,0x00, 0x7C,0x04,0x18,0x04,0x78, 0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38,
  0x7C,0x14,0x14,0x14,0x08, 0x08,0x14,0x14,0x18,0x7C, 0x7C,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x20,
  0x04,0x3F,0x44,0x40,0x20, 0x3C,0x40,0x40,0x20,0x7C, 0x1C,0x20,0x40,0x20,0x1C, 0x3C,0x40,0x30,0x40,0x3C,
  0x44,0x28,0x10,0x28,0x44, 0x0C,0x50,0x50,0x50,0x3C, 0x44,0x64,0x54,0x4C,0x44, 0x00,0x08,0x36,0x41,0x00,
  0x00,0x00,0x7F,0x00,0x00, 0x00,0x41,0x36,0x08,0x00, 0x08,0x04,0x08,0x10,0x08,
};

void oledCmds(const uint8_t *c, size_t n) {
  Wire.beginTransmission(oledAddr);
  Wire.write(0x00);  // control byte: commands follow
  Wire.write(c, n);
  Wire.endTransmission();
}

void oledSendPage(int p) {
  const uint8_t window[] = {0x21, 0, 127, 0x22, (uint8_t)p, (uint8_t)p};
  oledCmds(window, sizeof(window));
  for (int x = 0; x < 128; x += 32) {  // Wire's buffer is 128 bytes, so send in chunks
    Wire.beginTransmission(oledAddr);
    Wire.write(0x40);  // control byte: display data follows
    Wire.write(&fb[p][x], 32);
    Wire.endTransmission();
  }
}

void oledBegin() {
  Wire.begin(OLED_SDA, OLED_SCL, 400000);
  Wire.setTimeOut(20);
  for (uint8_t a : {0x3C, 0x3D}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { oledAddr = a; break; }
  }
  if (!oledAddr) { Wire.end(); return; }
  static const uint8_t init[] = {
    0xAE,        // display off
    0xD5, 0x80,  // clock
    0xA8, 0x1F,  // 32 rows
    0xD3, 0x00, 0x40,
    0x8D, 0x14,  // charge pump on (module runs from 3.3 V)
    0x20, 0x00,  // horizontal addressing
    0xA1, 0xC8,  // normal orientation (pins at the top-left)
    0xDA, 0x02,  // COM pin layout for 128x32
    0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40,
    0xA4, 0xA6, 0xAF,  // show RAM, not inverted, display on
  };
  oledCmds(init, sizeof(init));
}

// Draw text at a text row (0-3) and pixel column; big = double size over two rows.
int oledText(int row, int x, const String &s, bool big = false) {
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = s[i];
    if (c < 32 || c > 126) c = '?';
    const uint8_t *g = FONT + (c - 32) * 5;
    for (int col = 0; col < 6; col++) {
      uint8_t bits = col < 5 ? g[col] : 0;
      if (!big) {
        if (x < 128) fb[row][x] = bits;
        x++;
        continue;
      }
      uint16_t tall = 0;
      for (int b = 0; b < 8; b++)
        if (bits >> b & 1) tall |= 3u << (2 * b);
      for (int k = 0; k < 2; k++, x++)
        if (x < 128 && row < 3) { fb[row][x] = tall; fb[row + 1][x] = tall >> 8; }
    }
  }
  return x;
}

// Latest raw value of a logged PID, if the car has sent it this session.
bool liveVal(uint8_t pid, uint8_t &a, uint8_t &b) {
  for (int i = 0; i < NPIDS; i++)
    if (PIDS[i].pid == pid) {
      if (!(seenMask >> i & 1)) return false;
      a = vals[i][0];
      b = vals[i][1];
      return true;
    }
  return false;
}

String fit(const String &s) { return s.length() > 21 ? s.substring(0, 21) : s; }

// Signal bars at the top right, like a phone: 0-4 bars from the link's RSSI.
// Wi-Fi is live; Bluetooth shows the strength measured when the reader was found.
void oledBars(int rssi) {
  int bars = rssi >= -60 ? 4 : rssi >= -70 ? 3 : rssi >= -80 ? 2 : rssi >= -90 ? 1 : 0;
  for (int i = 0; i < 4; i++) {
    uint8_t col = i < bars ? (uint8_t)(0xFF << (6 - 2 * i)) : 0x80;  // lit bar or a dot
    for (int k = 0; k < 3; k++) fb[0][113 + 4 * i + k] = col;
  }
}

void oledCompose() {
  memset(fb, 0, sizeof(fb));
  bool linked = state == LINKED && active >= 0;
  oledBars(!linked ? -127 : readers[active].type == T_WIFI ? WiFi.RSSI() : linkRssi);

  if (!linked) {  // row 0: search round; rows 1-3: the current step, word-wrapped
    oledText(0, 0, "SEARCHING round " + String(searchRound));
    String rest = searchNote;
    for (int row = 1; row <= 3 && rest.length(); row++) {
      int cut = rest.length() <= 21 ? rest.length() : rest.lastIndexOf(' ', 21);
      if (cut <= 0) cut = std::min<int>(21, rest.length());
      oledText(row, 0, rest.substring(0, cut));
      rest = rest.substring(cut);
      rest.trim();
    }
    return;
  }

  // Row 0: what the board is doing with the reader (18 characters, left of the bars).
  if (fsReady && (logTotal == 0 || millis() - logUsedAt > 10000)) {
    logUsedAt = millis();
    logUsed = LittleFS.usedBytes();
    logTotal = std::max<size_t>(LittleFS.totalBytes(), 1);
  }
  String doing;
  if (owner != OWNER_LOGGER) doing = "MAC USING READER";
  else if (!logEnabled) doing = "LOGGING OFF";
  else if (!fsReady) doing = "LOG STORAGE ERROR";
  else if (lstate == L_INIT) doing = "STARTING LOGGER";
  else if (lstate == L_ENGINE_OFF) doing = "WAITING FOR ENGINE";
  else if (lstate == L_POLL) doing = "REC " + String(supportedCount()) + " PIDs";
  oledText(0, 0, doing.substring(0, 18));

  uint32_t m = millis() / 60000;
  String up = "up " + String(m / 60) + "h" + (m % 60 < 10 ? "0" : "") + String(m % 60) + "m";
  String logKb = fsReady ? "log " + String(logUsed / 1024) + "/" + String(logTotal / 1024) + " KB" : "";

  // Rows 2-3: live engine values, trouble codes, or log use and uptime.
  // With codes present the rows alternate: live values for 4 s, then each code for 2.5 s.
  bool carTalking = owner == OWNER_LOGGER && (lstate == L_POLL || lstate == L_ENGINE_OFF);
  std::vector<std::pair<String, bool>> codes;  // (code, stored)
  if (carTalking) {
    for (int k = 0; k < 2; k++) {
      const String &list = k == 0 ? carDtcs : carPending;
      for (int at = 0; at + 5 <= (int)list.length(); at += 6)  // "P0301 P0420 ..."
        codes.push_back({list.substring(at, at + 5), k == 0});
    }
  }
  uint8_t a, b;
  bool live = carTalking && lstate == L_POLL && liveVal(0x0C, a, b);
  int codeShown = -1;
  if (!codes.empty()) {
    uint32_t liveMs = live ? 4000 : 0, cycle = liveMs + 2500 * codes.size();
    uint32_t t = millis() % cycle;
    if (t >= liveMs) codeShown = (t - liveMs) / 2500;
  }
  if (live || codeShown >= 0)  // row 1 is free: log use and uptime on one line
    oledText(1, 0, fit((fsReady ? "log " + String(logUsed * 100 / logTotal) + "%  " : String("")) + up));
  if (codeShown >= 0) {
    oledText(2, 0, codes[codeShown].first, true);
    oledText(2, 66, codes[codeShown].second ? "STORED" : "PENDING");
    oledText(3, 66, String(codeShown + 1) + " of " + String(codes.size()));
  } else if (live) {
    oledText(2, 0, String((a * 256 + b) / 4), true);
    String r2 = "rpm", r3;
    if (liveVal(0x05, a, b)) r2 += " CLT " + String((a - 40) * 9 / 5 + 32) + "F";
    if (liveVal(0x0D, a, b)) r3 = String((int)(a * 0.6214f + 0.5f)) + "mph";
    if (liveVal(0x42, a, b)) r3 += " " + String((a * 256 + b) / 1000.0f, 1) + "V";
    else if (liveVal(0x04, a, b)) r3 += " LD " + String(a * 100 / 255) + "%";
    oledText(2, 56, r2);
    oledText(3, 56, r3);
  } else {
    oledText(2, 0, logKb);
    oledText(3, 0, up + (carTalking && dtcsChecked ? "  no codes" : ""));
  }
}

// Called from the main loop: sends one page per pass, starts a new frame every 500 ms.
void oledTick() {
  if (!oledAddr) return;
  if (oledPage < 4) { oledSendPage(oledPage++); return; }
  if ((int32_t)(millis() - nextOledDraw) < 0) return;
  nextOledDraw = millis() + OLED_REFRESH_MS;
  oledCompose();
  oledPage = 0;
}

// Search steps block the loop for seconds at a time, so show each one immediately.
void oledSearchNote(const String &note) {
  searchNote = note;
  if (!oledAddr) return;
  oledCompose();
  for (int p = 0; p < 4; p++) oledSendPage(p);
  oledPage = 4;
}

// --------------------------------------------------------------------- linking

void setLinked(int i, const String &detail) {
  state = LINKED;
  active = i;
  tcpFails = 0;
  owner = held ? OWNER_TO_MAC : OWNER_LOGGER;
  macPending.clear();
  status("READY reader=" + describe(i) + detail);
  if (owner == OWNER_TO_MAC) beginSync();   // a Mac program is waiting for the reader
  else loggerStart();
}

void dropLink(const String &why) {
  if (state == LINKED) status("LOST " + why + ", searching again");
  bool wasBt = state == LINKED && active >= 0 && readers[active].type == T_BT;
  loggerReset();
  syncing = false;
  state = SEARCHING;
  active = -1;
  owner = OWNER_LOGGER;
  macPending.clear();
  obd.stop();
  if (btReady && (wasBt || bt.connected())) {
    // Restart Bluetooth so bytes queued for the old reader can't reach the next one.
    bt.disconnect();
    bt.end();
    btReady = (bool)bt.begin(BT_NAME, true);
    if (!btReady) status("ERR Bluetooth restart failed");
  }
  WiFi.disconnect(true);
  nextSearch = millis() + 500;
}

// Wi-Fi readers here close idle TCP connections after ~2 s, so the TCP link is opened
// on demand whenever there is something to send.
bool ensureTcp() {
  if (active < 0 || readers[active].type != T_WIFI) return false;
  if (obd.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  const Reader &r = readers[active];
  if (obd.connect(r.host.c_str(), r.port, 3000)) {
    obd.setNoDelay(true);
    tcpFails = 0;
    return true;
  }
  status("ERR reader unreachable at " + r.host + ":" + String(r.port));
  if (state == LINKED && ++tcpFails >= 3) dropLink("reader TCP port not answering");
  return false;
}

void linkWrite(const uint8_t *d, size_t n) {
  if (state != LINKED || active < 0) return;
  if (readers[active].type == T_BT) { if (bt.connected()) bt.write(d, n); }
  else if (ensureTcp()) obd.write(d, n);
}

bool tryWifi() {
  std::vector<int> wanted;
  for (int i = 0; i < (int)readers.size(); i++)
    if (readers[i].type == T_WIFI && allowed(i)) wanted.push_back(i);
  if (wanted.empty()) return false;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);  // modem sleep is required while Bluetooth is also initialized
  int n = WiFi.scanNetworks();
  if (n < 0) {
    status("SEARCH wifi scan failed (" + String(n) + ")");
    WiFi.scanDelete();
    return false;
  }
  std::vector<std::pair<int, int>> seen;  // (rssi, reader index), strongest first
  for (int w : wanted) {
    int best = -1000;
    for (int k = 0; k < n; k++)
      if (WiFi.SSID(k) == readers[w].target) best = std::max(best, (int)WiFi.RSSI(k));
    if (best > -1000) seen.push_back({best, w});
  }
  WiFi.scanDelete();
  std::sort(seen.begin(), seen.end(), [](const std::pair<int, int> &a, const std::pair<int, int> &b) { return a.first > b.first; });
  status("SEARCH wifi: " + String(seen.size()) + " of " + String(wanted.size()) +
         " listed networks visible");

  for (auto &cand : seen) {
    const Reader &r = readers[cand.second];
    status("SEARCH joining " + r.target + " (" + String(cand.first) + "dBm)");
    WiFi.begin(r.target.c_str(), r.secret.length() ? r.secret.c_str() : nullptr);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_JOIN_MS) delay(100);
    if (WiFi.status() != WL_CONNECTED) {
      status("SEARCH could not join " + r.target);
      WiFi.disconnect(true);
      continue;
    }
    active = cand.second;  // ensureTcp() needs to know which reader
    if (ensureTcp()) {
      linkRssi = WiFi.RSSI();
      setLinked(cand.second, " rssi=" + String(WiFi.RSSI()) + "dBm");
      return true;
    }
    active = -1;
    WiFi.disconnect(true);
  }
  return false;
}

bool btPeerIs(const BTAddress &addr) {
  for (auto &p : bt.peers()) if (p == addr) return true;
  return false;
}

bool btConnectTo(int i, const BTAddress &addr) {
  Reader &r = readers[i];
  std::vector<String> pins;
  if (r.secret == "auto" || r.secret.length() == 0) pins = {"1234", "0000", "6789"};
  else pins = {r.secret};
  for (auto &pin : pins) {
    if (bt.connected()) bt.disconnect();
    bt.setPin(pin.c_str());
    status("SEARCH connecting to " + r.target + " (pin " + pin + ")");
    BTStatus st = bt.connect(addr);
    if (st && bt.connected() && btPeerIs(addr)) {
      if (r.secret != pin) {  // learned the PIN for next time
        r.secret = pin;
        saveAll();
        status("OK saved pin " + pin + " for reader " + String(i));
      }
      return true;
    }
    if (bt.connected()) bt.disconnect();  // a late or wrong connection
    status(String("SEARCH failed: ") + st.toString());
  }
  return false;
}

bool tryBt() {
  if (!btReady) return false;
  std::vector<int> wanted;
  for (int i = 0; i < (int)readers.size(); i++)
    if (readers[i].type == T_BT && allowed(i)) wanted.push_back(i);
  if (wanted.empty()) return false;

  WiFi.mode(WIFI_OFF);  // give Bluetooth the radio during inquiry and connection
  auto found = bt.discover(BT_DISCOVER_MS);
  struct Cand { int rssi; int idx; BTAddress addr; };
  std::vector<Cand> seen;
  for (int w : wanted) {
    const Reader &r = readers[w];
    bool hit = false;
    Cand c{-1000, w, BTAddress()};
    for (auto &d : found) {
      bool match = isMac(r.target) ? d.address.toString(true).equalsIgnoreCase(r.target)
                                   : d.name.equalsIgnoreCase(r.target);
      if (match && d.rssi > c.rssi) { c.rssi = d.rssi; c.addr = d.address; hit = true; }
    }
    if (hit) seen.push_back(c);
  }
  std::sort(seen.begin(), seen.end(), [](const Cand &a, const Cand &b) { return a.rssi > b.rssi; });
  status("SEARCH bt: " + String(seen.size()) + " of " + String(wanted.size()) +
         " listed readers visible");
  for (auto &c : seen) {
    if (btConnectTo(c.idx, c.addr)) {
      Reader &r = readers[c.idx];
      if (!isMac(r.target)) {  // matched by name: remember its address from now on
        r.target = c.addr.toString(true);
        saveAll();
      }
      linkRssi = c.rssi;
      setLinked(c.idx, " rssi=" + String(c.rssi) + "dBm");
      return true;
    }
  }
  return false;
}

void searchStep() {
  searchRound++;
  if (searchRound == 1 || searchRound % 10 == 0)
    status("SEARCH round " + String(searchRound) + " mode=" + mode);
  if (tryWifi()) return;
  if (tryBt()) return;
  nextSearch = millis() + ROUND_PAUSE_MS;
}

// -------------------------------------------------------------------- commands

void cmdScanWifi() {
  dropLink("Wi-Fi scan requested");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  int n = WiFi.scanNetworks();
  int shown = 0;
  for (int k = 0; k < n; k++) {
    String s = WiFi.SSID(k);
    bool listed = false;
    for (auto &r : readers) listed |= (r.type == T_WIFI && r.target == s);
    if (listed || looksLikeObd(s)) {  // other nearby networks are only counted
      status("SCAN found " + s + " rssi=" + String(WiFi.RSSI(k)) + "dBm ch=" +
             String(WiFi.channel(k)) + (listed ? " (listed)" : ""));
      shown++;
    }
  }
  status("SCAN done, " + String(n < 0 ? 0 : n) + " networks seen, " + String(shown) +
         " look like OBD readers");
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
}

void cmdScanBt() {
  if (!btReady) { status("ERR Bluetooth not available"); return; }
  dropLink("Bluetooth scan requested");
  WiFi.mode(WIFI_OFF);
  status("BTSCAN started (10 s)");
  auto found = bt.discover(10000);
  struct Dev { String addr; String name; int rssi; };
  std::vector<Dev> devs;  // merged by address: keep a non-empty name and the best signal
  for (auto &d : found) {
    String addr = d.address.toString(true);
    auto it = std::find_if(devs.begin(), devs.end(), [&](const Dev &x) { return x.addr == addr; });
    if (it == devs.end()) devs.push_back({addr, d.name, d.rssi});
    else {
      if (it->name.length() == 0) it->name = d.name;
      it->rssi = std::max(it->rssi, (int)d.rssi);
    }
  }
  int shown = 0;
  for (auto &d : devs) {
    bool listed = false;
    for (auto &r : readers)
      listed |= (r.type == T_BT && (r.target.equalsIgnoreCase(d.addr) || r.target.equalsIgnoreCase(d.name)));
    if (listed || looksLikeObd(d.name)) {  // other nearby devices are only counted
      status("BTSCAN found \"" + d.name + "\" " + d.addr + " rssi=" + String(d.rssi) + "dBm" +
             (listed ? " (listed)" : ""));
      shown++;
    }
  }
  status("BTSCAN done, " + String(devs.size()) + " devices seen, " + String(shown) +
         " look like OBD readers");
}

void reportStatus() {
  String s = "STATUS mode=" + mode + " state=" + (state == LINKED ? "linked" : "searching");
  if (state == LINKED && active >= 0) {
    s += " reader=" + String(active) + " type=" + (readers[active].type == T_WIFI ? "wifi" : "bt");
    s += " target=" + readers[active].target;
    if (readers[active].type == T_WIFI) s += " rssi=" + String(WiFi.RSSI()) + "dBm";
    s += String(" owner=") + (owner == OWNER_LOGGER ? "logger" : "mac");
  } else {
    s += " round=" + String(searchRound);
  }
  s += String(" log=") + (logEnabled ? lstateName() : "disabled");
  if (fsReady) {
    s += " log_used=" + String(LittleFS.usedBytes() / 1024) + "/" +
         String(LittleFS.totalBytes() / 1024) + "KB records=" + String(recordsWritten);
  } else {
    s += " log_storage=unavailable";
  }
  if (lstate == L_POLL || lstate == L_ENGINE_OFF)
    s += " car_protocol=" + String(carProtocol) + " values=" + String(supportedCount());
  s += " time=" + String(epochAtBoot ? "set" : "unknown");
  s += String(" display=") + (oledAddr ? "yes" : "none");
  s += " readers=" + String(readers.size()) + " heap=" + String(ESP.getFreeHeap()) +
       " uptime=" + String(millis() / 1000) + "s";
  status(s);
}

void cmdLogGet(String name) {
  name.trim();
  if (name.startsWith("/")) name = name.substring(1);
  if (name.indexOf('/') >= 0 || !name.endsWith(".bin")) { status("ERR bad file name"); return; }
  if (logFile && name == currentName()) flushLog(true);
  File f = LittleFS.open("/" + name, "r");
  if (!f) { status("ERR no such log file " + name); return; }
  status("LOGBEGIN " + name + " " + String(f.size()));
  uint8_t chunk[96];
  uint32_t crc = 0;
  size_t total = 0;
  while (true) {
    int n = f.read(chunk, sizeof(chunk));
    if (n <= 0) break;
    crc = crc32(chunk, n, crc);
    total += n;
    status("LOGDATA " + base64::encode(chunk, n));
  }
  f.close();
  char hex[9];
  snprintf(hex, sizeof(hex), "%08lx", (unsigned long)crc);
  status("LOGEND " + name + " " + String(total) + " " + hex);
}

void macTakeOver() {
  if (state != LINKED || owner != OWNER_LOGGER) return;
  owner = OWNER_TO_MAC;
  if (lstate != L_OFF) {
    if (sessionOpen) logEvent(3);
    lstate = L_PAUSED;
    status("LOG paused while the Mac uses the reader");
  }
  beginSync();
}

void macRelease() {
  if (state != LINKED) { owner = OWNER_LOGGER; return; }
  if (owner == OWNER_LOGGER) return;
  owner = OWNER_LOGGER;
  macPending.clear();
  if (logEnabled && fsReady) {
    status("LOG resuming");
    loggerStart();   // the Mac may have changed reader settings: redo start-up
  }
}

void handleCommand(String line) {
  line.trim();
  if (line == "#STATUS") {
    reportStatus();
  } else if (line == "#HOLD") {
    held = true;
    macTakeOver();
    status("OK held");
  } else if (line == "#RELEASE") {
    held = false;
    macRelease();
    status("OK released");
  } else if (line == "#LIST") {
    for (int i = 0; i < (int)readers.size(); i++) status("LIST " + describe(i));
    status("LIST done, " + String(readers.size()) + " readers, mode=" + mode);
  } else if (line.startsWith("#ADDWIFI ")) {
    auto f = fields(line.substring(9), 4);
    if (f[0].length() == 0) { status("ERR usage: #ADDWIFI ssid,password,host,port"); return; }
    if ((int)readers.size() >= MAX_READERS) { status("ERR list full, #DEL one first"); return; }
    for (int i = 0; i < (int)readers.size(); i++)
      if (readers[i].type == T_WIFI && readers[i].target == f[0]) {
        status("ERR already listed as reader " + String(i));
        return;
      }
    if (f[3].length() && (!isDigits(f[3]) || f[3].toInt() <= 0 || f[3].toInt() > 65535)) {
      status("ERR port must be 1-65535");
      return;
    }
    Reader r;
    r.type = T_WIFI; r.target = f[0]; r.secret = f[1];
    r.host = f[2].length() ? f[2] : String(DEF_HOST);
    r.port = f[3].length() ? f[3].toInt() : DEF_PORT;
    readers.push_back(r);
    saveAll();
    status("OK added reader " + describe(readers.size() - 1));
  } else if (line.startsWith("#ADDBT ")) {
    auto f = fields(line.substring(7), 2);
    if (f[0].length() == 0) { status("ERR usage: #ADDBT MAC-or-name,pin"); return; }
    if ((int)readers.size() >= MAX_READERS) { status("ERR list full, #DEL one first"); return; }
    for (int i = 0; i < (int)readers.size(); i++)
      if (readers[i].type == T_BT && readers[i].target.equalsIgnoreCase(f[0])) {
        status("ERR already listed as reader " + String(i));
        return;
      }
    Reader r;
    r.type = T_BT; r.target = f[0]; r.secret = f[1].length() ? f[1] : String("auto");
    if (isMac(r.target)) r.target.toUpperCase();
    readers.push_back(r);
    saveAll();
    status("OK added reader " + describe(readers.size() - 1));
  } else if (line.startsWith("#DEL ")) {
    String a = line.substring(5);
    a.trim();
    if (!isDigits(a) || a.toInt() >= (int)readers.size()) { status("ERR usage: #DEL n (see #LIST)"); return; }
    int i = a.toInt();
    if (state == LINKED && active == i) dropLink("reader removed");
    else if (state == LINKED && active > i) active--;
    String gone = describe(i);
    readers.erase(readers.begin() + i);
    // A mode pinned to a single reader no longer points at the same entry: go back to auto.
    if (mode != "auto" && mode != "wifi" && mode != "bt") mode = "auto";
    saveAll();
    status("OK removed reader " + gone);
  } else if (line.startsWith("#MODE ")) {
    String m = line.substring(6);
    m.trim();
    bool index = isDigits(m) && m.toInt() < (int)readers.size();
    if (m != "auto" && m != "wifi" && m != "bt" && !index) {
      status("ERR usage: #MODE auto|wifi|bt|n");
      return;
    }
    if (index) m = String(m.toInt());
    if (m == mode) { status("OK mode=" + mode); return; }
    mode = m;
    saveAll();
    if (state == LINKED && !allowed(active)) dropLink("mode changed");
    status("OK mode=" + mode);
  } else if (line == "#SCAN") {
    cmdScanWifi();
    nextSearch = millis() + 1000;
  } else if (line == "#BTSCAN") {
    cmdScanBt();
    nextSearch = millis() + 1000;
  } else if (line == "#RESET") {
    dropLink("settings reset");
    defaultReaders();
    logEnabled = true;
    saveAll();
    status("OK defaults restored, mode=auto");
  } else if (line.startsWith("#TIME ")) {
    String a = line.substring(6);
    a.trim();
    uint32_t t = isDigits(a) ? strtoul(a.c_str(), nullptr, 10) : 0;
    if (t < 1600000000UL) { status("ERR usage: #TIME unix-seconds"); return; }
    epochAtBoot = t - millis() / 1000;
    if (sessionOpen) {
      std::vector<uint8_t> r = {0xA6};
      put32(r, millis() - sessionStart);
      put32(r, t);
      appendRecord(r);
    }
    status("OK time set");
  } else if (line == "#LOG on" || line == "#LOG off") {
    logEnabled = line.endsWith("on");
    saveAll();
    if (!logEnabled) loggerReset();
    else if (state == LINKED && owner == OWNER_LOGGER && lstate == L_OFF) loggerStart();
    status(String("OK logging ") + (logEnabled ? "on" : "off"));
  } else if (line == "#LOGLIST") {
    if (!fsReady) { status("ERR log storage unavailable"); return; }
    if (logFile) flushLog(true);
    auto names = logFiles();
    for (auto &n : names) {
      File f = LittleFS.open("/" + n, "r");
      status("LOGFILE " + n + " " + String(f ? f.size() : 0));
      if (f) f.close();
    }
    status("LOGLIST done, " + String(names.size()) + " files, " +
           String(LittleFS.usedBytes() / 1024) + "/" + String(LittleFS.totalBytes() / 1024) + "KB used");
  } else if (line.startsWith("#LOGGET ")) {
    if (!fsReady) { status("ERR log storage unavailable"); return; }
    cmdLogGet(line.substring(8));
  } else if (line == "#LOGCLEAR") {
    if (!fsReady) { status("ERR log storage unavailable"); return; }
    bool wasOpen = sessionOpen;
    if (logFile) { flushLog(true); logFile.close(); }
    for (auto &n : logFiles()) LittleFS.remove("/" + n);
    recordsWritten = 0;
    if (wasOpen) { segment++; openSegment(active < 0 ? 255 : active); }
    status("OK logs cleared");
  } else {
    status("ERR unknown command");
  }
}

// ---------------------------------------------------------------------- main

#ifdef OLED_DEMO
// --------------------------------------------------------------------- display demo
// Test build only (-DOLED_DEMO, see firmware/wokwi/): no reader, no Bluetooth. Plays a
// one-minute script of fake states, values and codes so every screen can be checked in
// the Wokwi simulator. Times are seconds since boot; the script repeats.
//   0- 8 searching   8-12 starting   12-24 recording   24-40 recording + 2 codes
//   40-48 waiting for engine (no codes)   48-56 Mac using reader
void demoVal(uint8_t pid, uint16_t v) {
  for (int i = 0; i < NPIDS; i++)
    if (PIDS[i].pid == pid) { vals[i][0] = PIDS[i].bytes == 2 ? v >> 8 : v; vals[i][1] = PIDS[i].bytes == 2 ? v & 0xFF : 0; seenMask |= 1u << i; }
}

void demoTick() {
  static int lastPhase = -1, lastSec = -1;
  uint32_t t = (millis() / 1000) % 56;
  int phase = t < 8 ? 0 : t < 12 ? 1 : t < 24 ? 2 : t < 40 ? 3 : t < 48 ? 4 : 5;
  if (phase == 0) {
    state = SEARCHING; active = -1; searchRound = 3;
    if ((int)t != lastSec && (t == 0 || t == 4))
      status(t == 0 ? "SEARCH wifi: 0 of 1 listed networks visible"
                    : "SEARCH connecting to AA:BB:CC:DD:EE:FF (pin 1234)");
  } else {
    state = LINKED; active = 1; linkRssi = -67;  // three bars
    owner = phase == 5 ? OWNER_MAC : OWNER_LOGGER;
    lstate = phase == 1 ? L_INIT : phase == 4 ? L_ENGINE_OFF : L_POLL;
    supportKnown[0] = true; support[0] = 0xFFFFFFFF;
    dtcsChecked = phase >= 2;
    carDtcs = phase == 3 ? "P0301" : "";
    carPending = phase == 3 ? "P0171" : "";
    if (phase == 1 && lastPhase != 1) seenMask = 0;
    if (phase == 2 || phase == 3) {
      float w = millis() / 1000.0f;
      demoVal(0x0C, (uint16_t)((1850 + 650 * sinf(w * 0.7f)) * 4));  // rpm
      demoVal(0x05, 40 + 85 + (t > 30 ? 6 : t / 5));                  // coolant ~185-197 F
      demoVal(0x0D, (uint8_t)(56 + 20 * sinf(w * 0.3f)));              // km/h
      demoVal(0x04, (uint8_t)(60 + 25 * sinf(w * 0.9f)));              // load
    }
  }
  lastPhase = phase;
  lastSec = t;
}
#endif

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.setRxBufferSize(4096);  // holds Mac input while a blocking search step runs
  Serial.begin(115200);
  // The Bluetooth controller prints internal warnings ("ASSERT_WARN ... lc_task.c")
  // through ROM printf, straight into the middle of reader data. Disconnect ROM printf
  // from the UART; our own output uses the Serial driver and is unaffected.
  esp_rom_install_channel_putc(1, nullptr);
  oledBegin();
  delay(200);
  // Reset reasons: 1=power-on 3=software 4=crash 6=task watchdog 9=brownout (weak power)
  status("BOOT esp32 obd bridge v4 reset_reason=" + String((int)esp_reset_reason()));
  loadAll();
  fsReady = LittleFS.begin(true);  // formats the log partition on first use
  if (!fsReady) status("ERR log storage could not be mounted; logging disabled");
#ifdef OLED_DEMO
  status("DEMO display demo build: no reader, fake data");
#else
  BTStatus st = bt.begin(BT_NAME, true);  // true = we initiate connections
  btReady = (bool)st;
  if (!btReady) status(String("ERR Bluetooth init failed: ") + st.toString());
#endif
  status("MODE " + mode + " readers=" + String(readers.size()) + " log=" +
         (logEnabled ? "on" : "off") + " display=" + (oledAddr ? "yes" : "none") +
         " heap=" + String(ESP.getFreeHeap()));
  nextSearch = millis();
}

void loop() {
#ifdef OLED_DEMO
  demoTick();
  oledTick();
  delay(1);
  return;
#endif
  // Link health.
  if (state == LINKED) {
    if (active < 0 || active >= (int)readers.size()) dropLink("reader index invalid");
    else if (readers[active].type == T_BT && !bt.connected()) dropLink("Bluetooth reader disconnected");
    else if (readers[active].type == T_WIFI && WiFi.status() != WL_CONNECTED) dropLink("Wi-Fi reader network lost");
  }

  // Serial: "#" commands, or reader data from the Mac.
  while (Serial.available()) {
    char c = Serial.read();
    if (!inCmd && c == '#') { inCmd = true; cmdLine = "#"; continue; }
    if (inCmd) {
      if (c == '\n' || c == '\r') { inCmd = false; handleCommand(cmdLine); }
      else if (cmdLine.length() < 160) cmdLine += c;
      continue;
    }
    if (macPending.size() < 4096) macPending.push_back((uint8_t)c);
    lastMacData = millis();
  }

  if (!macPending.empty()) {
    if (state != LINKED) {
      if (millis() - lastNotLinkedMsg > 2000) {
        lastNotLinkedMsg = millis();
        status("ERR not connected to a reader yet; data dropped");
      }
      macPending.clear();
    } else if (owner == OWNER_LOGGER) {
      macTakeOver();                      // raw-serial use without #HOLD
    } else if (owner == OWNER_MAC) {
      linkWrite(macPending.data(), macPending.size());
      macPending.clear();
    }                                     // OWNER_TO_MAC: wait for the sync to finish
  }

  // Raw-serial fallback: give the reader back after the Mac has been quiet a while.
  if (state == LINKED && !held && owner == OWNER_MAC && millis() - lastMacData > MAC_IDLE_MS)
    macRelease();

  // Reader -> sync / Mac / logger.
  uint8_t buf[256];
  if (state == LINKED && active >= 0) {
    for (;;) {
      int n = 0;
      if (readers[active].type == T_BT) {
        int a = bt.connected() ? bt.available() : 0;
        if (a <= 0) break;
        n = std::min(a, (int)sizeof(buf));
        for (int i = 0; i < n; i++) buf[i] = bt.read();
      } else {
        if (!obd.connected() || !obd.available()) break;
        n = obd.read(buf, sizeof(buf));
        if (n <= 0) break;
      }
      if (syncing) syncLastByte = millis();          // discarded
      else if (owner == OWNER_MAC) Serial.write(buf, n);
      else if (owner == OWNER_LOGGER) loggerFeed(buf, n);
    }
  }

  // Finish a resync once the reader has gone quiet (or after SYNC_MAX_MS regardless).
  if (syncing) {
    uint32_t now = millis();
    if ((now - syncLastByte >= SYNC_QUIET_MS && now - syncStartMs >= SYNC_MIN_MS) ||
        now - syncStartMs >= SYNC_MAX_MS) {
      syncing = false;
      if (owner == OWNER_TO_MAC) {
        owner = OWNER_MAC;
        if (!macPending.empty()) { linkWrite(macPending.data(), macPending.size()); macPending.clear(); }
      }
    }
  }

  if (state == LINKED && owner == OWNER_LOGGER) loggerStep();
  flushLog(false);
  oledTick();

  // Search when not linked (each step blocks for a few seconds while scanning).
  if (state == SEARCHING && (int32_t)(millis() - nextSearch) >= 0) searchStep();

  // LED: solid = connected to a reader, fast blink = searching.
  digitalWrite(LED_PIN, state == LINKED ? HIGH : ((millis() / 150) % 2));
  delay(1);
}
