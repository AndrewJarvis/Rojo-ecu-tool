// ROJO Tuner sketch - UNOFFICIAL hobby tool. Not affiliated with, made or endorsed by ROJO /
// Zhejiang Langjie Electronics or any motorcycle maker. Writing settings to an engine control unit
// can damage the engine or ECU. Use entirely at your own risk; no warranty of any kind.
// See the terms and manual in rojo_tuner_app.html.
//
// ROJO R32-Z ECU - tuner: live data, reads, and GUARDED writes
// Arduino Nano + MCP2515 (autowp "arduino-mcp2515" library), 500 kbps
//
// Protocol (from the app's Java + your captures):
//   Requests go to CAN ID 0x201, replies/live data come from CAN ID 0x200.
//   A message is:  <cmd> <len_lo> <len_hi> <payload...> <crc>
//     len = payload bytes + 3, crc = XOR of len_lo, len_hi and payload.
//   Fits in one CAN frame -> first byte is just <cmd>. Otherwise each frame starts
//   with a byte whose top two bits are 01 first / 10 middle / 11 last, low 6 bits = cmd.
//
// Type a command in the terminal and press ENTER (115200 baud):
//   c            connect again
//   r            read idle/limit, tuning block (fuel factor) and run mode
//   d            decoded live data        x   raw hex of live data
//   l            toggle one-line-per-second live log
//   same         write back the CURRENT mode, fuel factor, idle and limit (no change) - first test
//   idle N       target idle rpm, 1200-2000
//   limit N      rev limit rpm, 3000-10000
//   fuel F       fuel factor, 1.000-3.000 (1.000 = neutral; only adds fuel)
//   mode N       run mode 0 Comfort, 1 Sport, 2 Competition
//   fuelon       allow fuel factor changes for this session (off after every restart); fueloff turns it off
//   fuel F       fuel factor 1.000-3.000 (1.000 = unchanged, higher = more fuel); needs fuelon first
//   clear        erase stored fault codes/history (asks y + ENTER; a fault still present comes back)
//   h            help
// Every write shows the current values, asks you to type y + ENTER, re-reads
// afterwards and says VERIFIED or MISMATCH. Writes are refused while the engine
// is running (rpm >= 50), while the ECU reports LOCKED, or with no live data.
// The ranges are the same ones the original app enforces.
//
// NOTE: Unlock (password) is NOT implemented. The ECU currently reports itself
// unlocked. If a write gets "NO ACK", tell me before trying anything else.
//
// Wiring: CS->D10 SI->D11 SO->D12 SCK->D13 VCC->5V
//         CANH black/white, CANL orange/black, GND blue (signal ground). J1 closed.

#include <SPI.h>
#include <mcp2515.h>

#define CS_PIN       10
#define CAN_XTAL     MCP_8MHZ
#define SERIAL_BAUD  115200
#define TX_ID        0x201
#define RX_ID        0x200
#define CMD_DAQ      62

MCP2515 mcp2515(CS_PIN);
struct can_frame txf;
struct can_frame rxf;

const uint16_t DAQ_MAX = 176;
const uint16_t RPL_MAX = 96;
uint8_t daqBuf[DAQ_MAX], daqMsg[DAQ_MAX];
uint8_t rplBuf[RPL_MAX], rplMsg[RPL_MAX];
uint16_t daqLen = 0, rplLen = 0;
uint8_t rplCmd = 0;
bool daqValid = false, rplReady = false;
uint32_t daqStamp = 0;
uint16_t daqCount = 0, badLen = 0, badCrc = 0;
uint8_t protVer = 0;
bool liveOn = false;

struct Ctx {
  uint8_t *buf;
  uint16_t cap;
  uint16_t got;
  bool active;
  uint8_t cmd;
};
Ctx daqCtx = { daqBuf, DAQ_MAX, 0, false, 0 };
Ctx rplCtx = { rplBuf, RPL_MAX, 0, false, 0 };

// types must be declared before the first function (Arduino IDE auto-prototypes)
enum Action { A_NONE, A_IDLE, A_LIMIT, A_FUEL, A_MODE, A_SAME, A_LEARN, A_CLEAR };

// ---------- helpers ----------
uint16_t rd16(const uint8_t *p)  { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
int16_t  rdi16(const uint8_t *p) { return (int16_t)rd16(p); }
int32_t  rdi32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

void ph(uint8_t b) {
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
}

void printHexBuf(const uint8_t *p, uint16_t n) {
  for (uint16_t i = 0; i < n; i++) {
    ph(p[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void printCode(uint16_t v) {
  Serial.print('P');
  if (v < 0x1000) Serial.print('0');
  if (v < 0x100)  Serial.print('0');
  if (v < 0x10)   Serial.print('0');
  Serial.print(v, HEX);
}

void startBus() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, CAN_XTAL);
  mcp2515.setNormalMode();
}

// ---------- receive and reassemble ----------
void ctxAppend(Ctx &c, const uint8_t *p, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    if (c.got < c.cap) c.buf[c.got] = p[i];
    c.got++;
  }
}

void ctxFinish(Ctx &c, uint8_t cmd) {
  c.active = false;
  uint16_t n = c.got;
  if (n < 3 || n > c.cap) { badLen++; return; }
  uint16_t expected = c.buf[0] | ((uint16_t)c.buf[1] << 8);
  if (expected != n) { badLen++; return; }
  uint8_t x = 0;
  for (uint16_t i = 0; i + 1 < n; i++) x ^= c.buf[i];
  if (x != c.buf[n - 1]) { badCrc++; return; }
  if (cmd == CMD_DAQ) {
    memcpy(daqMsg, c.buf, n);
    daqLen = n;
    daqStamp = millis();
    daqValid = true;
    daqCount++;
  } else {
    memcpy(rplMsg, c.buf, n);
    rplLen = n;
    rplCmd = cmd;
    rplReady = true;
  }
}

void pump() {
  while (mcp2515.readMessage(&rxf) == MCP2515::ERROR_OK) {
    if ((rxf.can_id & CAN_EFF_FLAG) || (rxf.can_id & CAN_EFF_MASK) != RX_ID) continue;
    if (rxf.can_dlc < 2) continue;
    uint8_t b0 = rxf.data[0];
    uint8_t type = b0 >> 6;
    uint8_t cmd = b0 & 0x3F;
    uint8_t n = rxf.can_dlc - 1;
    Ctx &c = (cmd == CMD_DAQ) ? daqCtx : rplCtx;
    if (type == 0) {
      c.got = 0;
      ctxAppend(c, rxf.data + 1, n);
      ctxFinish(c, cmd);
    } else if (type == 1) {
      c.got = 0;
      c.active = true;
      c.cmd = cmd;
      ctxAppend(c, rxf.data + 1, n);
    } else if (c.active && c.cmd == cmd) {
      ctxAppend(c, rxf.data + 1, n);
      if (type == 3) ctxFinish(c, cmd);
    }
  }
}

// ---------- send ----------
void sendFrame() {
  uint32_t t0 = millis();
  while (millis() - t0 < 50) {
    if (mcp2515.sendMessage(&txf) == MCP2515::ERROR_OK) return;
    pump();
  }
}

void sendMsg(uint8_t cmd, const uint8_t *pl, uint8_t plen) {
  uint8_t s[40];
  uint8_t n = 0;
  uint16_t len = plen + 3;
  s[n++] = len & 0xFF;
  s[n++] = len >> 8;
  for (uint8_t i = 0; i < plen; i++) s[n++] = pl[i];
  uint8_t x = 0;
  for (uint8_t i = 0; i < n; i++) x ^= s[i];
  s[n++] = x;

  txf.can_id = TX_ID;
  if (n <= 7) {                       // single frame
    txf.can_dlc = n + 1;
    txf.data[0] = cmd;
    for (uint8_t i = 0; i < n; i++) txf.data[1 + i] = s[i];
    sendFrame();
    return;
  }
  uint8_t pos = 0;
  bool first = true;
  while (pos < n) {                   // multi-frame (not used by the read commands)
    uint8_t chunk = (n - pos > 7) ? 7 : (n - pos);
    bool last = (pos + chunk >= n);
    txf.data[0] = (first ? 0x40 : (last ? 0xC0 : 0x80)) | cmd;
    for (uint8_t i = 0; i < chunk; i++) txf.data[1 + i] = s[pos + i];
    txf.can_dlc = chunk + 1;
    sendFrame();
    pos += chunk;
    first = false;
  }
}

bool request(uint8_t cmd, const uint8_t *pl, uint8_t plen, uint16_t timeoutMs) {
  pump();
  rplReady = false;
  sendMsg(cmd, pl, plen);
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    pump();
    if (rplReady && rplCmd == cmd) return true;
  }
  return false;
}

// ---------- commands ----------
void doConnect() {
  Serial.print(F("Connect: "));
  if (request(0x01, NULL, 0, 800) && rplLen >= 8) {
    protVer = rplMsg[2];
    uint32_t sw = (uint32_t)rplMsg[3] | ((uint32_t)rplMsg[4] << 8) |
                  ((uint32_t)rplMsg[5] << 16) | ((uint32_t)rplMsg[6] << 24);
    Serial.print(F("protocol version "));
    Serial.print(protVer);
    Serial.print(F(", software version 0x"));
    Serial.print(sw, HEX);
    Serial.print(F(" ("));
    Serial.print(sw);
    Serial.println(F(")"));
  } else {
    Serial.println(F("no reply"));
  }
}

void readIdle() {
  uint8_t pl[1] = { 0x10 };
  Serial.print(F("Idle/limit: "));
  if (request(0x05, pl, 1, 800) && rplLen >= 7) {
    Serial.print(F("target idle "));
    Serial.print(rd16(rplMsg + 2));
    Serial.print(F(" rpm, rev limit "));
    Serial.print(rd16(rplMsg + 4));
    Serial.println(F(" rpm"));
    Serial.print(F("  raw reply: "));
    printHexBuf(rplMsg, rplLen);
  } else {
    Serial.println(F("no reply"));
  }
}

void readTune() {
  uint8_t pl[1] = { 0x00 };
  Serial.print(F("Tuning block: "));
  if (request(0x06, pl, 1, 1500)) {
    Serial.print(rplLen);
    Serial.println(F(" bytes (len2 + payload + crc):"));
    printHexBuf(rplMsg, rplLen);
    if (rplLen >= 34) {
      Serial.print(F("fuel factor = "));
      Serial.println(rd16(rplMsg + 31) / 1000.0, 3);
    }
  } else {
    Serial.println(F("no reply (try again)"));
  }
}

// Fault list from the live data (same text format the app parses)
void printFaults() {
  const uint8_t *m = daqMsg;
  uint16_t o = (protVer >= 15) ? 164 : 159;
  if (daqLen > o + 2) {
    uint8_t cur = m[o], his = m[o + 1];
    Serial.print(F(" fault codes: current "));
    Serial.print(cur);
    Serial.print(F(", history "));
    Serial.println(his);
    uint16_t p = o + 2;
    for (uint8_t k = 0; k < cur && p + 1 < daqLen; k++, p += 2) {
      Serial.print(F("   current ")); printCode(rd16(m + p)); Serial.println();
    }
    for (uint8_t k = 0; k < his && p + 1 < daqLen; k++, p += 2) {
      Serial.print(F("   history ")); printCode(rd16(m + p)); Serial.println();
    }
  }
}

void printDaq() {
  if (!daqValid) { Serial.println(F("no live-data message received yet")); return; }
  const uint8_t *m = daqMsg;
  Serial.print(F("live data, age "));
  Serial.print(millis() - daqStamp);
  Serial.print(F(" ms, messages ok "));
  Serial.print(daqCount);
  Serial.print(F(", bad len "));
  Serial.print(badLen);
  Serial.print(F(", bad crc "));
  Serial.println(badCrc);
  if (daqLen < 150) { Serial.println(F("message too short")); return; }

  Serial.print(F(" rpm                  ")); Serial.println(rd16(m + 4));
  Serial.print(F(" throttle %           ")); Serial.println(rd16(m + 2) / 10.0, 1);
  Serial.print(F(" meas2 (unknown)      ")); Serial.println(rdi32(m + 34) / 1000.0, 3);
  Serial.print(F(" meas3 (ign angle?)   ")); Serial.println(rdi16(m + 6) / 10.0, 1);
  Serial.print(F(" meas4 (factor?)      ")); Serial.println(rd16(m + 22) / 10000.0, 4);
  Serial.print(F(" meas5 (factor?)      ")); Serial.println(rd16(m + 26) / 1000.0, 3);
  Serial.print(F(" O2 sensor mV         ")); Serial.println((uint32_t)rd16(m + 8) * 10);
  Serial.print(F(" battery V            ")); Serial.println(rd16(m + 20) / 1000.0, 3);
  Serial.print(F(" intake air temp C?   ")); Serial.println(rdi16(m + 18) / 10.0, 1);
  Serial.print(F(" cylinder head temp C ")); Serial.println(rdi16(m + 16) / 10.0, 1);
  Serial.print(F(" run mode            ")); Serial.println(m[130]);
  Serial.print(F(" ecu mode             ")); Serial.println(m[124]);
  Serial.print(F(" locked flag          ")); Serial.println(m[147] == 1 ? F("LOCKED") : F("unlocked"));
  Serial.print(F(" start count          ")); Serial.println(rd16(m + 122));
  Serial.print(F(" this run time        ")); Serial.println((uint32_t)rdi32(m + 87));
  Serial.print(F(" total run time       ")); Serial.println((uint32_t)rdi32(m + 91));

  printFaults();
}

void liveLine() {
  if (!daqValid || daqLen < 150) return;
  const uint8_t *m = daqMsg;
  Serial.print(millis());              Serial.print(',');
  Serial.print(rd16(m + 4));             Serial.print(',');   // rpm
  Serial.print(rd16(m + 2) / 10.0, 1);   Serial.print(',');   // meas1
  Serial.print(rdi32(m + 34) / 1000.0, 3); Serial.print(','); // meas2
  Serial.print(rdi16(m + 6) / 10.0, 1);   Serial.print(',');   // meas3
  Serial.print(rd16(m + 22) / 10000.0, 4); Serial.print(','); // meas4
  Serial.print(rd16(m + 26) / 1000.0, 3); Serial.print(',');  // meas5
  Serial.print((uint32_t)rd16(m + 8) * 10); Serial.print(','); // meas6
  Serial.print(rd16(m + 20) / 1000.0, 3); Serial.print(',');  // meas7
  Serial.print(rdi16(m + 18) / 10.0, 1);  Serial.print(',');   // meas8
  Serial.print(rdi16(m + 16) / 10.0, 1);  Serial.print(',');   // meas9
  Serial.print(m[130]);                Serial.print(',');   // run mode
  Serial.println(m[147] == 1 ? 1 : 0);                      // locked
}

void dumpDaq() {
  if (!daqValid) { Serial.println(F("no live-data message received yet")); return; }
  Serial.print(F("raw live-data message, "));
  Serial.print(daqLen);
  Serial.println(F(" bytes (index: bytes), index 0-1 = length"));
  for (uint16_t i = 0; i < daqLen; i += 16) {
    Serial.print(i);
    Serial.print(F(": "));
    for (uint16_t j = i; j < i + 16 && j < daqLen; j++) {
      ph(daqMsg[j]);
      Serial.print(' ');
    }
    Serial.println();
  }
}

// ---------- guarded writes ----------
#define REQUIRE_ENGINE_OFF 1
#define ENABLE_FUEL        1   // 1 = the fuel factor feature is built in; it stays OFF until you type fuelon
bool fuelEnabled = false;          // back to OFF every time the Arduino restarts
#define ENABLE_LEARN       0   // 1 = allow "learn" (sends Enter fast-learning mode). Off: the app calls it Unsupported for this ECU software

Action pending = A_NONE;
uint16_t newIdle = 0, newLimit = 0, newFuel = 0;
uint8_t newMode = 0;
uint16_t curIdle = 0, curLimit = 0, curFuel = 0;
uint8_t curMode = 255;

bool readValues() {
  uint8_t pl1[1] = { 0x10 };
  uint8_t pl2[1] = { 0x00 };
  if (!(request(0x05, pl1, 1, 800) && rplLen >= 7)) return false;
  curIdle = rd16(rplMsg + 2);
  curLimit = rd16(rplMsg + 4);
  if (!(request(0x06, pl2, 1, 1500) && rplLen >= 34)) return false;
  curFuel = rd16(rplMsg + 31);
  curMode = (daqValid && daqLen > 130) ? daqMsg[130] : 255;
  return true;
}

void printCurrent() {
  Serial.print(F("current: idle "));
  Serial.print(curIdle);
  Serial.print(F(" rpm, limit "));
  Serial.print(curLimit);
  Serial.print(F(" rpm, fuel factor "));
  Serial.print(curFuel / 1000.0, 3);
  Serial.print(F(", mode "));
  if (curMode == 255) Serial.println(F("unknown")); else Serial.println(curMode);
}

bool safeToWrite() {
  if (!daqValid || daqLen < 150) {
    Serial.println(F("no live data yet - wait a second and try again"));
    return false;
  }
  if (daqMsg[147] == 1) {
    Serial.println(F("ECU reports LOCKED - unlock is not implemented here, not writing"));
    return false;
  }
#if REQUIRE_ENGINE_OFF
  if (rd16(daqMsg + 4) >= 50) {
    Serial.println(F("engine appears to be running (rpm >= 50) - switch it off first"));
    return false;
  }
#endif
  return true;
}

bool ackWrite(uint8_t cmd, const uint8_t *pl, uint8_t plen, const __FlashStringHelper *what) {
  Serial.print(what);
  Serial.print(F(": "));
  bool ok = request(cmd, pl, plen, 1500);
  if (ok) {
    Serial.print(F("ack "));
    printHexBuf(rplMsg, rplLen);
  } else {
    Serial.println(F("NO ACK"));
  }
  return ok;
}

bool writeMode(uint8_t mode) {
  uint8_t pl[3] = { 0x02, 0x03, mode };
  return ackWrite(0x06, pl, 3, F("set mode"));
}

bool writeFuel(uint16_t fac) {
  uint8_t pl[4] = { 0x02, 0x0B, (uint8_t)(fac & 0xFF), (uint8_t)(fac >> 8) };
  return ackWrite(0x06, pl, 4, F("set fuel factor"));
}

bool writeRpm(uint16_t idle, uint16_t high) {
  uint8_t pl[5] = { 0x0F, (uint8_t)(idle & 0xFF), (uint8_t)(idle >> 8),
                    (uint8_t)(high & 0xFF), (uint8_t)(high >> 8) };
  return ackWrite(0x05, pl, 5, F("set idle/limit"));
}

void waitFreshDaq(uint16_t ms) {
  uint32_t t0 = millis();
  uint32_t stamp = daqStamp;
  while (millis() - t0 < ms) {
    pump();
    if (daqStamp != stamp) { delay(50); pump(); break; }
  }
}

void execute() {
  if (pending == A_LEARN) { enterLearn(); return; }
  if (pending == A_CLEAR) { doClear(); return; }
  if (!safeToWrite()) return;
  uint16_t eIdle = curIdle, eLimit = curLimit, eFuel = curFuel;
  uint8_t eMode = curMode;
  bool ok = true;
  switch (pending) {
    case A_IDLE:  eIdle = newIdle;   ok = writeRpm(newIdle, curLimit); break;
    case A_LIMIT: eLimit = newLimit; ok = writeRpm(curIdle, newLimit); break;
    case A_FUEL:  eFuel = newFuel;   ok = writeFuel(newFuel); break;
    case A_MODE:  eMode = newMode;   ok = writeMode(newMode); break;
    case A_SAME:
      if (curMode > 2) { Serial.println(F("current mode unknown - cannot run the write-back test")); return; }
      ok = writeMode(curMode);
      if (ok && fuelEnabled) ok = writeFuel(curFuel);   // fuel factor only if it has been switched on
      if (ok) ok = writeRpm(curIdle, curLimit);
      break;
    default: return;
  }
  if (!ok) {
    Serial.println(F("write failed - values may be partly changed. Press r to check, tell me what it says."));
    return;
  }
  waitFreshDaq(1500);
  if (!readValues()) { Serial.println(F("could not re-read to verify - press r")); return; }
  Serial.print(F("after write, "));
  printCurrent();
  bool same = (curIdle == eIdle) && (curLimit == eLimit) && (curFuel == eFuel) && (curMode == eMode);
  Serial.println(same ? F("VERIFIED - ECU reports the expected values") : F("MISMATCH - ECU does not report the expected values"));
}

void propose(Action a) {
  if (!safeToWrite()) return;
  if (!readValues()) { Serial.println(F("could not read the current values - try again")); return; }
  printCurrent();
  Serial.print(F("WRITE: "));
  switch (a) {
    case A_IDLE:  Serial.print(F("idle ")); Serial.print(curIdle); Serial.print(F(" -> ")); Serial.println(newIdle); break;
    case A_LIMIT: Serial.print(F("limit ")); Serial.print(curLimit); Serial.print(F(" -> ")); Serial.println(newLimit); break;
    case A_FUEL:  Serial.print(F("fuel factor ")); Serial.print(curFuel / 1000.0, 3); Serial.print(F(" -> ")); Serial.println(newFuel / 1000.0, 3); break;
    case A_MODE:  Serial.print(F("mode ")); Serial.print(curMode); Serial.print(F(" -> ")); Serial.println(newMode); break;
    default:      Serial.println(F("the same values back (no change)")); break;
  }
  pending = a;
  Serial.println(F("Write these down first. Type y + ENTER to confirm, anything else cancels."));
}

bool keyPressed() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    return true;
  }
  return false;
}

// Throttle check: shows what the ECU reads for closed and fully open throttle.
void tpsCheck() {
  if (!daqValid || daqLen < 150) { Serial.println(F("no live data yet - try again")); return; }
  float mn = 1000.0, mx = -1000.0;
  Serial.println(F("Throttle check: slowly open the throttle fully, close it fully, repeat. ENTER stops."));
  uint32_t t0 = millis(), last = 0;
  while (!keyPressed() && millis() - t0 < 90000UL) {
    pump();
    float v = rd16(daqMsg + 2) / 10.0;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    if (millis() - last >= 500) {
      last = millis();
      Serial.print(F("throttle "));
      Serial.print(v, 1);
      Serial.print(F(" %   min "));
      Serial.print(mn, 1);
      Serial.print(F("   max "));
      Serial.println(mx, 1);
    }
  }
  Serial.print(F("closed read "));
  Serial.print(mn, 1);
  Serial.print(F(" %, fully open read "));
  Serial.print(mx, 1);
  Serial.println(F(" %"));
  if (mn <= 1.0 && mx >= 98.0) Serial.println(F("Looks calibrated: about 0 % closed and about 100 % open."));
  else Serial.println(F("Not 0 / 100. If closed or open is clearly off, the manual's throttle calibration (also a factory reset) applies - see the notes."));
}

// Fast-learning (self-learning) - READ-ONLY watcher.
// From the app's SelfLrnActivity: ecuMode 2 = in fast-learning mode, ptIdx = current learning point,
// totSt bits 0-5 = points done (all six = 63 = finished, then switch the ignition off).
void printBits6(uint8_t v) {
  for (int8_t i = 5; i >= 0; i--) Serial.print((v >> i) & 1);
}

void learnMonitor() {
  Serial.println(F("Watching (read-only). ENTER stops."));
  Serial.println(F("s,rpm,throttle%,o2_mV,cyl_head_C,ecuMode,ptIdx,totSt(bit5..0)"));
  uint32_t t0 = millis(), last = 0;
  uint8_t lastPt = 255, lastSt = 255, lastMode = 255;
  while (!keyPressed() && millis() - t0 < 600000UL) {
    pump();
    if (!daqValid || daqLen < 150) continue;
    uint8_t pt = daqMsg[126], st = daqMsg[127], em = daqMsg[124];
    if (pt != lastPt || st != lastSt || em != lastMode) {
      Serial.print(F("** ecuMode "));
      Serial.print(em);
      Serial.print(F(", point "));
      Serial.print(pt);
      Serial.print(F(", points done "));
      printBits6(st);
      Serial.println();
      lastPt = pt;
      lastSt = st;
      lastMode = em;
    }
    if (millis() - last >= 500) {
      last = millis();
      Serial.print((millis() - t0) / 1000);              Serial.print(',');
      Serial.print(rd16(daqMsg + 4));                    Serial.print(',');
      Serial.print(rd16(daqMsg + 2) / 10.0, 1);          Serial.print(',');
      Serial.print((uint32_t)rd16(daqMsg + 8) * 10);     Serial.print(',');
      Serial.print(rdi16(daqMsg + 16) / 10.0, 1);        Serial.print(',');
      Serial.print(em);                                  Serial.print(',');
      Serial.print(pt);                                  Serial.print(',');
      printBits6(st);
      Serial.println();
    }
  }
}

void proposeLearn() {
#if ENABLE_LEARN
  Serial.println(F("FAST-LEARNING (self-learning). The app's notes:"));
  Serial.println(F(" - only with the bike stationary (the Chinese text says: no-load only, never on the road)"));
  Serial.println(F(" - engine fully warm (cylinder head temp in the green zone)"));
  Serial.println(F(" - this operation may stop the engine"));
  Serial.println(F(" - when it says done, switch the ignition off"));
  pending = A_LEARN;
  Serial.println(F("Type y + ENTER to enter fast-learning mode, anything else cancels."));
#else
  Serial.println(F("learn is disabled in this build."));
  Serial.println(F("The app shows 'Unsupported' for self-learning when the ECU software number is >= 400000000;"));
  Serial.println(F("yours is about 402 million, so the app's learning screen was never written for it."));
  Serial.println(F("Use lw to watch the ECU's learning state read-only."));
#endif
}

void enterLearn() {
  if (!daqValid || daqLen < 150) { Serial.println(F("no live data yet - try again")); return; }
  uint8_t pl[1] = { 0x13 };
  if (!ackWrite(0x05, pl, 1, F("enter fast-learning mode"))) return;
  learnMonitor();
}

// Clear the stored fault codes/history (the original app's "clear historical fault code").
void proposeClear() {
  if (!daqValid || daqLen < 150) { Serial.println(F("no live data yet - wait a second and try again")); return; }
  printFaults();
  Serial.println(F("CLEAR: erases the stored fault codes and history in the ECU. A fault that is still present comes back."));
  pending = A_CLEAR;
  Serial.println(F("Type y + ENTER to confirm, anything else cancels."));
}

void doClear() {
  if (!ackWrite(0x03, NULL, 0, F("clear fault codes"))) { Serial.println(F("clear failed - no acknowledgement")); return; }
  waitFreshDaq(1500);
  printFaults();
  uint16_t o = (protVer >= 15) ? 164 : 159;
  if (daqValid && daqLen > o + 2 && daqMsg[o] == 0 && daqMsg[o + 1] == 0)
    Serial.println(F("CLEARED - no fault codes remain"));
  else
    Serial.println(F("DONE - codes still reported (a fault that is still present comes back until its cause is fixed)"));
}

void help() {
  Serial.println(F("c | r | d | x | l | tps | lw | same | idle N | limit N | mode N | fuelon | fuel F | clear | h   (then ENTER)"));
}

void handleLine(char *s) {
  if (pending != A_NONE) {
    if (s[0] == 'y' || s[0] == 'Y') execute(); else Serial.println(F("cancelled"));
    pending = A_NONE;
    return;
  }
  if (!strcmp(s, "c")) doConnect();
  else if (!strcmp(s, "r")) {
    readIdle();
    readTune();
    if (daqValid && daqLen > 130) { Serial.print(F("run mode (live data): ")); Serial.println(daqMsg[130]); }
  }
  else if (!strcmp(s, "d")) printDaq();
  else if (!strcmp(s, "x")) dumpDaq();
  else if (!strcmp(s, "l")) {
    liveOn = !liveOn;
    if (liveOn) Serial.println(F("ms,rpm,throttle,m2,m3,m4,m5,o2_mV,battery,intake_air,cyl_head,runmode,locked"));
  }
  else if (!strcmp(s, "h")) help();
  else if (!strcmp(s, "same")) propose(A_SAME);
  else if (!strcmp(s, "clear")) proposeClear();
  else if (!strcmp(s, "tps")) tpsCheck();
  else if (!strcmp(s, "learn")) proposeLearn();
  else if (!strcmp(s, "lw")) learnMonitor();
  else if (!strncmp(s, "idle ", 5)) {
    int v = atoi(s + 5);
    if (v < 1200 || v > 2000) Serial.println(F("idle must be 1200-2000"));
    else { newIdle = v; propose(A_IDLE); }
  }
  else if (!strncmp(s, "limit ", 6)) {
    int v = atoi(s + 6);
    if (v < 3000 || v > 10000) Serial.println(F("limit must be 3000-10000"));
    else { newLimit = v; propose(A_LIMIT); }
  }
#if ENABLE_FUEL
  else if (!strcmp(s, "fuelon"))  { fuelEnabled = true;  Serial.println(F("fuel factor changes: ON (until restart or fueloff)")); }
  else if (!strcmp(s, "fueloff")) { fuelEnabled = false; Serial.println(F("fuel factor changes: off")); }
  else if (!strncmp(s, "fuel ", 5)) {
    if (!fuelEnabled) Serial.println(F("fuel factor changes are off - type fuelon first"));
    else {
      int v = (int)(atof(s + 5) * 1000.0 + 0.5);
      if (v < 1000 || v > 3000) Serial.println(F("fuel factor must be 1.000-3.000"));
      else { newFuel = v; propose(A_FUEL); }
    }
  }
#else
  else if (!strncmp(s, "fuel", 4)) Serial.println(F("fuel factor changes are not built into this sketch (ENABLE_FUEL 0)"));
#endif
  else if (!strncmp(s, "mode ", 5)) {
    int v = atoi(s + 5);
    if (v < 0 || v > 2) Serial.println(F("mode must be 0, 1 or 2"));
    else { newMode = v; propose(A_MODE); }
  }
  else if (!strcmp(s, "idle") || !strcmp(s, "limit") || !strcmp(s, "fuel") || !strcmp(s, "mode"))
    Serial.println(F("needs a value, e.g.  idle 1650   limit 9500   fuel 1.020   mode 1"));
  else Serial.println(F("unknown command - h for help"));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  SPI.begin();
  startBus();
  Serial.println(F("ROJO tuner v3.5. Request 0x201, reply 0x200, 500 kbps"));
  for (uint8_t t = 0; t < 4 && protVer == 0; t++) {   // first Connect after a reset is often missed
    delay(300);
    doConnect();
  }
  help();
}

void loop() {
  static uint32_t lastLive = 0, lastCheck = 0, lastConnect = 0;
  static char line[24];
  static uint8_t ll = 0;
  pump();

  uint32_t now = millis();
  if (now - lastCheck > 1000) {
    lastCheck = now;
    if (mcp2515.getErrorFlags() & 0x20) startBus();       // bus-off recovery
  }
  if ((now - daqStamp > 3000) && (now - lastConnect > 3000)) {
    lastConnect = now;                                      // stream stopped: ask again
    sendMsg(0x01, NULL, 0);
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (ll) {
        line[ll] = 0;
        Serial.println();
        handleLine(line);
        ll = 0;
      }
    } else if ((c == 8 || c == 127) && ll) {
      ll--;
    } else if (c >= 32 && ll < sizeof(line) - 1) {
      line[ll++] = c;
      Serial.write(c);
    }
  }

  if (liveOn && now - lastLive >= 1000) {
    lastLive = now;
    liveLine();
  }
}
