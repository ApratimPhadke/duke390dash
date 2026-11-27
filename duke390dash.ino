/*
  ============================================================================
   KTM Duke 390 (Gen-1, 2013-2016, Bosch ME17.8.5) — ESP32 + CJMCU-2551
   CAN logger, module/sensor discovery, signal finder and key-on health check
   Revision: read-only ABS candidate monitor, exact standard odometer if supported,
   validated OBD payloads, explicit unknown/stale states. No actuator commands.
  ============================================================================

  WHAT IT DOES
   1. Auto-detects the CAN bitrate in LISTEN-ONLY mode (never disturbs the bus
      while searching): tries 500k, 250k, 1M, 125k.
   2. Logs every frame: per-ID table (rate, count, changing bytes, automatic
      byte classification: constant / counter / flag bits / analog /
      checksum), 3000-frame ring buffer, candump-format download.
   3. MODULE DISCOVERY: every module that answers OBD-II, plus a
      TesterPresent sweep of diagnostic addresses (0x7E0-0x7E7, optional deep
      sweep 0x700-0x7F7). Each module found is asked READ-ONLY identification
      questions (UDS 22 F1xx, KWP2000 1A xx) and its fault memory
      (UDS 19 02 FF, KWP 18 02 FF00).
   4. SENSOR INVENTORY: every OBD-II PID the ECU says it supports is polled
      and shown with the sensor behind it and a plausibility status.
   5. ECU DATA BLOCKS: sweeps KWP2000 "read data by local identifier"
      (21 01..21 FE) and polls every block the ECU returns. Raw sensor values
      often live in these blocks.
   6. SIGNAL FINDER (in the browser): correlates every broadcast byte / 16-bit
      word / data-block byte with the OBD values (RPM, TPS, coolant, speed,
      MAP...) and an action detector (brake, clutch, side stand, indicators...)
      so you can name the signals the bike broadcasts. Names are saved on the
      phone.
   7. Health check every 0.5 s and a frozen "key-on self-test" report.
   8. CJMCU-2551 loopback check: our own frames are read back via TWAI
      self-reception to prove CTX -> transceiver -> bus -> CRX works.

  Everything the logger SENDS is read-only diagnostics (no writes, no
  security access, no flashing, no clearing of codes). Do it with the bike
  stationary; the address sweep refuses to run with the engine running.

  DASHBOARD:  Wi-Fi "DUKE390-CAN" / "duke390can"  ->  http://192.168.4.1

  WIRING with the CJMCU-2551 (MCP2551, 5 V part)
   Bike 12.5V -> buck 12V->5V (>=1A) -> ESP32 VIN  and  CJMCU-2551 VCC (5V)
   Bike GND   -> buck GND -> ESP32 GND -> CJMCU-2551 GND      (common ground)
   Bike yellow (CAN-H) -> CANH      Bike blue (CAN-L) -> CANL
   CJMCU-2551 CTX <- ESP32 GPIO5 (3.3 V is enough for the MCP2551 TXD input)
   CJMCU-2551 CRX -> 1k -> ESP32 GPIO4, and GPIO4 -> 2k -> GND  (5 V -> 3.3 V)
   MCP2551 pin 8 (Rs) must sit near 0 V (high-speed mode); above 0.75*VDD the
   chip is in standby. Remove any 120R across CANH/CANL on the module.

  Arduino-ESP32 core 3.x, board "ESP32 Dev Module". No external libraries.
  ============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <stdarg.h>
#include "driver/twai.h"
#include "esp_timer.h"
#include "esp_system.h"

// ============================ USER CONFIG ===================================
#define CAN_TX_PIN          GPIO_NUM_5        // -> CJMCU-2551 CTX
#define CAN_RX_PIN          GPIO_NUM_4        // <- CJMCU-2551 CRX (via divider)

#define ENABLE_ACTIVE_DIAG  1   // 0 = pure listen-only sniffer, never transmits
#define AUTO_LIVE_OBD       1   // keep polling sensors after the first scan
#define AUTO_MODULE_SCAN    1   // run the module + data-block scan once after
                                // key-on if the engine is not running

#define AP_SSID             "DUKE390-CAN"
#define AP_PASS             "duke390can"      // >= 8 characters
#define STA_SSID            ""                // optional: phone hotspot
#define STA_PASS            ""

#define VBAT_ADC_PIN        -1                // e.g. 34 with 100k/22k divider
#define VBAT_DIVIDER        (122.0f / 22.0f)

// Health thresholds (generic — check your manual)
#define COOLANT_WARN_C      105.0f
#define COOLANT_CRIT_C      115.0f
#define VBAT_OFF_WARN       12.2f
#define VBAT_OFF_CRIT       11.8f
#define VBAT_CHG_MIN        13.0f
#define VBAT_CHG_MAX        15.0f
#define VBAT_CRANK_MIN      9.6f
#define IDLE_RPM_MIN        1100.0f
#define IDLE_RPM_MAX        2200.0f
#define IDLE_RPM_STDEV_MAX  200.0f
#define FUEL_TRIM_MAX_PCT   20.0f

#define RAW_N               3000
#define MAX_IDS             128
#define MAX_MODS            8
#define MAX_BLOCKS          24
#define BLOCK_LEN           32
#define LIVE_GAP_MS         40
// ============================================================================

// ============================================================================
//   ALL TYPES ARE DEFINED HERE, BEFORE ANY FUNCTION.
//   (The Arduino IDE inserts auto-generated function prototypes above the
//    first function in the file, so every type used in a function signature
//    must already be defined by then.)
// ============================================================================
#define NIDENT 18
struct RawFrame { uint64_t tus; uint32_t id; uint8_t dlc, flags; uint8_t d[8]; };  // flags: 1 ext, 2 rtr, 4 tx
struct IdStat {
  uint32_t id;
  uint8_t ext, dlc, rxSeen;
  uint8_t d[8], mn[8], mx[8], tog[8], dlt[8];
  uint16_t chg[8], ctr[8];
  uint32_t chgMs[8];
  uint32_t n, csX, csS;
  uint64_t lastUs;
  uint32_t lastMs;
  float periodUs;
};
struct Sig { const char *key, *name, *unit, *src; double v; uint32_t ms; bool seen; };
struct PidDef { uint8_t pid, f; const char *sensor, *name, *unit; };
struct PidVal { double v; uint8_t raw[4]; uint8_t n; uint32_t ms; bool seen; };
struct IdentDef { uint8_t n; uint8_t d[4]; const char *name; };
struct IdentRes { uint8_t st, nrc, len; uint8_t d[48]; };  // st 0 not asked, 1 ok, 2 negative, 3 timeout, 4 skipped
struct Module {
  uint16_t tx, rx;
  char how[24];
  uint8_t first[8], firstLen;
  IdentRes ident[NIDENT];
  bool no22, no1A, no19, no18, no21;
  uint16_t lidPos, lidRange, lidOther;
  uint32_t lastMs;
};
struct Block { uint8_t mod, lid, len; uint8_t d[BLOCK_LEN]; uint32_t chgMs[BLOCK_LEN]; uint32_t ms; };
struct Check { uint8_t st; char detail[170]; };  // 0 n/a, 1 ok, 2 warn, 3 crit
struct Dtc { char code[6]; uint8_t pending; uint16_t from; };
struct DtcDesc { const char *c, *d; };
struct Req { uint16_t tx, rx; uint8_t n; uint8_t d[6]; uint8_t ctx, a, b; uint16_t tmo; };
struct ObdQ { uint8_t mode, pid; bool hasPid; };
struct IsoSlot { uint16_t rx; bool active; uint16_t total, got; uint8_t sn; uint8_t buf[200]; };

WebServer server(80);
SemaphoreHandle_t lockMx;
#define LOCK()   xSemaphoreTake(lockMx, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(lockMx)

// ----------------------------- raw ring buffer ------------------------------

RawFrame *ring = nullptr;
uint32_t ringSeq = 0;

// ----------------------------- per-ID statistics ----------------------------

IdStat ids[MAX_IDS];
int idN = 0;
uint32_t idOverflow = 0;

// ----------------------------- decoded signals ------------------------------
enum {
  S_RPM_BC, S_THR_BC, S_KILL_BC, S_GEAR_BC, S_CLUTCH_BC, S_WF_BC, S_WR_BC,
  S_LEAN_BC, S_FBRK_BC, S_CLT_BC, S_STAND_BC,
  S_RPM_OBD, S_CLT_OBD, S_IAT_OBD, S_MAP_OBD, S_TPS_OBD, S_LOAD_OBD, S_SPD_OBD,
  S_VMOD_OBD, S_STFT, S_LTFT, S_ADV, S_O2, S_RUNTIME, S_RPM_KWP, S_VBAT_ADC, S_ODO, S_CLR_DIST, S_MIL_DIST, S_FUEL_LEVEL, S_OIL_TEMP, S_FUEL_RATE, S_BARO,
  S_COUNT
};

Sig sigs[S_COUNT] = {
  {"rpm_bc", "Engine RPM", "rpm", "bcast 0x120 D0-1 (KTM map, unverified on 390)"},
  {"thr_bc", "Commanded throttle", "%", "bcast 0x120 D2 (unverified)"},
  {"kill_bc", "Kill switch RUN", "", "bcast 0x120 D3.4 (unverified)"},
  {"gear_bc", "Gear (0=N)", "", "bcast 0x129 D0 hi (unverified)"},
  {"clutch_bc", "Clutch pulled", "", "bcast 0x129 D0.3 (unverified)"},
  {"wf_bc", "Front wheel (raw)", "raw", "bcast 0x12B D0-1 (ABS, unverified)"},
  {"wr_bc", "Rear wheel (raw)", "raw", "bcast 0x12B D2-3 (ABS, unverified)"},
  {"lean_bc", "Lean (raw)", "raw", "bcast 0x12B D6-7 (1290 only)"},
  {"fbrk_bc", "Front brake pressure (raw)", "raw", "bcast 0x290 D0-1 (ABS, unverified)"},
  {"clt_bc", "Coolant", "C", "bcast 0x540 D6-7 /10 (unverified)"},
  {"stand_bc", "Side stand up", "", "bcast 0x540 D4.0 (unverified)"},
  {"rpm_obd", "Engine RPM", "rpm", "OBD 01 0C"},
  {"clt_obd", "Coolant", "C", "OBD 01 05"},
  {"iat_obd", "Intake air temp", "C", "OBD 01 0F"},
  {"map_obd", "Manifold pressure", "kPa", "OBD 01 0B"},
  {"tps_obd", "Throttle position", "%", "OBD 01 11"},
  {"load_obd", "Engine load", "%", "OBD 01 04"},
  {"spd_obd", "Vehicle speed", "km/h", "OBD 01 0D"},
  {"vmod_obd", "ECU supply voltage", "V", "OBD 01 42"},
  {"stft_obd", "Short-term fuel trim", "%", "OBD 01 06"},
  {"ltft_obd", "Long-term fuel trim", "%", "OBD 01 07"},
  {"adv_obd", "Ignition advance", "deg", "OBD 01 0E"},
  {"o2_obd", "O2 sensor B1S1", "V", "OBD 01 14"},
  {"run_obd", "Run time since start", "s", "OBD 01 1F"},
  {"rpm_kwp", "Engine RPM", "rpm", "KWP 21 21 (community-reported, (A*256+B)/4)"},
  {"vbat_adc", "Battery voltage", "V", "ESP32 ADC divider"},
  {"odo_obd", "ECU odometer", "km", "OBD 01 A6 (compare with instrument cluster)"},
  {"clr_dist", "Distance since DTC clear", "km", "OBD 01 31 (NOT odometer)"},
  {"mil_dist", "Distance with MIL on", "km", "OBD 01 21"},
  {"fuel_level", "Fuel level", "%", "OBD 01 2F"},
  {"oil_temp", "Oil temperature", "C", "OBD 01 5C"},
  {"fuel_rate", "Engine fuel rate", "L/h", "OBD 01 5E"},
  {"baro", "Barometric pressure", "kPa", "OBD 01 33"},
};
static inline void setSig(int i, double v) { sigs[i].v = v; sigs[i].ms = millis(); sigs[i].seen = true; }
static inline bool freshA(int i, uint32_t age) { return sigs[i].seen && (millis() - sigs[i].ms) < age; }
static inline bool fresh(int i) { return freshA(i, 3000); }

// ----------------------------- OBD-II PID table -----------------------------
enum : uint8_t { PF_A, PF_A40, PF_PCT, PF_TRIM, PF_RPM, PF_U16, PF_KPA3, PF_V1000, PF_O2, PF_ADV, PF_MAF, PF_LAMBDA, PF_CAT, PF_BITS, PF_PCT16, PF_ODO, PF_FUELRATE };

const PidDef PIDS[] = {
  {0x01, PF_BITS, "ECU self-diagnosis", "MIL / DTC count / readiness", ""},
  {0x03, PF_BITS, "O2 closed loop", "Fuel system status", ""},
  {0x04, PF_PCT, "(calculated)", "Engine load", "%"},
  {0x05, PF_A40, "Coolant temperature sensor", "Coolant temperature", "C"},
  {0x06, PF_TRIM, "O2 sensor feedback", "Short-term fuel trim B1", "%"},
  {0x07, PF_TRIM, "O2 sensor feedback", "Long-term fuel trim B1", "%"},
  {0x0A, PF_KPA3, "Fuel pressure sensor", "Fuel pressure", "kPa"},
  {0x0B, PF_A, "MAP sensor", "Intake manifold pressure", "kPa"},
  {0x0C, PF_RPM, "Crankshaft position sensor", "Engine speed", "rpm"},
  {0x0D, PF_A, "Vehicle / wheel speed sensor", "Vehicle speed", "km/h"},
  {0x0E, PF_ADV, "(calculated)", "Ignition advance", "deg"},
  {0x0F, PF_A40, "Intake air temperature sensor", "Intake air temperature", "C"},
  {0x10, PF_MAF, "MAF sensor", "Air flow rate", "g/s"},
  {0x11, PF_PCT, "Throttle position sensor", "Throttle position", "%"},
  {0x13, PF_BITS, "O2 sensors", "O2 sensors present", ""},
  {0x14, PF_O2, "O2 sensor bank1 sensor1", "O2 sensor voltage", "V"},
  {0x15, PF_O2, "O2 sensor bank1 sensor2", "O2 sensor voltage", "V"},
  {0x1C, PF_A, "(info)", "OBD standard", ""},
  {0x1F, PF_U16, "(timer)", "Run time since start", "s"},
  {0x21, PF_U16, "(info)", "Distance with MIL on", "km"},
  {0x2F, PF_PCT, "Fuel level sensor", "Fuel level", "%"},
  {0x30, PF_A, "(info)", "Warm-ups since codes cleared", ""},
  {0x31, PF_U16, "(info)", "Distance since codes cleared", "km"},
  {0x33, PF_A, "Barometric pressure sensor", "Barometric pressure", "kPa"},
  {0x3C, PF_CAT, "Catalyst temperature sensor", "Catalyst temperature", "C"},
  {0x42, PF_V1000, "ECU supply", "Control module voltage", "V"},
  {0x43, PF_PCT16, "(calculated)", "Absolute load", "%"},
  {0x44, PF_LAMBDA, "(calculated)", "Commanded lambda", ""},
  {0x45, PF_PCT, "Throttle position sensor", "Relative throttle position", "%"},
  {0x46, PF_A40, "Ambient air temperature sensor", "Ambient air temperature", "C"},
  {0x47, PF_PCT, "Throttle position sensor B", "Absolute throttle B", "%"},
  {0x49, PF_PCT, "Throttle grip sensor D", "Accelerator position D", "%"},
  {0x4A, PF_PCT, "Throttle grip sensor E", "Accelerator position E", "%"},
  {0x4C, PF_PCT, "Throttle actuator", "Commanded throttle actuator", "%"},
  {0x4D, PF_U16, "(timer)", "Time run with MIL on", "min"},
  {0x4E, PF_U16, "(timer)", "Time since codes cleared", "min"},
  {0x5A, PF_PCT, "Throttle grip sensor", "Relative accelerator position", "%"},
  {0x5C, PF_A40, "Oil temperature sensor", "Engine oil temperature", "C"},
  {0x5E, PF_FUELRATE, "(ECU calculated/reported)", "Engine fuel rate", "L/h"},
  {0xA6, PF_ODO, "(distance counter)", "Odometer (not DTC-clear distance)", "km"},
};
const int NPIDS = sizeof(PIDS) / sizeof(PIDS[0]);

PidVal pv[NPIDS];
int pidIndex(uint8_t pid) { for (int i = 0; i < NPIDS; i++) if (PIDS[i].pid == pid) return i; return -1; }

// ----------------------------- modules & data blocks ------------------------

const IdentDef IDENTS[] = {
  {3, {0x22, 0xF1, 0x90}, "VIN (UDS F190)"},
  {3, {0x22, 0xF1, 0x87}, "Spare part number (UDS F187)"},
  {3, {0x22, 0xF1, 0x8A}, "System supplier (UDS F18A)"},
  {3, {0x22, 0xF1, 0x8C}, "ECU serial number (UDS F18C)"},
  {3, {0x22, 0xF1, 0x91}, "ECU hardware number (UDS F191)"},
  {3, {0x22, 0xF1, 0x92}, "Supplier HW number (UDS F192)"},
  {3, {0x22, 0xF1, 0x94}, "Supplier SW number (UDS F194)"},
  {3, {0x22, 0xF1, 0x95}, "Supplier SW version (UDS F195)"},
  {3, {0x22, 0xF1, 0x97}, "System name (UDS F197)"},
  {2, {0x1A, 0x80}, "ECU identification table (KWP 1A 80)"},
  {2, {0x1A, 0x87}, "Spare part number (KWP 1A 87)"},
  {2, {0x1A, 0x90}, "VIN (KWP 1A 90)"},
  {2, {0x1A, 0x91}, "ECU hardware number (KWP 1A 91)"},
  {2, {0x1A, 0x92}, "Supplier HW number (KWP 1A 92)"},
  {2, {0x1A, 0x94}, "Supplier SW number (KWP 1A 94)"},
  {2, {0x1A, 0x97}, "System name (KWP 1A 97)"},
  {3, {0x19, 0x02, 0xFF}, "Fault memory (UDS 19 02 FF)"},
  {4, {0x18, 0x02, 0xFF, 0x00}, "Fault memory (KWP 18 02 FF00)"},
};
static_assert(sizeof(IDENTS) / sizeof(IDENTS[0]) == NIDENT, "update NIDENT");


Module mods[MAX_MODS];
int modN = 0;

Block blocks[MAX_BLOCKS];
int blockN = 0;
char modMsg[96] = "Not scanned yet.";
char modBusy[64] = "";

// ----------------------------- health checks --------------------------------
enum { C_BUS, C_XCVR, C_ECU, C_MIL, C_SENS, C_BATT, C_CHG, C_CLT, C_TPS, C_MAP, C_IAT, C_IDLE, C_TRIM, C_NODES, C_COUNT };
const char *CHK_NAMES[C_COUNT] = {
  "CAN bus link", "CJMCU-2551 TX/RX loop", "ECU communication", "Check-engine / fault codes", "Sensor plausibility",
  "Battery (engine off)", "Charging system", "Coolant temperature", "Throttle position sensor", "MAP sensor",
  "Intake air temp sensor", "Idle quality", "Fuel trims (mixture)", "Bus nodes alive"
};

Check checks[C_COUNT], bootReport[C_COUNT];
bool bootReportDone = false;
uint32_t bootReportMs = 0;
void setChk(int i, uint8_t st, const char *fmt, ...) {
  checks[i].st = st;
  va_list ap; va_start(ap, fmt);
  vsnprintf(checks[i].detail, sizeof(checks[i].detail), fmt, ap);
  va_end(ap);
}

// ----------------------------- CAN / OBD published state --------------------
struct CanPub {
  const char *phase = "SEARCHING";
  uint32_t rate = 0;
  bool normal = false;
  float fps = 0, load = 0;
  uint32_t total = 0, tec = 0, rec = 0, missed = 0, overrun = 0, busErr = 0, txFail = 0, arbLost = 0;
  uint32_t searchErr = 0;            // bus errors seen during bitrate search
  const char *state = "STOPPED";
  uint32_t searchCycles = 0;
  uint32_t loopTried = 0, loopOk = 0, rxValid = 0;
} canPub;


struct ObdPub {
  bool tried = false, avail = false, scanning = false, live = false, mil = false, supKnown = false;
  bool kwpRpm = true;
  int dtcReported = -1;
  uint32_t sup[6] = {};
  uint8_t supValid = 0;
  uint32_t lastEngineMs = 0;
  bool engineSeen = false, milSeen = false, storedSeen = false, pendingSeen = false;
  uint32_t milMs = 0, storedMs = 0, pendingMs = 0;
  char vin[18] = "";
  uint16_t ecuId = 0, responders = 0;
  uint32_t lastScanMs = 0, reqs = 0, resps = 0, timeouts = 0, negs = 0;
} obd;
Dtc dtcs[24];
int dtcN = 0;

volatile bool cmdScan = false, cmdLiveOn = false, cmdLiveOff = false, cmdClearIds = false, cmdLidScan = false, cmdAbsRead = false;
volatile uint8_t cmdModScan = 0;  // 1 normal, 2 deep

float rpmHist[40];
int rpmHistN = 0, rpmHistI = 0;
uint32_t runningSinceMs = 0;
float vHist[50];
int vHistI = 0;
float crankMinV = 0;

// ============================================================================
//                               helpers
// ============================================================================
String J;
void jf(const char *fmt, ...) {
  char b[512];
  va_list ap; va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  J += b;
}
void jstr(const char *s) {
  J += '"';
  for (; *s; s++) {
    if (*s == '"' || *s == '\\') { J += '\\'; J += *s; }
    else if ((uint8_t)*s < 0x20) J += ' ';
    else J += *s;
  }
  J += '"';
}
void jhex(const uint8_t *d, int n) { J += '"'; for (int i = 0; i < n; i++) jf("%02X", d[i]); J += '"'; }

const char *idLabel(uint32_t id, bool ext) {
  if (ext) return "";
  switch (id) {
    case 0x120: return "KTM: RPM / throttle / kill sw (690/1290 map)";
    case 0x128: return "KTM 1390: gear / clutch";
    case 0x129: return "KTM 690: gear / clutch";
    case 0x12A: return "KTM: throttle map / state";
    case 0x12B: return "KTM: wheel speeds / lean (ABS)";
    case 0x290: return "KTM: front brake pressure (ABS)";
    case 0x450: return "KTM: traction-control button";
    case 0x540: return "KTM: RPM / gear / side stand / coolant";
    case 0x7DF: return "OBD functional request (this logger)";
    case 0x7E8: return "Diagnostic response (usually engine ECU)";
  }
  for (int i = 0; i < modN; i++) {
    if (mods[i].rx == id) return "Diagnostic response of a discovered module";
    if (mods[i].tx == id) return "Diagnostic request to a discovered module";
  }
  if (id >= 0x7E0 && id <= 0x7E7) return "Diagnostic physical request";
  if (id >= 0x7E9 && id <= 0x7EF) return "Diagnostic response (other module, e.g. ABS)";
  return "";
}


const DtcDesc DTC_DB[] = {
  {"P0030", "O2 sensor heater control circuit (B1S1)"},
  {"P0105", "MAP/baro circuit malfunction"}, {"P0106", "MAP/baro circuit range/performance"},
  {"P0107", "MAP circuit low input"}, {"P0108", "MAP circuit high input"},
  {"P0110", "Intake air temp circuit malfunction"}, {"P0112", "Intake air temp circuit low"},
  {"P0113", "Intake air temp circuit high"}, {"P0115", "Coolant temp circuit malfunction"},
  {"P0116", "Coolant temp circuit range/performance"}, {"P0117", "Coolant temp circuit low"},
  {"P0118", "Coolant temp circuit high"}, {"P0120", "Throttle position sensor A circuit"},
  {"P0121", "TPS A range/performance"}, {"P0122", "TPS A circuit low"}, {"P0123", "TPS A circuit high"},
  {"P0130", "O2 sensor circuit (B1S1)"}, {"P0131", "O2 sensor low voltage (B1S1)"},
  {"P0132", "O2 sensor high voltage (B1S1)"}, {"P0133", "O2 sensor slow response (B1S1)"},
  {"P0134", "O2 sensor no activity (B1S1)"}, {"P0135", "O2 sensor heater circuit (B1S1)"},
  {"P0171", "System too lean (bank 1)"}, {"P0172", "System too rich (bank 1)"},
  {"P0201", "Injector circuit, cylinder 1"}, {"P0217", "Engine coolant over-temperature"},
  {"P0219", "Engine over-speed condition"}, {"P0230", "Fuel pump primary circuit"},
  {"P0261", "Cylinder 1 injector circuit low"}, {"P0262", "Cylinder 1 injector circuit high"},
  {"P0300", "Random/multiple misfire"}, {"P0301", "Cylinder 1 misfire"},
  {"P0335", "Crankshaft position sensor A circuit"}, {"P0336", "Crank sensor range/performance"},
  {"P0351", "Ignition coil A primary/secondary circuit"}, {"P0480", "Cooling fan 1 control circuit"},
  {"P0500", "Vehicle speed sensor A"}, {"P0505", "Idle air control system"},
  {"P0506", "Idle RPM lower than expected"}, {"P0507", "Idle RPM higher than expected"},
  {"P0560", "System voltage malfunction"}, {"P0562", "System voltage low"},
  {"P0563", "System voltage high"}, {"P0601", "ECU memory checksum error"},
  {"P0627", "Fuel pump control circuit open"}, {"P0650", "MIL control circuit"},
  {"U0001", "High-speed CAN communication bus"}, {"U0100", "Lost communication with ECM/PCM"},
  {"U0121", "Lost communication with ABS module"}, {"U0155", "Lost communication with instrument cluster"},
};
const char *dtcDesc(const char *c) {
  for (auto &e : DTC_DB) if (!strcmp(e.c, c)) return e.d;
  if (c[1] == '1' || c[1] == '3') return "Manufacturer-specific code (look up in KTM service data)";
  return "Generic code (not in local table)";
}

// ============================================================================
//                             TWAI driver control
// ============================================================================
bool canStart(uint32_t rate, twai_mode_t mode) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, mode);
  g.rx_queue_len = 256;
  g.tx_queue_len = 8;
  g.alerts_enabled = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                     TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED;
  twai_timing_config_t t;
  switch (rate) {
    case 1000000: { twai_timing_config_t x = TWAI_TIMING_CONFIG_1MBITS(); t = x; } break;
    case 250000:  { twai_timing_config_t x = TWAI_TIMING_CONFIG_250KBITS(); t = x; } break;
    case 125000:  { twai_timing_config_t x = TWAI_TIMING_CONFIG_125KBITS(); t = x; } break;
    default:      { twai_timing_config_t x = TWAI_TIMING_CONFIG_500KBITS(); t = x; } break;
  }
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  return true;
}
void canStopDrv() { twai_stop(); twai_driver_uninstall(); }

uint64_t bitsThisSec = 0;
uint32_t framesThisSec = 0;

void logFrame(const twai_message_t &m, bool tx) {
  uint64_t tus = esp_timer_get_time();
  uint32_t now = millis();
  uint8_t dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
  LOCK();
  RawFrame &r = ring[ringSeq % RAW_N];
  r.tus = tus; r.id = m.identifier; r.dlc = dlc;
  r.flags = (m.extd ? 1 : 0) | (m.rtr ? 2 : 0) | (tx ? 4 : 0);
  memcpy(r.d, m.data, 8);
  ringSeq++;
  canPub.total++;
  if (!tx) canPub.rxValid++;
  framesThisSec++;
  bitsThisSec += (uint64_t)((m.extd ? 67 : 47) + 8 * dlc) * 112 / 100;

  int k = -1;
  for (int i = 0; i < idN; i++) if (ids[i].id == m.identifier && ids[i].ext == m.extd) { k = i; break; }
  if (k < 0) {
    if (idN < MAX_IDS) {
      k = idN++;
      memset(&ids[k], 0, sizeof(IdStat));
      ids[k].id = m.identifier; ids[k].ext = m.extd;
    } else idOverflow++;
  }
  if (k >= 0) {
    IdStat &s = ids[k];
    if (!tx) s.rxSeen = 1;
    for (int b = 0; b < 8; b++) {
      uint8_t v = m.data[b];
      if (!s.n) { s.mn[b] = s.mx[b] = v; continue; }
      if (v < s.mn[b]) s.mn[b] = v;
      if (v > s.mx[b]) s.mx[b] = v;
      if (v != s.d[b]) {
        uint8_t dl = v - s.d[b];
        if (dl == s.dlt[b]) s.ctr[b]++;
        s.dlt[b] = dl;
        s.tog[b] |= v ^ s.d[b];
        if (s.chg[b] < 65535) s.chg[b]++;
        s.chgMs[b] = now;
      }
    }
    if (dlc >= 2) {
      uint8_t x = 0, sum = 0;
      for (int b = 0; b < dlc - 1; b++) { x ^= m.data[b]; sum += m.data[b]; }
      if (x == m.data[dlc - 1]) s.csX++;
      if (sum == m.data[dlc - 1]) s.csS++;
    }
    if (s.n) {
      float dt = (float)(tus - s.lastUs);
      s.periodUs = s.periodUs == 0 ? dt : s.periodUs * 0.9f + dt * 0.1f;
    }
    memcpy(s.d, m.data, 8);
    s.dlc = dlc; s.n++; s.lastUs = tus; s.lastMs = now;
  }
  UNLOCK();
}

// self-reception bookkeeping (CJMCU-2551 loop test)
uint32_t lastTxId = 0, lastTxMs = 0;
uint8_t lastTxData[8];
bool lastTxSelf = false;
uint32_t sendCount = 0;

bool canSend(uint32_t id, const uint8_t *d, uint8_t len) {
  twai_message_t m = {};
  m.identifier = id; m.data_length_code = 8; m.ss = 1;
  memset(m.data, 0x55, 8);
  memcpy(m.data, d, len);
  bool selfTest = (sendCount < 3) || (sendCount % 200 == 0);   // a few loop tests, then occasionally
  m.self = selfTest ? 1 : 0;
  sendCount++;
  if (twai_transmit(&m, 0) != ESP_OK) return false;
  lastTxId = id; lastTxMs = millis(); memcpy(lastTxData, m.data, 8); lastTxSelf = selfTest;
  if (selfTest) { LOCK(); canPub.loopTried++; UNLOCK(); }
  logFrame(m, true);
  return true;
}

// ============================================================================
//                    KTM broadcast decoding (unverified on 390)
// ============================================================================
uint16_t be16(const uint8_t *p) { return (uint16_t)p[0] << 8 | p[1]; }
int16_t signed12(uint16_t v) { v &= 0xFFF; return (v & 0x800) ? (int16_t)(v | 0xF000) : (int16_t)v; }

void decodeBroadcast(const twai_message_t &m) {
  if (m.extd || m.data_length_code < 8) return;
  const uint8_t *d = m.data;
  LOCK();
  switch (m.identifier) {
    case 0x120: {
      uint16_t rpm = be16(d);
      if (rpm < 14000) setSig(S_RPM_BC, rpm);
      setSig(S_THR_BC, d[2] * 100.0f / 255.0f);
      setSig(S_KILL_BC, (d[3] >> 4) & 1);
    } break;
    case 0x129:
      if ((d[0] >> 4) <= 6) setSig(S_GEAR_BC, d[0] >> 4);
      setSig(S_CLUTCH_BC, (d[0] >> 3) & 1);
      break;
    case 0x12B:
      setSig(S_WF_BC, be16(d));
      setSig(S_WR_BC, be16(d + 2));
      setSig(S_LEAN_BC, signed12(((uint16_t)(d[6] & 0x0F) << 8) | d[7]));
      break;
    case 0x290: setSig(S_FBRK_BC, be16(d)); break;
    case 0x540: {
      float c = be16(d + 6) / 10.0f;
      if (c > -40 && c < 150) setSig(S_CLT_BC, c);
      setSig(S_STAND_BC, d[4] & 1);
    } break;
  }
  UNLOCK();
}

// ============================================================================
//                   diagnostics engine (CAN-task-only state)
// ============================================================================
enum : uint8_t { X_OBD, X_SWEEP, X_IDENT, X_LIDSCAN, X_LIDPOLL, X_ABS };

Req cur;
bool inFlight = false;
uint32_t curDeadline = 0, lastDoneMs = 0, lastMaintMs = 0;
enum { D_IDLE, D_SCAN, D_LIVE, D_SWEEP, D_IDENT, D_LIDSCAN, D_ABS } dmode = D_IDLE;
bool liveWanted = AUTO_LIVE_OBD;
bool autoModScanDone = !AUTO_MODULE_SCAN;


ObdQ scanQ[16];
int scanN = 0, scanI = 0;
int liveI = 0, liveToggle = 0, liveBlockI = 0;

// sweep state
uint16_t sweepList[260];
int sweepN = 0, sweepI = 0;
uint32_t snapIds[MAX_IDS];
int snapN = 0;
// ident / lid scan state
int jobMod = 0, jobIdx = 0;


IsoSlot iso[6];

// --------- read-only ABS fault reader (targets the 0x7E1 -> 0x7E9 module) ----
// Each step is a pure READ: identification and stored/pending fault memory.
// slot: 0 name, 1 UDS 19 02 FF (fault), 2 KWP 18 02 FF00 (fault)
struct AbsReadQ { uint8_t n; uint8_t d[4]; uint8_t slot; };
const AbsReadQ ABSQ[] = {
  {2, {0x1A, 0x97}, 0},            // KWP system name
  {3, {0x22, 0xF1, 0x97}, 0},      // UDS system name (fallback)
  {3, {0x19, 0x02, 0xFF}, 1},      // UDS read DTC by status mask (all)
  {4, {0x18, 0x02, 0xFF, 0x00}, 2},// KWP read DTC by status
};
const int NABSQ = sizeof(ABSQ) / sizeof(ABSQ[0]);
struct AbsPub {
  uint16_t tx = 0x7E1, rx = 0x7E9;
  bool tried = false;
  uint32_t ms = 0;
  IdentRes name, uds19, kwp18;     // st: 0 not asked, 1 ok, 2 negative, 3 timeout
} absP;
int absStep = 0;

bool rpmRunning() {  // used for the sweep interlock (called with LOCK held)
  if (fresh(S_RPM_OBD) && sigs[S_RPM_OBD].v > 300) return true;
  if (fresh(S_RPM_KWP) && sigs[S_RPM_KWP].v > 300) return true;
  if (freshA(S_RPM_BC, 1000) && sigs[S_RPM_BC].v > 300) return true;
  return false;
}

bool pidSupported(uint8_t pid) {
  if (!obd.supKnown) return true;
  if (pid == 0) return true;
  if (pid > 0xC0) return false;
  int w = (pid - 1) / 32, b = (pid - 1) % 32;
  return (obd.supValid & (1U << w)) && ((obd.sup[w] >> (31 - b)) & 1);
}

int findMod(uint16_t rx) { for (int i = 0; i < modN; i++) if (mods[i].rx == rx) return i; return -1; }
int addModule(uint16_t tx, uint16_t rx, const char *how, const uint8_t *first, uint8_t flen) {  // LOCK held
  int i = findMod(rx);
  if (i >= 0) return i;
  if (modN >= MAX_MODS) return -1;
  Module &m = mods[modN];
  memset(&m, 0, sizeof(Module));
  m.tx = tx; m.rx = rx;
  strncpy(m.how, how, sizeof(m.how) - 1);
  if (first) { m.firstLen = flen > 8 ? 8 : flen; memcpy(m.first, first, m.firstLen); }
  m.lastMs = millis();
  return modN++;
}

bool isDiagRx(uint32_t id) {
  if (id >= 0x7E8 && id <= 0x7EF) return true;
  for (int i = 0; i < modN; i++) if (mods[i].rx == id) return true;
  return false;
}
uint16_t fcTxFor(uint16_t rx) {
  for (int i = 0; i < modN; i++) if (mods[i].rx == rx) return mods[i].tx;
  if (rx >= 0x7E8 && rx <= 0x7EF) return rx - 8;
  return 0;
}

void sendReq(const Req &r) {
  // Explicit allowlist: no actuator tests, routine control, security access, writes or code clearing.
  switch (r.d[0]) { case 0x01: case 0x03: case 0x07: case 0x09: case 0x19: case 0x18:
    case 0x1A: case 0x21: case 0x22: case 0x3E: break; default: lastDoneMs = millis(); return; }
  if (!r.n || r.n > sizeof(r.d)) return;
  uint8_t b[7];
  b[0] = r.n;
  memcpy(b + 1, r.d, r.n);
  cur = r;
  inFlight = true;
  curDeadline = millis() + r.tmo;
  LOCK(); obd.reqs++; UNLOCK();
  if (!canSend(r.tx, b, r.n + 1)) { inFlight = false; lastDoneMs = millis(); }
}
void reqDone() { inFlight = false; lastDoneMs = millis(); }

Req obdReq(uint8_t mode, uint8_t pid, bool hasPid) {
  Req r = {};
  r.tx = 0x7DF; r.rx = 0x7E8; r.ctx = X_OBD; r.tmo = 150; // engine responder observed on this bike; keep functional TX for existing discovery
  r.d[0] = mode; r.n = 1;
  if (hasPid) { r.d[1] = pid; r.n = 2; }
  return r;
}

void pushRpm(float r) { rpmHist[rpmHistI] = r; rpmHistI = (rpmHistI + 1) % 40; if (rpmHistN < 40) rpmHistN++; }

int pidBytes(uint8_t pid, uint8_t f) {
  if (pid == 0x01 || f == PF_ODO) return 4;
  if (pid == 0x03 || f == PF_O2 || f == PF_RPM || f == PF_U16 || f == PF_V1000 ||
      f == PF_MAF || f == PF_LAMBDA || f == PF_CAT || f == PF_PCT16 || f == PF_FUELRATE) return 2;
  return 1;
}
void parsePid(uint8_t pid, const uint8_t *v, int n) {  // LOCK held
  if (pid <= 0xA0 && (pid & 0x1F) == 0) {
    if (n == 4) { obd.sup[pid / 32] = (uint32_t)v[0] << 24 | (uint32_t)v[1] << 16 | (uint32_t)v[2] << 8 | v[3]; obd.supValid |= 1U << (pid / 32); obd.supKnown = (obd.supValid & 1) != 0; }
    return;
  }
  int pi = pidIndex(pid);
  if (pi < 0 || n != pidBytes(pid, PIDS[pi].f)) return;
  double A = v[0], B = n > 1 ? v[1] : 0, U = A * 256 + B;
  if (pi >= 0) {
    double x = 0;
    switch (PIDS[pi].f) {
      case PF_A: x = A; break;
      case PF_A40: x = A - 40; break;
      case PF_PCT: x = A * 100 / 255; break;
      case PF_TRIM: x = (A - 128) * 100 / 128; break;
      case PF_RPM: x = U / 4; break;
      case PF_U16: x = U; break;
      case PF_KPA3: x = A * 3; break;
      case PF_V1000: x = U / 1000; break;
      case PF_O2: x = A / 200; break;
      case PF_ADV: x = A / 2 - 64; break;
      case PF_MAF: x = U / 100; break;
      case PF_LAMBDA: x = U * 2 / 65536; break;
      case PF_CAT: x = U / 10 - 40; break;
      case PF_PCT16: x = U * 100 / 255; break;
      case PF_BITS: x = A; break;
      case PF_ODO: x = ((uint32_t(v[0]) << 24) | (uint32_t(v[1]) << 16) | (uint32_t(v[2]) << 8) | v[3]) / 10.0; break;
      case PF_FUELRATE: x = U / 20.0; break;
    }
    PidVal &p = pv[pi];
    p.v = x; p.n = n > 4 ? 4 : n; memcpy(p.raw, v, p.n); p.ms = millis(); p.seen = true;
  }
  switch (pid) {
    case 0x01: obd.mil = v[0] & 0x80; obd.dtcReported = v[0] & 0x7F; obd.milSeen = true; obd.milMs = millis(); break;
    case 0x04: setSig(S_LOAD_OBD, A * 100 / 255); break;
    case 0x05: setSig(S_CLT_OBD, A - 40); break;
    case 0x06: setSig(S_STFT, (A - 128) * 100 / 128); break;
    case 0x07: setSig(S_LTFT, (A - 128) * 100 / 128); break;
    case 0x0B: setSig(S_MAP_OBD, A); break;
    case 0x0C: setSig(S_RPM_OBD, U / 4); pushRpm(U / 4); break;
    case 0x0D: setSig(S_SPD_OBD, A); break;
    case 0x0E: setSig(S_ADV, A / 2 - 64); break;
    case 0x0F: setSig(S_IAT_OBD, A - 40); break;
    case 0x11: setSig(S_TPS_OBD, A * 100 / 255); break;
    case 0x14: setSig(S_O2, A / 200); break;
    case 0x1F: setSig(S_RUNTIME, U); break;
    case 0x42: setSig(S_VMOD_OBD, U / 1000); break;
    case 0x21: setSig(S_MIL_DIST, U); break;
    case 0x31: setSig(S_CLR_DIST, U); break;
    case 0x2F: setSig(S_FUEL_LEVEL, A * 100 / 255); break;
    case 0x33: setSig(S_BARO, A); break;
    case 0x5C: setSig(S_OIL_TEMP, A - 40); break;
    case 0x5E: setSig(S_FUEL_RATE, U / 20); break;
    case 0xA6: setSig(S_ODO, pv[pi].v); break;
  }
}

// 0 n/a, 1 ok, 2 suspect
uint8_t pidStatus(int i, const char **why) {
  const PidVal &p = pv[i];
  *why = "";
  if (!p.seen) return 0;
  if (millis() - p.ms > 10000) { *why = "stale"; return 0; }
  uint8_t A = p.raw[0];
  switch (PIDS[i].f) {
    case PF_A40: if (A == 0 || A == 255) { *why = "reads -40/215 C: sensor open or shorted"; return 2; } break;
    case PF_V1000: if (p.v < 8 || p.v > 16) { *why = "voltage implausible"; return 2; } break;
  }
  // Absolute TPS spans 0..100 by definition; closed-throttle offset is model-specific.
  if (PIDS[i].pid == 0x0B && (A == 0 || A == 255)) { *why = "MAP at rail: sensor/wiring fault"; return 2; }
  return 1;
}

void parseDtcs(uint16_t from, bool pending, const uint8_t *p, int len) {  // LOCK held
  int start;
  if (len >= 2 && (len - 2) == p[1] * 2) start = 2;
  else if ((len - 1) % 2 == 0) start = 1;
  else start = 2;
  int w = 0;
  for (int i = 0; i < dtcN; i++) if (!(dtcs[i].from == from && dtcs[i].pending == pending)) dtcs[w++] = dtcs[i];
  dtcN = w;
  for (int i = start; i + 1 < len && dtcN < 24; i += 2) {
    uint8_t a = p[i], b = p[i + 1];
    if (!a && !b) continue;
    Dtc &d = dtcs[dtcN++];
    const char L[] = "PCBU";
    snprintf(d.code, sizeof(d.code), "%c%d%X%X%X", L[a >> 6], (a >> 4) & 3, a & 0xF, b >> 4, b & 0xF);
    d.pending = pending; d.from = from;
  }
}

bool matchesCur(uint16_t from, const uint8_t *p, int len) {
  if (cur.rx) { if (from != cur.rx) return false; }
  else {
    if (from < 0x7E8 || from > 0x7EF) return false;
    if (obd.ecuId && from != obd.ecuId) return false;
  }
  if (len < 1 || (p[0] == 0x7F && len < 3)) return false;
  bool neg = p[0] == 0x7F;
  uint8_t svc = neg ? p[1] : (uint8_t)(p[0] - 0x40);
  if (svc != cur.d[0]) return false;
  if (!neg) {
    uint8_t s = cur.d[0];
    if ((s == 0x01 || s == 0x09 || s == 0x21 || s == 0x1A) && cur.n >= 2 && (len < 2 || p[1] != cur.d[1])) return false;
    if (s == 0x22 && (len < 3 || p[1] != cur.d[1] || p[2] != cur.d[2])) return false;
  }
  return true;
}

void markSvcNo(Module &m, uint8_t svc) {
  if (svc == 0x22) m.no22 = true;
  if (svc == 0x1A) m.no1A = true;
  if (svc == 0x19) m.no19 = true;
  if (svc == 0x18) m.no18 = true;
  if (svc == 0x21) m.no21 = true;
}
bool svcNo(const Module &m, uint8_t svc) {
  return (svc == 0x22 && m.no22) || (svc == 0x1A && m.no1A) || (svc == 0x19 && m.no19) || (svc == 0x18 && m.no18) || (svc == 0x21 && m.no21);
}
bool firstOfSvc(int idx) { return idx == 0 || IDENTS[idx - 1].d[0] != IDENTS[idx].d[0]; }

void onDiagMessage(uint16_t from, const uint8_t *p, int len) {
  if (len < 1) return;
  LOCK();
  obd.resps++;
  if (from >= 0x7E8 && from <= 0x7EF) obd.responders |= 1 << (from - 0x7E8);
  int mi = findMod(from);
  if (mi >= 0) mods[mi].lastMs = millis();
  bool neg = p[0] == 0x7F && len >= 3;
  if (neg && p[2] == 0x78) { if (inFlight && matchesCur(from, p, len)) curDeadline = millis() + 2500; UNLOCK(); return; }
  bool isCur = inFlight && matchesCur(from, p, len);
  if (isCur) {
    switch (cur.ctx) {
      case X_OBD:
        if (!neg) { obd.avail = true; obd.engineSeen = true; obd.lastEngineMs = millis(); }
        if (neg) { obd.negs++; if (p[1] == 0x21) obd.kwpRpm = false; break; }
        if (!obd.ecuId && p[0] == 0x41) obd.ecuId = from;
        if (p[0] == 0x41 && len >= 2) parsePid(p[1], p + 2, len - 2);
        else if ((p[0] == 0x43 || p[0] == 0x47) && len >= 2 &&
                 (len - 2) == p[1] * 2) {
          parseDtcs(from, p[0] == 0x47, p, len);
          if (p[0] == 0x43) { obd.storedSeen = true; obd.storedMs = millis(); }
          else { obd.pendingSeen = true; obd.pendingMs = millis(); }
        }
        else if (p[0] == 0x49 && len >= 3 && p[1] == 0x02) {
          int k = 0;
          for (int o = 3; o < len && k < 17; o++) if (p[o] >= 0x20 && p[o] < 0x7F) obd.vin[k++] = p[o];
          obd.vin[k] = 0;
        } else if (p[0] == 0x61 && len == 4) {
          float r = (p[2] * 256.0f + p[3]) / 4.0f;
          if (r < 14000) { setSig(S_RPM_KWP, r); if (!pidSupported(0x0C)) pushRpm(r); }
        }
        break;
      case X_IDENT: {
        IdentRes &ir = mods[cur.a].ident[cur.b];
        ir.st = neg ? 2 : 1;
        ir.nrc = neg ? p[2] : 0;
        ir.len = len > (int)sizeof(ir.d) ? sizeof(ir.d) : len;
        memcpy(ir.d, p, ir.len);
        if (neg && (p[2] == 0x11 || p[2] == 0x7F) && firstOfSvc(cur.b)) markSvcNo(mods[cur.a], cur.d[0]);
      } break;
      case X_LIDSCAN: {
        Module &m = mods[cur.a];
        if (!neg && len >= 2) {
          m.lidPos++;
          if (blockN < MAX_BLOCKS) {
            Block &b = blocks[blockN++];
            memset(&b, 0, sizeof(Block));
            b.mod = cur.a; b.lid = cur.d[1];
            b.len = (len - 2) > BLOCK_LEN ? BLOCK_LEN : len - 2;
            memcpy(b.d, p + 2, b.len);
            b.ms = millis();
          }
        } else if (neg) {
          if (p[2] == 0x11) m.no21 = true;
          else if (p[2] == 0x31 || p[2] == 0x12) m.lidRange++;
          else m.lidOther++;
        }
      } break;
      case X_LIDPOLL: {
        if (neg || len < 2 || cur.a >= blockN) break;
        Block &b = blocks[cur.a];
        uint8_t n = (len - 2) > BLOCK_LEN ? BLOCK_LEN : len - 2;
        uint32_t now = millis();
        for (int i = 0; i < n; i++) if (b.d[i] != p[2 + i]) b.chgMs[i] = now;
        memcpy(b.d, p + 2, n); b.len = n; b.ms = now;
        if (mods[b.mod].rx == obd.ecuId && b.lid == 0x21 && n >= 2) {
          float r = (p[2] * 256.0f + p[3]) / 4.0f;
          if (r < 14000) setSig(S_RPM_KWP, r);
        }
      } break;
      case X_ABS: {
        IdentRes &ir = cur.b == 0 ? absP.name : cur.b == 1 ? absP.uds19 : absP.kwp18;
        ir.st = neg ? 2 : 1;
        ir.nrc = neg ? p[2] : 0;
        ir.len = len > (int)sizeof(ir.d) ? sizeof(ir.d) : len;
        memcpy(ir.d, p, ir.len);
        absP.ms = millis();
      } break;
    }
    reqDone();
  } else if (!neg && len >= 2 && (len - 2) == p[1] * 2 && (p[0] == 0x43 || p[0] == 0x47)) {
    parseDtcs(from, p[0] == 0x47, p, len);   // other modules answering the functional DTC request
  }
  UNLOCK();
}

void onTimeout() {
  LOCK();
  obd.timeouts++;
  if (cur.ctx == X_IDENT) {
    mods[cur.a].ident[cur.b].st = 3;
    if (firstOfSvc(cur.b)) markSvcNo(mods[cur.a], cur.d[0]);
  } else if (cur.ctx == X_LIDSCAN) {
    mods[cur.a].lidOther++;
  } else if (cur.ctx == X_ABS) {
    IdentRes &ir = cur.b == 0 ? absP.name : cur.b == 1 ? absP.uds19 : absP.kwp18;
    if (ir.st == 0) ir.st = 3;
  }
  UNLOCK();
  reqDone();
}

void handleDiagFrame(const twai_message_t &m) {
  if (m.rtr || m.data_length_code < 1 || m.data_length_code > 8) return;
  const uint8_t *d = m.data;
  uint16_t rx = m.identifier;
  uint8_t pci = d[0] >> 4;
  if (pci == 0) {
    uint8_t n = d[0] & 0x0F;
    if (n >= 1 && n <= 7 && n + 1 <= m.data_length_code) onDiagMessage(rx, d + 1, n);
    return;
  }
  IsoSlot *s = nullptr;
  for (auto &x : iso) if (x.rx == rx && (x.active || pci == 1)) { s = &x; break; }
  if (!s && pci == 1) { for (auto &x : iso) if (!x.active) { s = &x; break; } if (!s) s = &iso[0]; }
  if (!s) return;
  if (pci == 1) {
    if (m.data_length_code != 8) return;
    s->rx = rx;
    s->total = ((d[0] & 0x0F) << 8) | d[1];
    if (s->total <= 7 || s->total > sizeof(s->buf)) { s->active = false; return; }
    memcpy(s->buf, d + 2, 6);
    s->got = 6; s->sn = 1; s->active = true;
    uint16_t tx = fcTxFor(rx);
    if (tx) { uint8_t fc[3] = {0x30, 0x00, 0x00}; canSend(tx, fc, 3); }
    if (inFlight) curDeadline = millis() + 1000;
  } else if (pci == 2 && s->active) {
    if ((d[0] & 0x0F) != s->sn) { s->active = false; return; }
    int n = s->total - s->got; if (n > 7) n = 7;
    if (n <= 0 || n + 1 > m.data_length_code) { s->active = false; return; }
    memcpy(s->buf + s->got, d + 1, n);
    s->got += n; s->sn = (s->sn + 1) & 0x0F;
    if (s->got >= s->total) { s->active = false; onDiagMessage(rx, s->buf, s->total); }
  }
}

// ---------------------------- job starters ----------------------------------
void startScan() {
  scanN = 0; scanI = 0;
  scanQ[scanN++] = {0x01, 0x00, true};
  scanQ[scanN++] = {0x01, 0x20, true};
  scanQ[scanN++] = {0x01, 0x40, true};
  scanQ[scanN++] = {0x01, 0x60, true};
  scanQ[scanN++] = {0x01, 0x80, true};
  scanQ[scanN++] = {0x01, 0xA0, true};
  scanQ[scanN++] = {0x01, 0xA6, true};
  scanQ[scanN++] = {0x01, 0x01, true};
  scanQ[scanN++] = {0x03, 0, false};
  scanQ[scanN++] = {0x07, 0, false};
  scanQ[scanN++] = {0x09, 0x02, true};
  scanQ[scanN++] = {0x21, 0x21, true};
  dmode = D_SCAN;
  LOCK(); obd.scanning = true; obd.tried = true; UNLOCK();
}

bool inSnap(uint32_t id) { for (int i = 0; i < snapN; i++) if (snapIds[i] == id) return true; return false; }

void startModScan(bool deep) {
  LOCK();
  if (rpmRunning() || (fresh(S_SPD_OBD) && sigs[S_SPD_OBD].v > 0)) {
    snprintf(modMsg, sizeof(modMsg), "Refused: engine running or vehicle moving. Stop the bike and engine (ignition ON).");
    UNLOCK(); return;
  }
  snapN = 0;
  for (int i = 0; i < idN; i++) if (ids[i].rxSeen && !ids[i].ext) snapIds[snapN++] = ids[i].id;
  for (int i = 0; i < 8; i++) if (obd.responders & (1 << i)) addModule(0x7E0 + i, 0x7E8 + i, "OBD-II responder", nullptr, 0);
  blockN = 0;
  for (int i = 0; i < modN; i++) {  // fresh results
    Module &m = mods[i];
    memset(m.ident, 0, sizeof(m.ident));
    m.no22 = m.no1A = m.no19 = m.no18 = m.no21 = false;
    m.lidPos = m.lidRange = m.lidOther = 0;
  }
  snprintf(modMsg, sizeof(modMsg), "Scanning...");
  UNLOCK();
  sweepN = 0; sweepI = 0;
  for (uint16_t id = 0x7E0; id <= 0x7E7; id++) sweepList[sweepN++] = id;
  if (deep) {
    for (uint16_t id = 0x700; id <= 0x7F7 && sweepN < 260; id++) {
      if (id >= 0x7DF && id <= 0x7EF) continue;
      if (inSnap(id)) continue;     // never inject frames with IDs that already carry data
      sweepList[sweepN++] = id;
    }
  }
  dmode = D_SWEEP;
}

void startLidScan() { jobMod = 0; jobIdx = 1; LOCK(); blockN = 0; UNLOCK(); dmode = D_LIDSCAN; }

void startAbsRead() {
  LOCK();
  // Target the ABS candidate. If a module scan already mapped the 0x7E9
  // responder, use its request ID; otherwise fall back to 0x7E1 -> 0x7E9.
  absP.tx = 0x7E1; absP.rx = 0x7E9;
  for (int i = 0; i < modN; i++) if (mods[i].rx == 0x7E9) { absP.tx = mods[i].tx; break; }
  absP.tried = true;
  memset(&absP.name, 0, sizeof(IdentRes));
  memset(&absP.uds19, 0, sizeof(IdentRes));
  memset(&absP.kwp18, 0, sizeof(IdentRes));
  snprintf(modBusy, sizeof(modBusy), "Reading ABS fault memory (0x%03X)...", absP.rx);
  UNLOCK();
  absStep = 0;
  dmode = D_ABS;
}

void finishJobs(const char *msg) {
  LOCK();
  snprintf(modMsg, sizeof(modMsg), "%s %d module(s), %d data block(s).", msg, modN, blockN);
  modBusy[0] = 0;
  UNLOCK();
  dmode = liveWanted ? D_LIVE : D_IDLE;
  lastMaintMs = millis();
}

// ---------------------------- main diag step --------------------------------
void diagStep() {
  uint32_t now = millis();
  if (cmdScan && !inFlight) { cmdScan = false; startScan(); }
  if (cmdLiveOn) { cmdLiveOn = false; liveWanted = true; if (dmode == D_IDLE) dmode = D_LIVE; }
  if (cmdLiveOff) { cmdLiveOff = false; liveWanted = false; if (dmode == D_LIVE) dmode = D_IDLE; }

  if (inFlight) {
    if ((int32_t)(now - curDeadline) > 0) onTimeout();
    else return;
  }
  // module commands only start between jobs
  if (cmdModScan && (dmode == D_LIVE || dmode == D_IDLE)) { bool deep = cmdModScan == 2; cmdModScan = 0; startModScan(deep); }
  if (cmdLidScan && (dmode == D_LIVE || dmode == D_IDLE)) { cmdLidScan = false; startLidScan(); }
  if (cmdAbsRead && (dmode == D_LIVE || dmode == D_IDLE)) { cmdAbsRead = false; startAbsRead(); }
  if (now - lastDoneMs < LIVE_GAP_MS) return;

  switch (dmode) {
    case D_SCAN: {
      while (scanI < scanN) {
        ObdQ q = scanQ[scanI++];
        bool useful = true;
        LOCK();
        if (q.mode == 1 && q.pid >= 0x20) useful = obd.supKnown && pidSupported(q.pid);
        if (q.mode == 0x21) useful = obd.kwpRpm;
        if (!(q.mode == 1 && q.pid == 0) && !obd.avail && scanI > 1) useful = false;
        UNLOCK();
        if (useful) { sendReq(obdReq(q.mode, q.pid, q.hasPid)); return; }
      }
      LOCK(); obd.scanning = false; obd.lastScanMs = now; bool avail = obd.avail; bool run = rpmRunning(); UNLOCK();
      lastMaintMs = now;
      dmode = liveWanted ? D_LIVE : D_IDLE;
      if (!bootReportMs) bootReportMs = now + 4000;
      if (!autoModScanDone && avail) {
        autoModScanDone = true;
        if (!run) startModScan(false);
      }
    } return;

    case D_SWEEP: {
      if (sweepI < sweepN) {
        uint16_t id = sweepList[sweepI++];
        LOCK(); snprintf(modBusy, sizeof(modBusy), "Address sweep %d/%d (0x%03X)", sweepI, sweepN, id); UNLOCK();
        Req r = {};
        r.tx = id; r.rx = 0xFFFF; r.n = 2; r.d[0] = 0x3E; r.d[1] = 0x00; r.ctx = X_SWEEP; r.tmo = 60;
        sendReq(r);
        return;
      }
      jobMod = 0; jobIdx = 0;
      dmode = D_IDENT;
    } return;

    case D_IDENT: {
      LOCK();
      while (jobMod < modN) {
        Module &m = mods[jobMod];
        while (jobIdx < NIDENT && svcNo(m, IDENTS[jobIdx].d[0])) { m.ident[jobIdx].st = 4; jobIdx++; }
        if (jobIdx < NIDENT) break;
        jobMod++; jobIdx = 0;
      }
      if (jobMod >= modN) { UNLOCK(); startLidScan(); return; }
      Module &m = mods[jobMod];
      snprintf(modBusy, sizeof(modBusy), "Identifying 0x%03X: %d/%d", m.rx, jobIdx + 1, NIDENT);
      Req r = {};
      r.tx = m.tx; r.rx = m.rx; r.n = IDENTS[jobIdx].n; memcpy(r.d, IDENTS[jobIdx].d, r.n);
      r.ctx = X_IDENT; r.a = jobMod; r.b = jobIdx; r.tmo = 150;
      jobIdx++;
      UNLOCK();
      sendReq(r);
    } return;

    case D_LIDSCAN: {
      LOCK();
      while (jobMod < modN) {
        Module &m = mods[jobMod];
        bool dead = m.no21 || (jobIdx > 8 && m.lidPos == 0 && m.lidRange == 0) || blockN >= MAX_BLOCKS;
        if (!dead && jobIdx <= 0xFE) break;
        jobMod++; jobIdx = 1;
      }
      if (jobMod >= modN) { UNLOCK(); finishJobs("Scan complete:"); return; }
      Module &m = mods[jobMod];
      snprintf(modBusy, sizeof(modBusy), "Data blocks 0x%03X: LID 0x%02X (%d found)", m.rx, jobIdx, blockN);
      Req r = {};
      r.tx = m.tx; r.rx = m.rx; r.n = 2; r.d[0] = 0x21; r.d[1] = jobIdx;
      r.ctx = X_LIDSCAN; r.a = jobMod; r.tmo = 80;
      jobIdx++;
      UNLOCK();
      sendReq(r);
    } return;

    case D_ABS: {
      // skip the UDS name fallback if the KWP name already answered
      while (absStep < NABSQ) {
        const AbsReadQ &q = ABSQ[absStep];
        bool skip = (q.slot == 0 && absP.name.st == 1 && absStep > 0);
        if (skip) { absStep++; continue; }
        break;
      }
      if (absStep >= NABSQ) {
        LOCK(); snprintf(modMsg, sizeof(modMsg), "ABS read done (0x%03X).", absP.rx); modBusy[0] = 0; UNLOCK();
        dmode = liveWanted ? D_LIVE : D_IDLE; lastMaintMs = millis();
        return;
      }
      const AbsReadQ &q = ABSQ[absStep];
      Req r = {};
      r.tx = absP.tx; r.rx = absP.rx; r.n = q.n; memcpy(r.d, q.d, q.n);
      r.ctx = X_ABS; r.b = q.slot; r.tmo = 250;   // ABS modulator can be slow
      absStep++;
      sendReq(r);
    } return;

    case D_LIVE: {
      bool avail; LOCK(); avail = obd.avail; obd.live = true; UNLOCK();
      if (!avail) {
        if (now - lastMaintMs > 20000) { lastMaintMs = now; startScan(); }
        return;
      }
      if (now - lastMaintMs > 30000) {
        lastMaintMs = now;
        scanN = 0; scanI = 0;
        scanQ[scanN++] = {0x01, 0x01, true};
        scanQ[scanN++] = {0x03, 0, false};
        scanQ[scanN++] = {0x07, 0, false};
        dmode = D_SCAN;
        LOCK(); obd.scanning = true; UNLOCK();
        return;
      }
      liveToggle = (liveToggle + 1) % 3;
      LOCK(); bool s0C = obd.supKnown && pidSupported(0x0C), kwp = obd.kwpRpm; int nb = blockN; UNLOCK();
      if (liveToggle == 0) {                       // RPM every third request
        if (s0C) { sendReq(obdReq(0x01, 0x0C, true)); return; }
        if (kwp) { sendReq(obdReq(0x21, 0x21, true)); return; }
      }
      if (liveToggle == 2 && nb > 0) {             // data blocks
        LOCK();
        liveBlockI %= blockN;
        Block &b = blocks[liveBlockI];
        Req r = {};
        r.tx = mods[b.mod].tx; r.rx = mods[b.mod].rx; r.n = 2; r.d[0] = 0x21; r.d[1] = b.lid;
        r.ctx = X_LIDPOLL; r.a = liveBlockI; r.tmo = 120;
        liveBlockI++;
        UNLOCK();
        sendReq(r);
        return;
      }
      for (int tries = 0; tries < NPIDS; tries++) {   // every supported PID
        const PidDef &d = PIDS[liveI]; liveI = (liveI + 1) % NPIDS;
        if (d.pid == 0x01 || d.pid == 0x0C) continue;
        LOCK(); bool ok = obd.supKnown && pidSupported(d.pid); UNLOCK();
        if (ok) { sendReq(obdReq(0x01, d.pid, true)); return; }
      }
    } return;

    default:
      LOCK(); obd.live = false; UNLOCK();
      return;
  }
}

// ============================================================================
//                            health evaluation
// ============================================================================
bool bestRpm(float &r) {
  if (fresh(S_RPM_OBD)) { r = sigs[S_RPM_OBD].v; return true; }
  if (fresh(S_RPM_KWP)) { r = sigs[S_RPM_KWP].v; return true; }
  // Unverified broadcast map is not used for health decisions.
  return false;
}

void evaluateHealth() {  // LOCK held
  uint32_t now = millis();
  // --- bus ---
  if (!strcmp(canPub.phase, "SEARCHING")) {
    if (canPub.searchCycles >= 1) {
      if (canPub.searchErr > 50)
        setChk(C_BUS, 3, "RX sees activity but no valid frame at any bitrate (%u errors): CANH/CANL swapped, or CRX signal bad (check the 1k/2k divider, CJMCU 5V supply).", canPub.searchErr);
      else
        setChk(C_BUS, 3, "Nothing on RX at any bitrate. Check yellow->CANH, blue->CANL, common GND, CJMCU-2551 VCC = 5V, CRX->GPIO4, ignition ON + kill switch RUN.");
    } else setChk(C_BUS, 0, "Searching bitrate...");
  } else if (!strcmp(canPub.state, "BUS_OFF") || !strcmp(canPub.state, "RECOVERING")) {
    setChk(C_BUS, 3, "Controller went BUS-OFF (TEC %u). Our transmitter is fighting the bus: wiring, Rs pin, or extra 120R.", canPub.tec);
  } else if (canPub.rec > 127 || canPub.tec > 127) {
    setChk(C_BUS, 2, "Error-passive (TEC %u, REC %u): marginal link. Twist the CAN pair, check ground and termination.", canPub.tec, canPub.rec);
  } else if (canPub.fps < 1 && !obd.avail) {
    setChk(C_BUS, 2, "Locked at %u bit/s but no traffic right now.", canPub.rate);
  } else {
    setChk(C_BUS, 1, "%u bit/s, %.0f frames/s, load %.1f%%, REC %u, TEC %u%s", canPub.rate, canPub.fps, canPub.load,
           canPub.rec, canPub.tec, canPub.load > 80 ? " (very high load)" : "");
  }
  // --- transceiver loop ---
  if (!canPub.rxValid) setChk(C_XCVR, 0, "No valid frame received yet.");
  else if (!canPub.normal) setChk(C_XCVR, 1, "RX path OK (%u valid frames). TX not tested (listen-only).", canPub.rxValid);
  else if (canPub.loopOk) setChk(C_XCVR, 1, "Full loop verified: GPIO5 -> CTX -> bus -> CRX -> GPIO4 (%u/%u self-received).", canPub.loopOk, canPub.loopTried);
  else if (canPub.loopTried >= 3 && obd.resps > 0)
    setChk(C_XCVR, 1, "TX path proven by %u module replies (self-reception copy not observed).", (unsigned)obd.resps);
  else if (canPub.loopTried >= 3)
    setChk(C_XCVR, 3, "Our frames never come back on CRX. Check CTX<-GPIO5, MCP2551 Rs (pin 8) near 0V (standby if >0.75*VCC), VCC=5V.");
  else setChk(C_XCVR, 0, "Testing (%u sent)...", canPub.loopTried);
  // --- ECU comms ---
  if (!ENABLE_ACTIVE_DIAG) setChk(C_ECU, 0, "Active diagnostics disabled (listen-only build).");
  else if (!obd.tried) setChk(C_ECU, 0, "Waiting for bus lock.");
  else if (obd.avail && obd.engineSeen && now - obd.lastEngineMs < 10000) {
    char resp[40] = ""; int o = 0;
    for (int i = 0; i < 8; i++) if (obd.responders & (1 << i)) o += snprintf(resp + o, sizeof(resp) - o, "%X ", 0x7E8 + i);
    setChk(C_ECU, 1, "ECU answers OBD-II (responders: %s). %u req / %u resp / %u timeouts.", resp, obd.reqs, obd.resps, obd.timeouts);
  } else if (obd.scanning) setChk(C_ECU, 0, "Scanning...");
  else if (canPub.txFail > 3 && canPub.fps > 1)
    setChk(C_ECU, 3, "Requests not ACKed while the bus is active: CTX wiring or MCP2551 Rs pin (standby).");
  else setChk(C_ECU, 2, "ECU did not answer OBD-II requests. Broadcast sniffing still works.");
  // --- MIL / DTC ---
  int nS = 0, nP = 0; char list[120] = ""; int lo = 0;
  for (int i = 0; i < dtcN; i++) {
    if (dtcs[i].pending) nP++; else nS++;
    if (lo < 100) lo += snprintf(list + lo, sizeof(list) - lo, "%s%s ", dtcs[i].code, dtcs[i].pending ? "(p)" : "");
  }
  if (!obd.avail) setChk(C_MIL, 0, "Needs ECU communication.");
  else if (!obd.milSeen || !obd.storedSeen || !obd.pendingSeen || now - obd.milMs > 90000 || now - obd.storedMs > 90000 || now - obd.pendingMs > 90000) setChk(C_MIL, 0, "Engine fault scan incomplete/stale; ABS fault memory is separate.");
  else if (obd.mil || nS) setChk(C_MIL, 3, "MIL %s, %d stored / %d pending: %s", obd.mil ? "ON" : "off", nS, nP, list);
  else if (nP) setChk(C_MIL, 2, "%d pending code(s): %s", nP, list);
  else setChk(C_MIL, 1, "MIL off, no stored or pending codes.");
  // --- sensor plausibility (all OBD sensors) ---
  {
    int nOk = 0, nBad = 0; char bad[130] = ""; int bo = 0;
    for (int i = 0; i < NPIDS; i++) {
      const char *why; uint8_t st = pidStatus(i, &why);
      if (st == 1) nOk++;
      if (st == 2) { nBad++; if (bo < 110) bo += snprintf(bad + bo, sizeof(bad) - bo, "%s: %s; ", PIDS[i].sensor, why); }
    }
    if (!nOk && !nBad) setChk(C_SENS, 0, "No sensor data yet.");
    else if (nBad) setChk(C_SENS, 2, "%d suspect: %s", nBad, bad);
    else setChk(C_SENS, 1, "%d ECU-reported values all plausible.", nOk);
  }
  // --- battery / charging ---
  float r = 0; bool haveRpm = bestRpm(r);
  bool running = haveRpm && r > 400;
  if (running && !runningSinceMs) {
    runningSinceMs = now;
    if (VBAT_ADC_PIN >= 0) { float mn = 99; for (float v : vHist) if (v > 1 && v < mn) mn = v; crankMinV = mn < 99 ? mn : 0; }
  }
  if (!running) runningSinceMs = 0;
  uint32_t runFor = running ? now - runningSinceMs : 0;
  int vi = fresh(S_VBAT_ADC) ? S_VBAT_ADC : (fresh(S_VMOD_OBD) ? S_VMOD_OBD : -1);
  if (vi < 0) { setChk(C_BATT, 0, "No voltage source (OBD PID 42 unsupported and no ADC divider)."); setChk(C_CHG, 0, "No voltage source."); }
  else {
    float v = sigs[vi].v;
    if (!running && haveRpm) {
      if (v < VBAT_OFF_CRIT) setChk(C_BATT, 3, "%.2f V engine off: battery discharged or failing.", v);
      else if (v < VBAT_OFF_WARN) setChk(C_BATT, 2, "%.2f V engine off: low state of charge.", v);
      else setChk(C_BATT, 1, "%.2f V engine off.", v);
    } else if (!haveRpm) setChk(C_BATT, 0, "%.2f V (engine state unknown).", v);
    else if (crankMinV > 0) setChk(C_BATT, crankMinV < VBAT_CRANK_MIN ? 2 : 1, "Cranking dip %.2f V%s", crankMinV, crankMinV < VBAT_CRANK_MIN ? ": weak battery or bad connection" : "");
    if (running && runFor > 10000 && r > 2500) {
      if (v > VBAT_CHG_MAX) setChk(C_CHG, 3, "%.2f V at %.0f rpm: OVERCHARGING, check regulator/rectifier.", v, r);
      else if (v < VBAT_CHG_MIN) setChk(C_CHG, 2, "%.2f V at %.0f rpm: not charging, check stator / regulator-rectifier / connectors.", v, r);
      else setChk(C_CHG, 1, "%.2f V at %.0f rpm.", v, r);
    } else if (checks[C_CHG].st == 0 || !running) setChk(C_CHG, 0, "Rev above 2500 rpm for 10 s to test (now %.2f V).", v);
  }
  // --- coolant ---
  int ci = fresh(S_CLT_OBD) ? S_CLT_OBD : -1;
  if (ci < 0) setChk(C_CLT, 0, "No coolant data.");
  else {
    float c = sigs[ci].v; const char *src = ci == S_CLT_OBD ? "" : " (unverified broadcast)";
    if (c <= -39) setChk(C_CLT, 3, "%.0f C: sensor reads open circuit%s.", c, src);
    else if (c >= COOLANT_CRIT_C) setChk(C_CLT, 3, "%.0f C: OVERHEATING%s. Stop, check fan, coolant level, radiator.", c, src);
    else if (c >= COOLANT_WARN_C) setChk(C_CLT, 2, "%.0f C: hot%s. Is the fan switching on?", c, src);
    else if (running && runFor > 900000 && c < 50) setChk(C_CLT, 2, "%.0f C after 15 min running%s: thermostat stuck open or sensor fault?", c, src);
    else setChk(C_CLT, 1, "%.0f C%s", c, src);
  }
  // --- TPS / MAP / IAT ---
  if (!fresh(S_TPS_OBD)) setChk(C_TPS, 0, "No OBD TPS data.");
  else {
    float t = sigs[S_TPS_OBD].v;
    setChk(C_TPS, 1, "%.1f%% absolute throttle (closed offset varies by model).", t);
  }
  if (!fresh(S_MAP_OBD)) setChk(C_MAP, 0, "No OBD MAP data.");
  else {
    float m = sigs[S_MAP_OBD].v;
    if (!running && haveRpm && (m < 75 || m > 108)) setChk(C_MAP, 2, "%.0f kPa with engine off: should be ~atmospheric (90-101 kPa). Sensor/hose?", m);
    else setChk(C_MAP, 1, "%.0f kPa%s", m, running ? " (running)" : "");
  }
  if (!fresh(S_IAT_OBD)) setChk(C_IAT, 0, "No OBD IAT data.");
  else {
    float a = sigs[S_IAT_OBD].v;
    if (a <= -39) setChk(C_IAT, 2, "%.0f C: sensor open circuit?", a);
    else if (a >= 100) setChk(C_IAT, 2, "%.0f C: implausibly hot, sensor short?", a);
    else setChk(C_IAT, 1, "%.0f C", a);
  }
  // --- idle ---
  if (!running) setChk(C_IDLE, 0, "Start the engine and let it idle.");
  else if (rpmHistN < 20) setChk(C_IDLE, 0, "Collecting RPM samples (%d/20)...", rpmHistN);
  else {
    float sum = 0, sq = 0, mx = 0; int n = rpmHistN;
    for (int i = 0; i < n; i++) { sum += rpmHist[i]; if (rpmHist[i] > mx) mx = rpmHist[i]; }
    float mean = sum / n;
    for (int i = 0; i < n; i++) sq += (rpmHist[i] - mean) * (rpmHist[i] - mean);
    float sd = sqrtf(sq / n);
    bool warm = (ci >= 0 && sigs[ci].v > 75);
    if (mx > 2600) setChk(C_IDLE, 0, "Engine not idling (max %.0f rpm in window).", mx);
    else if (sd > IDLE_RPM_STDEV_MAX) setChk(C_IDLE, 2, "Idle hunting: %.0f rpm +/- %.0f. Throttle body/ISC, vacuum leak, plug?", mean, sd);
    else if (warm && (mean < IDLE_RPM_MIN || mean > IDLE_RPM_MAX)) setChk(C_IDLE, 2, "Warm idle %.0f rpm outside %.0f-%.0f (check manual spec).", mean, IDLE_RPM_MIN, IDLE_RPM_MAX);
    else setChk(C_IDLE, 1, "%.0f rpm +/- %.0f%s", mean, sd, warm ? "" : " (still warming up)");
  }
  // --- fuel trims ---
  if (!fresh(S_STFT) && !fresh(S_LTFT)) setChk(C_TRIM, 0, "Fuel trim PIDs not available.");
  else {
    float s = fresh(S_STFT) ? sigs[S_STFT].v : 0, l = fresh(S_LTFT) ? sigs[S_LTFT].v : 0, t = s + l;
    bool warm = ci >= 0 && sigs[ci].v > 70;
    if (!running || runFor < 60000 || !warm) setChk(C_TRIM, 0, "STFT %.1f%% LTFT %.1f%% (judged after 60 s warm running).", s, l);
    else if (t > FUEL_TRIM_MAX_PCT) setChk(C_TRIM, 2, "Total +%.1f%%: running LEAN (air leak, weak pump, clogged injector).", t);
    else if (t < -FUEL_TRIM_MAX_PCT) setChk(C_TRIM, 2, "Total %.1f%%: running RICH (leaky injector, high fuel pressure, O2 sensor).", t);
    else setChk(C_TRIM, 1, "STFT %.1f%%, LTFT %.1f%%", s, l);
  }
  // --- nodes alive ---
  char lost[120] = ""; int lo2 = 0, lostN = 0, bcN = 0;
  for (int i = 0; i < idN; i++) {
    IdStat &s = ids[i];
    if (!s.rxSeen || (s.id >= 0x700 && s.id <= 0x7FF)) continue;
    if (s.n < 20 || s.periodUs <= 0) continue;
    bcN++;
    uint32_t lim = (uint32_t)(s.periodUs / 100.0f); if (lim < 1500) lim = 1500;
    if (now - s.lastMs > lim) { lostN++; if (lo2 < 100) lo2 += snprintf(lost + lo2, sizeof(lost) - lo2, "0x%03X ", (unsigned)s.id); }
  }
  if (!bcN) setChk(C_NODES, 0, "No periodic broadcast traffic seen.");
  else if (lostN) setChk(C_NODES, 2, "%d periodic ID(s) stopped: %s- a module dropped off the bus or a connector is loose.", lostN, lost);
  else setChk(C_NODES, 1, "%d periodic IDs all alive, %d diagnostic module(s) known.", bcN, modN);

  if (!bootReportDone && bootReportMs && (int32_t)(now - bootReportMs) > 0) {
    memcpy(bootReport, checks, sizeof(checks)); bootReportDone = true; bootReportMs = now;
  }
}

// ============================================================================
//                                 CAN task
// ============================================================================
const uint32_t RATES[] = {500000, 250000, 1000000, 125000};
uint32_t lockedRate = 0;

void processFrame(const twai_message_t &m) {
  // our own frame read back through the transceiver (self-reception)
  if (!m.extd && lastTxSelf && m.identifier == lastTxId && !memcmp(m.data, lastTxData, 8) && millis() - lastTxMs < 100) {
    lastTxSelf = false;
    LOCK(); canPub.loopOk++; UNLOCK();
    return;
  }
  logFrame(m, false);
  if (dmode == D_SWEEP && inFlight && cur.ctx == X_SWEEP && !m.extd && m.identifier != cur.tx) {
    bool diagRange = m.identifier >= 0x7E8 && m.identifier <= 0x7EF;
    bool validReply = !m.rtr && m.data_length_code >= 2 && m.data_length_code <= 8 &&
      (m.data[0] >> 4) == 0 && (m.data[0] & 15) + 1 <= m.data_length_code &&
      (m.data[1] == 0x7E || ((m.data[0] & 15) >= 3 && m.data[1] == 0x7F && m.data[2] == 0x3E));
    if (validReply && (diagRange || !inSnap(m.identifier))) {
      LOCK(); addModule(cur.tx, m.identifier, "TesterPresent sweep", m.data, m.data_length_code); UNLOCK();
      reqDone();
    }
  }
  if (!m.extd && isDiagRx(m.identifier)) handleDiagFrame(m);
  else decodeBroadcast(m);
}

bool listenFor(uint32_t ms, int needFrames, bool needDiag) {
  uint32_t t0 = millis(); int n = 0;
  twai_message_t m;
  while (millis() - t0 < ms) {
    if (twai_receive(&m, pdMS_TO_TICKS(10)) == ESP_OK) {
      processFrame(m);
      if (!needDiag || (m.identifier >= 0x7E8 && m.identifier <= 0x7EF)) n++;
      if (n >= needFrames) return true;
    }
  }
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) { LOCK(); canPub.searchErr += st.bus_error_count; UNLOCK(); }
  return false;
}

void lockTo(uint32_t rate, bool normalNow) {
  lockedRate = rate;
  if (ENABLE_ACTIVE_DIAG && !normalNow) { canStopDrv(); canStart(rate, TWAI_MODE_NORMAL); normalNow = true; }
  LOCK(); canPub.phase = "LOCKED"; canPub.rate = rate; canPub.normal = normalNow; UNLOCK();
  Serial.printf("[CAN] locked at %u bit/s, %s mode\n", (unsigned)rate, normalNow ? "normal" : "listen-only");
  if (ENABLE_ACTIVE_DIAG) startScan();
}

void autoBaud() {
  LOCK(); canPub.searchErr = 0; UNLOCK();
  for (uint32_t rate : RATES) {
    if (!canStart(rate, TWAI_MODE_LISTEN_ONLY)) continue;
    LOCK(); canPub.rate = rate; canPub.state = "LISTENING"; UNLOCK();
    if (listenFor(400, 3, false)) { lockTo(rate, false); return; }
    canStopDrv();
  }
  if (ENABLE_ACTIVE_DIAG && canStart(500000, TWAI_MODE_NORMAL)) {   // silent bus: ask once at 500k
    uint8_t req[3] = {0x02, 0x01, 0x00};
    canSend(0x7DF, req, 3);
    if (listenFor(400, 1, true)) { lockTo(500000, true); return; }
    canStopDrv();
  }
  LOCK(); canPub.searchCycles++; canPub.rate = 0; canPub.state = "STOPPED"; evaluateHealth(); UNLOCK();
  Serial.println("[CAN] no traffic at any bitrate, retrying");
}

void canTask(void *) {
  uint32_t lastStat = 0, lastEval = 0, lastAdc = 0, lastSec = millis();
  uint32_t prevTxFail = 0;
  for (;;) {
    if (cmdClearIds) { cmdClearIds = false; LOCK(); idN = 0; idOverflow = 0; UNLOCK(); }
    if (!lockedRate) { autoBaud(); continue; }

    twai_message_t m;
    int burst = 0;
    while (burst < 64 && twai_receive(&m, pdMS_TO_TICKS(burst ? 0 : 5)) == ESP_OK) { processFrame(m); burst++; }

    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK && (alerts & TWAI_ALERT_BUS_OFF)) {
      Serial.println("[CAN] BUS-OFF, recovering");
      twai_initiate_recovery();
    }
    if (alerts & TWAI_ALERT_BUS_RECOVERED) twai_start();

    if (ENABLE_ACTIVE_DIAG) diagStep();

    uint32_t now = millis();
    if (VBAT_ADC_PIN >= 0 && now - lastAdc >= 100) {
      lastAdc = now;
      uint32_t mv = 0; for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(VBAT_ADC_PIN);
      float v = mv / 8.0f / 1000.0f * VBAT_DIVIDER;
      vHist[vHistI] = v; vHistI = (vHistI + 1) % 50;
      LOCK(); setSig(S_VBAT_ADC, v); UNLOCK();
    }
    if (now - lastStat >= 250) {
      lastStat = now;
      twai_status_info_t st;
      if (twai_get_status_info(&st) == ESP_OK) {
        LOCK();
        canPub.tec = st.tx_error_counter; canPub.rec = st.rx_error_counter;
        canPub.missed = st.rx_missed_count; canPub.overrun = st.rx_overrun_count;
        canPub.busErr = st.bus_error_count; canPub.arbLost = st.arb_lost_count;
        if (st.tx_failed_count != prevTxFail) { canPub.txFail += st.tx_failed_count - prevTxFail; prevTxFail = st.tx_failed_count; }
        canPub.state = st.state == TWAI_STATE_RUNNING ? "RUNNING" : st.state == TWAI_STATE_BUS_OFF ? "BUS_OFF"
                     : st.state == TWAI_STATE_RECOVERING ? "RECOVERING" : "STOPPED";
        UNLOCK();
      }
    }
    if (now - lastSec >= 1000) {
      float dt = (now - lastSec) / 1000.0f; lastSec = now;
      LOCK();
      canPub.fps = framesThisSec / dt;
      canPub.load = lockedRate ? (float)bitsThisSec * 100.0f / dt / lockedRate : 0;
      framesThisSec = 0; bitsThisSec = 0;
      UNLOCK();
    }
    if (now - lastEval >= 500) { lastEval = now; LOCK(); evaluateHealth(); UNLOCK(); }
  }
}

// ============================================================================
//                                web server
// ============================================================================
extern const char INDEX_HTML[];

void sendJson() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", J);
}

void jChecks(const Check *c) {
  J += '[';
  for (int i = 0; i < C_COUNT; i++) {
    if (i) J += ',';
    jf("{\"n\":\"%s\",\"st\":%d,\"d\":", CHK_NAMES[i], c[i].st); jstr(c[i].detail); J += '}';
  }
  J += ']';
}

void handleState() {
  uint32_t now = millis();
  J = "";
  LOCK();
  jf("{\"up\":%u,\"can\":{\"phase\":\"%s\",\"rate\":%u,\"normal\":%d,\"fps\":%.1f,\"load\":%.1f,\"total\":%u,"
     "\"tec\":%u,\"rec\":%u,\"missed\":%u,\"overrun\":%u,\"busErr\":%u,\"txFail\":%u,\"arb\":%u,\"state\":\"%s\",\"ids\":%d,\"idOv\":%u,\"cycles\":%u,"
     "\"loopTried\":%u,\"loopOk\":%u},",
     (unsigned)now, canPub.phase, (unsigned)canPub.rate, canPub.normal, canPub.fps, canPub.load, (unsigned)canPub.total,
     (unsigned)canPub.tec, (unsigned)canPub.rec, (unsigned)canPub.missed, (unsigned)canPub.overrun, (unsigned)canPub.busErr,
     (unsigned)canPub.txFail, (unsigned)canPub.arbLost, canPub.state, idN, (unsigned)idOverflow, (unsigned)canPub.searchCycles,
     (unsigned)canPub.loopTried, (unsigned)canPub.loopOk);
  jf("\"obd\":{\"enabled\":%d,\"tried\":%d,\"avail\":%d,\"scanning\":%d,\"live\":%d,\"mil\":%d,\"dtcRep\":%d,\"supKnown\":%d,"
     "\"sup\":\"%08X %08X %08X %08X %08X %08X\",\"ecu\":\"%X\",\"resp\":%u,\"reqs\":%u,\"resps\":%u,\"tmo\":%u,\"negs\":%u,\"kwp\":%d,\"lastScan\":%u,\"vin\":",
     ENABLE_ACTIVE_DIAG, obd.tried, obd.avail, obd.scanning, obd.live, obd.mil, obd.dtcReported, obd.supKnown,
     (unsigned)obd.sup[0], (unsigned)obd.sup[1], (unsigned)obd.sup[2], (unsigned)obd.sup[3], (unsigned)obd.sup[4], (unsigned)obd.sup[5], obd.ecuId, obd.responders, (unsigned)obd.reqs,
     (unsigned)obd.resps, (unsigned)obd.timeouts, (unsigned)obd.negs, obd.kwpRpm, (unsigned)(obd.lastScanMs ? now - obd.lastScanMs : 0));
  jstr(obd.vin);
  jf(",\"engineFresh\":%d,\"faultComplete\":%d,\"odoState\":%d,\"absCandidateSeen\":%d", obd.engineSeen && now - obd.lastEngineMs < 10000,
     obd.milSeen && obd.storedSeen && obd.pendingSeen && now - obd.milMs < 90000 && now - obd.storedMs < 90000 && now - obd.pendingMs < 90000,
     sigs[S_ODO].seen ? 3 : (pidSupported(0xA6) && obd.supKnown) ? 2 : (obd.supKnown && !obd.scanning) ? 1 : 0,
     (obd.responders & 2) != 0);
  J += "},\"dtcs\":[";
  for (int i = 0; i < dtcN; i++) {
    if (i) J += ',';
    jf("{\"c\":\"%s\",\"p\":%d,\"from\":\"%X\",\"d\":", dtcs[i].code, dtcs[i].pending, dtcs[i].from);
    jstr(dtcDesc(dtcs[i].code)); J += '}';
  }
  J += "],\"sig\":[";
  bool first = true;
  for (int i = 0; i < S_COUNT; i++) {
    if (!sigs[i].seen) continue;
    if (!first) J += ',';
    first = false;
    jf("{\"k\":\"%s\",\"n\":\"%s\",\"u\":\"%s\",\"v\":%.3f,\"age\":%u,\"s\":", sigs[i].key, sigs[i].name, sigs[i].unit, sigs[i].v, (unsigned)(now - sigs[i].ms));
    jstr(sigs[i].src); J += '}';
  }
  J += "],\"checks\":";
  jChecks(checks);
  J += ",\"boot\":";
  if (!bootReportDone) J += "null";
  else { jf("{\"age\":%u,\"c\":", (unsigned)(now - bootReportMs)); jChecks(bootReport); J += '}'; }
  UNLOCK();
  J += '}';
  sendJson();
}

char clsOf(const IdStat &s, int b) {
  if (!s.chg[b]) return 'K';                                       // constant
  if (s.chg[b] >= 8 && s.ctr[b] * 10 >= s.chg[b] * 8) return 'C';  // counter
  if (__builtin_popcount(s.tog[b]) <= 2) return 'F';               // flag bits
  return 'A';                                                      // analog / value
}

void handleIds() {
  uint32_t now = millis();
  J = "";
  LOCK();
  J += "{\"ids\":[";
  for (int i = 0; i < idN; i++) {
    IdStat &s = ids[i];
    if (i) J += ',';
    jf("{\"id\":\"%0*X\",\"x\":%d,\"rx\":%d,\"l\":%d,\"d\":", s.ext ? 8 : 3, (unsigned)s.id, s.ext, s.rxSeen, s.dlc);
    jhex(s.d, s.dlc);
    J += ",\"ch\":[";
    for (int b = 0; b < s.dlc; b++) {
      uint32_t a = s.chgMs[b] ? now - s.chgMs[b] : 99999; if (a > 99999) a = 99999;
      jf(b ? ",%u" : "%u", (unsigned)a);
    }
    J += "],\"cls\":\"";
    for (int b = 0; b < s.dlc; b++) J += clsOf(s, b);
    J += "\",\"tog\":[";
    for (int b = 0; b < s.dlc; b++) jf(b ? ",%u" : "%u", s.tog[b]);
    J += "],\"mn\":"; jhex(s.mn, s.dlc);
    J += ",\"mx\":"; jhex(s.mx, s.dlc);
    const char *cs = (s.n >= 20 && s.csX == s.n) ? "xor" : (s.n >= 20 && s.csS == s.n) ? "sum" : "";
    jf(",\"cs\":\"%s\",\"hz\":%.1f,\"n\":%u,\"age\":%u,\"lbl\":", cs, s.periodUs > 0 ? 1e6f / s.periodUs : 0.0f, (unsigned)s.n, (unsigned)(now - s.lastMs));
    jstr(idLabel(s.id, s.ext)); J += '}';
  }
  UNLOCK();
  J += "]}";
  sendJson();
}

void handleSensors() {
  uint32_t now = millis();
  J = "";
  LOCK();
  J += "{\"msg\":"; jstr(modMsg);
  J += ",\"busy\":"; jstr(modBusy);
  jf(",\"active\":%d,\"mods\":[", ENABLE_ACTIVE_DIAG);
  for (int i = 0; i < modN; i++) {
    Module &m = mods[i];
    if (i) J += ',';
    jf("{\"tx\":\"%03X\",\"rx\":\"%03X\",\"how\":", m.tx, m.rx); jstr(m.how);
    J += ",\"first\":"; jhex(m.first, m.firstLen);
    jf(",\"age\":%u,\"lid\":[%u,%u,%u,%d],\"id\":[", (unsigned)(now - m.lastMs), m.lidPos, m.lidRange, m.lidOther, m.no21);
    bool f = true;
    for (int k = 0; k < NIDENT; k++) {
      IdentRes &r = m.ident[k];
      if (!r.st) continue;
      if (!f) J += ',';
      f = false;
      jf("{\"n\":\"%s\",\"st\":%d,\"nrc\":%d,\"d\":", IDENTS[k].name, r.st, r.nrc); jhex(r.d, r.len); J += '}';
    }
    J += "]}";
  }
  J += "],\"pids\":[";
  bool f = true;
  for (int i = 0; i < NPIDS; i++) {
    bool sup = obd.supKnown && pidSupported(PIDS[i].pid);
    // Include unsupported/unseen defined PIDs, rather than presenting fake zeroes.
    if (!f) J += ',';
    f = false;
    const char *why; uint8_t st = pidStatus(i, &why);
    jf("{\"pid\":\"%02X\",\"s\":\"%s\",\"n\":\"%s\",\"u\":\"%s\",\"v\":%.3f,\"raw\":", PIDS[i].pid, PIDS[i].sensor, PIDS[i].name, PIDS[i].unit, pv[i].v);
    jhex(pv[i].raw, pv[i].n);
    jf(",\"age\":%u,\"st\":%d,\"why\":\"%s\",\"sup\":%d,\"seen\":%d}", (unsigned)(pv[i].seen ? now - pv[i].ms : 0), st, why, sup, pv[i].seen);
  }
  J += "],\"blocks\":[";
  for (int i = 0; i < blockN; i++) {
    Block &b = blocks[i];
    if (i) J += ',';
    jf("{\"m\":\"%03X\",\"lid\":\"%02X\",\"d\":", mods[b.mod].rx, b.lid); jhex(b.d, b.len);
    J += ",\"ch\":[";
    for (int k = 0; k < b.len; k++) { uint32_t a = b.chgMs[k] ? now - b.chgMs[k] : 99999; if (a > 99999) a = 99999; jf(k ? ",%u" : "%u", (unsigned)a); }
    jf("],\"age\":%u}", (unsigned)(now - b.ms));
  }
  J += "],\"abs\":{";
  jf("\"tx\":\"%03X\",\"rx\":\"%03X\",\"tried\":%d,\"age\":%u,\"seen\":%d", absP.tx, absP.rx, absP.tried,
     (unsigned)(absP.ms ? now - absP.ms : 0), (obd.responders & 2) != 0);
  const IdentRes *slots[3] = {&absP.name, &absP.uds19, &absP.kwp18};
  const char *keys[3] = {"name", "uds19", "kwp18"};
  for (int i = 0; i < 3; i++) {
    jf(",\"%s\":{\"st\":%d,\"nrc\":%d,\"d\":", keys[i], slots[i]->st, slots[i]->nrc);
    jhex(slots[i]->d, slots[i]->len); J += '}';
  }
  J += "}";
  UNLOCK();
  J += "}";
  sendJson();
}

void handleFrames() {
  uint32_t since = server.hasArg("since") ? strtoul(server.arg("since").c_str(), nullptr, 10) : 0;
  J = "";
  LOCK();
  uint32_t head = ringSeq, oldest = head > RAW_N ? head - RAW_N : 0;
  if (since < oldest) since = oldest;
  if (since > head) since = head;
  uint32_t end = since + 300 < head ? since + 300 : head;
  jf("{\"head\":%u,\"oldest\":%u,\"next\":%u,\"f\":[", (unsigned)head, (unsigned)oldest, (unsigned)end);
  for (uint32_t s = since; s < end; s++) {
    RawFrame &r = ring[s % RAW_N];
    if (s != since) J += ',';
    jf("[%u,\"%llu.%06llu\",\"%0*X\",%d,", (unsigned)s, r.tus / 1000000ULL, r.tus % 1000000ULL, (r.flags & 1) ? 8 : 3, (unsigned)r.id, r.flags);
    jhex(r.d, r.dlc);
    J += ']';
  }
  UNLOCK();
  J += "]}";
  sendJson();
}

void handleCandump() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=duke390_candump.log");
  server.send(200, "text/plain", "");
  LOCK();
  uint32_t head = ringSeq, s = head > RAW_N ? head - RAW_N : 0;
  UNLOCK();
  String chunk; chunk.reserve(4200);
  char line[64];
  while (s < head) {
    chunk = "";
    LOCK();
    for (int k = 0; k < 80 && s < head; k++, s++) {
      RawFrame &r = ring[s % RAW_N];
      int o = snprintf(line, sizeof(line), "(%llu.%06llu) can0 %0*X#", r.tus / 1000000ULL, r.tus % 1000000ULL, (r.flags & 1) ? 8 : 3, (unsigned)r.id);
      for (int b = 0; b < r.dlc; b++) o += snprintf(line + o, sizeof(line) - o, "%02X", r.d[b]);
      line[o++] = '\n'; line[o] = 0;
      chunk += line;
    }
    UNLOCK();
    server.sendContent(chunk);
  }
  server.sendContent("");
}

void handleCmd() {
  String c = server.arg("c");
  if (c == "scan") cmdScan = true;
  else if (c == "live1") cmdLiveOn = true;
  else if (c == "live0") cmdLiveOff = true;
  else if (c == "clearids") cmdClearIds = true;
  else if (c == "modscan") cmdModScan = 1;
  else if (c == "deepscan") cmdModScan = 2;
  else if (c == "lidscan") cmdLidScan = true;
  else if (c == "rereport") { LOCK(); bootReportDone = false; bootReportMs = millis() + 4000; UNLOCK(); }
  else if (c == "absread") cmdAbsRead = true;
  server.send(200, "application/json", "{\"ok\":1}");
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Duke 390 CAN logger ===");
  lockMx = xSemaphoreCreateMutex();
  ring = (RawFrame *)calloc(RAW_N, sizeof(RawFrame));
  if (!ring) { Serial.println("ring alloc failed"); while (1) delay(1000); }
  J.reserve(24000);
  for (int i = 0; i < C_COUNT; i++) setChk(i, 0, "Waiting...");
  if (VBAT_ADC_PIN >= 0) analogSetPinAttenuation(VBAT_ADC_PIN, ADC_11db);

  Serial.printf("[boot] reset reason = %d (1=power-on, 3=SW, 4=panic, 5=INT-WDT, 6=TASK-WDT, 7=deep-sleep, 11=brownout, 12=SDIO)\n", (int)esp_reset_reason());
  if (strlen(STA_SSID)) { WiFi.mode(WIFI_AP_STA); WiFi.begin(STA_SSID, STA_PASS); }
  else WiFi.mode(WIFI_AP);
  // Fixed channel, allow up to 4 clients; the key line is setSleep(false): a
  // sleeping SoftAP stalls the page every few seconds (the "disconnected" bug).
  WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
  WiFi.setSleep(false);
  Serial.printf("[WiFi] AP %s  ->  http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  if (MDNS.begin("duke390")) MDNS.addService("http", "tcp", 80);

  server.on("/", []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api/state", handleState);
  server.on("/api/ids", handleIds);
  server.on("/api/sensors", handleSensors);
  server.on("/api/frames", handleFrames);
  server.on("/api/cmd", handleCmd);
  server.on("/candump.log", handleCandump);
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  server.begin();

  xTaskCreatePinnedToCore(canTask, "can", 8192, nullptr, 3, nullptr, 1);
}

void loop() {
  server.handleClient();
  static uint32_t t = 0;
  if (strlen(STA_SSID) && millis() - t > 10000) {
    t = millis();
    if (WiFi.status() == WL_CONNECTED) Serial.printf("[WiFi] STA IP %s\n", WiFi.localIP().toString().c_str());
  }
  delay(2);
}

// ============================================================================
//                               dashboard page
// ============================================================================
const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Duke 390 CAN</title>
<style>
:root{--bg:#0d1015;--card:#161a21;--line:#252b35;--tx:#e7eaf0;--mut:#8b94a4;--ok:#2fbf71;--warn:#f0a020;--crit:#ef4444;--acc:#ff6a00}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{position:sticky;top:0;z-index:5;background:#0a0c10f0;border-bottom:1px solid var(--line);padding:10px 14px;display:flex;flex-wrap:wrap;gap:6px;align-items:center}
h1{font-size:16px;margin:0 10px 0 0;color:var(--acc);letter-spacing:.3px}
.pill{padding:2px 9px;border-radius:99px;background:var(--card);border:1px solid var(--line);font-size:12px;color:var(--mut);white-space:nowrap}
.pill b{color:var(--tx);font-weight:600}
nav{display:flex;gap:4px;padding:8px 14px;border-bottom:1px solid var(--line);overflow-x:auto}
nav button,.btn{background:var(--card);border:1px solid var(--line);color:var(--tx);padding:6px 12px;border-radius:8px;cursor:pointer;font:inherit;white-space:nowrap}
nav button{background:none;border-color:transparent;color:var(--mut)}nav button.on{background:var(--card);border-color:var(--line);color:var(--tx)}
.btn:hover{border-color:var(--acc)}.btn.on{background:var(--acc);border-color:var(--acc);color:#111}.btn.sm{padding:3px 8px;font-size:12px}
main{padding:14px;max-width:1250px;margin:auto}section[hidden]{display:none}
h2{font-size:13px;text-transform:uppercase;letter-spacing:.8px;color:var(--mut);margin:18px 0 8px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(165px,1fr));gap:10px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
.m .l{color:var(--mut);font-size:12px}.m .v{font-size:28px;font-weight:650;font-variant-numeric:tabular-nums;line-height:1.2}
.m .u{font-size:13px;color:var(--mut);margin-left:3px;font-weight:400}.m .s{font-size:11px;color:var(--mut);margin-top:2px;min-height:15px}
.m.stale .v{color:#4d5563}.bar{height:5px;background:#222833;border-radius:3px;margin-top:8px;overflow:hidden}.bar i{display:block;height:100%;background:var(--acc);transition:width .25s}
.banner{border-radius:12px;padding:14px 16px;font-size:17px;font-weight:600;cursor:pointer}
.banner.ok{background:#12301f;border:1px solid #1f5c38}.banner.warn{background:#33260b;border:1px solid #6b4d10}.banner.crit{background:#3a1212;border:1px solid #7a2222}.banner.na{background:var(--card);border:1px solid var(--line)}
.chk{display:grid;grid-template-columns:26px 210px 1fr;gap:8px;padding:8px 10px;border-bottom:1px solid var(--line);align-items:start}
.chk:last-child{border:0}.dot{width:12px;height:12px;border-radius:50%;margin-top:4px;background:#3a414d}
.st1 .dot{background:var(--ok)}.st2 .dot{background:var(--warn)}.st3 .dot{background:var(--crit)}
.chk .n{font-weight:600}.chk .d{color:var(--mut)}.st2 .d{color:#f3c46b}.st3 .d{color:#ff8a8a}
@media(max-width:640px){.chk{grid-template-columns:22px 1fr}.chk .d{grid-column:2}}
table{width:100%;border-collapse:collapse;font-size:13px}th,td{text-align:left;padding:6px 8px;border-bottom:1px solid var(--line);vertical-align:top}
th{color:var(--mut);font-weight:500;position:sticky;top:0;background:var(--card)}tr.click{cursor:pointer}tr.click:hover{background:#1c212a}
.mono,.b{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
.b{display:inline-block;width:2.4ch;text-align:center;border-radius:3px}.b.c{background:#ff6a0045;color:#ffc08a}.b.c2{background:#ff6a0018}
.cl{font-size:10px;color:var(--mut)}.cl .b{width:2.4ch}
.tw{overflow:auto;max-height:70vh;border-radius:12px;border:1px solid var(--line);background:var(--card)}
pre{margin:0;padding:10px;font-size:12px;white-space:pre;overflow:auto;max-height:65vh;background:var(--card);border:1px solid var(--line);border-radius:12px}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:10px}
input{background:var(--card);border:1px solid var(--line);color:var(--tx);border-radius:8px;padding:6px 10px;font:inherit;width:150px}
.mut{color:var(--mut)}.tag{font-size:11px;padding:1px 6px;border-radius:4px;background:#252b35;color:var(--mut);white-space:nowrap}
.tag.ok{background:#12301f;color:#7fe0a8}.tag.w{background:#33260b;color:#f3c46b}.tag.e{background:#3a1212;color:#ff9a9a}
.dtc{display:flex;gap:10px;padding:8px 10px;border-bottom:1px solid var(--line)}.dtc b{font-family:ui-monospace,monospace;color:#ff8a8a;min-width:58px}
.kv{display:grid;grid-template-columns:180px 1fr;gap:4px 12px}.kv div:nth-child(odd){color:var(--mut)}
.mod{margin-bottom:10px}.mod h3{margin:0 0 6px;font-size:15px}.note{font-size:12px;color:var(--mut);margin:6px 0}
.big{font-size:22px;font-weight:700;color:var(--acc)}
</style></head><body>
<header><h1>DUKE 390 &middot; CAN</h1>
<span class="pill">bus <b id="pBus">-</b></span><span class="pill">mode <b id="pMode">-</b></span>
<span class="pill"><b id="pFps">0</b> fr/s</span><span class="pill">load <b id="pLoad">0</b>%</span>
<span class="pill">frames <b id="pTot">0</b></span><span class="pill">ECU <b id="pEcu">-</b></span><span class="pill" id="pConn">connecting</span></header>
<nav><button data-t="ov" class="on">Overview</button><button data-t="health">Health check</button><button data-t="sens">Sensors &amp; modules</button><button data-t="find">Signal finder</button><button data-t="ids">Bus IDs</button><button data-t="raw">Raw log</button><button data-t="diag">Diagnostics</button><button data-t="abs">ABS service</button></nav>
<main>
<section id="ov">
 <div id="banner" class="banner na">Waiting for data&hellip;</div>
 <h2>Live engine data</h2><div class="grid" id="metrics"></div>
 <h2>Vehicle mileage</h2><div class="card" id="odoInfo">Waiting for ECU capability scan.</div>
 <h2>All decoded signals</h2><div class="tw"><table><thead><tr><th>Signal</th><th>Value</th><th>Source</th><th>Age</th></tr></thead><tbody id="sigT"></tbody></table></div>
</section>
<section id="health" hidden>
 <div class="row"><button class="btn" onclick="cmd('scan')">Run full scan now</button><button class="btn" onclick="cmd('rereport')">Re-take key-on report</button></div>
 <h2>Live health check</h2><div class="card" id="chkLive"></div>
 <h2>Key-on self-test (frozen after first scan)</h2><div class="card" id="chkBoot"><span class="mut">Not taken yet.</span></div>
</section>
<section id="sens" hidden>
 <div class="row"><button class="btn" onclick="cmd('modscan')">Scan modules</button><button class="btn" onclick="cmd('deepscan')">Deep scan 0x700-0x7F7</button><button class="btn" onclick="cmd('lidscan')">Re-scan ECU data blocks</button><span class="mut" id="modMsg"></span></div>
 <div class="note">Module scans are read-only and refuse to run with the engine running. Keep the bike on its stand, ignition ON.</div>
 <h2>Modules on the CAN bus</h2><div id="modL"><span class="mut">No modules yet.</span></div>
 <h2>Sensors reported by the engine ECU (OBD-II)</h2><div class="tw"><table><thead><tr><th>PID</th><th>Sensor</th><th>Measurement</th><th>Value</th><th>Raw</th><th>Status</th></tr></thead><tbody id="pidT"></tbody></table></div>
 <h2>ECU data blocks (KWP 21 xx) &mdash; raw internal values</h2><div class="note">Orange = changed in the last 0.6 s. Use the Signal finder to work out which byte is which sensor.</div>
 <div class="tw"><table><thead><tr><th>Module</th><th>LID</th><th>Len</th><th>Data</th><th>Age</th></tr></thead><tbody id="blkT"></tbody></table></div>
 <h2>Sensors &amp; switches seen in broadcast traffic</h2><div class="tw"><table><thead><tr><th>Name</th><th>Value</th><th>Source</th></tr></thead><tbody id="bcT"></tbody></table></div>
</section>
<section id="find" hidden>
 <h2>1 &middot; Auto-match against ECU values</h2>
 <div class="note">Engine running: blip the throttle a few times over ~30 s, let it warm up, roll the bike for speed. Every broadcast byte, 16-bit word and data-block byte is correlated with the OBD values. |r| close to 1 = same signal.</div>
 <div class="row"><button class="btn" id="bCorr">Pause matching</button><button class="btn" onclick="ACC.clear()">Reset</button><span class="mut" id="corrInfo"></span></div>
 <div id="corrR" class="card"><span class="mut">Collecting samples&hellip;</span></div>
 <h2>2 &middot; Action detector (switches, levers, buttons)</h2>
 <div class="note">Press Start, keep everything still for 5 s, then operate ONE thing repeatedly for 6 s (front brake, rear brake, clutch, side stand, horn, indicators, high beam, kill switch, gear lever, ABS button...). Bits that change only during the action are listed.</div>
 <div class="row"><button class="btn" id="bAct">Start</button><span class="big" id="actPh"></span></div>
 <div id="actR" class="card"><span class="mut">No result yet.</span></div>
 <h2>3 &middot; My named signals</h2>
 <div class="row"><button class="btn sm" id="bExp">Export</button><button class="btn sm" id="bClrL">Delete all</button></div>
 <div class="tw"><table><thead><tr><th>Name</th><th>Source</th><th>Value</th><th></th></tr></thead><tbody id="lblT"></tbody></table></div>
</section>
<section id="ids" hidden>
 <div class="row"><span class="mut" id="idInfo"></span><button class="btn" onclick="cmd('clearids')">Reset table</button></div>
 <div class="note">Orange bytes changed in the last 0.6 s. Letter under each byte: K constant &middot; C counter &middot; F flag bits &middot; A value. &Sigma; = last byte is a checksum. Tap a row to filter the raw log.</div>
 <div class="tw"><table><thead><tr><th>ID</th><th>DLC</th><th>Data</th><th>Hz</th><th>Count</th><th>Meaning</th></tr></thead><tbody id="idT"></tbody></table></div>
</section>
<section id="raw" hidden>
 <div class="row"><input id="flt" placeholder="filter ID e.g. 120"><button class="btn" id="bPause">Pause</button>
 <button class="btn" id="bRec">&#9679; Record</button><button class="btn" id="bDl">Download recording</button>
 <a class="btn" href="/candump.log" style="text-decoration:none">Download last 3000 (ESP32 buffer)</a><span class="mut" id="recInfo"></span></div>
 <pre id="rawP" class="mono"></pre>
</section>
<section id="diag" hidden>
 <div class="row"><button class="btn" onclick="cmd('scan')">Run full scan</button><button class="btn" id="bLive">Live OBD polling</button></div>
 <h2>Fault codes (DTC)</h2><div class="card" id="dtcL"></div>
 <h2>ECU / OBD status</h2><div class="card kv" id="obdKV"></div>
 <h2>CAN controller</h2><div class="card kv" id="canKV"></div>
</section>
<section id="abs" hidden>
 <h2>Bosch ABS 9MB &mdash; read-only fault diagnosis</h2>
 <div class="row"><button class="btn" onclick="cmd('absread')">Read ABS fault codes</button><span class="mut" id="absBusy"></span></div>
 <div class="note">Reads the ABS module's stored fault memory (ISO 14229 service 19 / KWP 18). Read-only: no pump or valve is ever commanded. Ignition ON; engine may be running or off.</div>
 <div class="card" id="absInfo">Waiting for module discovery.</div>
 <h2>ABS fault codes</h2><div class="card" id="absDtc"><span class="mut">Press "Read ABS fault codes".</span></div>
 <h2>ABS module identification</h2><div class="card" id="absId"><span class="mut">No data yet.</span></div>
 <h2>Pump-dead diagnosis (do this with a multimeter)</h2>
 <div class="card"><ol style="margin:0;padding-left:18px;line-height:1.7">
  <li><b>Read the codes above first.</b> A "pump motor circuit" or "motor supply" code names the fault directly.</li>
  <li><b>ABS pump-motor fuse.</b> The 9MB has a separate high-current fuse for the pump besides the ECU fuse. A blown motor fuse is the most common cause of a dead pump. Check the KTM fuse legend.</li>
  <li><b>Motor supply.</b> Ignition ON, confirm battery voltage reaches the motor-supply pin at the ABS connector.</li>
  <li><b>Motor ground.</b> Near-zero ohms from the motor-ground pin to battery negative.</li>
  <li><b>Motor winding.</b> Connector off, measure across the pump-motor terminals. Open circuit = burnt motor/brushes; a few ohms = motor OK, fault is in supply/ground or the module's driver.</li>
 </ol>
 <p class="note">Actuator bleeding and any pump run must be done with the KTM dealer tool (or a scanner with a 9MB bleed routine), which runs the interlocks. Confirm brake function before riding.</p></div>
 <h2>Raw ABS diagnostic traffic</h2><div id="absMods" class="card">No data yet.</div>
 <p class="note">Use Raw log to record and download the full diagnostic exchange. A missing or negative response is not proof of a healthy ABS.</p>
</section>
</main>
<script>
const $=s=>document.querySelector(s);
const S={tab:'ov',since:null,paused:false,rec:false,recL:[],lines:[],dropped:0,st:null,ok:0,latest:{},sens:null,corr:true};
let LBL=[];try{LBL=JSON.parse(localStorage.getItem('d390lbl')||'[]')}catch(e){LBL=[]}
function saveLbl(){try{localStorage.setItem('d390lbl',JSON.stringify(LBL))}catch(e){}renderLbl()}
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>{S.tab=b.dataset.t;document.querySelectorAll('nav button').forEach(x=>x.classList.toggle('on',x==b));document.querySelectorAll('section').forEach(s=>s.hidden=s.id!=S.tab);});
const go=t=>document.querySelector('nav button[data-t='+t+']').click();
async function J(u){try{const r=await fetch(u,{cache:'no-store'});if(!r.ok)throw 0;const data=await r.json();S.ok=Date.now();return data}catch(e){return null}}
function cmd(c){fetch('/api/cmd?c='+c)}
const esc=s=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const hb=h=>{const a=[];for(let i=0;i<h.length;i+=2)a.push(parseInt(h.substr(i,2),16));return a};
const NRC={0x10:'general reject',0x11:'service not supported',0x12:'sub-function not supported',0x13:'wrong length',0x22:'conditions not correct',0x31:'out of range',0x33:'security access denied',0x7E:'not in this session',0x7F:'not in this session'};
const METRICS=[
 {n:'Engine RPM',k:['rpm_obd','rpm_kwp'],min:0,max:11000,d:0},
 {n:'Coolant',k:['clt_obd'],min:-20,max:130,d:0},
 {n:'Throttle',k:['tps_obd'],min:0,max:100,d:1},
 {n:'Voltage',k:['vbat_adc','vmod_obd'],min:9,max:16,d:2},
 {n:'Intake air',k:['iat_obd'],min:-20,max:80,d:0},
 {n:'Manifold pressure',k:['map_obd'],min:0,max:110,d:0},
 {n:'Speed',k:['spd_obd'],min:0,max:180,d:0},
 {n:'Gear (unverified)',k:['gear_bc'],min:0,max:6,d:0},
 {n:'Engine load',k:['load_obd'],min:0,max:100,d:0},
 {n:'Ignition advance',k:['adv_obd'],min:-10,max:50,d:1},
 {n:'Short fuel trim',k:['stft_obd'],min:-25,max:25,d:1},
 {n:'Long fuel trim',k:['ltft_obd'],min:-25,max:25,d:1},
 {n:'O2 sensor',k:['o2_obd'],min:0,max:1.1,d:3},
 {n:'Front brake (raw, unverified)',k:['fbrk_bc'],min:0,max:8500,d:0},
 {n:'Engine run time',k:['run_obd'],min:0,max:3600,d:0},
 {n:'Fuel level',k:['fuel_level'],min:0,max:100,d:1},
 {n:'Oil temperature',k:['oil_temp'],min:0,max:150,d:0},
 {n:'Fuel rate',k:['fuel_rate'],min:0,max:15,d:2},
 {n:'Barometric pressure',k:['baro'],min:0,max:110,d:0},
];
$('#metrics').innerHTML=METRICS.map((m,i)=>`<div class="card m stale" id="m${i}"><div class="l">${m.n}</div><div class="v">--<span class="u"></span></div><div class="s"></div><div class="bar"><i style="width:0"></i></div></div>`).join('');
function chkHtml(c){return c.map(x=>`<div class="chk st${x.st}"><span class="dot"></span><span class="n">${esc(x.n)}</span><span class="d">${esc(x.d)}</span></div>`).join('')}
function renderState(st){
 S.st=st;const c=st.can,o=st.obd;
 $('#pBus').textContent=c.phase=='LOCKED'?(c.rate/1000)+'k':'searching';
 $('#pMode').textContent=c.phase=='LOCKED'?(c.normal?'normal':'listen-only'):'-';
 $('#pFps').textContent=c.fps.toFixed(0);$('#pLoad').textContent=c.load.toFixed(1);$('#pTot').textContent=c.total;
 $('#pEcu').textContent=!o.enabled?'off':o.engineFresh?'online':o.scanning?'scanning':o.tried?'silent':'-';
 const sig={};st.sig.forEach(s=>sig[s.k]=s);
 METRICS.forEach((m,i)=>{const el=$('#m'+i);let s=null;for(const k of m.k){const x=sig[k];if(x&&x.age<3000){s=x;break}}
  if(!s){for(const k of m.k)if(sig[k]){s=sig[k];break}}
  if(!s){el.classList.add('stale');el.querySelector('.v').innerHTML='--';el.querySelector('.s').textContent='no data';el.querySelector('i').style.width='0';return}
  const stale=s.age>3000;el.classList.toggle('stale',stale);
  el.querySelector('.v').innerHTML=s.v.toFixed(m.d)+`<span class="u">${s.u}</span>`;
  el.querySelector('.s').textContent=s.s+(stale?' · stale':'');
  el.querySelector('i').style.width=Math.max(0,Math.min(100,(s.v-m.min)/(m.max-m.min)*100))+'%';});
 const ageT=a=>a<1000?a+' ms':(a/1000).toFixed(0)+' s';
 const odo=sig.odo_obd, clr=sig.clr_dist, mil=sig.mil_dist;
 const fmtDist=x=>x?x.v.toFixed(1)+' km (age '+ageT(x.age)+')':'unavailable';
 $('#odoInfo').innerHTML='<b>ECU odometer:</b> '+(odo?esc(fmtDist(odo)):(o.odoState===1?'Not advertised by ECU / capability chain incomplete':o.odoState===2?'Supported; awaiting valid response':'Capability unknown'))+'<br><b>Distance since codes cleared:</b> '+esc(fmtDist(clr))+' (not total mileage)<br><b>Distance with MIL on:</b> '+esc(fmtDist(mil))+'<p class="note">01 A6 resolution is 0.1 km. Compare with the instrument-cluster ODO. No distance estimate is substituted for an unavailable odometer.</p>';
 $('#sigT').innerHTML=st.sig.map(s=>`<tr><td>${esc(s.n)}</td><td class="mono">${(+s.v).toFixed(2)} ${esc(s.u)}</td><td class="mut">${esc(s.s)}</td><td class="mut">${ageT(s.age)}</td></tr>`).join('');
 $('#bcT').innerHTML=st.sig.filter(s=>s.k.endsWith('_bc')).map(s=>`<tr><td>${esc(s.n)}</td><td class="mono">${(+s.v).toFixed(1)} ${esc(s.u)}</td><td class="mut">${esc(s.s)}</td></tr>`).join('')||'<tr><td colspan=3 class="mut">None of the known KTM broadcast IDs seen. Use the Signal finder.</td></tr>';
 const worst=Math.max(0,...st.checks.map(x=>x.st));const bad=st.checks.filter(x=>x.st>=2);
 const b=$('#banner');b.className='banner '+(worst==3?'crit':worst==2?'warn':worst==1?'ok':'na');
 b.innerHTML=worst==3?'&#9888; '+bad.length+' problem(s) found: '+esc(bad.map(x=>x.n).join(', ')):worst==2?'&#9888; Check: '+esc(bad.map(x=>x.n).join(', ')):worst==1?'No issues in completed checks; untested systems are not certified':'Collecting data&hellip;';
 b.onclick=()=>go('health');
 $('#chkLive').innerHTML=chkHtml(st.checks);
 if(st.boot)$('#chkBoot').innerHTML=`<div class="mut" style="padding:4px 10px">Taken ${(st.boot.age/1000).toFixed(0)} s ago</div>`+chkHtml(st.boot.c);
 $('#dtcL').innerHTML=!o.avail?'<span class="mut">ECU not answering OBD (or diagnostics disabled).</span>':st.dtcs.length?st.dtcs.map(d=>`<div class="dtc"><b>${d.c}</b><span>${esc(d.d)} <span class="tag">${d.p?'pending':'stored'}</span> <span class="tag">from ${d.from}</span></span></div>`).join(''):o.faultComplete?'<span style="color:var(--ok)">No engine stored or pending fault codes reported.</span> <span class="mut">MIL '+(o.mil?'ON':'off')+'</span>':'<span class="mut">Engine fault scan incomplete or stale; ABS status unknown.</span>';
 const sup=[];if(o.supKnown){o.sup.split(' ').forEach((h,w)=>{const v=parseInt(h,16);for(let i=0;i<32;i++)if(v>>>(31-i)&1)sup.push((w*32+i+1).toString(16).toUpperCase().padStart(2,'0'))})}
 const kv=a=>a.map(([k,v])=>`<div>${k}</div><div class="mono">${esc(v)}</div>`).join('');
 $('#obdKV').innerHTML=kv([['Active diagnostics',o.enabled?'enabled':'disabled (listen-only build)'],['ECU state',o.engineFresh?'online (0x'+o.ecu+')':o.scanning?'scanning':o.tried?'not answering':'-'],
  ['MIL',o.mil?'ON':'off'],['DTC count (PID 01)',o.dtcRep<0?'-':o.dtcRep],['VIN',o.vin||'-'],['Supported PIDs',sup.join(' ')||'-'],
  ['RPM via KWP 21 21',o.kwp?'trying/ok':'rejected by ECU'],['Requests / responses',o.reqs+' / '+o.resps],['Timeouts / negative',o.tmo+' / '+o.negs],
  ['Live polling',o.live?'on':'off'],['Last scan',o.lastScan?(o.lastScan/1000).toFixed(0)+' s ago':'-']]);
 $('#bLive').classList.toggle('on',o.live);$('#bLive').onclick=()=>cmd(o.live?'live0':'live1');
 $('#canKV').innerHTML=kv([['Phase',c.phase+(c.phase!='LOCKED'?' (cycle '+c.cycles+')':'')],['Controller state',c.state],['Bitrate',c.rate||'-'],
  ['RX / TX error counters',c.rec+' / '+c.tec],['Bus errors',c.busErr],['RX missed / overrun',c.missed+' / '+c.overrun],['Our TX failures',c.txFail],
  ['TX->RX loop (self-reception)',c.loopOk+' / '+c.loopTried],['Arbitration lost',c.arb],['Unique IDs',c.ids+(c.idOv?' (+'+c.idOv+' not tracked)':'')],['Uptime',(st.up/1000).toFixed(0)+' s']]);
 if(S.corr)sampleCorr(st);
}
const CLS={K:'constant',C:'counter',F:'flag bits',A:'value'};
function renderIds(r){
 $('#idInfo').textContent=r.ids.length+' IDs';
 r.ids.sort((a,b)=>parseInt(a.id,16)-parseInt(b.id,16));
 $('#idT').innerHTML=r.ids.map(x=>{let d='',cl='';const mn=hb(x.mn),mx=hb(x.mx);for(let i=0;i<x.l;i++){const a=x.ch[i];d+=`<span class="b ${a<600?'c':a<3000?'c2':''}">${x.d.substr(i*2,2)}</span>`;
   const k=(x.cs&&i==x.l-1)?'&Sigma;':x.cls[i];cl+=`<span class="b" title="${CLS[x.cls[i]]||'checksum'} · range ${mn[i]}..${mx[i]} · bits ${x.tog[i].toString(2).padStart(8,'0')}">${k}</span>`}
  const mine=LBL.filter(l=>l.src==x.id).map(l=>l.name).join(', ');
  return `<tr class="click" onclick="$('#flt').value='${x.id}';go('raw')"><td class="mono">${x.id}${x.x?' <span class=tag>ext</span>':''}${x.rx?'':' <span class=tag>TX</span>'}</td><td>${x.l}</td><td>${d}<div class="cl">${cl}</div></td><td class="mono">${x.hz.toFixed(1)}</td><td class="mono">${x.n}</td><td class="mut">${esc(x.lbl)}${mine?' <span class="tag ok">'+esc(mine)+'</span>':''}${x.age>3000?' <span class=tag>silent '+(x.age/1000).toFixed(0)+'s</span>':''}</td></tr>`}).join('');
}
function identVal(it){
 const d=hb(it.d);if(it.st==3)return '<span class="mut">no answer</span>';if(it.st==4)return '<span class="mut">skipped (service not supported)</span>';
 if(it.st==2)return `<span class="tag w">NRC 0x${(it.nrc).toString(16).toUpperCase()} ${NRC[it.nrc]||''}</span>`;
 const svc=d[0]-0x40;let p=d.slice(svc==0x22?3:svc==0x1A?2:1);
 if(svc==0x19){if(d.length<3||d[1]!==2||(d.length-3)%4!==0)return '<span class="mut">Raw fault response: '+esc(it.d)+'</span>';const recs=[],h2=x=>x.toString(16).padStart(2,'0').toUpperCase();for(let o=3;o+3<d.length;o+=4)recs.push(dtcStr(d[o],d[o+1])+'-'+h2(d[o+2])+' st '+h2(d[o+3]));return recs.length?'<b style="color:#ff8a8a">'+recs.join(', ')+'</b>':'<span style="color:var(--ok)">no codes</span>'}
 if(svc==0x18){if(d.length<2||d.length!==2+d[1]*3)return '<span class="mut">Raw fault response (format unverified): '+esc(it.d)+'</span>';const n=d[1]||0,recs=[];for(let i=0;i<n;i++){const o=2+i*3;if(o+2>=d.length)break;recs.push(dtcStr(d[o],d[o+1])+' st '+d[o+2].toString(16).padStart(2,'0'))}return recs.length?'<b style="color:#ff8a8a">'+recs.join(', ')+'</b>':'<span style="color:var(--ok)">no codes</span>'}
 const asc=p.map(b=>b>=32&&b<127?String.fromCharCode(b):'.').join('');const pr=p.filter(b=>b>=32&&b<127).length;
 return (pr>=p.length*0.7&&p.length?'<b>'+esc(asc)+'</b> ':'')+'<span class="mono mut">'+p.map(b=>b.toString(16).padStart(2,'0').toUpperCase()).join(' ')+'</span>';
}
function dtcStr(a,b){return 'PCBU'[a>>6]+((a>>4)&3)+(a&15).toString(16).toUpperCase()+(b>>4).toString(16).toUpperCase()+(b&15).toString(16).toUpperCase()}
// UDS DTC status mask (ISO 14229-1) - standardized bit meanings
const UDSST=['testFailed','failedThisCycle','pending','confirmed','notCompleteSinceClear','failedSinceClear','notCompleteThisCycle','warningIndicatorOn'];
function statusBits(b){const o=[];for(let i=0;i<8;i++)if(b>>i&1)o.push(UDSST[i]);return o.join(', ')||'no status bits set'}
// Exact Bosch 9MB code meanings are KTM-specific; give an honest category hint only.
function absHint(code){const L=code[0];const n=code.slice(1);
 if(L=='C')return 'Chassis/ABS fault — exact meaning is KTM/Bosch-specific (look up '+code+' in KTM service data). Pump/motor, valve-relay and wheel-speed faults all appear as C-codes.';
 if(L=='U')return 'Communication fault (a module or the CAN bus).';
 if(L=='P')return dtcDesc2(code);
 return 'Body/network fault.'}
function dtcDesc2(c){return 'Powertrain code (reported by the ABS module).'}
// Decode a 59 02 ... (UDS) fault response: 4 bytes per record [b0 b1 failureType status]
function decUds(hex){const d=hb(hex);if(d.length<3)return{err:'short response'};const recs=[];for(let o=3;o+3<d.length;o+=4){const code=dtcStr(d[o],d[o+1]);recs.push({code,ft:d[o+2],st:d[o+3]})}return{recs}}
// Decode an 58 NN ... (KWP) fault response: 3 bytes per record [b0 b1 status]
function decKwp(hex){const d=hb(hex);if(d.length<2)return{err:'short response'};const n=d[1];const recs=[];for(let i=0;i<n;i++){const o=2+i*3;if(o+2>=d.length)break;recs.push({code:dtcStr(d[o],d[o+1]),ft:null,st:d[o+2]})}return{recs}}
function absResHtml(ir,dec){
 if(!ir||ir.st==0)return null;
 if(ir.st==3)return '<span class="mut">no answer (module did not respond)</span>';
 if(ir.st==2)return '<span class="tag w">module refused: NRC 0x'+ir.nrc.toString(16).toUpperCase()+' '+(NRC[ir.nrc]||'')+'</span>';
 const res=dec(ir.d);if(res.err)return '<span class="mut">unparsed: '+esc(ir.d)+'</span>';
 if(!res.recs.length)return '<span style="color:var(--ok)">No fault codes stored.</span>';
 return res.recs.map(r=>`<div class="dtc"><b>${r.code}${r.ft!=null?'-'+r.ft.toString(16).padStart(2,'0').toUpperCase():''}</b><span>${esc(absHint(r.code))}<br><span class="mut">status 0x${r.st.toString(16).padStart(2,'0').toUpperCase()}: ${esc(statusBits(r.st))}</span></span></div>`).join('')}
function renderSens(r){
 S.sens=r;$('#modMsg').textContent=(r.busy?r.busy+' — ':'')+r.msg+(r.active?'':' (listen-only build: active scans disabled)');
 $('#modL').innerHTML=r.mods.length?r.mods.map(m=>`<div class="card mod"><h3>Module 0x${m.tx} &rarr; 0x${m.rx} <span class="tag">${esc(m.how)}</span> <span class="tag ${m.age<5000?'ok':'w'}">${m.age<5000?'alive':'last seen '+(m.age/1000).toFixed(0)+' s'}</span></h3>
  <div class="note">${m.rx=='7E8'?'Most likely the engine ECU (Bosch ME17).':m.rx>='7E9'&&m.rx<='7EF'?'Additional responder; module identity unverified.':'Found by address sweep.'} Data-block scan: ${m.lid[0]} blocks, ${m.lid[1]} out-of-range, ${m.lid[2]} other${m.lid[3]?' (service 21 not supported)':''}.</div>
  ${m.id.length?'<table>'+m.id.map(i=>`<tr><td class="mut" style="width:260px">${esc(i.n)}</td><td>${identVal(i)}</td></tr>`).join('')+'</table>':'<span class="mut">Not identified yet.</span>'}</div>`).join(''):'<span class="mut">No modules yet. Ignition ON, engine off, then press Scan modules.</span>';
 const ab=r.abs;
 if(ab){$('#absBusy').textContent=r.busy&&r.busy.indexOf('ABS')>=0?r.busy:'';
  const name=ab.name&&ab.name.st==1?hb(ab.name.d).filter(b=>b>=32&&b<127).map(b=>String.fromCharCode(b)).join('').replace(/^[^A-Za-z0-9]+/,''):'';
  $('#absInfo').innerHTML=ab.seen?('ABS responder seen on 0x'+ab.rx+' (request 0x'+ab.tx+').'+(name?' Module: <b>'+esc(name)+'</b>.':'')+(ab.tried?'':' Press "Read ABS fault codes".')):'No 0x7E9 diagnostic responder observed yet. The read still tries 0x7E1 on demand.';
  const uds=absResHtml(ab.uds19,decUds), kwp=absResHtml(ab.kwp18,decKwp);
  let html='';
  if(!ab.tried)html='<span class="mut">Press "Read ABS fault codes".</span>';
  else{if(uds)html+='<div class="note">UDS service 19 (ISO 14229):</div>'+uds;
       if(kwp)html+='<div class="note" style="margin-top:8px">KWP service 18 (fallback):</div>'+kwp;
       if(!uds&&!kwp)html='<span class="mut">No response yet. If both stay blank, the ABS may use a different request ID or need a diagnostic session this read-only tool does not open.</span>';}
  $('#absDtc').innerHTML=html;
  const idr=ab.name&&ab.name.st==1?identVal({d:ab.name.d,st:1,nrc:0}):(ab.name&&ab.name.st==2?'<span class="tag w">NRC 0x'+ab.name.nrc.toString(16).toUpperCase()+'</span>':'<span class="mut">not read</span>');
  $('#absId').innerHTML='<table><tr><td class="mut" style="width:200px">Request / response ID</td><td class="mono">0x'+esc(ab.tx)+' / 0x'+esc(ab.rx)+'</td></tr><tr><td class="mut">System name</td><td>'+idr+'</td></tr></table>';}
 $('#absMods').innerHTML=r.mods.filter(m=>m.rx==='7E9').map(m=>'<b>0x'+esc(m.tx)+' → 0x'+esc(m.rx)+'</b> · last response '+(m.age/1000).toFixed(1)+' s ago'+(m.id.length?'<table>'+m.id.map(i=>'<tr><td>'+esc(i.n)+'</td><td>'+identVal(i)+'</td></tr>').join('')+'</table>':'<p>Identity not read yet.</p>')).join('')||'<span class="mut">No identified 0x7E9 candidate yet.</span>';
 $('#pidT').innerHTML=r.pids.length?r.pids.map(p=>`<tr><td class="mono">${p.pid}</td><td>${esc(p.s)}</td><td class="mut">${esc(p.n)}</td><td class="mono">${p.seen?(p.pid==='01'?'MIL '+((parseInt(p.raw.slice(0,2),16)&128)?'ON':'off')+' / DTC '+(parseInt(p.raw.slice(0,2),16)&127):p.pid==='03'||p.pid==='13'?'0x'+p.raw:(+p.v).toFixed(p.pid==='A6'?1:2)+' '+esc(p.u)):'—'}</td><td class="mono mut">${p.raw}</td><td>${p.st==1?'<span class="tag ok">reported</span>':p.st==2?'<span class="tag e">'+esc(p.why)+'</span>':'<span class="tag">'+(p.seen?'stale':p.sup?'waiting':'not advertised / unknown')+'</span>'}</td></tr>`).join(''):'<tr><td colspan=6 class="mut">ECU has not reported its supported PIDs yet.</td></tr>';
 $('#blkT').innerHTML=r.blocks.length?r.blocks.map(b=>{let d='';for(let i=0;i<b.d.length/2;i++){const a=b.ch[i];d+=`<span class="b ${a<600?'c':a<3000?'c2':''}">${b.d.substr(i*2,2)}</span>`}return `<tr><td class="mono">${b.m}</td><td class="mono">${b.lid}</td><td>${b.d.length/2}</td><td style="max-width:640px">${d}</td><td class="mut">${(b.age/1000).toFixed(1)} s</td></tr>`}).join(''):'<tr><td colspan=5 class="mut">No data blocks (the ECU may not support KWP service 21, or the scan has not run).</td></tr>';
 r.blocks.forEach(b=>{const k='LID '+b.m+':'+b.lid;const d=hb(b.d);feedAct(k,d);S.latest[k]={d,t:Date.now()-b.age}});
}
/* ---------- frames, raw log, recording ---------- */
function ingest(f){const[seq,t,id,fl,hex]=f;if(fl&4)return;const n=parseInt(id,16);if(!(fl&1)&&n>=0x700&&n<=0x7FF&&isDiag(id))return;
 const d=hb(hex);feedAct(id,d);S.latest[id]={d,t:Date.now()}}
function isDiag(id){const n=parseInt(id,16);if(n>=0x7DF&&n<=0x7EF)return true;if(S.sens)for(const m of S.sens.mods)if(m.rx==id||m.tx==id)return true;return false}
async function pollFrames(){
 const r=await J('/api/frames?since='+(S.since==null?'0':S.since));if(!r)return;
 if(S.since==null){S.since=Math.max(r.oldest,r.head-200);if(r.head>200)return;}
 if(S.since<r.oldest)S.dropped+=r.oldest-S.since;
 r.f.forEach(f=>{const[seq,t,id,fl,hex]=f;ingest(f);const x={id,txt:`${t.padStart(14)}  ${(fl&4)?'TX':'RX'}  ${id.padStart(3)}  [${hex.length/2}]  ${hex.replace(/(..)/g,'$1 ').trim()}`};
  if(S.rec)S.recL.push(`(${t}) can0 ${id}#${hex}`);S.lines.push(x)});
 S.since=r.next;if(S.lines.length>2000)S.lines.splice(0,S.lines.length-2000);
 $('#recInfo').textContent=(S.rec?'recording: ':'recorded: ')+S.recL.length+' frames'+(S.dropped?' · '+S.dropped+' dropped':'');
 if(S.tab=='raw'&&!S.paused){const q=$('#flt').value.trim().toUpperCase();const L=(q?S.lines.filter(x=>x.id.includes(q)):S.lines).slice(-300);$('#rawP').textContent=L.map(x=>x.txt).join('\n');$('#rawP').scrollTop=1e9}
}
$('#bPause').onclick=e=>{S.paused=!S.paused;e.target.textContent=S.paused?'Resume':'Pause'};
$('#bRec').onclick=e=>{S.rec=!S.rec;if(S.rec){S.recL=[];S.dropped=0}e.target.classList.toggle('on',S.rec);e.target.innerHTML=S.rec?'&#9632; Stop':'&#9679; Record'};
function dl(name,txt){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([txt],{type:'text/plain'}));a.download=name;a.click()}
$('#bDl').onclick=()=>{if(!S.recL.length)return alert('Nothing recorded yet');dl('duke390_rec_'+Date.now()+'.log',S.recL.join('\n')+'\n')};
/* ---------- signal finder: correlation ---------- */
const REFS=[{k:'rpm',n:'Engine RPM',keys:['rpm_obd','rpm_kwp'],min:150},{k:'tps',n:'Throttle position',keys:['tps_obd'],min:3},{k:'clt',n:'Coolant temp',keys:['clt_obd'],min:2},
 {k:'spd',n:'Vehicle speed',keys:['spd_obd'],min:3},{k:'map',n:'Manifold pressure',keys:['map_obd'],min:4},{k:'iat',n:'Intake air temp',keys:['iat_obd'],min:2},{k:'load',n:'Engine load',keys:['load_obd'],min:3},{k:'volt',n:'ECU voltage',keys:['vmod_obd'],min:0.2}];
const ACC=new Map();let corrSamples=0;
function cands(d){const c=[];for(let i=0;i<d.length;i++)c.push(['u8',i,d[i]]);for(let i=0;i+1<d.length;i++){c.push(['be16',i,d[i]*256+d[i+1]]);c.push(['le16',i,d[i]+d[i+1]*256])}return c}
function sampleCorr(st){const sig={};st.sig.forEach(s=>sig[s.k]=s);const now=Date.now();const rv={};
 REFS.forEach(r=>{for(const k of r.keys){const s=sig[k];if(s&&s.age<1200){rv[r.k]=s.v;break}}});if(!Object.keys(rv).length)return;corrSamples++;
 for(const src in S.latest){const o=S.latest[src];if(now-o.t>1500)continue;for(const[kind,i,x]of cands(o.d))for(const r in rv){const key=src+'|'+kind+'|'+i+'|'+r;let a=ACC.get(key);
  if(!a){a={n:0,sx:0,sy:0,sxx:0,syy:0,sxy:0};ACC.set(key,a)}const y=rv[r];a.n++;a.sx+=x;a.sy+=y;a.sxx+=x*x;a.syy+=y*y;a.sxy+=x*y}}}
function corrResults(){const best={};for(const[key,a]of ACC){if(a.n<20)continue;const mx=a.sx/a.n,my=a.sy/a.n,vx=a.sxx/a.n-mx*mx,vy=a.syy/a.n-my*my;
  const[src,kind,i,r]=key.split('|');const ref=REFS.find(x=>x.k==r);if(vx<=1e-9||vy<=0||Math.sqrt(vy)<ref.min)continue;
  const cov=a.sxy/a.n-mx*my,rr=cov/Math.sqrt(vx*vy),slope=cov/vx,off=my-slope*mx;if(Math.abs(rr)<0.85)continue;(best[r]=best[r]||[]).push({src,kind,i:+i,r:rr,slope,off,n:a.n})}
 for(const r in best)best[r]=best[r].sort((a,b)=>Math.abs(b.r)-Math.abs(a.r)).slice(0,4);return best}
const kindT=(k,i)=>k=='u8'?'byte '+i:k=='be16'?'bytes '+i+'-'+(i+1)+' (big-endian)':'bytes '+i+'-'+(i+1)+' (little-endian)';
let CR={};
function renderCorr(){CR=corrResults();$('#corrInfo').textContent=corrSamples+' samples, '+ACC.size+' candidates';
 const h=REFS.filter(r=>CR[r.k]).map(r=>`<div style="margin-bottom:10px"><b>${r.n}</b>`+CR[r.k].map((c,j)=>`<div class="row" style="margin:4px 0 0 10px"><span class="mono">${esc(c.src)}</span> <span>${kindT(c.kind,c.i)}</span> <span class="tag ${Math.abs(c.r)>0.97?'ok':'w'}">r=${c.r.toFixed(3)}</span> <span class="mut mono">&asymp; raw&times;${c.slope.toPrecision(4)} ${c.off>=0?'+':'-'} ${Math.abs(c.off).toPrecision(4)}</span> <button class="btn sm" onclick="saveCorr('${r.k}',${j})">Save as "${r.n}"</button></div>`).join('')+'</div>').join('');
 $('#corrR').innerHTML=h||'<span class="mut">No strong match yet. Needs the ECU online (OBD values) and the values must change: rev, warm up, ride.</span>'}
function saveCorr(rk,j){const c=CR[rk][j],r=REFS.find(x=>x.k==rk);LBL.push({src:c.src,kind:c.kind,i:c.i,bit:0,name:r.n,scale:c.slope,off:c.off});saveLbl()}
$('#bCorr').onclick=e=>{S.corr=!S.corr;e.target.textContent=S.corr?'Pause matching':'Resume matching'};
/* ---------- signal finder: action detector ---------- */
const ACT={ph:'idle',base:{},act:{},res:[]};
function feedAct(src,d){if(ACT.ph!='base'&&ACT.ph!='act')return;const M=ACT.ph=='base'?ACT.base:ACT.act;let m=M[src];const L=S.latest[src];
 if(!m)m=M[src]={tog:new Array(d.length).fill(0),mn:d.slice(),mx:d.slice()};const p=L?L.d:null;
 for(let i=0;i<d.length&&i<m.tog.length;i++){if(p&&i<p.length)m.tog[i]|=d[i]^p[i];if(d[i]<m.mn[i])m.mn[i]=d[i];if(d[i]>m.mx[i])m.mx[i]=d[i]}}
function actStart(){ACT.ph='base';ACT.base={};ACT.act={};ACT.res=[];let t=5;$('#actPh').textContent='Keep still… '+t;
 const iv=setInterval(()=>{t--;if(ACT.ph=='base'){if(t>0)$('#actPh').textContent='Keep still… '+t;else{ACT.ph='act';t=6;try{navigator.vibrate&&navigator.vibrate(200)}catch(e){}$('#actPh').textContent='NOW: operate it repeatedly… '+t}}
  else if(ACT.ph=='act'){if(t>0)$('#actPh').textContent='NOW: operate it repeatedly… '+t;else{clearInterval(iv);ACT.ph='done';$('#actPh').textContent='Done';actResults()}}},1000)}
function actResults(){const R=[];for(const src in ACT.act){const a=ACT.act[src],b=ACT.base[src];if(!b){R.push({src,kind:'new',i:0,bit:0,note:'only appeared during the action'});continue}
 for(let i=0;i<a.tog.length;i++){const ra=a.mx[i]-a.mn[i],rb=(b.mx[i]-b.mn[i])||0;if(ra>=4&&ra>3*rb+2){R.push({src,kind:'u8',i,bit:0,note:`value ${a.mn[i]}..${a.mx[i]} (still: ${b.mn[i]}..${b.mx[i]})`});continue}
  const nb=a.tog[i]&~(b.tog[i]||0);for(let k=0;k<8;k++)if(nb>>k&1)R.push({src,kind:'bit',i,bit:k,note:'bit toggled only during the action'})}}
 ACT.res=R.slice(0,40);$('#actR').innerHTML=ACT.res.length?'<table>'+ACT.res.map((x,j)=>`<tr><td class="mono">${esc(x.src)}</td><td>${x.kind=='bit'?'byte '+x.i+' bit '+x.bit:x.kind=='u8'?'byte '+x.i:'whole ID'}</td><td class="mut">${esc(x.note)}</td><td><input id="an${j}" placeholder="name e.g. Front brake"> <button class="btn sm" onclick="saveAct(${j})">Save</button></td></tr>`).join('')+'</table>'
 :'<span class="mut">Nothing changed only during the action. That input may not be on CAN (many switches are wired straight to the ECU/dash), or try again with a longer, clearer action.</span>'}
function saveAct(j){const x=ACT.res[j],n=$('#an'+j).value.trim();if(!n)return alert('Type a name first');LBL.push({src:x.src,kind:x.kind=='new'?'u8':x.kind,i:x.i,bit:x.bit,name:n,scale:1,off:0});saveLbl()}
$('#bAct').onclick=actStart;
/* ---------- labels ---------- */
function lblVal(l){const o=S.latest[l.src];if(!o)return null;const d=o.d,i=l.i;if(i>=d.length)return null;let x;
 if(l.kind=='bit')x=(d[i]>>l.bit)&1;else if(l.kind=='be16'){if(i+1>=d.length)return null;x=d[i]*256+d[i+1]}else if(l.kind=='le16'){if(i+1>=d.length)return null;x=d[i]+d[i+1]*256}else x=d[i];return x*(l.scale||1)+(l.off||0)}
function renderLbl(){$('#lblT').innerHTML=LBL.length?LBL.map((l,j)=>{const v=lblVal(l);return `<tr><td>${esc(l.name)}</td><td class="mono mut">${esc(l.src)} ${l.kind=='bit'?'B'+l.i+'.'+l.bit:kindT(l.kind,l.i)}</td><td class="mono">${v==null?'-':(+v).toFixed(l.kind=='bit'?0:2)}</td><td><button class="btn sm" onclick="LBL.splice(${j},1);saveLbl()">&times;</button></td></tr>`}).join(''):'<tr><td colspan=4 class="mut">Nothing named yet.</td></tr>';
 const rows=LBL.map(l=>{const v=lblVal(l);return `<tr><td>${esc(l.name)} <span class="tag ok">mine</span></td><td class="mono">${v==null?'-':(+v).toFixed(l.kind=='bit'?0:2)}</td><td class="mut">${esc(l.src)} ${l.kind=='bit'?'B'+l.i+'.'+l.bit:kindT(l.kind,l.i)}</td></tr>`}).join('');
 if(rows&&S.tab=='sens')$('#bcT').insertAdjacentHTML('beforeend',rows)}
$('#bExp').onclick=()=>dl('duke390_signals.json',JSON.stringify(LBL,null,1));
$('#bClrL').onclick=()=>{if(confirm('Delete all named signals?')){LBL=[];saveLbl()}};
/* ---------- loops ---------- */
async function loopState(){const r=await J('/api/state');if(r)renderState(r);if(S.tab=='find')renderCorr();renderLbl();setTimeout(loopState,500)}
async function loopIds(){if(S.tab=='ids'){const r=await J('/api/ids');if(r)renderIds(r)}setTimeout(loopIds,700)}
async function loopSens(){if(S.tab=='sens'||S.tab=='find'||S.tab=='abs'||!S.sens){const r=await J('/api/sensors');if(r)renderSens(r)}setTimeout(loopSens,1000)}
async function loopFrames(){if(S.tab=='raw'||S.rec){await pollFrames()}else{S.since=null}setTimeout(loopFrames,S.rec?150:S.tab=='raw'?250:1000)}
setInterval(()=>{const on=Date.now()-S.ok<4000;const p=$('#pConn');p.textContent=on?'connected':'disconnected';p.style.color=on?'var(--ok)':'var(--crit)'},1000);
loopState();loopIds();loopSens();loopFrames();
</script></body></html>)HTML";