// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 TheRepairForge
// Host decode tests: the REAL read path (readAllData -> sendCommand -> OneWire) against a
// scripted pack (fakepack.h), then the REAL verdict (computeVerdict, findHardwareFault).
// Run: python tools/hostsim/hostsim.py --test     (exit code 0 = every case passed)
//
// Pack values are illustrative, chosen to sit on the edges the cases test; they are not
// captures. Commands are the ones in protocol.cpp. A case named after an issue reproduces
// the behaviour that issue reported, so a regression shows up here before a tester sees it.
// Guarded whole: PlatformIO builds every .cpp under src_dir = . (D14), this one included.
#ifdef POCKETOBI_HOSTTEST
#include "../../pocketobi.h"
#include "../../decode.h"
#include "../../unlock.h"
#include "../../protocol.h"
#include "../../bridge.h"
#include "fakepack.h"
#include "shim_gpio.h"
#include <cmath>

static int g_fail = 0, g_checks = 0;
static const char *g_case = "";
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fail++; \
  printf("  FAIL %s: %s (line %d)\n", g_case, #cond, __LINE__); } } while (0)
static bool near(float a, float b) { return fabsf(a - b) < 0.005f; }

// ---- Pack building blocks (bus answers, in the firmware's own frame layouts) ----
static const char *K_ROM = "33", *K_MSG = "33 AA 00", *K_MODEL = "CC DC 0C", *K_D7 = "CC D7 00 00 FF";
static const char *K_CLEAR = "CC F0 00", *K_F0513_ID = "31", *K_F0513_T = "CC 52";
static const char *K_VCELL[5] = { "CC 31", "CC 32", "CC 33", "CC 34", "CC 35" };

static Bytes rom() { return Bytes{ 21, 6, 14, 0x13, 0x00, 0x00, 0x00, 0x9A }; }
// A static frame with consistent checksums (no lock), 5 Ah, `charges` cycles.
static Bytes msgFrame(uint16_t charges, bool chargerLock) {
  uint8_t f[32] = {0}, out[32];
  f[16] = 0x05;                                   // capacity, whole-Ah encoding
  f[26] = nibbleSwap((charges >> 8) & 0x0F); f[27] = nibbleSwap(charges & 0xFF);
  buildRepairedFrame(f, out);                     // checksums right, lock nybble clear
  if (chargerLock) nybSet(out, 34, 3);
  return Bytes(out, out + 32);
}
static uint16_t dK(float c) { return (uint16_t)lroundf((c + 273.15f) * 10); }   // 1/10 K
// The 29-byte D7 live answer: pack mV, five cells mV, two temperatures.
static Bytes d7(const float *cells, float t1 = 22.0f, float t2 = 23.0f) {
  Bytes b(29, 0);
  float sum = 0; for (int i = 0; i < 5; i++) sum += cells[i];
  auto put = [&](int at, uint16_t v) { b[at] = v & 0xFF; b[at + 1] = v >> 8; };
  put(0, (uint16_t)lroundf(sum * 1000));
  for (int i = 0; i < 5; i++) put(2 + 2 * i, (uint16_t)lroundf(cells[i] * 1000));
  put(14, dK(t1)); put(16, dK(t2));
  return b;
}
static Bytes mv(float v) { return le16b((uint16_t)lroundf(v * 1000)); }
static void f0513Cells(const float *c) { for (int i = 0; i < 5; i++) fakepackSet(K_VCELL[i], { mv(c[i]) }); }

static void standardPack(const char *model, bool chargerLock = false) {
  fakepackSet(K_ROM, { rom() });
  fakepackSet(K_MSG, { msgFrame(127, chargerLock) });
  if (model) fakepackSet(K_MODEL, { ascii(model, 16) });   // nullptr: MODEL stays silent
}
static void f0513Pack(uint8_t lo, uint8_t hi) {       // model "BL<hi><lo>" from the 0x31 raw read
  fakepackSet(K_F0513_ID, { Bytes{ lo, hi } });
  fakepackSet(K_F0513_T, { le16b(2972) });
}
static void begin(const char *name) {
  g_case = name; printf("- %s\n", name);
  fakepackClear(); bat = BatteryData(); cellCount = 5;
}

// ---- Cases ----
static void caseHealthy() {
  begin("standard BL1850B, healthy");
  standardPack("BL1850B");
  const float c[5] = { 3.95f, 3.96f, 3.94f, 3.95f, 3.96f };
  fakepackSet(K_D7, { d7(c) });
  CHECK(readAllData());
  CHECK(strcmp(bat.model, "BL1850B") == 0);
  CHECK(cellCount == 5 && !packReadOnly());
  CHECK(near(bat.cell[2], 3.94f) && fabsf(bat.tempCell - 22.0f) < 0.1f);
  CHECK(bat.chargeCount == 127 && near(bat.capacityAh, 5.0f));
  int g; CHECK(findHardwareFault(&g, nullptr, 0) == HW_NONE);
  CHECK(computeVerdict() == V_HEALTHY);
}
static void caseChargerLocked() {
  begin("standard BL1850B, charger lock nybble set");
  standardPack("BL1850B", true);
  const float c[5] = { 3.80f, 3.81f, 3.80f, 3.79f, 3.80f };
  fakepackSet(K_D7, { d7(c) });
  CHECK(readAllData());
  CHECK(bat.chargerLocked && (lockCauses(bat.msg) & LF_N34));
  CHECK(computeVerdict() == V_REPAIRABLE);
}
static void caseIntermittentFrame() {
  begin("old pack: static frame silent twice, then answers (retry)");
  fakepackSet(K_ROM, { rom() });
  fakepackSet(K_MSG, { silent(32), silent(32), msgFrame(362, false) });
  const float c[5] = { 3.70f, 3.71f, 3.70f, 3.69f, 3.70f };
  fakepackSet(K_D7, { d7(c) });
  CHECK(readAllData());
  CHECK(strcmp(bat.model, "LXT ?") == 0 && strcmp(bat.commandVersion, "") == 0);
  CHECK(fakepackCount(K_MSG) == 3);
  CHECK(computeVerdict() == V_HEALTHY);
}
static void caseD7Dropout() {
  begin("D7 answer trending to 0xFF is no read, never a verdict (#54)");
  standardPack("BL1850B");
  Bytes bad(29, 0xFF); bad[2] = 0x10; bad[3] = 0x0F;     // one cell plausible, the rest 0xFFFF
  fakepackSet(K_D7, { bad });
  CHECK(!readAllData());
  CHECK(computeVerdict() == V_UNKNOWN);
}
static void caseSenseWire() {
  begin("standard pack, cell 3 at 0 V on every read: sense-wire fault");
  standardPack("BL1850B");
  const float c[5] = { 3.90f, 3.91f, 0.00f, 3.90f, 3.92f };
  fakepackSet(K_D7, { d7(c) });
  CHECK(readAllData());
  int g; CHECK(findHardwareFault(&g, nullptr, 0) == HW_SENSE_WIRE && g == 3);
}
static void case43Bl1440() {
  begin("#43 BL1440: silent model, D7 silent, fifth F0513 reading 6.5 V -> 4S read-only");
  standardPack(nullptr);                                  // MODEL silent -> "LXT ?"
  const float c[5] = { 3.70f, 3.71f, 3.69f, 3.70f, 6.50f };
  f0513Cells(c); fakepackSet(K_F0513_T, { le16b(2972) });
  CHECK(readAllData());
  CHECK(cellCount == 4 && packReadOnly());
  CHECK(near(bat.packVoltage, 14.80f));
  CHECK(fakepackCount(K_CLEAR) == 0);                     // CLEARs only on an F0513 pack
  CHECK(computeVerdict() == V_HEALTHY);
}
static void caseIdentified18vHighCell5() {
  begin("identified BL1830B, D7 silent, fifth reading 7.4 V -> rejected read, not 4S (2.9.1)");
  standardPack("BL1830B");   // the ASCII test needs 7 printable bytes
  const float c[5] = { 3.60f, 3.61f, 3.60f, 3.59f, 7.40f };
  f0513Cells(c); fakepackSet(K_F0513_T, { le16b(2972) });
  CHECK(!readAllData());
  CHECK(cellCount == 5 && !packReadOnly());
  CHECK(computeVerdict() == V_UNKNOWN);
}
static void case86MissedAnswer() {
  begin("#86 F0513 BL1830: C1 and C2 answer 00 00 once -> read again, no sense-wire verdict");
  f0513Pack(0x30, 0x18);
  const float c[5] = { 3.60f, 3.61f, 3.62f, 3.60f, 3.61f };
  f0513Cells(c);
  fakepackSet(K_VCELL[0], { le16b(0), mv(3.60f) });
  fakepackSet(K_VCELL[1], { le16b(0), mv(3.61f) });
  CHECK(readAllData());
  CHECK(strcmp(bat.model, "BL1830") == 0 && strcmp(bat.commandVersion, "F0513") == 0);
  CHECK(near(bat.cell[0], 3.60f) && near(bat.cell[1], 3.61f));
  CHECK(fakepackCount(K_VCELL[0]) == 2 && fakepackCount(K_VCELL[2]) == 1);
  CHECK(fakepackCount(K_CLEAR) == 3);                     // one after the id read, two before the cells
  int g; CHECK(findHardwareFault(&g, nullptr, 0) == HW_NONE);
  CHECK(computeVerdict() == V_HEALTHY);
}
static void case86RealZero() {
  begin("#86 F0513: C1 at 0 V on every read keeps its fault verdict");
  f0513Pack(0x30, 0x18);
  const float c[5] = { 0.00f, 3.61f, 3.62f, 3.60f, 3.61f };
  f0513Cells(c);
  CHECK(readAllData());
  CHECK(near(bat.cell[0], 0.0f));
  CHECK(fakepackCount(K_VCELL[0]) == 1 + F0513_ZERO_REREADS);
  CHECK(computeVerdict() == V_FAULT);
  int g; CHECK(findHardwareFault(&g, nullptr, 0) != HW_NONE && g == 1);   // names group 1, not 4 (#88)
}
static void case86HalfAnswer() {
  begin("#86 F0513: a re-read answered 00 FF is a no-read, never a verdict (L31)");
  f0513Pack(0x30, 0x18);
  const float c[5] = { 3.60f, 3.61f, 3.62f, 3.60f, 3.61f };
  f0513Cells(c);
  fakepackSet(K_VCELL[1], { le16b(0), Bytes{ 0x00, 0xFF } });
  CHECK(!readAllData());
  CHECK(computeVerdict() == V_UNKNOWN);
}
static void case86AllZero() {
  begin("#86 F0513: every cell 0 V -> no re-reads, no read");
  f0513Pack(0x30, 0x18);
  const float c[5] = { 0, 0, 0, 0, 0 };
  f0513Cells(c);
  CHECK(!readAllData());
  CHECK(fakepackCount(K_VCELL[0]) == 1 && fakepackCount(K_VCELL[4]) == 1);
}
// #41: some older packs switch their output on and off while ENABLE is held low for more than
// ~50 ms. After an operation the screen is drawn (~200 ms, loop() does not run), so the release
// must not wait for loop(). delay(250) stands for that screen.
static void case41ReleasedDuringScreen() {
  begin("#41: ENABLE released after a read, while the screen is drawn");
  standardPack("BL1850B");
  const float c[5] = { 3.95f, 3.96f, 3.94f, 3.95f, 3.96f };
  fakepackSet(K_D7, { d7(c) });
  pinMode(ENABLE_PIN, OUTPUT); digitalWrite(ENABLE_PIN, LOW);   // as setup() leaves it
  shimPinReset();
  CHECK(readAllData());
  delay(250);
  CHECK(!shimPinDriven(ENABLE_PIN));
  CHECK(shimPinMaxLowUs(ENABLE_PIN) <= 40000);
}
// The bus power-cycle after Reset error is a deliberate 100 ms low: it must stay whole, and the
// line must still be released once it is over.
static void case41PowerCycleKept() {
  begin("#41: Reset error keeps its 100 ms power-cycle low, then releases ENABLE");
  standardPack("BL1850B");
  pinMode(ENABLE_PIN, OUTPUT); digitalWrite(ENABLE_PIN, LOW);   // as setup() leaves it
  shimPinReset();
  resetErrors();
  CHECK(shimPinReleases(ENABLE_PIN) == 0);      // never released between its commands (30 ms gaps)
  delay(250);
  CHECK(!shimPinDriven(ENABLE_PIN));
  CHECK(shimPinReleases(ENABLE_PIN) == 1);
  uint32_t lo = shimPinMaxLowUs(ENABLE_PIN);
  CHECK(lo >= 100000 && lo <= 110000);
}

// An unlock draws its progress between the store and the bus power-cycle that commits it. That
// redraw must not let the idle timer release ENABLE inside the operation (the line stayed low
// from the store through the power-cycle in 2.9.0). delay(10) stands for the redraw.
static void progressRedraw(int, int) { delay(10); }
static void case41UnlockHeld() {
  begin("#41: an unlock never releases ENABLE between its store and its power-cycle");
  standardPack("BL1850B", true);
  const float c[5] = { 3.95f, 3.96f, 3.94f, 3.95f, 3.96f };
  fakepackSet(K_D7, { d7(c) });
  CHECK(readAllData());
  shimPinReset();
  unlockRepair(progressRedraw);
  CHECK(shimPinReleases(ENABLE_PIN) == 0);
  delay(250);
  CHECK(!shimPinDriven(ENABLE_PIN));
}
// At boot the splash holds the main loop for 1.5 s: ENABLE must not sit low through it.
static void case41BootReleased() {
  begin("#41: ENABLE is released during the boot splash");
  shimPinReset();
  enableBoot();
  delay(1500);                                  // drawSplash() + delay(1500) in setup()
  CHECK(!shimPinDriven(ENABLE_PIN));
  CHECK(shimPinMaxLowUs(ENABLE_PIN) <= 40000);
}

// PC bridge (#88): each PC frame is one transaction. PackScope's unlock sends the store, then
// TESTMODE; the low between them is what commits the frame over the bridge (2.9.0 held it the
// whole gap). A 90 ms PC gap must stay one unbroken low; a PC that stops talking gets a release.
static void bridgeFrame(const uint8_t *f, size_t n) { Serial.shimFeed(f, n); serviceBridge(); }
static void case88BridgeGapHeld() {
  begin("#88: bridge, the low between the store and the next PC frame is not cut");
  standardPack("BL1850B");
  static const uint8_t STORE[] = { 0x01, 0x02, 0x08, 0x33, 0x55, 0xA5 };
  static const uint8_t TESTMODE[] = { 0x01, 0x03, 0x09, 0x33, 0xD9, 0x96, 0xA5 };
  bridgeFrame(STORE, sizeof(STORE));
  shimPinReset();
  delay(90);                                    // USB + PackScope between two frames
  bridgeFrame(TESTMODE, sizeof(TESTMODE));
  CHECK(shimPinReleases(ENABLE_PIN) == 0);
  CHECK(shimPinMaxLowUs(ENABLE_PIN) >= 90000);
  delay(250);                                   // the PC stops talking
  CHECK(!shimPinDriven(ENABLE_PIN));
}

static void caseNoPack() {
  begin("no pack on the bus");
  CHECK(!readAllData());
  CHECK(!bat.valid && computeVerdict() == V_UNKNOWN);
}

int main() {
  printf("PocketOBI host decode tests (fakepack)\n");
  caseHealthy(); caseChargerLocked(); caseIntermittentFrame(); caseD7Dropout(); caseSenseWire();
  case43Bl1440(); caseIdentified18vHighCell5();
  case86MissedAnswer(); case86RealZero(); case86HalfAnswer(); case86AllZero();
  case41ReleasedDuringScreen(); case41PowerCycleKept(); case41UnlockHeld(); case41BootReleased(); case88BridgeGapHeld();
  caseNoPack();
  printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
#endif // POCKETOBI_HOSTTEST
