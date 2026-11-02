// W10 ISM Waterfall - SX1262 RSSI sweep -> ST7789 240x240 waterfall
// https://github.com/anubisdarkwatch  (Anubis Darkwatch)
// Meshnology W10 (ESP32-S3). Receive-only. BOOT button cycles span.
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <algorithm>
#include <initializer_list>

// ---- Pins (from Meshnology W10 demos) ----
#define I2C_SDA 8
#define I2C_SCL 7
#define MCP_ADDR 0x20
#define SPI_SCK 12
#define SPI_MISO 11
#define SPI_MOSI 13
#define LORA_CS 14
#define LCD_CS 10
#define LCD_DC 16
#define LCD_BL 6
#define BTN 0
// MCP23017 bits: GPA1 = LCD reset, GPA3 = LoRa NRST, GPB2 = LoRa BUSY

// ---- Layout ----
#define W 240
#define HDR_H 30
#define WF_Y 30
#define WF_H 154
#define SP_Y 184
#define SP_H 56
#define RANGE_DB 30.0f

SPISettings lcdSPI(40000000, MSBFIRST, SPI_MODE0);
SPISettings loraSPI(8000000, MSBFIRST, SPI_MODE0);

struct Span { const char* name; uint32_t lo, hi; };
Span spans[] = {
  {"ISM 902-928", 902000000UL, 928000000UL},
  {"MESH US 904-910", 904000000UL, 910000000UL},
  {"LoRaWAN 902-915", 902000000UL, 915000000UL},
};
const int NSPAN = sizeof(spans) / sizeof(spans[0]);
int mode = 0;
uint32_t curLo = 902000000UL, curHi = 928000000UL;
bool custom = false;
#define IMU_ADDR 0x6B
bool imuOk = false;
float pitch0 = 0, roll0 = 0;

uint16_t* fb = nullptr;          // waterfall framebuffer, big-endian RGB565
uint16_t swapBuf[W * SP_H];      // byte-swap buffer for pushing canvases
uint16_t palBE[256], palN[256];  // waterfall palette (big-endian for fb, native for canvas)
int16_t rssiRow[W];
float smoothRow[W];
float peakHold[W];
float floorDb = -115.0f;
uint32_t wfRow = 0;
GFXcanvas16 hdr(W, HDR_H);
GFXcanvas16 spc(W, SP_H);
uint32_t dwellUs = 300;

// ================= MCP23017 =================
void mcpW(uint8_t r, uint8_t v) { Wire.beginTransmission(MCP_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
int mcpR(uint8_t r) {
  Wire.beginTransmission(MCP_ADDR); Wire.write(r);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)MCP_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}
bool loraBusy() { int v = mcpR(0x13); return v < 0 ? true : (v & 0x04); }
bool waitBusy(uint32_t ms = 500) {
  uint32_t t = millis();
  while (loraBusy()) { if (millis() - t > ms) return false; delayMicroseconds(20); }
  return true;
}

// ================= SX1262 =================
void loraXfer(const uint8_t* d, int n, uint8_t* rx = nullptr) {
  SPI.beginTransaction(loraSPI);
  digitalWrite(LORA_CS, LOW);
  for (int i = 0; i < n; i++) { uint8_t r = SPI.transfer(d[i]); if (rx) rx[i] = r; }
  digitalWrite(LORA_CS, HIGH);
  SPI.endTransaction();
}
void cmd(std::initializer_list<uint8_t> l) { uint8_t b[16]; int n = 0; for (auto x : l) b[n++] = x; loraXfer(b, n); }
uint8_t getStatus() { uint8_t t[2] = {0xC0, 0}, r[2]; loraXfer(t, 2, r); return r[1]; }
uint16_t devErrors() { uint8_t t[4] = {0x17, 0, 0, 0}, r[4]; loraXfer(t, 4, r); return (r[2] << 8) | r[3]; }
uint8_t readReg(uint16_t a) { uint8_t t[5] = {0x1D, (uint8_t)(a >> 8), (uint8_t)a, 0, 0}, r[5]; loraXfer(t, 5, r); return r[4]; }
void writeReg(uint16_t a, uint8_t v) { cmd({0x0D, (uint8_t)(a >> 8), (uint8_t)a, v}); }
int rssiInst() { uint8_t t[3] = {0x15, 0, 0}, r[3]; loraXfer(t, 3, r); return -(int)r[2] / 2; }
void setFreq(uint32_t hz) {
  uint32_t f = (uint32_t)(((uint64_t)hz << 25) / 32000000ULL);
  cmd({0x86, (uint8_t)(f >> 24), (uint8_t)(f >> 16), (uint8_t)(f >> 8), (uint8_t)f});
}

bool loraInit() {
  int iodira = mcpR(0x00), olata = mcpR(0x14);
  if (iodira < 0 || olata < 0) { Serial.println("MCP23017 not responding"); return false; }
  mcpW(0x00, iodira & ~0x08);           // GPA3 output
  mcpW(0x14, olata & ~0x08); delay(10); // NRST low
  mcpW(0x14, olata | 0x08);  delay(50); // NRST high
  if (!waitBusy(1000)) Serial.println("BUSY never went low after reset (continuing)");

  cmd({0x80, 0x00}); waitBusy();        // STDBY_RC
  Serial.printf("SX1262 status after reset: 0x%02X\n", getStatus());
  Serial.printf("LoRa sync word regs 0x0740/41: 0x%02X 0x%02X (expect 0x14 0x24)\n", readReg(0x0740), readReg(0x0741));

  cmd({0x96, 0x01}); waitBusy();        // DC-DC regulator
  cmd({0x07, 0x00, 0x00}); waitBusy();  // clear device errors
  cmd({0x80, 0x01}); delay(5); waitBusy(); // try STDBY_XOSC on plain crystal
  uint16_t e = devErrors();
  Serial.printf("Device errors after XOSC start: 0x%04X\n", e);
  if (e & 0x0020) {
    Serial.println("XOSC start error -> enabling TCXO on DIO3 (1.8V)");
    cmd({0x07, 0x00, 0x00}); waitBusy();
    cmd({0x80, 0x00}); waitBusy();
    cmd({0x97, 0x02, 0x00, 0x01, 0x40}); waitBusy(); // TCXO 1.8V, 5ms
    cmd({0x89, 0x7F}); delay(10); waitBusy();        // calibrate all
    Serial.printf("Device errors after TCXO: 0x%04X\n", devErrors());
  }
  cmd({0x80, 0x00}); waitBusy();
  cmd({0x9D, 0x01}); waitBusy();        // DIO2 drives RF switch (low = RX path)
  cmd({0x8A, 0x01}); waitBusy();        // packet type LoRa
  cmd({0x98, 0xE1, 0xE9}); delay(5); waitBusy(); // image calibration 902-928
  cmd({0x8B, 0x07, 0x04, 0x01, 0x00}); waitBusy(); // SF7, BW125, CR4/5
  writeReg(0x08AC, 0x96);               // boosted RX gain
  cmd({0x80, 0x01}); waitBusy();        // STDBY_XOSC for fast hops
  uint8_t st = getStatus();
  Serial.printf("SX1262 ready, status 0x%02X (chip mode %d)\n", st, (st >> 4) & 7);
  return true;
}

int measure(uint32_t hz) {
  cmd({0x80, 0x01}); delayMicroseconds(40);  // standby xosc
  setFreq(hz);       delayMicroseconds(40);
  cmd({0x82, 0xFF, 0xFF, 0xFF});               // RX continuous
  delayMicroseconds(dwellUs);
  int best = -200;
  for (int k = 0; k < 3; k++) { int r = rssiInst(); if (r > best) best = r; delayMicroseconds(40); }
  return best;
}

void radioDiag() {
  Serial.println("--- radio diag @ 906.875 MHz ---");
  cmd({0x80, 0x01}); waitBusy();
  setFreq(906875000UL); waitBusy();
  cmd({0x82, 0xFF, 0xFF, 0xFF}); waitBusy();
  delay(5);
  uint8_t st = getStatus();
  Serial.printf("status 0x%02X chip mode %d (5 = RX), errors 0x%04X\n", st, (st >> 4) & 7, devErrors());
  for (int i = 0; i < 5; i++) { Serial.printf("  RSSI %d dBm\n", rssiInst()); delay(20); }
  cmd({0x80, 0x01}); waitBusy();
  // auto-calibrate dwell: shortest time after RX start that gives a real RSSI
  const uint32_t tries[] = {300, 500, 800, 1200, 1800, 2500, 4000};
  dwellUs = 4000;
  for (uint32_t d : tries) {
    dwellUs = d;
    int worst = 0;
    for (int k = 0; k < 6; k++) { int r = measure(906875000UL + k * 1000000UL); if (r < worst) worst = r; }
    Serial.printf("  dwell %4lu us -> worst RSSI %d dBm\n", (unsigned long)d, worst);
    if (worst > -126) break;
  }
  Serial.printf("Using dwell %lu us\n", (unsigned long)dwellUs);
}

// ================= ST7789 =================
void lcdBegin() { SPI.beginTransaction(lcdSPI); digitalWrite(LCD_CS, LOW); }
void lcdEnd() { digitalWrite(LCD_CS, HIGH); SPI.endTransaction(); }
void lcdCmd(uint8_t c) { digitalWrite(LCD_DC, LOW); SPI.transfer(c); digitalWrite(LCD_DC, HIGH); }
void lcdWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  lcdCmd(0x2A); SPI.transfer16(x0); SPI.transfer16(x1);
  lcdCmd(0x2B); SPI.transfer16(y0); SPI.transfer16(y1);
  lcdCmd(0x2C);
}
void pushRectBE(int x, int y, int w, int h, uint16_t* px) { // px already big-endian
  lcdBegin(); lcdWindow(x, y, x + w - 1, y + h - 1);
  SPI.writeBytes((uint8_t*)px, (uint32_t)w * h * 2);
  lcdEnd();
}
void lcdInit() {
  int iodira = mcpR(0x00), olata = mcpR(0x14);
  mcpW(0x00, iodira & ~0x02);           // GPA1 output (LCD reset)
  mcpW(0x14, olata | 0x02); delay(10);
  mcpW(0x14, olata & ~0x02); delay(10);
  mcpW(0x14, olata | 0x02); delay(120);
  lcdBegin(); lcdCmd(0x01); lcdEnd(); delay(150);   // SWRESET
  lcdBegin(); lcdCmd(0x11); lcdEnd(); delay(120);   // SLPOUT
  lcdBegin();
  lcdCmd(0x3A); SPI.transfer(0x55);                 // 16-bit color
  lcdCmd(0x36); SPI.transfer(0x00);                 // MADCTL
  lcdCmd(0x21);                                     // inversion on (IPS)
  lcdCmd(0x13);                                     // normal mode
  lcdCmd(0x29);                                     // display on
  lcdEnd();
  delay(20);
}


// ================= IMU (QMI8658) =================
void imuW(uint8_t r, uint8_t v) { Wire.beginTransmission(IMU_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool imuRead(float& pitch, float& roll) {
  Wire.beginTransmission(IMU_ADDR); Wire.write(0x35);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)6) != 6) return false;
  int16_t ax = Wire.read() | (Wire.read() << 8);
  int16_t ay = Wire.read() | (Wire.read() << 8);
  int16_t az = Wire.read() | (Wire.read() << 8);
  float gx = ax / 8192.0f, gy = ay / 8192.0f, gz = az / 8192.0f;
  pitch = atan2f(-gx, sqrtf(gy * gy + gz * gz)) * 57.2958f;
  roll = atan2f(gy, gz) * 57.2958f;
  return true;
}
void imuInit() {
  imuW(0x02, 0x40); imuW(0x03, 0x15); imuW(0x08, 0x01); delay(50);
  float p, r, sp = 0, sr = 0; int n = 0;
  for (int i = 0; i < 10; i++) { if (imuRead(p, r)) { sp += p; sr += r; n++; } delay(10); }
  imuOk = n > 0;
  if (imuOk) { pitch0 = sp / n; roll0 = sr / n; }
  Serial.printf("IMU %s, neutral pitch %.0f roll %.0f\n", imuOk ? "ok" : "MISSING", pitch0, roll0);
}
void applyPreset() { curLo = spans[mode].lo; curHi = spans[mode].hi; custom = false; }
// tilt: roll pans, pitch zooms (relative to how the board sat at boot)
bool tiltControl(uint32_t centerHint) {
  if (!imuOk) return false;
  float p, r; if (!imuRead(p, r)) return false;
  float dp = p - pitch0, dr = r - roll0;
  if (dr > 180) dr -= 360;
  if (dr < -180) dr += 360;
  int64_t lo = curLo, hi = curHi, span = hi - lo, c = (lo + hi) / 2;
  bool changed = false;
  if (fabsf(dp) > 30) {
    if (dp > 0 && span > 1000000) { span = span * 4 / 5; c = (c * 3 + (int64_t)centerHint) / 4; changed = true; }
    if (dp < 0 && span < 26000000) { span = span * 5 / 4; changed = true; }
  }
  if (fabsf(dr) > 30) { c += (dr > 0 ? 1 : -1) * span / 10; changed = true; }
  if (!changed) return false;
  if (span > 26000000) span = 26000000;
  if (span < 1000000) span = 1000000;
  lo = c - span / 2; hi = c + span / 2;
  if (lo < 902000000LL) { lo = 902000000LL; hi = lo + span; }
  if (hi > 928000000LL) { hi = 928000000LL; lo = hi - span; }
  curLo = lo; curHi = hi; custom = !(curLo == spans[mode].lo && curHi == spans[mode].hi);
  return true;
}

// ================= Audio (ES8311 codec + NS4150B amp) =================
#include "ESP_I2S.h"
#define ES_ADDR 0x18
#define AUD_RATE 16000
I2SClass i2s;
bool audioOk = false, soundOn = true;
uint32_t lastBeep = 0;
void esW(uint8_t r, uint8_t v) { Wire.beginTransmission(ES_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
void audioInit() {
  // speaker amp enable: MCP23017 GPA7 high (read-modify-write keeps LCD/LoRa reset lines)
  int iodira = mcpR(0x00), olata = mcpR(0x14);
  if (iodira >= 0 && olata >= 0) { mcpW(0x00, iodira & ~0x80); mcpW(0x14, olata | 0x80); }
  // ES8311: slave I2S, 16-bit, MCLK 4.096 MHz (256 x 16 kHz) from MCLK pin
  esW(0x00, 0x1F); delay(20); esW(0x00, 0x00); esW(0x00, 0x80);
  esW(0x01, 0x3F); esW(0x02, 0x00); esW(0x03, 0x10); esW(0x04, 0x10); esW(0x05, 0x00);
  esW(0x06, 0x03); esW(0x07, 0x00); esW(0x08, 0xFF);
  esW(0x09, 0x0C); esW(0x0A, 0x0C);
  esW(0x0D, 0x01); esW(0x0E, 0x02); esW(0x12, 0x00); esW(0x13, 0x00); // DAC on, line out to amp
  esW(0x1C, 0x6A); esW(0x37, 0x08);
  esW(0x32, 0xB2); esW(0x31, 0x00); // volume ~70%, unmute
  i2s.setPins(2, 4, 5, 3, 1);       // BCK, LRCK, DOUT, DIN, MCLK
  audioOk = i2s.begin(I2S_MODE_STD, AUD_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  Serial.printf("Audio %s\n", audioOk ? "ok" : "I2S init FAILED");
}
void tone_ms(uint16_t hz, uint16_t ms, int16_t amp = 9000) {
  if (!audioOk || !soundOn) return;
  const int N = 128; int16_t buf[N * 2];
  uint32_t total = (uint32_t)AUD_RATE * ms / 1000, fade = AUD_RATE / 200; // 5 ms fade in/out (no clicks)
  float ph = 0, dph = 2.0f * PI * hz / AUD_RATE;
  for (uint32_t done = 0; done < total; ) {
    int n = min((uint32_t)N, total - done);
    for (int i = 0; i < n; i++, done++) {
      float env = 1.0f;
      if (done < fade) env = (float)done / fade;
      else if (total - done < fade) env = (float)(total - done) / fade;
      int16_t v = (int16_t)(sinf(ph) * amp * env); ph += dph; if (ph > 2 * PI) ph -= 2 * PI;
      buf[i * 2] = v; buf[i * 2 + 1] = v;
    }
    i2s.write((uint8_t*)buf, n * 4);
  }
  memset(buf, 0, sizeof(buf));
  for (int k = 0; k < 6; k++) i2s.write((uint8_t*)buf, sizeof(buf)); // flush silence
}
// beep on signals: pitch rises with strength above the floor, rate-limited
void signalBeep(int pkDb, float floor_) {
  float over = pkDb - floor_;
  if (over < 15 || millis() - lastBeep < 250) return;
  lastBeep = millis();
  float t = (over - 15) / 45.0f; if (t > 1) t = 1;
  tone_ms((uint16_t)(600 + t * 1900), 40);
}


// ================= Graphics =================
uint16_t be(uint16_t c) { return (c >> 8) | (c << 8); }
uint16_t rgbRaw(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); } uint8_t g8(uint8_t v) { return (uint8_t)(powf(v / 255.0f, 1.8f) * 255.0f + 0.5f); } uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) { return rgbRaw(g8(r), g8(g), g8(b)); } // gamma: panel shows dark tones bright
#define C_BG     rgb(6, 9, 18)
#define C_PANEL  rgb(14, 20, 36)
#define C_GRID   rgb(28, 38, 60)
#define C_TEXT   rgb(150, 165, 190)
#define C_CYAN   rgb(0, 230, 210)
#define C_AMBER  rgb(255, 190, 40)
#define C_MESH   rgb(0, 200, 90)
#define C_RED    rgb(255, 50, 60)

void buildPal() { // inferno-style: black -> indigo -> magenta -> orange -> yellow -> white
  const uint8_t s[][3] = {{0,0,4},{20,11,60},{87,16,110},{160,42,99},{221,81,58},{250,153,21},{252,230,90},{255,255,255}};
  const int NS = 7;
  for (int i = 0; i < 256; i++) {
    float t = powf(i / 255.0f, 0.85f) * NS; int k = min(NS - 1, (int)t); float f = t - k;
    uint16_t c = rgbRaw(s[k][0] + (s[k+1][0] - s[k][0]) * f, s[k][1] + (s[k+1][1] - s[k][1]) * f, s[k][2] + (s[k+1][2] - s[k][2]) * f);
    palN[i] = c; palBE[i] = be(c);
  }
}
// level 0..255 relative to floor (+2 dB so plain noise stays black)
int lvl(float db) { int v = (int)((db - floorDb - 2.0f) * 255.0f / RANGE_DB); return v < 0 ? 0 : (v > 255 ? 255 : v); }
int freqX(float mhz) { float lo = curLo / 1e6f, hi = curHi / 1e6f; return (int)((mhz - lo) / (hi - lo) * (W - 1) + 0.5f); }
float gridStep() { float sp = (curHi - curLo) / 1e6f; return sp > 10 ? 2.0f : (sp > 4 ? 1.0f : (sp > 1.5f ? 0.5f : 0.1f)); }

void pushCanvas(GFXcanvas16& c, int y, int h) {
  uint16_t* src = c.getBuffer();
  for (int i = 0; i < W * h; i++) swapBuf[i] = be(src[i]);
  pushRectBE(0, y, W, h, swapBuf);
}

void drawHeader(int pkDb, uint32_t pkHz) {
  hdr.fillScreen(C_PANEL);
  hdr.setTextSize(1);
  hdr.setTextColor(C_CYAN); hdr.setCursor(3, 2); hdr.print(custom ? "TILT ZOOM" : spans[mode].name);
  char b[24];
  snprintf(b, sizeof(b), "%.3f MHz", pkHz / 1e6);
  hdr.setTextColor(C_AMBER); hdr.setCursor(3, 11); hdr.print(b);
  if (pkDb > -60) { hdr.fillRoundRect(78, 10, 22, 10, 3, C_RED); hdr.setTextColor(rgb(255,255,255)); hdr.setCursor(80, 11); hdr.print("OVL"); }
  // big peak readout on the right
  snprintf(b, sizeof(b), "%d", pkDb);
  uint16_t pc = pkDb > -60 ? C_RED : (pkDb > floorDb + 15 ? C_AMBER : C_TEXT);
  hdr.setTextSize(2); hdr.setTextColor(pc);
  int bw = strlen(b) * 12;
  hdr.setCursor(W - 22 - bw, 2); hdr.print(b);
  hdr.setTextSize(1); hdr.setTextColor(C_TEXT); hdr.setCursor(W - 20, 2); hdr.print("dBm");
  if (!soundOn) { hdr.setCursor(W - 20, 11); hdr.setTextColor(C_GRID); hdr.print("mute"); }
  // frequency axis
  hdr.fillRect(0, 20, W, 10, C_BG);
  float st = gridStep();
  float lo = curLo / 1e6f, hi = curHi / 1e6f;
  for (float f = ceilf(lo / st) * st; f <= hi + 0.001f; f += st) {
    int x = freqX(f);
    hdr.drawFastVLine(x, 26, 4, C_GRID);
  }
  hdr.setTextColor(C_TEXT);
  snprintf(b, sizeof(b), (hi - lo) > 10 ? "%.0f" : "%.1f", lo); hdr.setCursor(1, 21); hdr.print(b);
  snprintf(b, sizeof(b), "%.1f", (lo + hi) / 2); hdr.setCursor(W / 2 - strlen(b) * 3, 21); hdr.print(b);
  snprintf(b, sizeof(b), (hi - lo) > 10 ? "%.0f" : "%.1f", hi); hdr.setCursor(W - 1 - strlen(b) * 6, 21); hdr.print(b);
  if (906.875f >= lo && 906.875f <= hi) { int x = freqX(906.875f); hdr.fillTriangle(x - 3, 25, x + 3, 25, x, 29, C_MESH); }
  pushCanvas(hdr, 0, HDR_H);
}

// one new waterfall line: smooth interpolation, noise -> black, faint dotted gridlines
void waterfallLine(const int16_t* binsDb, int nb) {
  memmove(fb + W, fb, (WF_H - 1) * W * 2);
  for (int x = 0; x < W; x++) {
    float pos = (float)x * (nb - 1) / (W - 1);
    int i0 = (int)pos; int i1 = min(nb - 1, i0 + 1); float f = pos - i0;
    float db = binsDb[i0] * (1 - f) + binsDb[i1] * f;
    rssiRow[x] = (int16_t)db;
    fb[x] = palBE[lvl(db)];
  }
  if (wfRow % 3 == 0) {
    float st = gridStep(), lo = curLo / 1e6f, hi = curHi / 1e6f;
    for (float fq = ceilf(lo / st) * st; fq <= hi + 0.001f; fq += st) { int x = freqX(fq); if (fb[x] == palBE[0]) fb[x] = be(C_GRID); }
    if (906.875f >= lo && 906.875f <= hi) { int x = freqX(906.875f); if (fb[x] == palBE[0]) fb[x] = be(rgb(0, 90, 45)); }
  }
  wfRow++;
  pushRectBE(0, WF_Y, W, WF_H, fb);
}

// spectrum: dB grid, gradient fill, bright trace, peak-hold line, peak marker
void drawSpectrum(int pkX) {
  spc.fillScreen(C_BG);
  for (int d = 10; d < RANGE_DB + 10; d += 10) {
    int y = SP_H - 1 - (int)(d / (RANGE_DB + 8.0f) * (SP_H - 4));
    for (int x = 0; x < W; x += 4) spc.drawPixel(x, y, C_GRID);
  }
  if (906.875f >= curLo / 1e6f && 906.875f <= curHi / 1e6f) { int x = freqX(906.875f); for (int y = 0; y < SP_H; y += 3) spc.drawPixel(x, y, rgb(0, 90, 45)); }
  int prevY = -1;
  for (int x = 0; x < W; x++) {
    smoothRow[x] = smoothRow[x] * 0.45f + rssiRow[x] * 0.55f;
    float l = (smoothRow[x] - floorDb + 2) / (RANGE_DB + 8.0f); if (l < 0) l = 0; if (l > 1) l = 1;
    int h = (int)(l * (SP_H - 4));
    int yTop = SP_H - 1 - h;
    for (int y = yTop; y < SP_H; y++) {
      int li = (SP_H - 1 - y) * 255 / SP_H;            // gradient by height
      int dim = 70 + (y - yTop) * 140 / (h + 1);       // fade toward the bottom
      uint16_t c = palN[min(255, 40 + li)];
      uint8_t r = ((c >> 11) & 0x1F) << 3, g = ((c >> 5) & 0x3F) << 2, bb = (c & 0x1F) << 3;
      int k = 255 - dim; if (k < 40) k = 40;
      spc.drawPixel(x, y, rgbRaw(r * k / 255, g * k / 255, bb * k / 255));
    }
    if (prevY >= 0) spc.drawLine(x - 1, prevY, x, yTop, rgb(220, 240, 255)); else spc.drawPixel(x, yTop, rgb(220, 240, 255));
    prevY = yTop;
    peakHold[x] = max(peakHold[x] - 1.5f, (float)h);
    int py = SP_H - 1 - (int)peakHold[x];
    if (py >= 0 && py < yTop) spc.drawPixel(x, py, C_AMBER);
  }
  // peak marker
  if (pkX >= 0) {
    float l = (smoothRow[pkX] - floorDb + 2) / (RANGE_DB + 8.0f); if (l < 0) l = 0; if (l > 1) l = 1;
    int y = SP_H - 1 - (int)(l * (SP_H - 4)) - 3;
    if (y < 5) y = 5;
    spc.fillTriangle(pkX - 3, y - 5, pkX + 3, y - 5, pkX, y, C_AMBER);
  }
  pushCanvas(spc, SP_Y, SP_H);
}

void splash() {
  spc.fillScreen(C_BG);
  for (int x = 0; x < W; x++) fb[x] = 0;
  // full-screen splash using the waterfall buffer
  for (int y = 0; y < WF_H; y++) for (int x = 0; x < W; x++) {
    float d = fabsf(x - W / 2) / (W / 2.0f);
    int v = (int)(255 * expf(-d * d * 6) * (0.4f + 0.6f * sinf(y * 0.08f) * sinf(y * 0.08f)) * (y > WF_H / 2 ? 0.5f : 1));
    fb[y * W + x] = palBE[max(0, min(255, v / 2))];
  }
  pushRectBE(0, WF_Y, W, WF_H, fb);
  hdr.fillScreen(C_PANEL);
  hdr.setTextSize(2); hdr.setTextColor(C_CYAN); hdr.setCursor(18, 7); hdr.print("W10 // SCANNER");
  pushCanvas(hdr, 0, HDR_H);
  spc.setTextSize(1); spc.setTextColor(C_TEXT);
  spc.setCursor(46, 12); spc.print("902-928 MHz  ISM BAND");
  spc.setTextColor(C_AMBER); spc.setCursor(64, 30); spc.print("calibrating radio...");
  pushCanvas(spc, SP_Y, SP_H);
}

void clearAll() {
  memset(fb, 0, W * WF_H * 2);
  for (int i = 0; i < W; i++) { peakHold[i] = 0; smoothRow[i] = floorDb; }
  pushRectBE(0, WF_Y, W, WF_H, fb);
}

// ================= Main =================
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);
  Serial.println("\n=== W10 ISM Waterfall ===");

  pinMode(BTN, INPUT_PULLUP);
  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(LCD_CS, OUTPUT);  digitalWrite(LCD_CS, HIGH);
  pinMode(LCD_DC, OUTPUT);  digitalWrite(LCD_DC, HIGH);
  pinMode(LCD_BL, OUTPUT);  digitalWrite(LCD_BL, LOW);

  pinMode(I2C_SDA, INPUT_PULLUP); pinMode(I2C_SCL, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(10000);
  Wire.setTimeOut(1000);
  delay(100);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  fb = (uint16_t*)ps_malloc(W * WF_H * 2);
  if (!fb) fb = (uint16_t*)malloc(W * WF_H * 2);
  buildPal();

  lcdInit();
  splash();
  digitalWrite(LCD_BL, HIGH);
  Serial.println("Display up");

  if (!loraInit()) Serial.println("LoRa init FAILED");
  Wire.setClock(400000);
  radioDiag();
  imuInit();
  audioInit();
  tone_ms(880, 80); tone_ms(1320, 80);
  applyPreset();
  clearAll();
  Serial.println("Tilt: roll = pan, pitch = zoom. BOOT tap = next preset / exit zoom, BOOT hold 1s = sound on/off.");
}

uint32_t lastBtn = 0;
uint32_t rows = 0, lastStat = 0;
int16_t binDb[240];

void loop() {
  if (digitalRead(BTN) == LOW && millis() - lastBtn > 400) {
    uint32_t t0 = millis();
    while (digitalRead(BTN) == LOW && millis() - t0 < 1500) delay(10);
    lastBtn = millis();
    if (millis() - t0 >= 1000) {
      soundOn = !soundOn;
      if (soundOn) tone_ms(1320, 80);
      Serial.printf("Sound %s\n", soundOn ? "ON" : "OFF");
    } else {
      if (!custom) mode = (mode + 1) % NSPAN;
      applyPreset();
      clearAll();
      Serial.printf("Span -> %s\n", spans[mode].name);
    }
  }

  static bool reverse = false;
  reverse = !reverse;
  // wide view: 120 bins @ BW250 (fast); narrow view: 240 bins @ BW125 (detail)
  int bins = (curHi - curLo) > 10000000UL ? 120 : 240;
  static int lastBins = 0;
  if (bins != lastBins) {
    cmd({0x80, 0x01}); delayMicroseconds(60);
    cmd({0x8B, 0x07, (uint8_t)(bins == 120 ? 0x05 : 0x04), 0x01, 0x00}); delayMicroseconds(60);
    lastBins = bins;
    Serial.printf("Sweep mode: %d bins, BW %s kHz\n", bins, bins == 120 ? "250" : "125");
  }
  int pk = -200, pkB = 0;
  for (int n = 0; n < bins; n++) {
    int i = reverse ? (bins - 1 - n) : n;
    uint32_t hz = curLo + (uint32_t)((uint64_t)(curHi - curLo) * i / (bins - 1));
    int r = measure(hz);
    binDb[i] = r;
    if (r > pk) { pk = r; pkB = i; }
  }
  cmd({0x80, 0x01});

  // noise floor = 25th percentile of bins, smoothed
  int16_t tmp[240]; memcpy(tmp, binDb, bins * sizeof(int16_t));
  std::nth_element(tmp, tmp + bins / 4, tmp + bins);
  floorDb = floorDb * 0.9f + tmp[bins / 4] * 0.1f;

  waterfallLine(binDb, bins);
  int pkX = (int)((float)pkB * (W - 1) / (bins - 1) + 0.5f);
  drawSpectrum(pk > floorDb + 8 ? pkX : -1);
  uint32_t pkHz = curLo + (uint32_t)((uint64_t)(curHi - curLo) * pkB / (bins - 1));
  drawHeader(pk, pkHz);
  rows++;
  // tiltControl(pkHz);  // tilt zoom off: keep full view

  signalBeep(pk, floorDb);
  if (pk > floorDb + 12) Serial.printf("[burst] %d dBm @ %.3f MHz (floor %.1f)\n", pk, pkHz / 1e6, floorDb);
  if (millis() - lastStat > 5000) {
    lastStat = millis();
    Serial.printf("rows/s %.1f  floor %.1f dBm  peak %d dBm @ %.3f MHz  span %.2f-%.2f\n", rows / 5.0f, floorDb, pk, pkHz / 1e6, curLo / 1e6, curHi / 1e6);
    rows = 0;
  }
}
