/*
  USBMOUSE2PC98.ino

  USB HIDマウスをPC-9801用バスマウス信号へ変換するRP2040向けファームウェア。

  今回の前提:
    - 出力段は 74HCT07 / 74AHCT07 / 74HCT3G07 系の非反転オープンドレインバッファ。
    - RP2040 GPIO -> HCT07入力。
    - HCT07出力 -> PC-98バスマウス信号線。
    - HCT07側は5V動作。
    - PC-98側信号は+5Vプルアップ、またはHCT07出力側に外付けプルアップ。

  HCT07は非反転のオープンドレイン/オープンコレクタ系バッファなので、
    RP2040 GPIO LOW  -> HCT07出力LOW  -> PC-98側LOW
    RP2040 GPIO HIGH -> HCT07出力Hi-Z -> PC-98側HIGH、プルアップ任せ
  として扱う。

  Arduino IDE:
    - Earle Philhower版 arduino-pico
    - USB Stack = Adafruit TinyUSB Host
    - USBマウスはRP2040 Zero等の本体USB端子へOTG経由で接続
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "hardware/sync.h"

#if __has_include(<Adafruit_NeoPixel.h>)
#include <Adafruit_NeoPixel.h>
#define HAVE_NEOPIXEL 1
#else
#define HAVE_NEOPIXEL 0
#endif

// -----------------------------------------------------------------------------
// GPIO割り当て
// -----------------------------------------------------------------------------
// 指定条件:
//   L / R / YB は GPIO0〜8 側へ配置。
//   XA / XB / YA は GPIO14 / 15 / 26 / 27 / 28 / 29 側へ配置。
//
// 今回の具体割り当て:
//   L  = GPIO0
//   R  = GPIO1
//   YB = GPIO8
//   XA = GPIO14
//   XB = GPIO15
//   YA = GPIO26
//
// GPIO27/28/29は未使用。配線都合で使う場合は下の定数だけ変更する。

static constexpr uint8_t PIN_LB = 0;
static constexpr uint8_t PIN_RB = 1;
static constexpr uint8_t PIN_YB = 8;
static constexpr uint8_t PIN_XA = 14;
static constexpr uint8_t PIN_XB = 15;
static constexpr uint8_t PIN_YA = 26;
static constexpr uint8_t PIN_WS2812 = 16;  // RP2040 Zero上のRGB LED。不要なら未使用でよい。

// -----------------------------------------------------------------------------
// HCT07出力設定
// -----------------------------------------------------------------------------

// HCT07は非反転なので通常false。
static constexpr bool HCT07_OUTPUT_INVERT = false;

// 2相信号の初期位相。
static constexpr uint8_t PC98_QUAD_INITIAL_PHASE = 0;

// -----------------------------------------------------------------------------
// 検証用テストモード
// -----------------------------------------------------------------------------
// 通常動作は0。
// 出力段の波形だけを確認したい場合は、USBマウスを使わず以下のモードを試す。
//   0 = 通常動作。USBマウスを読み、PC-98バスマウス信号を出す。
//   1 = 待機状態固定。XA/XB/YA/YB=00、LEFT/RIGHT=HIGH。
//   2 = ボタン出力テスト。LEFT/RIGHTを順番にON/OFFする。
//   3 = X forward。XA/XBを 00->10->11->01->00 で出力。
//   4 = X reverse。XA/XBを 00->01->11->10->00 で出力。
//   5 = Y forward。YA/YBを 00->10->11->01->00 で出力。
//   6 = Y reverse。YA/YBを 00->01->11->10->00 で出力。
//   7 = 全信号ウォーキングテスト。各信号を1本ずつHIGHにする。
static constexpr uint8_t HCT07_TEST_MODE = 0;
static constexpr uint32_t HCT07_TEST_INTERVAL_MS = 500;

// -----------------------------------------------------------------------------
// マウス移動量とPC-98出力タイミングの調整
// -----------------------------------------------------------------------------

static constexpr bool INVERT_X = false;
static constexpr bool INVERT_Y = false;

// 速すぎる場合は 1/2, 1/4 のように落とす。
static constexpr int32_t MOTION_SCALE_NUM = 1;
static constexpr int32_t MOTION_SCALE_DEN = 4;

static constexpr int32_t ACCUM_LIMIT = 768;

// main_mouse.c と同じ時間配分。
static constexpr uint32_t REPORT_TIME_SINGLE_AXIS_US = 10000;
static constexpr uint32_t REPORT_TIME_DIAGONAL_AXIS_US = 5000;

static constexpr int32_t MOTION_BATCH_LIMIT = 127;
static constexpr uint32_t MIN_STEP_INTERVAL_US = 40;
static constexpr uint32_t MAX_STEP_INTERVAL_US = 10000;

// USB HIDレポートのボタンビット。
static constexpr uint8_t USB_BTN_LEFT = 0x01;
static constexpr uint8_t USB_BTN_RIGHT = 0x02;

// -----------------------------------------------------------------------------
// USBホスト
// -----------------------------------------------------------------------------

Adafruit_USBH_Host USBHost;

static constexpr uint8_t MAX_USB_ADDR = 8;
static constexpr uint8_t MAX_HID_INST = 4;
static bool g_hidIsMouse[MAX_USB_ADDR][MAX_HID_INST];
static bool g_hidHasReportId[MAX_USB_ADDR][MAX_HID_INST];
static volatile uint8_t g_mouseMountedCount = 0;

// 特殊なトラックボールで認識しない場合のみtrueを試す。
// 通常はfalse。キーボード一体型レシーバーの誤読防止のため。
static constexpr bool ACCEPT_UNKNOWN_HID_AS_MOUSE = false;

// -----------------------------------------------------------------------------
// 共有状態
// -----------------------------------------------------------------------------

static volatile int32_t g_pendingX = 0;
static volatile int32_t g_pendingY = 0;
static volatile int32_t g_fracX = 0;
static volatile int32_t g_fracY = 0;
static volatile uint8_t g_buttons = 0;
static volatile uint32_t g_reportCount = 0;
static volatile uint32_t g_stepCount = 0;

// 2相パルスの位相。
// 0 = A/BともLOW、1 = AのみHIGH、2 = A/BともHIGH、3 = BのみHIGH。
// forward: 0 -> 1 -> 2 -> 3 -> 0
// reverse: 0 -> 3 -> 2 -> 1 -> 0
static uint8_t g_phaseX = 0;
static uint8_t g_phaseY = 0;

// 現在出力中の1軸分の処理状態。
// axis: 0=待機、1=X軸、2=Y軸。dir: +1=forward、-1=reverse。
static uint8_t g_currentAxis = 0;
static int8_t g_currentDir = 0;
static int32_t g_currentRemain = 0;
static uint32_t g_currentIntervalUs = MAX_STEP_INTERVAL_US;
static uint32_t g_nextStepUs = 0;

// X軸出力後に続けて処理するY軸分。
static int32_t g_deferredY = 0;
static uint32_t g_deferredBaseUs = REPORT_TIME_SINGLE_AXIS_US;

#if HAVE_NEOPIXEL
static Adafruit_NeoPixel g_pixel(1, PIN_WS2812, NEO_GRB + NEO_KHZ800);
#endif

// -----------------------------------------------------------------------------
// 共通ユーティリティ
// -----------------------------------------------------------------------------

static inline int32_t clamp32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline bool timePassed(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

static void setPixel(uint8_t r, uint8_t g, uint8_t b) {
#if HAVE_NEOPIXEL
  g_pixel.setPixelColor(0, g_pixel.Color(r, g, b));
  g_pixel.show();
#else
  (void)r;
  (void)g;
  (void)b;
#endif
}

static void initStatusLED() {
#if HAVE_NEOPIXEL
  g_pixel.begin();
  g_pixel.setBrightness(24);
  setPixel(8, 0, 0);
#endif
}

static void updateStatusLED() {
  static uint32_t lastMs = 0;
  static uint32_t lastStepCount = 0;

  uint32_t nowMs = millis();
  if (nowMs - lastMs < 80) return;
  lastMs = nowMs;

  uint32_t currentSteps = g_stepCount;
  bool movedRecently = (currentSteps != lastStepCount);
  lastStepCount = currentSteps;

  if (movedRecently) {
    setPixel(0, 0, 24);  // 青: PC-98側へパルス出力中。
  } else if (g_mouseMountedCount != 0) {
    setPixel(0, 18, 0);  // 緑: USBマウス認識済み。
  } else {
    if ((nowMs / 400) & 1) setPixel(16, 0, 0);
    else setPixel(0, 0, 0);  // 赤点滅: USBマウス未接続。
  }
}

// -----------------------------------------------------------------------------
// PC-98バスマウス出力処理 HCT07版
// -----------------------------------------------------------------------------

static void writePc98Level(uint8_t pin, bool levelHigh) {
  // HCT07は非反転オープンドレインバッファ。
  // GPIO LOW  -> HCT07出力LOW。
  // GPIO HIGH -> HCT07出力Hi-Z、PC-98側はプルアップによりHIGH。
  bool outHigh = HCT07_OUTPUT_INVERT ? !levelHigh : levelHigh;
  digitalWrite(pin, outHigh ? HIGH : LOW);
}

static void applyQuadraturePhase(uint8_t pinA, uint8_t pinB, uint8_t phase) {
  // 0/1はPC-98側へ出る実レベル LOW/HIGH。
  switch (phase & 0x03) {
    case 0:
      writePc98Level(pinA, false);  // A=LOW
      writePc98Level(pinB, false);  // B=LOW
      break;
    case 1:
      writePc98Level(pinA, true);   // A=HIGH
      writePc98Level(pinB, false);  // B=LOW
      break;
    case 2:
      writePc98Level(pinA, true);  // A=HIGH
      writePc98Level(pinB, true);  // B=HIGH
      break;
    case 3:
      writePc98Level(pinA, false);  // A=LOW
      writePc98Level(pinB, true);   // B=HIGH
      break;
  }
}

static void setButtons(uint8_t buttons) {
  // LEFT/RIGHTは負論理。通常時HIGH、押下時LOW。
  writePc98Level(PIN_LB, (buttons & USB_BTN_LEFT) == 0);
  writePc98Level(PIN_RB, (buttons & USB_BTN_RIGHT) == 0);
}

static void setupPc98Outputs() {
  // 起動時の瞬間的なLOWを避けるため、先にHIGHを書いてから出力にする。
  digitalWrite(PIN_XA, HIGH);
  digitalWrite(PIN_XB, HIGH);
  digitalWrite(PIN_YA, HIGH);
  digitalWrite(PIN_YB, HIGH);
  digitalWrite(PIN_LB, HIGH);
  digitalWrite(PIN_RB, HIGH);

  pinMode(PIN_XA, OUTPUT);
  pinMode(PIN_XB, OUTPUT);
  pinMode(PIN_YA, OUTPUT);
  pinMode(PIN_YB, OUTPUT);
  pinMode(PIN_LB, OUTPUT);
  pinMode(PIN_RB, OUTPUT);

  // 参考ソースの初期LATBと同じ意味にする。
  // XA/XB/YA/YB = LOW、LEFT/RIGHT = HIGH。
  g_phaseX = PC98_QUAD_INITIAL_PHASE & 0x03;
  g_phaseY = PC98_QUAD_INITIAL_PHASE & 0x03;
  applyQuadraturePhase(PIN_XA, PIN_XB, g_phaseX);
  applyQuadraturePhase(PIN_YA, PIN_YB, g_phaseY);
  setButtons(0);
}

static void advanceX(int dir) {
  if (dir > 0) g_phaseX = (g_phaseX + 1) & 0x03;
  else g_phaseX = (g_phaseX + 3) & 0x03;
  applyQuadraturePhase(PIN_XA, PIN_XB, g_phaseX);
  g_stepCount++;
}

static void advanceY(int dir) {
  if (dir > 0) g_phaseY = (g_phaseY + 1) & 0x03;
  else g_phaseY = (g_phaseY + 3) & 0x03;
  applyQuadraturePhase(PIN_YA, PIN_YB, g_phaseY);
  g_stepCount++;
}

static uint32_t calcStepIntervalUs(uint32_t baseUs, int32_t count) {
  int32_t absCount = count < 0 ? -count : count;
  if (absCount <= 0) return MAX_STEP_INTERVAL_US;

  uint32_t interval = baseUs / static_cast<uint32_t>(absCount);
  if (interval < MIN_STEP_INTERVAL_US) interval = MIN_STEP_INTERVAL_US;
  if (interval > MAX_STEP_INTERVAL_US) interval = MAX_STEP_INTERVAL_US;
  return interval;
}

static bool startAxisOutput(uint8_t axis, int32_t count, uint32_t baseUs, uint32_t now) {
  if (count == 0) return false;

  g_currentAxis = axis;
  g_currentDir = (count > 0) ? 1 : -1;
  g_currentRemain = count > 0 ? count : -count;
  g_currentIntervalUs = calcStepIntervalUs(baseUs, count);
  g_nextStepUs = now;
  return true;
}

static bool startNextMotionBatch(uint32_t now) {
  if (g_deferredY != 0) {
    int32_t y = g_deferredY;
    uint32_t baseUs = g_deferredBaseUs;
    g_deferredY = 0;
    return startAxisOutput(2, y, baseUs, now);
  }

  int32_t x = 0;
  int32_t y = 0;

  uint32_t irq = save_and_disable_interrupts();

  if (g_pendingX != 0) {
    x = clamp32(g_pendingX, -MOTION_BATCH_LIMIT, MOTION_BATCH_LIMIT);
    g_pendingX -= x;
  }

  if (g_pendingY != 0) {
    y = clamp32(g_pendingY, -MOTION_BATCH_LIMIT, MOTION_BATCH_LIMIT);
    g_pendingY -= y;
  }

  restore_interrupts(irq);

  if (x == 0 && y == 0) return false;

  uint32_t baseUs = (x != 0 && y != 0) ? REPORT_TIME_DIAGONAL_AXIS_US : REPORT_TIME_SINGLE_AXIS_US;
  g_deferredY = y;
  g_deferredBaseUs = baseUs;

  if (x != 0) return startAxisOutput(1, x, baseUs, now);
  return startNextMotionBatch(now);
}

static void servicePc98MouseOutput() {
  setButtons(g_buttons);

  uint32_t now = micros();
  if (!timePassed(now, g_nextStepUs)) return;

  if (g_currentAxis == 0) {
    if (!startNextMotionBatch(now)) return;
  }

  if (g_currentAxis == 1) {
    advanceX(g_currentDir);
  } else if (g_currentAxis == 2) {
    advanceY(g_currentDir);
  }

  if (g_currentRemain > 0) g_currentRemain--;
  g_nextStepUs = now + g_currentIntervalUs;

  if (g_currentRemain <= 0) {
    g_currentAxis = 0;
  }
}

// -----------------------------------------------------------------------------
// HCT07検証用の単体出力テスト
// -----------------------------------------------------------------------------

static bool serviceHct07TestMode() {
  if (HCT07_TEST_MODE == 0) return false;

  static uint32_t lastMs = 0;
  static uint8_t state = 0;

  uint32_t nowMs = millis();
  if (nowMs - lastMs < HCT07_TEST_INTERVAL_MS) return true;
  lastMs = nowMs;

  switch (HCT07_TEST_MODE) {
    case 1:
      g_phaseX = PC98_QUAD_INITIAL_PHASE & 0x03;
      g_phaseY = PC98_QUAD_INITIAL_PHASE & 0x03;
      applyQuadraturePhase(PIN_XA, PIN_XB, g_phaseX);
      applyQuadraturePhase(PIN_YA, PIN_YB, g_phaseY);
      setButtons(0);
      break;

    case 2:
      switch (state & 0x03) {
        case 0: setButtons(0); break;
        case 1: setButtons(USB_BTN_LEFT); break;
        case 2: setButtons(USB_BTN_RIGHT); break;
        case 3: setButtons(USB_BTN_LEFT | USB_BTN_RIGHT); break;
      }
      state++;
      break;

    case 3:
      advanceX(+1);
      break;

    case 4:
      advanceX(-1);
      break;

    case 5:
      advanceY(+1);
      break;

    case 6:
      advanceY(-1);
      break;

    case 7:
      writePc98Level(PIN_XA, false);
      writePc98Level(PIN_XB, false);
      writePc98Level(PIN_YA, false);
      writePc98Level(PIN_YB, false);
      writePc98Level(PIN_LB, false);
      writePc98Level(PIN_RB, false);
      switch (state % 6) {
        case 0: writePc98Level(PIN_XA, true); break;
        case 1: writePc98Level(PIN_XB, true); break;
        case 2: writePc98Level(PIN_YA, true); break;
        case 3: writePc98Level(PIN_YB, true); break;
        case 4: writePc98Level(PIN_LB, true); break;
        case 5: writePc98Level(PIN_RB, true); break;
      }
      state++;
      break;

    default:
      break;
  }

  setPixel(16, 8, 0);  // 橙: HCT07テストモード動作中。
  return true;
}

// -----------------------------------------------------------------------------
// HIDマウスレポート処理
// -----------------------------------------------------------------------------

static bool hidDescContainsBytePair(uint8_t const* desc, uint16_t len, uint8_t a, uint8_t b) {
  if (!desc || len < 2) return false;
  for (uint16_t i = 0; i + 1 < len; ++i) {
    if (desc[i] == a && desc[i + 1] == b) return true;
  }
  return false;
}

static bool hidDescHasReportId(uint8_t const* desc, uint16_t len) {
  if (!desc || len < 2) return false;
  for (uint16_t i = 0; i + 1 < len; ++i) {
    if (desc[i] == 0x85) return true;
  }
  return false;
}

static bool hidDescLooksLikeMouse(uint8_t const* desc, uint16_t len) {
  if (!desc || len < 4) return false;

  bool genericDesktop = hidDescContainsBytePair(desc, len, 0x05, 0x01);
  bool usageMouse = hidDescContainsBytePair(desc, len, 0x09, 0x02);
  bool usageX = hidDescContainsBytePair(desc, len, 0x09, 0x30);
  bool usageY = hidDescContainsBytePair(desc, len, 0x09, 0x31);

  return genericDesktop && usageMouse && usageX && usageY;
}

static void accumulateMouseDelta(int8_t rawDx, int8_t rawDy, uint8_t buttons) {
  int32_t dx = rawDx;
  int32_t dy = rawDy;

  if (INVERT_X) dx = -dx;
  if (INVERT_Y) dy = -dy;

  uint32_t irq = save_and_disable_interrupts();

  int32_t scaledX = dx * MOTION_SCALE_NUM + g_fracX;
  int32_t scaledY = dy * MOTION_SCALE_NUM + g_fracY;

  int32_t stepX = scaledX / MOTION_SCALE_DEN;
  int32_t stepY = scaledY / MOTION_SCALE_DEN;

  g_fracX = scaledX % MOTION_SCALE_DEN;
  g_fracY = scaledY % MOTION_SCALE_DEN;

  g_pendingX = clamp32(g_pendingX + stepX, -ACCUM_LIMIT, ACCUM_LIMIT);
  g_pendingY = clamp32(g_pendingY + stepY, -ACCUM_LIMIT, ACCUM_LIMIT);
  g_buttons = buttons & (USB_BTN_LEFT | USB_BTN_RIGHT);
  g_reportCount++;

  restore_interrupts(irq);
}

static bool parseMouseReport(uint8_t dev_addr, uint8_t instance,
                             uint8_t const* report, uint16_t len,
                             uint8_t* buttons, int8_t* dx, int8_t* dy) {
  if (!report || len < 3) return false;

  bool hasReportId = false;
  if (dev_addr < MAX_USB_ADDR && instance < MAX_HID_INST) {
    hasReportId = g_hidHasReportId[dev_addr][instance];
  }

  if (hasReportId) {
    // Report ID付き: [Report ID, Buttons, X, Y, ...]
    // Boot Mouseへ切り替え済みのインターフェースでは、ここへ入れてはいけない。
    if (len < 4) return false;
    *buttons = report[1];
    *dx = static_cast<int8_t>(report[2]);
    *dy = static_cast<int8_t>(report[3]);
    return true;
  }

  if (len == 3 || len == 4) {
    // Boot Mouse: [Buttons, X, Y, optional Wheel]
    *buttons = report[0];
    *dx = static_cast<int8_t>(report[1]);
    *dy = static_cast<int8_t>(report[2]);
    return true;
  }

  if (len >= 5 && (report[1] & 0xF0) == 0x00) {
    // Report ID付きの保険。
    *buttons = report[1];
    *dx = static_cast<int8_t>(report[2]);
    *dy = static_cast<int8_t>(report[3]);
    return true;
  }

  *buttons = report[0];
  *dx = static_cast<int8_t>(report[1]);
  *dy = static_cast<int8_t>(report[2]);
  return true;
}

extern "C" void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                                 uint8_t const* desc_report, uint16_t desc_len) {
  bool isMouse = false;
  bool hasReportId = hidDescHasReportId(desc_report, desc_len);

  if (dev_addr < MAX_USB_ADDR && instance < MAX_HID_INST) {
    uint8_t proto = tuh_hid_interface_protocol(dev_addr, instance);

    isMouse = (proto == HID_ITF_PROTOCOL_MOUSE) || hidDescLooksLikeMouse(desc_report, desc_len);

    if (isMouse) {
      if (!g_hidIsMouse[dev_addr][instance]) {
        g_mouseMountedCount++;
      }
      g_hidIsMouse[dev_addr][instance] = true;

      if (proto == HID_ITF_PROTOCOL_MOUSE) {
        // Boot Mouseプロトコルへ切り替えると、実際に届くレポートは
        // [Buttons, X, Y, optional Wheel] になる。
        // レポート記述子にReport IDが残っていても、ここではIDなしとして読む。
        // これを間違えると、X移動をボタン、Y移動をXとして誤読する。
        tuh_hid_set_protocol(dev_addr, instance, HID_PROTOCOL_BOOT);
        g_hidHasReportId[dev_addr][instance] = false;
      } else {
        // Boot Mouseではない複合HIDは、レポート記述子のReport ID有無に従う。
        g_hidHasReportId[dev_addr][instance] = hasReportId;
      }
    } else {
      g_hidIsMouse[dev_addr][instance] = false;
      g_hidHasReportId[dev_addr][instance] = false;
    }
  }

  tuh_hid_receive_report(dev_addr, instance);
}

extern "C" void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  if (dev_addr < MAX_USB_ADDR && instance < MAX_HID_INST) {
    if (g_hidIsMouse[dev_addr][instance]) {
      g_hidIsMouse[dev_addr][instance] = false;
      g_hidHasReportId[dev_addr][instance] = false;
      if (g_mouseMountedCount != 0) g_mouseMountedCount--;
    } else {
      g_hidHasReportId[dev_addr][instance] = false;
    }
  }

  if (g_mouseMountedCount == 0) {
    uint32_t irq = save_and_disable_interrupts();
    g_pendingX = 0;
    g_pendingY = 0;
    g_fracX = 0;
    g_fracY = 0;
    g_buttons = 0;
    restore_interrupts(irq);

    g_currentAxis = 0;
    g_currentRemain = 0;
    g_currentDir = 0;
    g_deferredY = 0;
    setupPc98Outputs();
  }
}

extern "C" void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                           uint8_t const* report, uint16_t len) {
  bool accept = false;

  if (dev_addr < MAX_USB_ADDR && instance < MAX_HID_INST) {
    accept = g_hidIsMouse[dev_addr][instance] || ACCEPT_UNKNOWN_HID_AS_MOUSE;
  }

  if (accept) {
    uint8_t buttons = 0;
    int8_t dx = 0;
    int8_t dy = 0;

    if (parseMouseReport(dev_addr, instance, report, len, &buttons, &dx, &dy)) {
      accumulateMouseDelta(dx, dy, buttons);
    }
  }

  tuh_hid_receive_report(dev_addr, instance);
}

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup() {
  initStatusLED();
  setPixel(12, 4, 0);

  setupPc98Outputs();

  if (HCT07_TEST_MODE == 0) {
    USBHost.begin(0);
    setPixel(0, 0, 12);
  } else {
    setPixel(16, 8, 0);
  }
}

void loop() {
  if (serviceHct07TestMode()) {
    return;
  }

  USBHost.task();
  servicePc98MouseOutput();
  updateStatusLED();
}
