# USBMOUSE2PC98

RP2040ボードを使い、USB HIDマウスをPC-9801 / PC-98系のバスマウス入力へ変換するアダプター用ファームウェアです。

USBマウスはRP2040ボードのUSB端子へOTG経由で接続し、PC-98側にはXA / XB / YA / YBの2相信号と、LEFT / RIGHTのボタン信号を出力します。

## 特徴

- USB HIDマウスをPC-98バスマウス信号へ変換
- RP2040 / RP2040 Zero対応
- Arduino IDE対応
- Adafruit TinyUSB Host使用
- HCT07系オープンドレインバッファ前提
- PC-98用XA / XB / YA / YB 2相信号生成
- LEFT / RIGHTボタン対応
- USBマウスのHIDレポート誤読対策済み
- 出力確認用テストモード搭載

## 使用ハードウェア

### マイコン

- RP2040 / RP2040 Zero互換ボード

### USBマウス/トラックボール

USBマウス/トラックボールは、RP2040ボードのUSB端子へOTG経由で接続します。

### 出力バッファ

PC-98側のマウス信号は5V系のため、RP2040 GPIOを直接接続しないでください。

本ファームウェアでは、HCT07系のオープンドレインバッファを前提にしています。

```text
RP2040 GPIO -> HCT07入力
HCT07出力 -> PC-98マウス信号
HCT07 VCC -> +5V
RP2040 GND / HCT07 GND / PC-98 GND 共通
```

## GPIO割り当て

| PC-98信号 | RP2040 GPIO |
|---|---:|
| LEFT | GPIO0 |
| RIGHT | GPIO1 |
| YB | GPIO8 |
| XA | GPIO14 |
| XB | GPIO15 |
| YA | GPIO26 |


## 信号論理

HCT07出力はオープンドレインとして扱います。

```text
RP2040 GPIO LOW  -> HCT07出力LOW  -> PC-98信号LOW
RP2040 GPIO HIGH -> HCT07出力Hi-Z -> PC-98信号HIGH、プルアップ任せ
```

LEFT / RIGHT は負論理です。

```text
ボタン未押下 : HIGH
ボタン押下   : LOW
```

XA / XB、YA / YB は2相信号です。

```text
正方向 : 00 -> 10 -> 11 -> 01 -> 00
逆方向 : 00 -> 01 -> 11 -> 10 -> 00
```

## マウス移動量調整

トラックボールや高DPIマウスでは移動量が大きすぎる場合があります。  
以下の値で調整できます。

```cpp
static constexpr bool INVERT_X = false;
static constexpr bool INVERT_Y = false;
static constexpr int32_t MOTION_SCALE_NUM = 1;
static constexpr int32_t MOTION_SCALE_DEN = 4;
```

例:

- 移動が速すぎる場合: `MOTION_SCALE_DEN` を大きくする
- 向きが逆の場合: `INVERT_X` / `INVERT_Y` を `true`


## Arduino IDE設定

Earle Philhower版RP2040コアを使用してください。

推奨設定：

```text
Board     : RP2040 Zero互換ボード
USB Stack : Adafruit TinyUSB Host
```

以下のエラーが出る場合：

```text
#error TinyUSB is not selected, please select it in "Tools->Menu->USB Stack"
```

Arduino IDEで次のように設定してください。

```text
Tools -> USB Stack -> Adafruit TinyUSB Host
```

## テストモード

USBマウスを使わず、PC-98側への出力信号だけを確認するためのテストモードを搭載しています。

ソースコード内の以下の値を変更します。

```cpp
static constexpr uint8_t HCT07_TEST_MODE = 0;
```

| 値 | モード |
|---:|---|
| 0 | 通常動作 |
| 1 | 待機状態出力テスト |
| 2 | LEFT / RIGHTボタン出力テスト |
| 3 | X正方向2相信号テスト |
| 4 | X逆方向2相信号テスト |
| 5 | Y正方向2相信号テスト |
| 6 | Y逆方向2相信号テスト |
| 7 | 全信号ウォーキングテスト |

## 注意事項
 
USB HIDマウスをTinyUSB Hostで直接読み取り、PC-98用バスマウス信号を生成します。

PC-98側は5V系のため、RP2040 GPIOを直接PC-98マウス端子へ接続しないでください。  
HCT07系オープンドレインバッファ、または同等の5V対応オープンドレイン出力段を使用してください。
