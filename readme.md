# 引き継ぎメモ
> **このファイルは Claude Code への引き継ぎ用です。作業完了後に削除してください。**

---

## 現在地

STEP2 実装済み・動作確認中。  
ペアリングと制御パケット受信がまだ未確認。

---

## ファイル構成

```
src/main.cpp        共通ソース (ROLE_ROBOT / ROLE_CTRLR で分岐)
platformio.ini
```

### 書き込みコマンド

```bash
pio run -e atoms3r-robot  --target upload   # AtomS3R-CAM
pio run -e atoms3r-ctrlr  --target upload   # AtomS3R
```

---

## ハードウェア

| env | 機種 | 役割 |
|-----|------|------|
| `atoms3r-robot` | AtomS3R-CAM (8MB PSRAM) | 映像送信・サーボ駆動 |
| `atoms3r-ctrlr` | AtomS3R | 映像表示・可変抵抗・制御送信 |

### ピン配置

| 側 | 信号 | GPIO |
|----|------|------|
| ctrlr | 可変抵抗 H軸 | G8 |
| ctrlr | 可変抵抗 V軸 | G7 |
| robot | サーボ1 | G5 (LEDC_TIMER_1 / CH_1) |
| robot | サーボ2 | G6 (LEDC_TIMER_1 / CH_2) |
| robot | カメラ電源 | G18 (Low=ON) |

カメラが `LEDC_TIMER_0 / CH_0` を占有するため、サーボは `TIMER_1` 以降を使用。

---

## 通信設計

すべてのパケットを `radio.sendData()` / `setRecvCallback()` 経由で送受信。  
`esp_now_register_recv_cb` は**使わない**（ESPNowCam のコールバックを上書きしてしまうため）。

映像とパケットの識別は先頭バイトで行う。

| 識別 | 条件 | 方向 | サイズ |
|------|------|------|--------|
| 映像 | 先頭 `0xFF 0xD8` (JPEG) | robot → ctrlr | 可変 |
| `sin1` | 制御パケット | ctrlr → robot | 7 byte |
| `senq` | ペアリング要求 + 自MAC | ctrlr → robot | 10 byte |
| `sack` | ペアリング応答 + 自MAC | robot → ctrlr | 10 byte |
| `smac` | 疎通確認 ping + 自MAC | ctrlr → robot | 10 byte |
| `spon` | 疎通確認 pong + 自MAC | robot → ctrlr | 10 byte |

制御パケット構造 (`sin1`):
```
[s][i][n][1][左モーター+90][右モーター+90][ボタンフラグ]
```

---

## MAC 履歴管理 (ctrlr 側)

- SPIFFS `/mac.txt` : 最大5件 × 6byte バイナリリスト
- 起動時に `smac` ping → `spon` pong で疎通確認
- 応答した MAC を先頭に移動して使用
- 全件無応答 → `/mac.txt` 削除 → ブロードキャストにフォールバック
- ペアリング時は新 MAC を先頭追加 (6件超で末尾を捨てる)

---

## ペアリング手順

1. 両機種を起動
2. ctrlr の **Aボタン長押し** → ENQ 送信
3. robot が ACK を返す
4. ctrlr が ACK 受信 → MAC 保存 → 自動再起動

---

## ADC・制御パラメータ

| 定数 | 値 | 意味 |
|------|----|------|
| `CTRL_INTERVAL_US` | 20,000 (20ms) | 制御パケット送信間隔 |
| `LOG_SIZE` | 5 | 移動平均サンプル数 |
| `DEADBAND` | 0.1 | 不感帯 (±10%) |
| `MAX_SPEED` | 60 | 最大サーボ角度 |
| `RECV_TIMEOUT_MS` | 300 | 無受信でサーボ停止するまでの時間 |
| `PING_TIMEOUT_MS` | 800 | 疎通確認タイムアウト |

タンクデフ演算 (joy_v は前後反転済み):
```cpp
left  = -(MAX_SPEED * (-joy_v - joy_h))
right =   MAX_SPEED * (-joy_v + joy_h)
```

---

## STEP3 残タスク

- [ ] ペアリング・制御パケット受信の動作確認
- [ ] モード切替 (Aボタンダブルクリック) + SPIFFS 保存
- [ ] サーボキャリブレーション (センター・スパン調整)
- [ ] 仕様書 `firmware_specification.md` の残機能を確認して取捨選択

---

## 既知の問題・注意点

- `Using old EspNow implementation :(` はライブラリの警告で動作には影響しない
- ESPNowCam の `setRecvCallback` は**フレーム完成時**に呼ばれる。短いパケットも同じコールバックで届く（タンクサンプルと同じ設計）
- WiFi 使用中は ADC2 系ピンが使えない。ctrlr の可変抵抗は ADC1 系の G7/G8 を使用
- SPIFFS は `SPIFFS.begin(true)` でフォーマットを自動実行している

---

## 参考リンク

- [ESPNowCam タンクサンプル](https://deepwiki.com/hpsaturn/ESPNowCam/4.1-tank-control-system)
- [AtomS3R-CAM ドキュメント](https://docs.m5stack.com/en/core/AtomS3R%20Cam)
- [AtomS3R ドキュメント](https://docs.m5stack.com/en/core/AtomS3R)