/**
 * @file    main.cpp
 * @brief   STEP2 — ペアリング + MAC履歴管理 + P2P制御通信 + 映像伝送
 *
 * データフロー
 *   atoms3r-robot  ──[radio.sendData 映像]──────▶  atoms3r-ctrlr
 *   atoms3r-robot  ◀──[radio.sendData 制御/ENQ等]── atoms3r-ctrlr
 *
 * ============================================================
 * 設計方針
 *   ESPNowCam は制御パケットも radio.sendData() / setRecvCallback()
 *   で扱える汎用ストリーマ。esp_now_register_recv_cb は使わない。
 *   全パケットを先頭4バイトの識別子で振り分ける。
 *
 *   識別子 (4byte)
 *     "sin1" + [L+90][R+90][btn]  制御パケット     7byte
 *     "senq" + [mac 6byte]        ペアリング要求  10byte  ctrlr→robot
 *     "sack" + [mac 6byte]        ペアリング応答  10byte  robot→ctrlr
 *     "smac" + [mac 6byte]        疎通確認ping   10byte  ctrlr→robot
 *     "spon" + [mac 6byte]        疎通確認pong   10byte  robot→ctrlr
 *   映像フレームは JPEG ヘッダ (0xFF 0xD8) で識別
 * ============================================================
 *
 * MAC履歴ロジック (ctrlr側)
 *   /mac.txt : 最大5件 × 6byte のバイナリリスト
 *   起動時に smac ping → spon pong で疎通確認。
 *   応答した MAC を先頭に移動して使用。
 *   全件無応答なら /mac.txt を削除しブロードキャストにフォールバック。
 *   ペアリング時は新 MAC を先頭に追加 (超過分は末尾を捨てる)。
 *
 * ============================================================
 * [ペアリング手順]
 *   1. 両機種を起動
 *   2. atoms3r-ctrlr の Aボタン長押し → ENQ 送信
 *   3. atoms3r-robot が ACK を返す
 *   4. atoms3r-ctrlr が ACK 受信 → MAC 保存 → 自動再起動
 * ============================================================
 *
 * [ピン配置]
 *   atoms3r-ctrlr 可変抵抗 H軸 : G8
 *   atoms3r-ctrlr 可変抵抗 V軸 : G7
 *   atoms3r-robot サーボ1      : G5  (LEDC_TIMER_1 / CH_1)
 *   atoms3r-robot サーボ2      : G6  (LEDC_TIMER_1 / CH_2)
 */

#include <M5Unified.h>
#include <ESPNowCam.h>
#include <esp_camera.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_timer.h>
#include "FS.h"
#include "SPIFFS.h"

// ================================================================
//  共通定数
// ================================================================
static constexpr int     WIFI_CHANNEL     = 6;
static constexpr int     JPEG_QUALITY     = 24;
static constexpr int     SERVO_OFFSET     = 90;
static constexpr int     MAX_SPEED        = 60;
static constexpr float   ADC_MAX_VALUE    = 4095.0f;
static constexpr float   DEADBAND         = 0.1f;
static constexpr int     LOG_SIZE         = 5;
static constexpr int     CTRL_INTERVAL_US = 20 * 1000;
static constexpr int     RECV_TIMEOUT_MS  = 300;
static constexpr int     MAC_LIST_MAX     = 5;
static constexpr int     PING_TIMEOUT_MS  = 800;

// パケット識別子
static constexpr uint8_t ID_CTRL[4] = {'s','i','n','1'};
static constexpr uint8_t ID_ENQ[4]  = {'s','e','n','q'};
static constexpr uint8_t ID_ACK[4]  = {'s','a','c','k'};
static constexpr uint8_t ID_PING[4] = {'s','m','a','c'};
static constexpr uint8_t ID_PONG[4] = {'s','p','o','n'};

static constexpr int PKT_MAC_LEN  = 10;
static constexpr int PKT_CTRL_LEN = 7;

static uint8_t BROADCAST_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const char* MAC_FILE = "/mac.txt";

// ================================================================
//  共通ユーティリティ
// ================================================================
static bool matchId(const uint8_t* data, const uint8_t* id) {
    return data[0]==id[0] && data[1]==id[1] && data[2]==id[2] && data[3]==id[3];
}

static bool isJpeg(const uint8_t* data) {
    return data[0] == 0xFF && data[1] == 0xD8;
}

static float deadbanded(float val, float band) {
    if (val < band && val > -band) return 0.0f;
    return (val > 0) ? val - band : val + band;
}


// ================================================================
//  ROLE_ROBOT : atoms3r-robot (AtomS3R-CAM)
//  映像送信 + 制御パケット受信 + サーボ駆動
// ================================================================
#ifdef ROLE_ROBOT

// カメラ GPIO
static constexpr int CAM_POWER_PIN = 18;
static constexpr int CAM_PIN_XCLK  = 21;
static constexpr int CAM_PIN_SIOD  = 12;
static constexpr int CAM_PIN_SIOC  = 9;
static constexpr int CAM_PIN_VSYNC = 10;
static constexpr int CAM_PIN_HREF  = 14;
static constexpr int CAM_PIN_PCLK  = 40;
static constexpr int CAM_PIN_Y9    = 13;
static constexpr int CAM_PIN_Y8    = 11;
static constexpr int CAM_PIN_Y7    = 17;
static constexpr int CAM_PIN_Y6    = 4;
static constexpr int CAM_PIN_Y5    = 48;
static constexpr int CAM_PIN_Y4    = 46;
static constexpr int CAM_PIN_Y3    = 42;
static constexpr int CAM_PIN_Y2    = 3;

// サーボ (LEDC_TIMER_1: カメラが TIMER_0 を占有)
static constexpr int SERVO_PIN1  = 5;
static constexpr int SERVO_PIN2  = 6;
static constexpr int LEDC_CH_SV1 = 1;
static constexpr int LEDC_CH_SV2 = 2;
static constexpr int SERVO_FREQ  = 50;
static constexpr int SERVO_BITS  = 10;
static constexpr int SERVO_MIN_W = 26;
static constexpr int SERVO_MAX_W = 125;

ESPNowCam radio;

// 受信バッファ (映像 + 制御パケット兼用)
static uint8_t recvBuf[64 * 1024];

static volatile int           sv_left      = 0;
static volatile int           sv_right     = 0;
static volatile uint8_t       sv_btn       = 0;
static volatile unsigned long last_recv_ms = 0;

// recv コールバック内から esp_now_send は禁止 (ESP-IDF 5.x)
// ACK/PONG はメインループで送信するための defer バッファ
static volatile bool   pending_reply     = false;
static uint8_t         pending_reply_pkt[PKT_MAC_LEN];

// --- サーボ ---
static void initServo() {
    pinMode(SERVO_PIN1, OUTPUT);
    ledcSetup(LEDC_CH_SV1, SERVO_FREQ, SERVO_BITS);
    ledcAttachPin(SERVO_PIN1, LEDC_CH_SV1);
    pinMode(SERVO_PIN2, OUTPUT);
    ledcSetup(LEDC_CH_SV2, SERVO_FREQ, SERVO_BITS);
    ledcAttachPin(SERVO_PIN2, LEDC_CH_SV2);
    Serial.println("[SERVO] init OK  G5/CH1  G6/CH2");
}

static void setServo(int d1, int d2) {
    ledcWrite(LEDC_CH_SV1, map(d1, -90, 90, SERVO_MIN_W, SERVO_MAX_W));
    ledcWrite(LEDC_CH_SV2, map(d2, -90, 90, SERVO_MIN_W, SERVO_MAX_W));
}

// --- ESPNowCam 受信コールバック ---
static void onDataRecv(uint32_t length) {
    if (length < 4) return;

    // 映像フレームは無視 (robot は受信しない)
    if (isJpeg(recvBuf)) return;

    uint8_t myMac[6];
    esp_read_mac(myMac, ESP_MAC_WIFI_STA);

    // ENQ → ACK 返信
    if (matchId(recvBuf, ID_ENQ) && length >= (uint32_t)PKT_MAC_LEN) {
        uint8_t* src = recvBuf + 4;
        Serial.printf("[PAIR] ENQ from %02X:%02X:%02X:%02X:%02X:%02X\n",
            src[0],src[1],src[2],src[3],src[4],src[5]);
        // ACK をメインループで送信 (recv CB 内から esp_now_send 禁止)
        memcpy(pending_reply_pkt,     ID_ACK, 4);
        memcpy(pending_reply_pkt + 4, myMac,  6);
        pending_reply = true;
        return;
    }

    // PING → PONG 返信
    if (matchId(recvBuf, ID_PING) && length >= (uint32_t)PKT_MAC_LEN) {
        uint8_t* src = recvBuf + 4;
        Serial.printf("[PING] from %02X:%02X:%02X:%02X:%02X:%02X\n",
            src[0],src[1],src[2],src[3],src[4],src[5]);
        memcpy(pending_reply_pkt,     ID_PONG, 4);
        memcpy(pending_reply_pkt + 4, myMac,   6);
        pending_reply = true;
        return;
    }

    // 制御パケット
    if (matchId(recvBuf, ID_CTRL) && length >= (uint32_t)PKT_CTRL_LEN) {
        sv_left      = (int)recvBuf[4] - SERVO_OFFSET;
        sv_right     = (int)recvBuf[5] - SERVO_OFFSET;
        sv_btn       = recvBuf[6];
        last_recv_ms = millis();
        Serial.printf("[CTRL] L=%+d R=%+d btn=0x%02X\n", sv_left, sv_right, sv_btn);
        return;
    }
}

// --- カメラ初期化 ---
static bool initCamera() {
    pinMode(CAM_POWER_PIN, OUTPUT);
    digitalWrite(CAM_POWER_PIN, LOW);
    delay(1500);
    camera_config_t cfg = {};
    cfg.pin_pwdn = -1; cfg.pin_reset = -1;
    cfg.pin_xclk     = CAM_PIN_XCLK;
    cfg.pin_sscb_sda = CAM_PIN_SIOD; cfg.pin_sscb_scl = CAM_PIN_SIOC;
    cfg.pin_d7=CAM_PIN_Y9; cfg.pin_d6=CAM_PIN_Y8;
    cfg.pin_d5=CAM_PIN_Y7; cfg.pin_d4=CAM_PIN_Y6;
    cfg.pin_d3=CAM_PIN_Y5; cfg.pin_d2=CAM_PIN_Y4;
    cfg.pin_d1=CAM_PIN_Y3; cfg.pin_d0=CAM_PIN_Y2;
    cfg.pin_vsync    = CAM_PIN_VSYNC;
    cfg.pin_href     = CAM_PIN_HREF;
    cfg.pin_pclk     = CAM_PIN_PCLK;
    cfg.xclk_freq_hz = 20000000;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.pixel_format = PIXFORMAT_RGB565;
    cfg.frame_size   = FRAMESIZE_QQVGA;
    cfg.jpeg_quality = 0;
    cfg.fb_count     = 2;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
    if (esp_camera_init(&cfg) != ESP_OK) {
        Serial.println("[CAM] init failed"); return false;
    }
    sensor_t* s = esp_camera_sensor_get();
    if (s) { s->set_hmirror(s, 1); s->set_vflip(s, 1); }
    Serial.println("[CAM] init OK");
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== atoms3r-robot (STEP2) ===");
    SPIFFS.begin(true);

    if (!initCamera()) { while (true) delay(1000); }

    initServo();
    setServo(0, 0);

    // ESPNowCam 初期化
    radio.setRecvBuffer(recvBuf);
    radio.setRecvCallback(onDataRecv);
    radio.setChannel(WIFI_CHANNEL);
    radio.init();

    // ESPNowCam は sendData() 時に protobuf ヘッダ [0x08][n][0x12][n] を
    // 先頭4バイトに付加して送信する。radio.init() の recv_cb はこれを除去して
    // ユーザーコールバックを呼ぶが、送信ロール(robot)では呼ばれないため直接登録。
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 1)
    esp_now_register_recv_cb(
        [](const esp_now_recv_info_t*, const uint8_t* data, int len) {
#else
    esp_now_register_recv_cb(
        [](const uint8_t*, const uint8_t* data, int len) {
#endif
            // ヘッダ形式: [0x08][n][0x12][n][payload n bytes]
            if (len >= 5 && data[0] == 0x08 && data[2] == 0x12 && data[1] == data[3]) {
                uint8_t n = data[3];
                if (len == (int)(4 + n) && (size_t)n <= sizeof(recvBuf)) {
                    memcpy(recvBuf, data + 4, n);
                    onDataRecv((uint32_t)n);
                }
            }
        }
    );
    Serial.println("[RADIO] init OK");
    Serial.println("Ready");
}

void loop() {
    // ACK / PONG の deferred 送信 (recv CB 内から esp_now_send 禁止のため)
    if (pending_reply) {
        pending_reply = false;
        radio.sendData(pending_reply_pkt, PKT_MAC_LEN);
    }

    bool recent = (millis() - last_recv_ms) < (unsigned long)RECV_TIMEOUT_MS;
    setServo(recent ? sv_left  : 0,
             recent ? sv_right : 0);

    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
        uint8_t* jpg = nullptr; size_t jpg_len = 0;
        if (frame2jpg(fb, JPEG_QUALITY, &jpg, &jpg_len)) {
            radio.sendData(jpg, jpg_len);
            free(jpg);
        }
        esp_camera_fb_return(fb);
    }
}

#endif // ROLE_ROBOT


// ================================================================
//  ROLE_CTRLR : atoms3r-ctrlr (AtomS3R)
//  LCD表示 + ADC + 制御パケット送信 + ペアリング + MAC履歴管理
// ================================================================
#ifdef ROLE_CTRLR

ESPNowCam radio;

static constexpr int LCD_W = 128;
static constexpr int LCD_H = 128;

static uint8_t           recvBuf[64 * 1024];
static volatile bool     frameReady = false;
static volatile uint32_t frameLen   = 0;

// ADC
static constexpr int ADC_H_PIN = 8;
static constexpr int ADC_V_PIN = 7;

static float h_log[LOG_SIZE] = {};
static float v_log[LOG_SIZE] = {};
static int   log_cnt = 0;
static float joy_h   = 0.0f;
static float joy_v   = 0.0f;

// MAC 管理
static uint8_t macList[MAC_LIST_MAX][6];
static int     macCount  = 0;
static bool    is_paired = false;
static uint8_t target_addr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// 応答待ちフラグ
static volatile bool pong_received = false;
static volatile bool ack_received  = false;
static uint8_t       response_mac[6] = {};

// L/R モード
static const char* PARAM_FILE = "/param.ini";
static bool lr_reversed = false;

// ----------------------------------------------------------------
//  SPIFFS — param.ini (L/R モード)
// ----------------------------------------------------------------
static void loadParam() {
    if (!SPIFFS.exists(PARAM_FILE)) return;
    File f = SPIFFS.open(PARAM_FILE, FILE_READ);
    if (!f) return;
    String line = f.readStringUntil('\n');
    f.close();
    lr_reversed = (line.indexOf('L') >= 0);
    Serial.printf("[PARAM] mode=%c\n", lr_reversed ? 'L' : 'R');
}

static void saveParam() {
    File f = SPIFFS.open(PARAM_FILE, FILE_WRITE);
    if (!f) { Serial.println("[PARAM] write failed"); return; }
    f.printf("mode=%c\n", lr_reversed ? 'L' : 'R');
    f.close();
    Serial.printf("[PARAM] saved mode=%c\n", lr_reversed ? 'L' : 'R');
}

// ----------------------------------------------------------------
//  SPIFFS — mac.txt (MAC 履歴)
// ----------------------------------------------------------------
static void loadMacList() {
    macCount = 0;
    if (!SPIFFS.exists(MAC_FILE)) return;
    File f = SPIFFS.open(MAC_FILE, FILE_READ);
    if (!f) return;
    int cnt = (int)(f.size() / 6);
    if (cnt > MAC_LIST_MAX) cnt = MAC_LIST_MAX;
    for (int i = 0; i < cnt; i++)
        for (int j = 0; j < 6; j++) macList[i][j] = (uint8_t)f.read();
    f.close();
    macCount = cnt;
    Serial.printf("[MAC] loaded %d entries\n", macCount);
}

static void saveMacList() {
    File f = SPIFFS.open(MAC_FILE, FILE_WRITE);
    if (!f) { Serial.println("[MAC] write failed"); return; }
    for (int i = 0; i < macCount; i++)
        for (int j = 0; j < 6; j++) f.write(macList[i][j]);
    f.close();
    Serial.printf("[MAC] saved %d entries\n", macCount);
}

// MAC を先頭に追加、重複除去、超過分は末尾を捨てる
static void prependMac(const uint8_t* mac) {
    uint8_t tmp[MAC_LIST_MAX][6];
    int newCount = 0;
    for (int i = 0; i < macCount; i++) {
        if (memcmp(macList[i], mac, 6) != 0 && newCount < MAC_LIST_MAX - 1)
            memcpy(tmp[newCount++], macList[i], 6);
    }
    memcpy(macList[0], mac, 6);
    for (int i = 0; i < newCount; i++) memcpy(macList[i+1], tmp[i], 6);
    macCount = newCount + 1;
    saveMacList();
}

// ----------------------------------------------------------------
//  ESPNowCam 受信コールバック (映像 + 制御応答 共通)
// ----------------------------------------------------------------
static void onDataRecv(uint32_t length) {
    if (length < 2) return;

    // 映像フレーム (JPEG ヘッダで識別)
    if (isJpeg(recvBuf)) {
        frameLen   = length;
        frameReady = true;
        return;
    }

    if (length < 4) return;

    // ACK 受信 (ペアリング応答)
    if (matchId(recvBuf, ID_ACK) && length >= (uint32_t)PKT_MAC_LEN) {
        memcpy(response_mac, recvBuf + 4, 6);
        ack_received = true;
        return;
    }
    // PONG 受信 (疎通確認応答)
    if (matchId(recvBuf, ID_PONG) && length >= (uint32_t)PKT_MAC_LEN) {
        memcpy(response_mac, recvBuf + 4, 6);
        pong_received = true;
        return;
    }
}

// ----------------------------------------------------------------
//  L/R モード適用 + モード切替アニメーション (old_src draw_matrix 移植)
// ----------------------------------------------------------------

// old_src から移植: 5×5ドットフォント (ASCII 27-126, 5byte/char)
static constexpr int      MX_ASCII_START = 27;
static constexpr int      MX_CHAR_BYTES  = 5;
static constexpr int      MX_SHIFT       = 15;   // グリッド原点オフセット (px)
static constexpr int      MX_STEP        = 25;   // ドット間隔 (px)
static constexpr int      MX_RADIUS      = 8;    // ドット半径 (px)
static constexpr int      MX_DELAY_MS    = 120;  // 文字間ディレイ (MATRIX_CHAR_DELAY_MS_FAST)
static constexpr uint32_t MX_WHITE       = 0xFFFFFF;

// old_src FONTDATA[500] をそのままコピー
static const uint8_t FONTDATA[500] = {
    0x0A,0x0A,0x00,0x11,0x0E, 0x00,0x0A,0x00,0x11,0x0E, 0x0A,0x0A,0x00,0x11,0x0E, 0x00,0x0A,0x00,0x11,0x0E,
    0x0,0x0,0x0,0x0,0x5,  0x0,0x0,0x0,0x0,0x0,
    0x8,0x8,0x8,0x0,0x8,  0xa,0x4a,0x40,0x0,0x0, 0xa,0x5f,0xea,0x5f,0xea, 0xe,0xd9,0x2e,0xd3,0x6e,
    0x19,0x32,0x44,0x89,0x33, 0xc,0x92,0x4c,0x92,0x4d, 0x8,0x8,0x0,0x0,0x0,
    0x4,0x88,0x8,0x8,0x4, 0x8,0x4,0x84,0x84,0x88, 0x0,0xa,0x44,0x8a,0x40,
    0x0,0x4,0x8e,0xc4,0x80, 0x0,0x0,0x0,0x4,0x88, 0x0,0x0,0xe,0xc0,0x0,
    0x0,0x0,0x0,0x8,0x0,  0x1,0x22,0x44,0x88,0x10,
    0xc,0x92,0x52,0x52,0x4c, 0x4,0x8c,0x84,0x84,0x8e, 0x1c,0x82,0x4c,0x90,0x1e,
    0x1e,0xc2,0x44,0x92,0x4c, 0x6,0xca,0x52,0x5f,0xe2, 0x1f,0xf0,0x1e,0xc1,0x3e,
    0x2,0x44,0x8e,0xd1,0x2e,  0x1f,0xe2,0x44,0x88,0x10, 0xe,0xd1,0x2e,0xd1,0x2e,
    0xe,0xd1,0x2e,0xc4,0x88,  0x0,0x8,0x0,0x8,0x0,   0x0,0x4,0x80,0x4,0x88,
    0x2,0x44,0x88,0x4,0x82,   0x0,0xe,0xc0,0xe,0xc0,  0x8,0x4,0x82,0x44,0x88,
    0xe,0xd1,0x26,0xc0,0x4,   0xe,0xd1,0x35,0xb3,0x6c,
    0xc,0x92,0x5e,0xd2,0x52,  0x1c,0x92,0x5c,0x92,0x5c, 0xe,0xd0,0x10,0x10,0xe,
    0x1c,0x92,0x52,0x52,0x5c,  0x1e,0xd0,0x1c,0x90,0x1e, 0x1e,0xd0,0x1c,0x90,0x10,
    0xe,0xd0,0x13,0x71,0x2e,   0x12,0x52,0x5e,0xd2,0x52, 0x1c,0x88,0x8,0x8,0x1c,
    0x1f,0xe2,0x42,0x52,0x4c,  0x12,0x54,0x98,0x14,0x92, 0x10,0x10,0x10,0x10,0x1e,
    0x11,0x3b,0x75,0xb1,0x31,  0x11,0x39,0x35,0xb3,0x71, 0xc,0x92,0x52,0x52,0x4c,
    0x1c,0x92,0x5c,0x90,0x10,  0xc,0x92,0x52,0x4c,0x86,  0x1c,0x92,0x5c,0x92,0x51,
    0xe,0xd0,0xc,0x82,0x5c,    0x1f,0xe4,0x84,0x84,0x84, 0x12,0x52,0x52,0x52,0x4c,
    0x11,0x31,0x31,0x2a,0x44,  0x11,0x31,0x35,0xbb,0x71, 0x12,0x52,0x4c,0x92,0x52,
    0x11,0x2a,0x44,0x84,0x84,  0x1e,0xc4,0x88,0x10,0x1e,
    0xe,0xc8,0x8,0x8,0xe,   0x10,0x8,0x4,0x82,0x41,  0xe,0xc2,0x42,0x42,0x4e,
    0x4,0x8a,0x40,0x0,0x0,  0x0,0x0,0x0,0x0,0x1f,    0x8,0x4,0x80,0x0,0x0,
    0x0,0xe,0xd2,0x52,0x4f,  0x10,0x10,0x1c,0x92,0x5c, 0x0,0xe,0xd0,0x10,0xe,
    0x2,0x42,0x4e,0xd2,0x4e,  0xc,0x92,0x5c,0x90,0xe,  0x6,0xc8,0x1c,0x88,0x8,
    0xe,0xd2,0x4e,0xc2,0x4c,  0x10,0x10,0x1c,0x92,0x52, 0x8,0x0,0x8,0x8,0x8,
    0x2,0x40,0x2,0x42,0x4c,   0x10,0x14,0x98,0x14,0x92, 0x8,0x8,0x8,0x8,0x6,
    0x0,0x1b,0x75,0xb1,0x31,  0x0,0x1c,0x92,0x52,0x52,  0x0,0xc,0x92,0x52,0x4c,
    0x0,0x1c,0x92,0x5c,0x90,  0x0,0xe,0xd2,0x4e,0xc2,   0x0,0xe,0xd0,0x10,0x10,
    0x0,0x6,0xc8,0x4,0x98,    0x8,0x8,0xe,0xc8,0x7,     0x0,0x12,0x52,0x52,0x4f,
    0x0,0x11,0x31,0x2a,0x44,  0x0,0x11,0x31,0x35,0xbb,   0x0,0x12,0x4c,0x8c,0x92,
    0x0,0x11,0x2a,0x44,0x98,  0x0,0x1e,0xc4,0x88,0x1e,   0x6,0xc4,0x8c,0x84,0x86,
    0x8,0x8,0x8,0x8,0x8,      0x18,0x8,0xc,0x88,0x18,    0x0,0x0,0xc,0x83,0x60
};

static void applyLrMode() {
    // old_src: M5.Display.setRotation(lefty?2:0)
    M5.Display.setRotation(lr_reversed ? 2 : 0);
}

// old_src draw_matrix() のLCD移植版: 5×5ドットマトリクスで1文字描画
static void draw_matrix(char ch, uint32_t color, uint32_t bColor) {
    if (ch < MX_ASCII_START || ch > 126) return;
    int start = ((int)ch - MX_ASCII_START) * MX_CHAR_BYTES;
    for (int row = 0; row < 5; row++) {
        for (int col = 0; col < 5; col++) {
            uint32_t c = (FONTDATA[start + row] & (1 << abs(col - 4))) ? color : bColor;
            M5.Display.fillCircle(
                MX_SHIFT + col * MX_STEP,
                MX_SHIFT + row * MX_STEP,
                MX_RADIUS, c);
        }
    }
}

// old_src draw_matrix_str() 移植: 文字列を1文字ずつ切り替えて表示
static void draw_matrix_str(const char* str, int delay_ms, uint32_t color, uint32_t bColor) {
    for (int i = 0; str[i] != '\0'; i++) {
        draw_matrix(str[i], color, bColor);
        if (str[i + 1] != '\0') delay(delay_ms);
    }
}

static void showModeAnim() {
    M5.Display.fillScreen(TFT_BLACK);
    // old_src: draw_matrix_str("L Controler ", MATRIX_CHAR_DELAY_MS_FAST, COLOR_WHITE, 0)
    draw_matrix_str(lr_reversed ? "L con " : "R con ", MX_DELAY_MS, MX_WHITE, 0);
}

// ----------------------------------------------------------------
//  ADC + 制御パケット送信
// ----------------------------------------------------------------
static void readADC() {
    h_log[log_cnt] = 1.0f - 2.0f * analogRead(ADC_H_PIN) / ADC_MAX_VALUE;
    v_log[log_cnt] = 1.0f - 2.0f * analogRead(ADC_V_PIN) / ADC_MAX_VALUE;
    if (++log_cnt >= LOG_SIZE) log_cnt = 0;
    float sh = 0, sv = 0;
    for (int i = 0; i < LOG_SIZE; i++) { sh += h_log[i]; sv += v_log[i]; }
    float sign = lr_reversed ? -1.0f : 1.0f;
    joy_h = sign * deadbanded(sh / LOG_SIZE, DEADBAND);
    joy_v = sign * deadbanded(sv / LOG_SIZE, DEADBAND);
}

static void sendCtrlPacket() {
    int left  = constrain(-(int)(MAX_SPEED * (-joy_v - joy_h)), -90, 90);
    int right = constrain( (int)(MAX_SPEED * (-joy_v + joy_h)), -90, 90);
    uint8_t btn = M5.BtnA.isPressed() ? 0x01 : 0x00;
    uint8_t pkt[PKT_CTRL_LEN] = {
        ID_CTRL[0], ID_CTRL[1], ID_CTRL[2], ID_CTRL[3],
        (uint8_t)(left  + SERVO_OFFSET),
        (uint8_t)(right + SERVO_OFFSET),
        btn
    };
    radio.sendData(pkt, PKT_CTRL_LEN);
}

static void ctrlTimerCb(void* /*arg*/) {
    readADC();
    sendCtrlPacket();
}

// ----------------------------------------------------------------
//  疎通確認 ping
// ----------------------------------------------------------------
static bool pingMac(const uint8_t* mac) {
    uint8_t myMac[6];
    esp_read_mac(myMac, ESP_MAC_WIFI_STA);
    uint8_t pkt[PKT_MAC_LEN];
    memcpy(pkt,     ID_PING, 4);
    memcpy(pkt + 4, myMac,   6);
    pong_received = false;

    // ping はブロードキャスト送信 (robot が誰でも応答できるよう)
    radio.sendData(pkt, PKT_MAC_LEN);

    unsigned long t = millis();
    while (millis() - t < (unsigned long)PING_TIMEOUT_MS) {
        if (pong_received) {
            // 応答 MAC が期待する MAC と一致するか確認
            if (memcmp(response_mac, mac, 6) == 0) return true;
            pong_received = false; // 別機器からの応答は無視
        }
        delay(5);
    }
    return false;
}

static bool resolveMac() {
    for (int i = 0; i < macCount; i++) {
        Serial.printf("[MAC] ping %02X:%02X:%02X:%02X:%02X:%02X\n",
            macList[i][0],macList[i][1],macList[i][2],
            macList[i][3],macList[i][4],macList[i][5]);
        if (pingMac(macList[i])) {
            Serial.println("[MAC] pong OK");
            prependMac(macList[i]);
            memcpy(target_addr, macList[i], 6);
            return true;
        }
    }
    Serial.println("[MAC] all entries failed → delete mac.txt");
    SPIFFS.remove(MAC_FILE);
    macCount = 0;
    return false;
}

// ----------------------------------------------------------------
//  ペアリング
// ----------------------------------------------------------------
static void doPairing() {
    Serial.println("[PAIR] sending ENQ...");
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(2, 2);
    M5.Display.println("Pairing...");

    uint8_t myMac[6];
    esp_read_mac(myMac, ESP_MAC_WIFI_STA);
    uint8_t pkt[PKT_MAC_LEN];
    memcpy(pkt,     ID_ENQ, 4);
    memcpy(pkt + 4, myMac,  6);

    ack_received = false;
    radio.sendData(pkt, PKT_MAC_LEN);

    unsigned long t = millis();
    while (millis() - t < 3000) {
        if (ack_received) {
            Serial.printf("[PAIR] ACK from %02X:%02X:%02X:%02X:%02X:%02X\n",
                response_mac[0],response_mac[1],response_mac[2],
                response_mac[3],response_mac[4],response_mac[5]);
            prependMac(response_mac);
            memcpy(target_addr, response_mac, 6);
            is_paired = true;
            M5.Display.fillScreen(TFT_GREEN);
            M5.Display.setTextColor(TFT_BLACK, TFT_GREEN);
            M5.Display.setCursor(2, 2);
            M5.Display.println("PAIRED!");
            delay(1000);
            esp_restart();
            return;
        }
        delay(10);
    }
    Serial.println("[PAIR] timeout");
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5.Display.setCursor(2, 2);
    M5.Display.println("PAIR FAIL");
    delay(1000);
}

// ----------------------------------------------------------------
//  setup / loop
// ----------------------------------------------------------------
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    Serial.println("=== atoms3r-ctrlr (STEP2) ===");

    SPIFFS.begin(true);

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(2, 2);
    M5.Display.println("Starting...");

    pinMode(ADC_H_PIN, ANALOG);
    pinMode(ADC_V_PIN, ANALOG);

    // ESPNowCam 映像受信 + 制御応答受信
    radio.setRecvBuffer(recvBuf);
    radio.setRecvCallback(onDataRecv);
    radio.setChannel(WIFI_CHANNEL);
    radio.init();
    Serial.println("[RADIO] init OK");

    // パラメータ読み込み → 表示回転適用
    loadParam();
    applyLrMode();

    // MAC リスト読み込み → 疎通確認
    loadMacList();
    if (macCount > 0) {
        M5.Display.setCursor(2, 14);
        M5.Display.println("Checking MAC...");
        is_paired = resolveMac();
    }

    if (is_paired) {
        Serial.println("[MAC] P2P mode");
        M5.Display.fillScreen(TFT_BLACK);
        M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(2, 2);
        M5.Display.println("PAIRED");
    } else {
        // ブロードキャストのまま使用
        memcpy(target_addr, BROADCAST_ADDR, 6);
        Serial.println("[MAC] broadcast mode");
        M5.Display.fillScreen(TFT_BLACK);
        M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(2, 2);
        M5.Display.println("NO PAIR");
        M5.Display.println("Hold A:pair");
    }

    // 制御タイマー (20ms)
    esp_timer_handle_t t;
    esp_timer_create_args_t ta = {};
    ta.callback = ctrlTimerCb;
    ta.name     = "ctrl";
    esp_timer_create(&ta, &t);
    esp_timer_start_periodic(t, CTRL_INTERVAL_US);

    Serial.println("Ready");
    delay(800);
}

void loop() {
    M5.update();

    // Aボタンダブルクリック → L/R モード切替 + /param.ini 保存
    if (M5.BtnA.wasDoubleClicked()) {
        lr_reversed = !lr_reversed;
        saveParam();
        applyLrMode();
        showModeAnim();
    }

    // Aボタン長押し → ペアリング
    if (M5.BtnA.wasHold()) {
        doPairing();
    }

    // 映像表示 (LCD全面 128×128)
    if (frameReady) {
        frameReady = false;
        M5.Display.drawJpg(recvBuf, frameLen,
                           0, 0, LCD_W, LCD_H,
                           0, 0, JPEG_DIV_NONE);
    }
}

#endif // ROLE_CTRLR