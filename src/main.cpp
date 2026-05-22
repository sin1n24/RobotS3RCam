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
 * [動作モード — ROLE_CTRLR]
 *   Mode::R_CON  右手持ちコントローラ (Aダブルクリックで切替)
 *   Mode::L_CON  左手持ちコントローラ
 *   Mode::ROBOT  ロボット動作 (M5Avatar表示、サーボ駆動)
 *   → /param.ini に保存、切替時に esp_restart()
 * ============================================================
 *
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
 *   atoms3r-robot / ctrlr-robot サーボ1 : G5  (LEDC CH_1)
 *   atoms3r-robot / ctrlr-robot サーボ2 : G6  (LEDC CH_2)
 *   atoms3r-robot / ctrlr-robot サーボ3 : G7  (LEDC CH_3)
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
static constexpr uint8_t ID_PING[4] = {'s','p','i','n'};
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
static constexpr int SERVO_PIN3  = 7;
static constexpr int LEDC_CH_SV1 = 1;
static constexpr int LEDC_CH_SV2 = 2;
static constexpr int LEDC_CH_SV3 = 3;
static constexpr int SERVO_FREQ  = 50;
static constexpr int SERVO_BITS  = 10;
static constexpr int SERVO_MIN_W = 26;
static constexpr int SERVO_MAX_W = 125;
static constexpr int ARM_ANGLE_A  = -70;
static constexpr int ARM_ANGLE_B  = -55;
static constexpr int ARM_ANGLE_C  = -10;
static constexpr int ARM_ANGLE_NG =  70;

ESPNowCam radio;

// 受信バッファ (映像 + 制御パケット兼用)
static uint8_t recvBuf[64 * 1024];

static volatile int           sv_left      = 0;
static volatile int           sv_right     = 0;
static volatile uint8_t       sv_btn       = 0;
static volatile unsigned long last_recv_ms = 0;

// アーム状態 (servo3)
static bool sv_arm_mode_b = false;
static bool sv_ng_hold    = false;
static bool ok_prev       = false;
static bool ng_prev       = false;

// サーボ出力バッファ (タイムアウト時イージング用)
static int            sv_out_left  = 0;
static int            sv_out_right = 0;
static int            sv_out_arm   = 0;
static unsigned long  ease_last_ms = 0;
static constexpr int  EASE_DEG_PER_SEC = 90;

// recv コールバック内から esp_now_send は禁止 (ESP-IDF 5.x)
// ACK/PONG はメインループで送信するための defer バッファ
static volatile bool   pending_reply     = false;
static uint8_t         pending_reply_pkt[PKT_MAC_LEN];

// ペアリング済みコントローラー MAC (SPIFFS /ctrl.mac に永続化)
static uint8_t         ctrl_mac[6]      = {};
static bool            ctrl_mac_set     = false;
static uint8_t         recv_src_mac[6]  = {};
static constexpr const char* CTRL_MAC_FILE = "/ctrl.mac";

// --- ctrl_mac SPIFFS 永続化 ---
static void loadCtrlMac() {
    if (!SPIFFS.exists(CTRL_MAC_FILE)) return;
    File f = SPIFFS.open(CTRL_MAC_FILE, FILE_READ);
    if (!f || f.size() < 6) { if (f) f.close(); return; }
    for (int i = 0; i < 6; i++) ctrl_mac[i] = (uint8_t)f.read();
    f.close();
    ctrl_mac_set = true;
    radio.setTarget(ctrl_mac);  // 映像をペアのCtrlr宛ユニキャスト送信
    Serial.printf("[CTRL_MAC] %02X:%02X:%02X:%02X:%02X:%02X\n",
        ctrl_mac[0],ctrl_mac[1],ctrl_mac[2],ctrl_mac[3],ctrl_mac[4],ctrl_mac[5]);
}

static void saveCtrlMac(const uint8_t* mac) {
    memcpy(ctrl_mac, mac, 6);
    ctrl_mac_set = true;
    radio.setTarget(ctrl_mac);  // 映像をペアのCtrlr宛ユニキャスト送信
    File f = SPIFFS.open(CTRL_MAC_FILE, FILE_WRITE);
    if (!f) return;
    for (int i = 0; i < 6; i++) f.write(ctrl_mac[i]);
    f.close();
}

// --- サーボ ---
static void initServo() {
    pinMode(SERVO_PIN1, OUTPUT);
    ledcSetup(LEDC_CH_SV1, SERVO_FREQ, SERVO_BITS);
    ledcAttachPin(SERVO_PIN1, LEDC_CH_SV1);
    pinMode(SERVO_PIN2, OUTPUT);
    ledcSetup(LEDC_CH_SV2, SERVO_FREQ, SERVO_BITS);
    ledcAttachPin(SERVO_PIN2, LEDC_CH_SV2);
    pinMode(SERVO_PIN3, OUTPUT);
    ledcSetup(LEDC_CH_SV3, SERVO_FREQ, SERVO_BITS);
    ledcAttachPin(SERVO_PIN3, LEDC_CH_SV3);
    Serial.println("[SERVO] init OK  G5/CH1  G6/CH2  G7/CH3");
}

static void setServo(int d1, int d2) {
    ledcWrite(LEDC_CH_SV1, map(d1, -90, 90, SERVO_MIN_W, SERVO_MAX_W));
    ledcWrite(LEDC_CH_SV2, map(d2, -90, 90, SERVO_MIN_W, SERVO_MAX_W));
}

static void setArm(int deg) {
    ledcWrite(LEDC_CH_SV3, map(deg, -90, 90, SERVO_MIN_W, SERVO_MAX_W));
}

static int easeToward(int cur, int target, int step) {
    if (cur < target) return min(cur + step, target);
    if (cur > target) return max(cur - step, target);
    return cur;
}

// --- ESPNowCam 受信コールバック ---
static void onDataRecv(uint32_t length) {
    if (length < 4) return;

    // 映像フレームは無視 (robot は受信しない)
    if (isJpeg(recvBuf)) return;

    uint8_t myMac[6];
    esp_read_mac(myMac, ESP_MAC_WIFI_STA);

    // ENQ → ACK 返信 + ctrl_mac 更新 (再ペアリング許可・常に受け付ける)
    if (matchId(recvBuf, ID_ENQ) && length >= (uint32_t)PKT_MAC_LEN) {
        Serial.printf("[PAIR] ENQ from %02X:%02X:%02X:%02X:%02X:%02X\n",
            recv_src_mac[0],recv_src_mac[1],recv_src_mac[2],
            recv_src_mac[3],recv_src_mac[4],recv_src_mac[5]);
        saveCtrlMac(recv_src_mac);
        memcpy(pending_reply_pkt,     ID_ACK, 4);
        memcpy(pending_reply_pkt + 4, myMac,  6);
        pending_reply = true;
        return;
    }

    // ペアリング済みの場合、未ペアの送信元を拒否
    if (ctrl_mac_set && memcmp(recv_src_mac, ctrl_mac, 6) != 0) return;

    // PING → PONG 返信
    if (matchId(recvBuf, ID_PING) && length >= (uint32_t)PKT_MAC_LEN) {
        Serial.printf("[PING] from %02X:%02X:%02X:%02X:%02X:%02X\n",
            recv_src_mac[0],recv_src_mac[1],recv_src_mac[2],
            recv_src_mac[3],recv_src_mac[4],recv_src_mac[5]);
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

    loadCtrlMac();
    initServo();
    setServo(0, 0);
    setArm(0);

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
        [](const esp_now_recv_info_t* info, const uint8_t* data, int len) {
            if (info) memcpy(recv_src_mac, info->src_addr, 6);
#else
    esp_now_register_recv_cb(
        [](const uint8_t* src, const uint8_t* data, int len) {
            if (src) memcpy(recv_src_mac, src, 6);
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

    // アーム状態マシン
    bool cur_ok  = recent && ((sv_btn >> 1) & 1);
    bool cur_ng  = recent && ((sv_btn >> 2) & 1);
    bool cur_trg = recent && ((sv_btn >> 4) & 1);

    if (cur_ok && !ok_prev) { sv_arm_mode_b = !sv_arm_mode_b; sv_ng_hold = false; }
    ok_prev = cur_ok;
    if (cur_ng && !ng_prev) sv_ng_hold = !sv_ng_hold;
    ng_prev = cur_ng;

    int arm_target;
    if (!recent)         arm_target = 0;
    else if (cur_trg)    arm_target = ARM_ANGLE_C;
    else if (sv_ng_hold) arm_target = ARM_ANGLE_NG;
    else                 arm_target = sv_arm_mode_b ? ARM_ANGLE_B : ARM_ANGLE_A;

    if (recent) {
        // 通常制御: スナップ
        sv_out_left  = sv_left;
        sv_out_right = sv_right;
        sv_out_arm   = arm_target;
        ease_last_ms = millis();
    } else {
        // タイムアウト: 車輪はすぐ停止、アームのみゆっくりイージング
        sv_out_left  = 0;
        sv_out_right = 0;
        unsigned long now     = millis();
        unsigned long elapsed = now - ease_last_ms;
        if (elapsed > 500) elapsed = 500;
        int step = max(1, (int)((long)EASE_DEG_PER_SEC * (long)elapsed / 1000));
        ease_last_ms = now;
        sv_out_arm   = easeToward(sv_out_arm, 0, step);
    }
    setServo(sv_out_left, sv_out_right);
    setArm(sv_out_arm);

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
//  ROLE_CTRLR : atoms3r-ctrlr / atoms3-ctrlr
//  動作モード (Aダブルクリックで切替・再起動):
//    Mode::R_CON  右手持ちコントローラ — ADC + 制御送信 + 映像受信
//    Mode::L_CON  左手持ちコントローラ — 同上 (反転)
//    Mode::ROBOT  ロボット動作 — 制御受信 + サーボ駆動 + M5Avatar
// ================================================================
#ifdef ROLE_CTRLR

#include <Avatar.h>
using namespace m5avatar;

// ---- 動作モード ----
enum Mode : uint8_t { MODE_R_CON = 0, MODE_L_CON = 1, MODE_ROBOT = 2 };
static Mode current_mode = MODE_R_CON;

ESPNowCam radio;

static constexpr int LCD_W = 128;
static constexpr int LCD_H = 128;

// 受信バッファ (Con モードで映像を受ける可能性があるため 64KB)
static uint8_t recvBuf[64 * 1024];

// ---- Con モード: 映像受信 ----
static volatile bool          frameReady    = false;
static volatile uint32_t      frameLen      = 0;
static volatile unsigned long last_frame_ms = 0;
static bool                   was_live      = false;
static constexpr unsigned long FRAME_TIMEOUT_MS = 2000;

// ---- Con モード: ADC + ハードウェアボタン ----
static constexpr int ADC_H_PIN  = 8;
static constexpr int ADC_V_PIN  = 7;
static constexpr int TRG_SW_PIN = 5;
static constexpr int OK_SW_PIN  = 38;
static constexpr int NG_SW_PIN  = 39;

static float h_log[LOG_SIZE] = {};
static float v_log[LOG_SIZE] = {};
static int   log_cnt = 0;
static float joy_h   = 0.0f;
static float joy_v   = 0.0f;

// ---- Con モード: MAC 管理 ----
static uint8_t macList[MAC_LIST_MAX][6];
static int     macCount  = 0;
static bool    is_paired = false;
static uint8_t target_addr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static volatile bool pong_received = false;
static volatile bool ack_received  = false;
static uint8_t       response_mac[6] = {};

// ---- Robot モード: サーボ (G5/G6/G7) ----
static constexpr int SV_PIN1   = 5;
static constexpr int SV_PIN2   = 6;
static constexpr int SV_PIN3   = 7;
static constexpr int SV_CH1    = 1;
static constexpr int SV_CH2    = 2;
static constexpr int SV_CH3    = 3;
static constexpr int SV_FREQ   = 50;
static constexpr int SV_BITS   = 10;
static constexpr int SV_MIN_W  = 26;
static constexpr int SV_MAX_W  = 125;
static constexpr int SV_ARM_A  = -70;
static constexpr int SV_ARM_B  = -55;
static constexpr int SV_ARM_C  = -10;
static constexpr int SV_ARM_NG =  70;

static volatile int           sv_left       = 0;
static volatile int           sv_right      = 0;
static volatile uint8_t       sv_btn        = 0;
static volatile unsigned long sv_recv_ms    = 0;

static bool sv_arm_mode_b = false;
static bool sv_ng_hold    = false;
static bool sv_ok_prev    = false;
static bool sv_ng_prev    = false;

static int            sv_out_left  = 0;
static int            sv_out_right = 0;
static int            sv_out_arm   = 0;
static unsigned long  sv_ease_ms   = 0;
static constexpr int  SV_EASE_DPS  = 90;

// ---- 共通: deferred 返信 (ENQ/PING 応答、Robot モードで使用) ----
static volatile bool  pending_reply = false;
static uint8_t        pending_reply_pkt[PKT_MAC_LEN];

// ---- M5Avatar (Robot モード) ----
static Avatar avatar;

// ----------------------------------------------------------------
//  SPIFFS — param.ini (モード保存)
//  文字: 'R'=R_CON  'L'=L_CON  'O'=rObot
// ----------------------------------------------------------------
static const char* PARAM_FILE = "/param.ini";

static void loadParam() {
    current_mode = MODE_R_CON;
    if (!SPIFFS.exists(PARAM_FILE)) return;
    File f = SPIFFS.open(PARAM_FILE, FILE_READ);
    if (!f) return;
    String line = f.readStringUntil('\n');
    f.close();
    if      (line.indexOf('L') >= 0) current_mode = MODE_L_CON;
    else if (line.indexOf('O') >= 0) current_mode = MODE_ROBOT;
    Serial.printf("[PARAM] mode=%d\n", (int)current_mode);
}

static void saveParam() {
    File f = SPIFFS.open(PARAM_FILE, FILE_WRITE);
    if (!f) { Serial.println("[PARAM] write failed"); return; }
    char ch = (current_mode == MODE_L_CON) ? 'L'
            : (current_mode == MODE_ROBOT)  ? 'O' : 'R';
    f.printf("mode=%c\n", ch);
    f.close();
    Serial.printf("[PARAM] saved mode=%d\n", (int)current_mode);
}

// ----------------------------------------------------------------
//  SPIFFS — mac.txt (MAC 履歴、Con モードのみ使用)
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
//  ESPNowCam 受信コールバック (モードで処理を切替)
// ----------------------------------------------------------------
static void onDataRecv(uint32_t length) {
    if (current_mode == MODE_ROBOT) {
        // Robot モード: 制御パケット + ENQ/PING 受信
        if (length < 4) return;
        uint8_t myMac[6];
        esp_read_mac(myMac, ESP_MAC_WIFI_STA);
        if (matchId(recvBuf, ID_ENQ) && length >= (uint32_t)PKT_MAC_LEN) {
            uint8_t* src = recvBuf + 4;
            Serial.printf("[PAIR] ENQ from %02X:%02X:%02X:%02X:%02X:%02X\n",
                src[0],src[1],src[2],src[3],src[4],src[5]);
            memcpy(pending_reply_pkt,     ID_ACK, 4);
            memcpy(pending_reply_pkt + 4, myMac,  6);
            pending_reply = true;
            return;
        }
        if (matchId(recvBuf, ID_PING) && length >= (uint32_t)PKT_MAC_LEN) {
            uint8_t* src = recvBuf + 4;
            Serial.printf("[PING] from %02X:%02X:%02X:%02X:%02X:%02X\n",
                src[0],src[1],src[2],src[3],src[4],src[5]);
            memcpy(pending_reply_pkt,     ID_PONG, 4);
            memcpy(pending_reply_pkt + 4, myMac,   6);
            pending_reply = true;
            return;
        }
        if (matchId(recvBuf, ID_CTRL) && length >= (uint32_t)PKT_CTRL_LEN) {
            sv_left    = (int)recvBuf[4] - SERVO_OFFSET;
            sv_right   = (int)recvBuf[5] - SERVO_OFFSET;
            sv_btn     = recvBuf[6];
            sv_recv_ms = millis();
            Serial.printf("[CTRL] L=%+d R=%+d btn=0x%02X\n", sv_left, sv_right, sv_btn);
        }
        return;
    }

    // Con モード: 映像フレーム + ACK/PONG 受信
    if (length < 2) return;
    if (isJpeg(recvBuf)) {
        frameLen      = length;
        frameReady    = true;
        last_frame_ms = millis();
        return;
    }
    if (length < 4) return;
    if (matchId(recvBuf, ID_ACK) && length >= (uint32_t)PKT_MAC_LEN) {
        memcpy(response_mac, recvBuf + 4, 6);
        ack_received = true;
        return;
    }
    if (matchId(recvBuf, ID_PONG) && length >= (uint32_t)PKT_MAC_LEN) {
        memcpy(response_mac, recvBuf + 4, 6);
        pong_received = true;
        return;
    }
}

// ----------------------------------------------------------------
//  ステータスバー (Con モード)
// ----------------------------------------------------------------
static char last_status[32] = "";

static void setStatus(const char* msg) {
    strncpy(last_status, msg, sizeof(last_status) - 1);
    last_status[sizeof(last_status) - 1] = '\0';
}

static void drawStatusBar() {
    if (last_status[0] == '\0') return;
    M5.Display.fillRect(0, LCD_H - 10, LCD_W, 10, TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(2, LCD_H - 9);
    M5.Display.print(last_status);
}

// ----------------------------------------------------------------
//  LCD 回転適用
// ----------------------------------------------------------------
static void applyLrMode() {
    M5.Display.setRotation(current_mode == MODE_L_CON ? 2 : 0);
}

// ----------------------------------------------------------------
//  5×5 ドットマトリクスフォント (Con モード起動アニメーション)
// ----------------------------------------------------------------
static constexpr int      MX_ASCII_START = 27;
static constexpr int      MX_CHAR_BYTES  = 5;
static constexpr int      MX_SHIFT       = 15;
static constexpr int      MX_STEP        = 25;
static constexpr int      MX_RADIUS      = 8;
static constexpr int      MX_DELAY_MS    = 120;
static constexpr uint32_t MX_WHITE       = 0xFFFFFF;

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

static void draw_matrix_str(const char* str, int delay_ms, uint32_t color, uint32_t bColor) {
    for (int i = 0; str[i] != '\0'; i++) {
        draw_matrix(str[i], color, bColor);
        if (str[i + 1] != '\0') delay(delay_ms);
    }
}

// ----------------------------------------------------------------
//  起動アニメーション
// ----------------------------------------------------------------

// Robot モード: "Robot " マトリクスアニメーション (緑色)
static void showRobotAnim() {
    M5.Display.setRotation(0);
    M5.Display.fillScreen(TFT_BLACK);
    draw_matrix_str("Robot ", MX_DELAY_MS, 0x00FF00, 0);
}

// Con モード: "R con" / "L con" マトリクスアニメーション + ステータスバー
static void showConAnim() {
    M5.Display.fillScreen(TFT_BLACK);
    draw_matrix_str(current_mode == MODE_L_CON ? "L con " : "R con ", MX_DELAY_MS, MX_WHITE, 0);
    drawStatusBar();
}

// ----------------------------------------------------------------
//  Robot モード: サーボ
// ----------------------------------------------------------------
static void initServo() {
    pinMode(SV_PIN1, OUTPUT);
    ledcSetup(SV_CH1, SV_FREQ, SV_BITS);
    ledcAttachPin(SV_PIN1, SV_CH1);
    pinMode(SV_PIN2, OUTPUT);
    ledcSetup(SV_CH2, SV_FREQ, SV_BITS);
    ledcAttachPin(SV_PIN2, SV_CH2);
    pinMode(SV_PIN3, OUTPUT);
    ledcSetup(SV_CH3, SV_FREQ, SV_BITS);
    ledcAttachPin(SV_PIN3, SV_CH3);
    Serial.println("[SERVO] init OK  G5/CH1  G6/CH2  G7/CH3");
}

static void setServo(int d1, int d2) {
    ledcWrite(SV_CH1, map(d1, -90, 90, SV_MIN_W, SV_MAX_W));
    ledcWrite(SV_CH2, map(d2, -90, 90, SV_MIN_W, SV_MAX_W));
}

static void setArm(int deg) {
    ledcWrite(SV_CH3, map(deg, -90, 90, SV_MIN_W, SV_MAX_W));
}

static int easeToward(int cur, int target, int step) {
    if (cur < target) return min(cur + step, target);
    if (cur > target) return max(cur - step, target);
    return cur;
}

// ----------------------------------------------------------------
//  Con モード: ADC + 制御パケット送信
// ----------------------------------------------------------------
static void readADC() {
    float sign = (current_mode == MODE_L_CON) ? -1.0f : 1.0f;
    h_log[log_cnt] = sign * (1.0f - 2.0f * analogRead(ADC_H_PIN) / ADC_MAX_VALUE);
    v_log[log_cnt] = sign * (1.0f - 2.0f * analogRead(ADC_V_PIN) / ADC_MAX_VALUE);
    if (++log_cnt >= LOG_SIZE) log_cnt = 0;
    float sh = 0, sv = 0;
    for (int i = 0; i < LOG_SIZE; i++) { sh += h_log[i]; sv += v_log[i]; }
    joy_h = deadbanded(sh / LOG_SIZE, DEADBAND);
    joy_v = deadbanded(sv / LOG_SIZE, DEADBAND);
}

static void sendCtrlPacket() {
    int left  = constrain(-(int)(MAX_SPEED * (-joy_v - joy_h)), -90, 90);
    int right = constrain( (int)(MAX_SPEED * (-joy_v + joy_h)), -90, 90);
    uint8_t btn = 0;
    if (M5.BtnA.isPressed())      btn |= (1 << 0);
    if (!digitalRead(OK_SW_PIN))  btn |= (1 << 1);
    if (!digitalRead(NG_SW_PIN))  btn |= (1 << 2);
    if (!digitalRead(TRG_SW_PIN)) btn |= (1 << 4);
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
//  Con モード: 疎通確認 ping / ペアリング
// ----------------------------------------------------------------
static bool pingMac(const uint8_t* mac) {
    uint8_t myMac[6];
    esp_read_mac(myMac, ESP_MAC_WIFI_STA);
    uint8_t pkt[PKT_MAC_LEN];
    memcpy(pkt,     ID_PING, 4);
    memcpy(pkt + 4, myMac,   6);
    pong_received = false;
    radio.sendData(pkt, PKT_MAC_LEN);
    unsigned long t = millis();
    while (millis() - t < (unsigned long)PING_TIMEOUT_MS) {
        if (pong_received) {
            if (memcmp(response_mac, mac, 6) == 0) return true;
            pong_received = false;
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
            radio.setTarget(target_addr);  // 制御パケットをRobot宛ユニキャスト送信
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
    setStatus("PAIR FAIL");
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
    loadParam();

    radio.setRecvBuffer(recvBuf);
    radio.setRecvCallback(onDataRecv);
    radio.setChannel(WIFI_CHANNEL);
    radio.init();
    Serial.println("[RADIO] init OK");

    if (current_mode == MODE_ROBOT) {
        // ---- Robot モード ----
        initServo();
        setServo(0, 0);
        setArm(0);
        Serial.println("Ready (robot mode)");
        showRobotAnim();
        M5.Display.setRotation(2);
        avatar.setScale(0.4f);
        avatar.setPosition(-56, -96);
        avatar.init();
        avatar.setExpression(Expression::Sleepy);
    } else {
        // ---- Con モード (R / L) ----
        applyLrMode();
        pinMode(ADC_H_PIN, ANALOG);
        pinMode(ADC_V_PIN, ANALOG);
        pinMode(TRG_SW_PIN, INPUT_PULLUP);
        pinMode(OK_SW_PIN,  INPUT_PULLUP);
        pinMode(NG_SW_PIN,  INPUT_PULLUP);

        loadMacList();
        if (macCount > 0) is_paired = resolveMac();

        if (is_paired) {
            radio.setTarget(target_addr);  // 制御パケットをRobot宛ユニキャスト送信
            Serial.println("[MAC] P2P mode");
            setStatus("PAIRED");
        } else {
            memcpy(target_addr, BROADCAST_ADDR, 6);
            Serial.println("[MAC] broadcast mode");
            setStatus("NO PAIR  Hold A:pair");
        }

        esp_timer_handle_t t;
        esp_timer_create_args_t ta = {};
        ta.callback = ctrlTimerCb;
        ta.name     = "ctrl";
        esp_timer_create(&ta, &t);
        esp_timer_start_periodic(t, CTRL_INTERVAL_US);

        Serial.println("Ready");
        showConAnim();
    }
}

void loop() {
    M5.update();

    // Aダブルクリック → モード切替 (R→L→Robot→R...) → 再起動
    if (M5.BtnA.wasDoubleClicked()) {
        current_mode = (Mode)((current_mode + 1) % 3);
        saveParam();
        esp_restart();
    }

    if (current_mode == MODE_ROBOT) {
        // ---- Robot モード ----
        if (pending_reply) {
            pending_reply = false;
            radio.sendData(pending_reply_pkt, PKT_MAC_LEN);
        }

        bool recent = (millis() - sv_recv_ms) < (unsigned long)RECV_TIMEOUT_MS;

        bool cur_ok  = recent && ((sv_btn >> 1) & 1);
        bool cur_ng  = recent && ((sv_btn >> 2) & 1);
        bool cur_trg = recent && ((sv_btn >> 4) & 1);

        if (cur_ok && !sv_ok_prev) { sv_arm_mode_b = !sv_arm_mode_b; sv_ng_hold = false; }
        sv_ok_prev = cur_ok;
        if (cur_ng && !sv_ng_prev) sv_ng_hold = !sv_ng_hold;
        sv_ng_prev = cur_ng;

        int arm_target;
        if (!recent)          arm_target = 0;
        else if (cur_trg)     arm_target = SV_ARM_C;
        else if (sv_ng_hold)  arm_target = SV_ARM_NG;
        else                  arm_target = sv_arm_mode_b ? SV_ARM_B : SV_ARM_A;

        if (recent) {
            sv_out_left  = sv_left;
            sv_out_right = sv_right;
            sv_out_arm   = arm_target;
            sv_ease_ms   = millis();
        } else {
            sv_out_left  = 0;
            sv_out_right = 0;
            unsigned long now     = millis();
            unsigned long elapsed = now - sv_ease_ms;
            if (elapsed > 500) elapsed = 500;
            int step = max(1, (int)((long)SV_EASE_DPS * (long)elapsed / 1000));
            sv_ease_ms  = now;
            sv_out_arm  = easeToward(sv_out_arm, 0, step);
        }
        setServo(sv_out_left, sv_out_right);
        setArm(sv_out_arm);

        // アバター表情: 走行状態 + ボタンで決定
        {
            static Expression prev_expr = Expression::Sleepy;
            static constexpr int MOVE_THRESH = 15;
            Expression expr;
            if (!recent) {
                expr = Expression::Sleepy;
            } else if (cur_trg) {
                expr = Expression::Happy;
            } else {
                bool fwd  = sv_out_left >  MOVE_THRESH && sv_out_right >  MOVE_THRESH;
                bool back = sv_out_left < -MOVE_THRESH && sv_out_right < -MOVE_THRESH;
                bool spin = (sv_out_left >  MOVE_THRESH && sv_out_right < -MOVE_THRESH) ||
                            (sv_out_left < -MOVE_THRESH && sv_out_right >  MOVE_THRESH);
                if      (fwd)  expr = Expression::Happy;
                else if (back) expr = Expression::Sad;
                else if (spin) expr = Expression::Angry;
                else           expr = Expression::Neutral;
            }
            if (expr != prev_expr) {
                prev_expr = expr;
                avatar.setExpression(expr);
            }
        }
        return;
    }

    // ---- Con モード ----

    // Aボタン長押し → ペアリング
    if (M5.BtnA.wasHold()) {
        doPairing();
        showConAnim();
    }

    // 映像タイムアウト監視 → ON AIR / NO SIGNAL 切替
    bool is_live = (last_frame_ms != 0) &&
                   ((millis() - last_frame_ms) < FRAME_TIMEOUT_MS);
    if (is_live && !was_live) {
        setStatus(is_paired ? "ON AIR P2P" : "ON AIR BCAST");
    } else if (!is_live && was_live) {
        setStatus("NO SIGNAL");
        drawStatusBar();
    }
    was_live = is_live;

    // 映像表示 (LCD全面 128×128) + ステータスバー重ね描き
    if (frameReady) {
        frameReady = false;
        M5.Display.drawJpg(recvBuf, frameLen,
                           0, 0, LCD_W, LCD_H,
                           0, 0, JPEG_DIV_NONE);
        drawStatusBar();
    }
}

#endif // ROLE_CTRLR
