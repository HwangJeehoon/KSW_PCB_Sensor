#include <Arduino.h>
#include <SPI.h>
#include "ads1220.hpp"

#define DEBUG_TEXT 0  // 0 = 25-byte 바이너리 (t_us 포함, logger_ts.py 용)
                      // 1 = 텍스트 디버그 출력
                      // 2 = 21-byte 바이너리 (t_us 없음, 기존 logger.py 호환)

// ==================== Pin Assignment =======================
static const uint8_t PIN_CS_ENC   = 5;
static const uint8_t PIN_CS_LC    = 7;
static const uint8_t PIN_DRDY_ENC = 4;
static const uint8_t PIN_DRDY_LC  = 6;

// ====================== ADC Objects ========================
ADS1220 adc_enc(PIN_CS_ENC, PIN_DRDY_ENC);
ADS1220 adc_lc (PIN_CS_LC,  PIN_DRDY_LC);

// ======================== Constants ========================
static constexpr uint32_t SPI_CLOCK_HZ    = 2000000;
static constexpr uint32_t DRDY_TIMEOUT_US = 1000;
static constexpr uint32_t TX_INTERVAL_US  = 2000;  // 500 Hz = 2000 µs

// ================== Binary Frame Format ====================
// Mode 0: SOF(2) + t_us(4) + seq(2) + enc0(4) + enc1(4) + lc0(4) + lc1(4) + checksum(1) = 25 bytes
// Mode 2: SOF(2)           + seq(2) + enc0(4) + enc1(4) + lc0(4) + lc1(4) + checksum(1) = 21 bytes
static const uint8_t SOF0 = 0xAA;
static const uint8_t SOF1 = 0x55;

static uint16_t g_seq = 0;

// =================== ADC Channel State =====================
static ADS1220::Config enc_cfgs[2];
static ADS1220::Config lc_cfgs[2];
static uint8_t enc_ch = 0;
static uint8_t lc_ch  = 0;

static int32_t enc_latest[2] = {0, 0};
static int32_t lc_latest[2]  = {0, 0};

// ======================== Utilities ========================
static uint8_t xorChecksum(const uint8_t* data, size_t n) {
    uint8_t x = 0;
    for (size_t i = 0; i < n; ++i) x ^= data[i];
    return x;
}

static void writeLeU16(uint8_t* p, uint16_t v) {
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
}

static void writeLeU32(uint8_t* p, uint32_t v) {
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF;
    p[3] = (v >> 24) & 0xFF;
}

static void writeLeI32(uint8_t* p, int32_t v) {
    writeLeU32(p, (uint32_t)v);
}

// ====================== Serial TX ==========================
// Mode 0: 25-byte frame (t_us 포함)
static void sendBinaryFrame25(uint32_t t_us, uint16_t seq,
                               int32_t enc0, int32_t enc1,
                               int32_t lc0,  int32_t lc1) {
    uint8_t f[25];
    f[0] = SOF0;
    f[1] = SOF1;
    writeLeU32(&f[2],  t_us);
    writeLeU16(&f[6],  seq);
    writeLeI32(&f[8],  enc0);
    writeLeI32(&f[12], enc1);
    writeLeI32(&f[16], lc0);
    writeLeI32(&f[20], lc1);
    f[24] = xorChecksum(f, 24);
    Serial.write(f, 25);
}

// Mode 2: 21-byte frame (기존 logger.py 호환)
static void sendBinaryFrame21(uint16_t seq,
                               int32_t enc0, int32_t enc1,
                               int32_t lc0,  int32_t lc1) {
    uint8_t f[21];
    f[0] = SOF0;
    f[1] = SOF1;
    writeLeU16(&f[2],  seq);
    writeLeI32(&f[4],  enc0);
    writeLeI32(&f[8],  enc1);
    writeLeI32(&f[12], lc0);
    writeLeI32(&f[16], lc1);
    f[20] = xorChecksum(f, 20);
    Serial.write(f, 21);
}

static void sendDebugFrame(uint32_t t_us, uint16_t seq,
                            int32_t enc0, int32_t enc1,
                            int32_t lc0,  int32_t lc1) {
    static uint32_t t0_us = 0;
    static uint32_t count = 0;
    static float    hz    = 0.0f;

    if (t0_us == 0) t0_us = t_us;
    count++;

    const uint32_t dt_us = t_us - t0_us;
    if (dt_us < 1000000UL) return;

    hz    = (float)count * 1e6f / (float)dt_us;
    t0_us = t_us;
    count = 0;

    Serial.print("t_us="); Serial.print(t_us);
    Serial.print(" seq=");  Serial.print(seq);
    Serial.print(" enc0="); Serial.print(enc0);
    Serial.print(" enc1="); Serial.print(enc1);
    Serial.print(" lc0=");  Serial.print(lc0);
    Serial.print(" lc1=");  Serial.print(lc1);
    Serial.print(" hz=");   Serial.println(hz, 1);
}

// ======================= Arduino ===========================
void setup() {
    Serial.begin(230400);

    adc_enc.begin(SPI, SPI_CLOCK_HZ);
    adc_lc.begin(SPI, SPI_CLOCK_HZ);

    adc_enc.reset();
    adc_lc.reset();

    // Encoder: single-ended, gain x1, 2000 SPS turbo, AVDD ref
    ADS1220::Config enc_base;
    enc_base.gain       = ADS1220::Gain::X1;
    enc_base.pga_bypass = ADS1220::PgaBypass::BYPASSED;
    enc_base.vref       = ADS1220::Vref::AVDD_AVSS;
    enc_base.dr         = ADS1220::DataRate::DR6;
    enc_base.op_mode    = ADS1220::OpMode::TURBO;
    enc_base.conv_mode  = ADS1220::ConvMode::SINGLE_SHOT;
    enc_cfgs[0] = enc_base;  enc_cfgs[0].mux = ADS1220::Mux::SE_AIN0_AVSS;
    enc_cfgs[1] = enc_base;  enc_cfgs[1].mux = ADS1220::Mux::SE_AIN1_AVSS;

    // Loadcell: differential, gain x128, 2000 SPS turbo
    ADS1220::Config lc_base;
    lc_base.gain      = ADS1220::Gain::X128;
    lc_base.dr        = ADS1220::DataRate::DR6;
    lc_base.op_mode   = ADS1220::OpMode::TURBO;
    lc_base.conv_mode = ADS1220::ConvMode::SINGLE_SHOT;
    lc_cfgs[0] = lc_base;  lc_cfgs[0].mux = ADS1220::Mux::DIFF_AIN0_AIN1;
    lc_cfgs[1] = lc_base;  lc_cfgs[1].mux = ADS1220::Mux::DIFF_AIN2_AIN3;

    enc_ch = 0;
    lc_ch  = 0;

    adc_enc.applyConfig(enc_cfgs[enc_ch]);
    adc_lc.applyConfig(lc_cfgs[lc_ch]);

    adc_enc.startSync();
    adc_lc.startSync();
}

void loop() {
    // --- Encoder ADC (full speed, 2000 SPS) ---
    {
        int32_t raw = 0;
        if (adc_enc.readRaw(raw, DRDY_TIMEOUT_US)) {
            enc_latest[enc_ch] = raw;
            enc_ch ^= 1;
            adc_enc.applyConfig(enc_cfgs[enc_ch]);
            adc_enc.startSync();
        }
    }

    // --- Loadcell ADC (full speed, 2000 SPS) ---
    {
        int32_t raw = 0;
        if (adc_lc.readRaw(raw, DRDY_TIMEOUT_US)) {
            lc_latest[lc_ch] = raw;
            lc_ch ^= 1;
            adc_lc.applyConfig(lc_cfgs[lc_ch]);
            adc_lc.startSync();
        }
    }

    // --- Serial TX at 500 Hz ---
    static uint32_t last_tx_us = 0;
    const uint32_t t_us = micros();
    if (TX_INTERVAL_US == 0 || t_us - last_tx_us >= TX_INTERVAL_US) {
        last_tx_us = t_us;
#if   DEBUG_TEXT == 1
        sendDebugFrame(t_us, g_seq,
                       enc_latest[0], enc_latest[1],
                       lc_latest[0],  lc_latest[1]);
#elif DEBUG_TEXT == 2
        sendBinaryFrame21(g_seq,
                          enc_latest[0], enc_latest[1],
                          lc_latest[0],  lc_latest[1]);
#else  // 0: 25-byte (기본)
        sendBinaryFrame25(t_us, g_seq,
                          enc_latest[0], enc_latest[1],
                          lc_latest[0],  lc_latest[1]);
#endif
        g_seq++;
    }
}
