#pragma once
#include <Arduino.h>
#include <SPI.h>

class ADS1220 {
public:
  // ------------------------------------------------------------
  // Register addresses (8.6.1)
  // ------------------------------------------------------------
  enum class Reg : uint8_t {
    CONFIG0 = 0x00,
    CONFIG1 = 0x01,
    CONFIG2 = 0x02,
    CONFIG3 = 0x03
  };

  // ------------------------------------------------------------
  // SPI commands (8.5.3)
  // ------------------------------------------------------------
  enum class Command : uint8_t {
    POWERDOWN = 0x02,   // 0000 001x
    RESET     = 0x06,   // 0000 011x
    STARTSYNC = 0x08,   // 0000 100x
    RDATA     = 0x10    // 0001 xxxx
    // RREG / WREG are formed dynamically because rr, nn are embedded
  };

  // ------------------------------------------------------------
  // CONFIG0 (00h): MUX[3:0], GAIN[2:0], PGA_BYPASS
  // ------------------------------------------------------------
  enum class Mux : uint8_t {
    DIFF_AIN0_AIN1 = 0b0000,
    DIFF_AIN0_AIN2 = 0b0001,
    DIFF_AIN0_AIN3 = 0b0010,
    DIFF_AIN1_AIN2 = 0b0011,
    DIFF_AIN1_AIN3 = 0b0100,
    DIFF_AIN2_AIN3 = 0b0101,
    DIFF_AIN1_AIN0 = 0b0110,
    DIFF_AIN3_AIN2 = 0b0111,
    SE_AIN0_AVSS   = 0b1000,
    SE_AIN1_AVSS   = 0b1001,
    SE_AIN2_AVSS   = 0b1010,
    SE_AIN3_AVSS   = 0b1011,
    MON_REFP_REFN_DIV4 = 0b1100,
    MON_AVDD_AVSS_DIV4 = 0b1101,
    SHORT_MID_SUPPLY   = 0b1110
    // 1111 reserved
  };

  enum class Gain : uint8_t {
    X1   = 0b000,
    X2   = 0b001,
    X4   = 0b010,
    X8   = 0b011,
    X16  = 0b100,
    X32  = 0b101,
    X64  = 0b110,
    X128 = 0b111
  };

  enum class PgaBypass : uint8_t {
    ENABLED  = 0,   // PGA enabled (default)
    BYPASSED = 1    // PGA disabled and bypassed
  };

  // ------------------------------------------------------------
  // CONFIG1 (01h): DR[2:0], MODE[1:0], CM, TS, BCS
  // ------------------------------------------------------------
  // DR bit pattern is mode-dependent:
  // Normal:     20, 45, 90, 175, 330, 600, 1000 SPS
  // Duty-cycle: 5, 11.25, 22.5, 44, 82.5, 150, 250 SPS
  // Turbo:      40, 90, 180, 350, 660, 1200, 2000 SPS
  enum class DataRate : uint8_t {
    DR0 = 0b000,
    DR1 = 0b001,
    DR2 = 0b010,
    DR3 = 0b011,
    DR4 = 0b100,
    DR5 = 0b101,
    DR6 = 0b110 // turbo + DR6 = 2000 SPS
    // 111 reserved
  };

  enum class OpMode : uint8_t {
    NORMAL     = 0b00, 
    DUTY_CYCLE = 0b01,
    TURBO      = 0b10 
    // 11 reserved
  };

  enum class ConvMode : uint8_t {
    SINGLE_SHOT = 0, // default
    CONTINUOUS  = 1
  };

  enum class TempSensor : uint8_t {
    DISABLED = 0, // default
    ENABLED  = 1
  };

  enum class BurnoutCurrent : uint8_t {
    OFF = 0, // default
    ON  = 1
  };

  // ------------------------------------------------------------
  // CONFIG2 (02h): VREF[1:0], 50/60[1:0], PSW, IDAC[2:0]
  // ------------------------------------------------------------
  enum class Vref : uint8_t {
    INTERNAL_2V048 = 0b00, // internal 2.048V (default)
    EXT_REF0       = 0b01, // REFP0 / REFN0
    EXT_REF1       = 0b10, // AIN0/REFP1, AIN3/REFN1
    AVDD_AVSS      = 0b11
  };

  enum class FirMode : uint8_t {
    NONE      = 0b00, // default
    REJECT_50_60 = 0b01,
    REJECT_50    = 0b10,
    REJECT_60    = 0b11
  };

  enum class LowSideSwitch : uint8_t {
    OPEN_ALWAYS = 0, // default
    AUTO_SWITCH = 1
  };

  enum class IdacCurrent : uint8_t {
    OFF      = 0b000, // default
    UA_10    = 0b001,
    UA_50    = 0b010,
    UA_100   = 0b011,
    UA_250   = 0b100,
    UA_500   = 0b101,
    UA_1000  = 0b110,
    UA_1500  = 0b111
  };

  // ------------------------------------------------------------
  // CONFIG3 (03h): I1MUX[2:0], I2MUX[2:0], DRDYM, 0
  // ------------------------------------------------------------
  enum class IdacMux : uint8_t {
    DISABLED   = 0b000, // default
    AIN0_REFP1 = 0b001,
    AIN1       = 0b010,
    AIN2       = 0b011,
    AIN3_REFN1 = 0b100,
    REFP0      = 0b101,
    REFN0      = 0b110
    // 111 reserved
  };

  enum class DrdyMode : uint8_t {
    DRDY_PIN_ONLY = 0, // default
    BOTH_PINS     = 1   // DRDY and DOUT/DRDY
  };

  // ------------------------------------------------------------
  // Register preset
  // ------------------------------------------------------------
  struct Config {
    Mux mux = Mux::DIFF_AIN0_AIN1;
    Gain gain = Gain::X1;
    PgaBypass pga_bypass = PgaBypass::ENABLED;

    DataRate dr = DataRate::DR6;
    OpMode op_mode = OpMode::TURBO;
    ConvMode conv_mode = ConvMode::SINGLE_SHOT;
    TempSensor temp_sensor = TempSensor::DISABLED;
    BurnoutCurrent burnout = BurnoutCurrent::OFF;

    Vref vref = Vref::INTERNAL_2V048;
    FirMode fir = FirMode::NONE;
    LowSideSwitch psw = LowSideSwitch::OPEN_ALWAYS;
    IdacCurrent idac = IdacCurrent::OFF;

    IdacMux idac1_mux = IdacMux::DISABLED;
    IdacMux idac2_mux = IdacMux::DISABLED;
    DrdyMode drdy_mode = DrdyMode::DRDY_PIN_ONLY;
  };

  ADS1220(uint8_t cs_pin, int8_t drdy_pin = -1);

  void begin(SPIClass& spi = SPI, uint32_t spi_hz = 2000000);

  // basic device control
  void reset();
  void startSync();
  void powerDown();

  // register access
  uint8_t readRegister(Reg reg);
  void readRegisters(Reg start_reg, uint8_t* dst, uint8_t count);
  void writeRegister(Reg reg, uint8_t value);
  void writeRegisters(Reg start_reg, const uint8_t* src, uint8_t count);

  // config helpers
  void applyConfig(const Config& cfg);
  void readConfig(Config& cfg);

  // raw data access
  bool readRaw(int32_t& out_raw, uint32_t timeout_us = 0);
  bool waitDrdy(uint32_t timeout_us);

private:
  uint8_t cs_;
  int8_t drdy_;
  SPIClass* spi_{nullptr};
  SPISettings spi_set_{2000000, MSBFIRST, SPI_MODE1};

  void csLow_();
  void csHigh_();

  // command builders
  uint8_t buildRregCmd_(Reg start_reg, uint8_t count) const;
  uint8_t buildWregCmd_(Reg start_reg, uint8_t count) const;

  // register pack/unpack
  uint8_t buildConfig0_(Mux mux, Gain gain, PgaBypass pga_bypass) const;
  uint8_t buildConfig1_(DataRate dr, OpMode op_mode, ConvMode conv_mode,
                        TempSensor ts, BurnoutCurrent bcs) const;
  uint8_t buildConfig2_(Vref vref, FirMode fir, LowSideSwitch psw,
                        IdacCurrent idac) const;
  uint8_t buildConfig3_(IdacMux i1mux, IdacMux i2mux, DrdyMode drdym) const;

  void parseConfig0_(uint8_t reg, Config& cfg) const;
  void parseConfig1_(uint8_t reg, Config& cfg) const;
  void parseConfig2_(uint8_t reg, Config& cfg) const;
  void parseConfig3_(uint8_t reg, Config& cfg) const;
};
