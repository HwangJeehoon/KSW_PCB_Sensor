#include "ads1220.hpp"

ADS1220::ADS1220(uint8_t cs_pin, int8_t drdy_pin)
: cs_(cs_pin), drdy_(drdy_pin)
{}

void ADS1220::begin(SPIClass& spi, uint32_t spi_hz){
  spi_ = &spi;
  spi_set_ = SPISettings(spi_hz, MSBFIRST, SPI_MODE1);

  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);

  if (drdy_ >= 0) {
    pinMode((uint8_t)drdy_, INPUT_PULLUP);
  }

  spi_->begin();
}

void ADS1220::csLow_(){
  digitalWrite(cs_, LOW);
}

void ADS1220::csHigh_(){
  digitalWrite(cs_, HIGH);
}

// ------------------------------------------------------------
// command helpers
// ------------------------------------------------------------
uint8_t ADS1220::buildRregCmd_(Reg start_reg, uint8_t count) const{
  // RREG: 0010 rrnn
  // rr = start register address (2bit)
  // nn = number of registers - 1 (2bit)
  const uint8_t rr = static_cast<uint8_t>(start_reg) & 0x03;
  const uint8_t nn = (count - 1u) & 0x03;
  return static_cast<uint8_t>(0x20 | (rr << 2) | nn);
}

uint8_t ADS1220::buildWregCmd_(Reg start_reg, uint8_t count) const{
  // WREG: 0100 rrnn
  // rr = start register address (2bit)
  // nn = number of registers - 1 (2bit)
  const uint8_t rr = static_cast<uint8_t>(start_reg) & 0x03;
  const uint8_t nn = (count - 1u) & 0x03;
  return static_cast<uint8_t>(0x40 | (rr << 2) | nn);
}

// ------------------------------------------------------------
// device commands
// ------------------------------------------------------------
void ADS1220::reset(){
  spi_->beginTransaction(spi_set_);
  csLow_();
  spi_->transfer(static_cast<uint8_t>(Command::RESET));
  csHigh_();
  spi_->endTransaction();

  delayMicroseconds(50);
}

void ADS1220::startSync(){
  spi_->beginTransaction(spi_set_);
  csLow_();
  spi_->transfer(static_cast<uint8_t>(Command::STARTSYNC));
  csHigh_();
  spi_->endTransaction();
}

void ADS1220::powerDown(){
  spi_->beginTransaction(spi_set_);
  csLow_();
  spi_->transfer(static_cast<uint8_t>(Command::POWERDOWN));
  csHigh_();
  spi_->endTransaction();
}

// ------------------------------------------------------------
// register read / write
// ------------------------------------------------------------
uint8_t ADS1220::readRegister(Reg reg){
  uint8_t value = 0;

  spi_->beginTransaction(spi_set_);
  csLow_();
  spi_->transfer(buildRregCmd_(reg, 1));
  value = spi_->transfer(0x00);
  csHigh_();
  spi_->endTransaction();

  return value;
}

void ADS1220::readRegisters(Reg start_reg, uint8_t* dst, uint8_t count){
  spi_->beginTransaction(spi_set_);
  csLow_();
  spi_->transfer(buildRregCmd_(start_reg, count));

  for (uint8_t i = 0; i < count; ++i) {
    dst[i] = spi_->transfer(0x00);
  }

  csHigh_();
  spi_->endTransaction();
}

void ADS1220::writeRegister(Reg reg, uint8_t value){
  spi_->beginTransaction(spi_set_);
  csLow_();
  spi_->transfer(buildWregCmd_(reg, 1));
  spi_->transfer(value);
  csHigh_();
  spi_->endTransaction();
}

void ADS1220::writeRegisters(Reg start_reg, const uint8_t* src, uint8_t count){
  spi_->beginTransaction(spi_set_);
  csLow_();
  spi_->transfer(buildWregCmd_(start_reg, count));

  for (uint8_t i = 0; i < count; ++i) {
    spi_->transfer(src[i]);
  }

  csHigh_();
  spi_->endTransaction();
}

// ------------------------------------------------------------
// (Important) config packers
// ------------------------------------------------------------
uint8_t ADS1220::buildConfig0_(Mux mux, Gain gain, PgaBypass pga_bypass) const{
  uint8_t reg = 0;
  reg |= (static_cast<uint8_t>(mux) & 0x0F) << 4;   // MUX[7:4]
  reg |= (static_cast<uint8_t>(gain) & 0x07) << 1;  // GAIN[3:1]
  reg |= (static_cast<uint8_t>(pga_bypass) & 0x01); // PGA_BYPASS[0]
  return reg;
}

uint8_t ADS1220::buildConfig1_(DataRate dr, OpMode op_mode, ConvMode conv_mode,
                               TempSensor ts, BurnoutCurrent bcs) const{
  uint8_t reg = 0;
  reg |= (static_cast<uint8_t>(dr) & 0x07) << 5;         // DR[7:5]
  reg |= (static_cast<uint8_t>(op_mode) & 0x03) << 3;    // MODE[4:3]
  reg |= (static_cast<uint8_t>(conv_mode) & 0x01) << 2;  // CM[2]
  reg |= (static_cast<uint8_t>(ts) & 0x01) << 1;         // TS[1]
  reg |= (static_cast<uint8_t>(bcs) & 0x01);             // BCS[0]
  return reg;
}

uint8_t ADS1220::buildConfig2_(Vref vref, FirMode fir, LowSideSwitch psw,
                               IdacCurrent idac) const{
  uint8_t reg = 0;
  reg |= (static_cast<uint8_t>(vref) & 0x03) << 6;   // VREF[7:6]
  reg |= (static_cast<uint8_t>(fir) & 0x03) << 4;    // 50/60[5:4]
  reg |= (static_cast<uint8_t>(psw) & 0x01) << 3;    // PSW[3]
  reg |= (static_cast<uint8_t>(idac) & 0x07);        // IDAC[2:0]
  return reg;
}

uint8_t ADS1220::buildConfig3_(IdacMux i1mux, IdacMux i2mux, DrdyMode drdym) const{
  uint8_t reg = 0;
  reg |= (static_cast<uint8_t>(i1mux) & 0x07) << 5;  // I1MUX[7:5]
  reg |= (static_cast<uint8_t>(i2mux) & 0x07) << 2;  // I2MUX[4:2]
  reg |= (static_cast<uint8_t>(drdym) & 0x01) << 1;  // DRDYM[1]
  // bit0 reserved = 0
  return reg;
}

// ------------------------------------------------------------
// config parsers
// ------------------------------------------------------------
void ADS1220::parseConfig0_(uint8_t reg, Config& cfg) const{
  cfg.mux = static_cast<Mux>((reg >> 4) & 0x0F);
  cfg.gain = static_cast<Gain>((reg >> 1) & 0x07);
  cfg.pga_bypass = static_cast<PgaBypass>(reg & 0x01);
}

void ADS1220::parseConfig1_(uint8_t reg, Config& cfg) const{
  cfg.dr = static_cast<DataRate>((reg >> 5) & 0x07);
  cfg.op_mode = static_cast<OpMode>((reg >> 3) & 0x03);
  cfg.conv_mode = static_cast<ConvMode>((reg >> 2) & 0x01);
  cfg.temp_sensor = static_cast<TempSensor>((reg >> 1) & 0x01);
  cfg.burnout = static_cast<BurnoutCurrent>(reg & 0x01);
}

void ADS1220::parseConfig2_(uint8_t reg, Config& cfg) const{
  cfg.vref = static_cast<Vref>((reg >> 6) & 0x03);
  cfg.fir = static_cast<FirMode>((reg >> 4) & 0x03);
  cfg.psw = static_cast<LowSideSwitch>((reg >> 3) & 0x01);
  cfg.idac = static_cast<IdacCurrent>(reg & 0x07);
}

void ADS1220::parseConfig3_(uint8_t reg, Config& cfg) const{
  cfg.idac1_mux = static_cast<IdacMux>((reg >> 5) & 0x07);
  cfg.idac2_mux = static_cast<IdacMux>((reg >> 2) & 0x07);
  cfg.drdy_mode = static_cast<DrdyMode>((reg >> 1) & 0x01);
}

// ------------------------------------------------------------
// (Important) high-level config helpers
// ------------------------------------------------------------
void ADS1220::applyConfig(const Config& cfg){
  uint8_t regs[4];
  regs[0] = buildConfig0_(cfg.mux, cfg.gain, cfg.pga_bypass);
  regs[1] = buildConfig1_(cfg.dr, cfg.op_mode, cfg.conv_mode,
                          cfg.temp_sensor, cfg.burnout);
  regs[2] = buildConfig2_(cfg.vref, cfg.fir, cfg.psw, cfg.idac);
  regs[3] = buildConfig3_(cfg.idac1_mux, cfg.idac2_mux, cfg.drdy_mode);

  writeRegisters(Reg::CONFIG0, regs, 4);
}

void ADS1220::readConfig(Config& cfg){
  uint8_t regs[4] = {0, 0, 0, 0};
  readRegisters(Reg::CONFIG0, regs, 4);

  parseConfig0_(regs[0], cfg);
  parseConfig1_(regs[1], cfg);
  parseConfig2_(regs[2], cfg);
  parseConfig3_(regs[3], cfg);
}

// ------------------------------------------------------------
// data ready wait
// ------------------------------------------------------------
bool ADS1220::waitDrdy(uint32_t timeout_us){
  const uint32_t t0 = micros();

  if (drdy_ >= 0) {
    while (digitalRead((uint8_t)drdy_) != LOW) {
      if (timeout_us > 0 && (micros() - t0) > timeout_us) {
        return false;
      }
    }
    return true;
  }

  if (timeout_us > 0) {
    while ((micros() - t0) < timeout_us) {
      /* wait */
    }
  }

  return true;
}

// ------------------------------------------------------------
// raw conversion read
// ------------------------------------------------------------
bool ADS1220::readRaw(int32_t& out_raw, uint32_t timeout_us){
  if (!waitDrdy(timeout_us)) {
    return false;
  }

  uint8_t b0 = 0;
  uint8_t b1 = 0;
  uint8_t b2 = 0;

  spi_->beginTransaction(spi_set_);
  csLow_();

  spi_->transfer(static_cast<uint8_t>(Command::RDATA));
  b0 = spi_->transfer(0x00);
  b1 = spi_->transfer(0x00);
  b2 = spi_->transfer(0x00);

  csHigh_();
  spi_->endTransaction();

  int32_t raw = (static_cast<int32_t>(b0) << 16) |
                (static_cast<int32_t>(b1) << 8)  |
                (static_cast<int32_t>(b2));

  // 24-bit sign extension
  if (raw & 0x800000L) {
    raw |= 0xFF000000L;
  }

  out_raw = raw;
  return true;
}
