#include <Arduino.h>
#include <SPI.h>
#include "ads1220.hpp"

// ============================================================
// 여기를 실제 배선에 맞게 수정
// ============================================================
constexpr uint8_t ADS_CS_PIN   = 9;
constexpr int8_t  ADS_DRDY_PIN = 2;

// constexpr uint8_t ADS_CS_PIN   = 10;
// constexpr int8_t  ADS_DRDY_PIN = 3;

// static const uint8_t PIN_CS_ENC   = 9; //PCB pin setup
// static const uint8_t PIN_CS_LC    = 10;
// static const uint8_t PIN_DRDY_ENC = 2;
// static const uint8_t PIN_DRDY_LC  = 3;

// SPI 생존 테스트에서는 일부러 느리게 사용
constexpr uint32_t SPI_SPEED = 100000;   // 100 kHz

ADS1220 adc(ADS_CS_PIN, ADS_DRDY_PIN);


// ------------------------------------------------------------
// 1 byte를 2자리 HEX로 출력
// 예: 0x08 -> "08"
// ------------------------------------------------------------
void printHexByte(uint8_t value)
{
  if (value < 0x10) {
    Serial.print("0");
  }

  Serial.print(value, HEX);
}


// ------------------------------------------------------------
// 4개 register 출력
// ------------------------------------------------------------
void printRegisters(const char* title, const uint8_t regs[4])
{
  Serial.print(title);
  Serial.print(": ");

  for (int i = 0; i < 4; i++) {
    Serial.print("0x");
    printHexByte(regs[i]);

    if (i < 3) {
      Serial.print("  ");
    }
  }

  Serial.println();
}


// ------------------------------------------------------------
// 두 register 배열이 같은지 비교
// ------------------------------------------------------------
bool compareRegisters(const uint8_t a[4], const uint8_t b[4])
{
  for (int i = 0; i < 4; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }

  return true;
}


// ============================================================
// ADS1220 SPI 생존 테스트
// ============================================================
void testADS1220()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("ADS1220 SPI register read/write test");
  Serial.println("========================================");


  // ==========================================================
  // STEP 1. RESET
  // ==========================================================
  Serial.println();
  Serial.println("[STEP 1] RESET");

  adc.reset();

  // ADS1220 RESET 후 충분히 기다린다.
  // 현재 ads1220.cpp의 reset() 안에는 50 us delay가 있지만
  // 여기서 추가 여유 시간을 준다.
  delayMicroseconds(200);


  // ==========================================================
  // STEP 2. RESET 직후 register 읽기
  // ==========================================================
  Serial.println("[STEP 2] Read registers after RESET");

  uint8_t resetRegs[4] = {0, 0, 0, 0};

  adc.readRegisters(
    ADS1220::Reg::CONFIG0,
    resetRegs,
    4
  );

  printRegisters("Read", resetRegs);

  const uint8_t expectedReset[4] = {
    0x00,
    0x00,
    0x00,
    0x00
  };

  if (compareRegisters(resetRegs, expectedReset)) {
    Serial.println("RESET register check : OK");
  }
  else {
    Serial.println("RESET register check : WRONG");
  }


  // ==========================================================
  // STEP 3. 우리가 정한 값을 ADS1220에 써본다.
  // ==========================================================
  Serial.println();
  Serial.println("[STEP 3] Write test values");

  const uint8_t testRegs[4] = {
    0x08,   // CONFIG0
    0x04,   // CONFIG1
    0x10,   // CONFIG2
    0x00    // CONFIG3
  };

  printRegisters("Write", testRegs);

  adc.writeRegisters(
    ADS1220::Reg::CONFIG0,
    testRegs,
    4
  );

  delayMicroseconds(20);


  // ==========================================================
  // STEP 4. 방금 쓴 값을 다시 읽는다.
  // ==========================================================
  Serial.println();
  Serial.println("[STEP 4] Read them back");

  uint8_t readbackRegs[4] = {0, 0, 0, 0};

  adc.readRegisters(
    ADS1220::Reg::CONFIG0,
    readbackRegs,
    4
  );

  printRegisters("Read ", readbackRegs);


  // ==========================================================
  // STEP 5. 쓴 값과 읽은 값을 비교
  // ==========================================================
  Serial.println();
  Serial.println("[STEP 5] Compare");

  if (compareRegisters(testRegs, readbackRegs)) {

    Serial.println();
    Serial.println("*************** PASS ***************");
    Serial.println("ADS1220 SPI communication is working.");
    Serial.println("Register WRITE and READ both succeeded.");
    Serial.println("************************************");

  }
  else {

    Serial.println();
    Serial.println("*************** FAIL ***************");
    Serial.println("Register readback does not match.");
    Serial.println("Check power, CS, SCLK, MOSI/DIN,");
    Serial.println("MISO/DOUT, soldering, and SPI pins.");
    Serial.println("************************************");
  }


  // ==========================================================
  // STEP 6. 테스트하면서 바꾼 register를 원상복구
  // ==========================================================
  Serial.println();
  Serial.println("[STEP 6] RESET again to restore defaults");

  adc.reset();
  delayMicroseconds(200);

  uint8_t finalRegs[4] = {0, 0, 0, 0};

  adc.readRegisters(
    ADS1220::Reg::CONFIG0,
    finalRegs,
    4
  );

  printRegisters("Final", finalRegs);

  Serial.println();
  Serial.println("Test finished.");
}


void setup()
{
  Serial.begin(115200);

  delay(1500);

  Serial.println();
  Serial.println("Starting ADS1220...");

  // 첨부한 라이브러리의 begin()이
  // SPI_MODE1 + MSBFIRST를 설정함
  adc.begin(SPI, SPI_SPEED);

  delay(100);

  testADS1220();
}


void loop()
{
  // 한 번만 검사하므로 비워둔다.
}