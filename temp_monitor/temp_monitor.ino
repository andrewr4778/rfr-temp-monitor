// RFR temp monitor - Nano + CD74HC4067 mux, 16 diode temp sensors, reports over I2C
// sensors: 10k pullup to 5V + diode to gnd, diode voltage DROPS ~2mV per degC
// reg map:
//   0x00 status  (bit0 = fault, bit1 = sensor error, bit2 = still booting)
//   0x01 fault map high, 0x02 fault map low
//   0x03 heartbeat (goes up by 1 every full scan, master should check it changes)
//   0x04+ temps, 2 bytes each (high, low), degC x10
//         0x7FFF = channel not used, 0x7FFE = sensor bad (open/short)
// NOTE: wire buffer is only 32 bytes so master has to read the map in 2 chunks

#include <Wire.h>
#include <avr/wdt.h>

// #define PHYSICAL_CD4051BE   // only uncomment this for the old 8 ch CD4051 board

#ifdef PHYSICAL_CD4051BE
  const uint8_t NUM_CH        = 8;
  const uint8_t MUX_SEL_BITS  = 3;
  const uint16_t SETTLE_US    = 500;
#else
  const uint8_t NUM_CH        = 16;
  const uint8_t MUX_SEL_BITS  = 4;
  const uint16_t SETTLE_US    = 500;   // 1k + 100nF filter is ~100us, wait ~5x that
#endif

const int16_t TEMP_UNUSED = 0x7FFF;
const int16_t TEMP_BAD    = 0x7FFE;

const uint8_t I2C_ADDR   = 0x40;
const uint8_t MUX_S[4]   = {2, 3, 4, 5};
// mux enable pin is tied to gnd on the board so we dont use it
const uint8_t LED_FAULT  = 9;   // matches schematic (R20 + red LED on D9)
const uint8_t FAULT_N    = 8;   // not wired in the schematic yet, needs external pullup, we only pull it low
const uint8_t ADC_PIN    = A0;

const int16_t  FAULT_SET_X10 = 600;   // 60.0C trips
const int16_t  FAULT_CLR_X10 = 580;   // 58.0C clears (hysteresis)
const uint8_t  OVER_COUNT    = 2;     // scans in a row over the limit before we fault
const uint16_t ADC_RAIL      = 1023;
const uint16_t ADC_OPEN_MAX  = 10;    // ~1C, anything lower we call a shorted/open sensor
const uint8_t  N_AVG         = 8;

// internal ref is NOT exactly 1.1V (datasheet says 1.0 - 1.2V)
// TODO: measure AREF with a multimeter on each board and put the real number here
const uint32_t VREF_MV = 1100;

// diode sensor: temp (C x10) = 250 + (VF25 - mV) * 5   (2mV per degC, 1mV = 0.5C = 5 in x10)
// TODO: measure each diode at a known temp (room temp ~25C) and put its voltage in here,
//       diodes vary a lot from part to part
const int16_t VF25_MV[16] = {600, 600, 600, 600, 600, 600, 600, 600,
                             600, 600, 600, 600, 600, 600, 600, 600};

const uint8_t REG_STATUS = 0x00;
const uint8_t REG_FMAP_H = 0x01;
const uint8_t REG_FMAP_L = 0x02;
const uint8_t REG_BEAT   = 0x03;
const uint8_t REG_TEMP0  = 0x04;
const uint8_t REG_SIZE   = 0x24;

volatile uint8_t regs[REG_SIZE];
volatile uint8_t regPtr = 0;

uint16_t faultMap = 0;
uint16_t errMap   = 0;
uint8_t  overCount[16];
uint8_t  beat = 0;
bool     booting = true;   // stays true until we finish the first full scan

void selectChannel(uint8_t ch) {
  for (uint8_t i = 0; i < MUX_SEL_BITS; i++) {
    digitalWrite(MUX_S[i], (ch >> i) & 1);
  }
}

int16_t readTempX10(uint8_t ch, uint16_t &adcOut) {
  selectChannel(ch);
  delayMicroseconds(SETTLE_US);
  analogRead(ADC_PIN);   // throw away the first one after switching the mux

  uint32_t sum = 0;
  for (uint8_t i = 0; i < N_AVG; i++) sum += analogRead(ADC_PIN);
  uint16_t adc = sum / N_AVG;
  adcOut = adc;

  // adc -> mV, then diode voltage -> temp (lower voltage = hotter)
  int16_t mv = (int16_t)((adc * VREF_MV) / 1024UL);
  return 250 + (VF25_MV[ch] - mv) * 5;
}

void updateChannelState(uint8_t ch, int16_t tX10, uint16_t adc) {
  uint16_t bit = (1u << ch);

  bool bad = (adc >= ADC_RAIL) || (adc <= ADC_OPEN_MAX);
  if (bad) errMap |= bit; else errMap &= ~bit;

  if (bad) {
    // bad sensor = fault right away
    faultMap |= bit;
    overCount[ch] = 0;
  } else if (tX10 > FAULT_SET_X10) {
    // over temp, make sure its not just one noisy scan
    if (overCount[ch] < OVER_COUNT) overCount[ch]++;
    if (overCount[ch] >= OVER_COUNT) faultMap |= bit;
  } else {
    overCount[ch] = 0;
    if ((faultMap & bit) && tX10 < FAULT_CLR_X10) {
      faultMap &= ~bit;
    }
  }
}

void publishChannel(uint8_t ch, int16_t tX10) {
  uint8_t idx = REG_TEMP0 + 2 * ch;
  noInterrupts();
  regs[idx]     = (uint8_t)(tX10 >> 8);
  regs[idx + 1] = (uint8_t)(tX10 & 0xFF);
  interrupts();
}

void publishStatus() {
  uint8_t status = 0;
  if (faultMap) status |= 0x01;
  if (errMap)   status |= 0x02;
  if (booting)  status |= 0x04;

  noInterrupts();
  regs[REG_STATUS] = status;
  regs[REG_FMAP_H] = (uint8_t)(faultMap >> 8);
  regs[REG_FMAP_L] = (uint8_t)(faultMap & 0xFF);
  regs[REG_BEAT]   = beat;
  interrupts();
}

void driveFaultOutputs() {
  // also hold the fault line low while booting so "no data yet" doesnt look like "all good"
  bool fault = (faultMap != 0) || booting;
  digitalWrite(LED_FAULT, fault ? HIGH : LOW);

  if (fault) {
    digitalWrite(FAULT_N, LOW);
    pinMode(FAULT_N, OUTPUT);
  } else {
    pinMode(FAULT_N, INPUT);   // let the pullup take it high
  }
}

void onI2CReceive(int n) {
  if (n < 1) return;
  uint8_t p = Wire.read();
  regPtr = (p < REG_SIZE) ? p : 0;
  while (Wire.available()) Wire.read();
}

void onI2CRequest() {
  uint8_t len = REG_SIZE - regPtr;
  if (len > 32) len = 32;   // wire buffer limit
  Wire.write((const uint8_t *)&regs[regPtr], len);
}

void setup() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MUX_S[i], OUTPUT);
    digitalWrite(MUX_S[i], LOW);
  }

  pinMode(LED_FAULT, OUTPUT);

  // start out in fault/booting state
  pinMode(FAULT_N, INPUT);
  digitalWrite(LED_FAULT, HIGH);
  digitalWrite(FAULT_N, LOW);
  pinMode(FAULT_N, OUTPUT);

  // temps start as "unused" so master never reads a fake 0.0C
  for (uint8_t ch = 0; ch < 16; ch++) publishChannel(ch, TEMP_UNUSED);
  publishStatus();   // sets the booting bit

  analogReference(INTERNAL);
  for (uint8_t i = 0; i < 4; i++) analogRead(ADC_PIN);

  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  // if the loop ever hangs the nano resets itself
  // (old nano bootloader can get stuck in a reset loop with the watchdog,
  //  if that happens flash the optiboot bootloader or take this line out)
  wdt_enable(WDTO_250MS);
}

void loop() {
  for (uint8_t ch = 0; ch < NUM_CH; ch++) {
    uint16_t adc;
    int16_t tX10 = readTempX10(ch, adc);
    updateChannelState(ch, tX10, adc);

    bool bad = errMap & (1u << ch);
    publishChannel(ch, bad ? TEMP_BAD : tX10);
  }

  booting = false;   // got through one full scan
  beat++;

  publishStatus();
  driveFaultOutputs();
  wdt_reset();
}
