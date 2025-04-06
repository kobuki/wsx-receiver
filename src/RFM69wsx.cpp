#include "RFM69registers.h"
#include "RFM69wsx.h"

int16_t RFM69wsx::FEI;

bool RFM69wsx::initialize(uint32_t freq)
{
  _interruptNum = digitalPinToInterrupt(_interruptPin);
  if (_interruptNum == (uint8_t)NOT_AN_INTERRUPT) return false;
#ifdef RF69_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    _interruptNum = _interruptPin;
#endif
  const uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, 0x07}, // 17.241 kbps
    /* 0x04 */ { REG_BITRATELSB, 0x40},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_35000}, // WS90 Fdev = 35 kHz
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_35000},
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    /* 0x1E */ { REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON | RF_AFCFEI_AFCAUTO_ON },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    /* 0x2D */ { REG_PREAMBLELSB, 6 }, // WS90 preamble length is 6 (6 x 0xAA)
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, 0x2D }, // WS90 sync word is 0x2DD4
    /* 0x30 */ { REG_SYNCVALUE2, 0xD4 },
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_CRCAUTOCLEAR_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 32 },
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };

  digitalWrite(_slaveSelectPin, HIGH);
  pinMode(_slaveSelectPin, OUTPUT);
  if(_spi == nullptr){
    _spi = &SPI;
  }
#if defined(ESP32)
  _spi->begin(18,19,23,5); //SPI3  (SCK,MISO,MOSI,CS)
  //_spi->begin(14,12,13,15); //SPI2  (SCK,MISO,MOSI,CS) 
#else
  _spi->begin();
#endif  

#ifdef SPI_HAS_TRANSACTION
  _settings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
#endif

  uint32_t start = millis();
  uint8_t timeout = 50;
  do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1) != 0xaa && millis() - start < timeout);
  start = millis();
  do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55 && millis() - start < timeout);

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);

  RFM69::setFrequency(freq);
  encrypt(0);

  setHighPower(_isRFM69HW);
  setMode(RF69_MODE_STANDBY);
  start = millis();
  while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis() - start < timeout);
  if (millis() - start >= timeout)
    return false;
  attachInterrupt(_interruptNum, RFM69::isr0, RISING);

  return true;
}

void RFM69wsx::interruptHandler() {
  if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    setMode(RF69_MODE_STANDBY);
    DATALEN = PAYLOADLEN = 32;

    select();
    _spi->transfer(REG_FIFO & 0x7F);
    for (uint8_t i = 0; i < DATALEN; i++) DATA[i] = _spi->transfer(0);
    unselect();

    setMode(RF69_MODE_RX);
  }
  RSSI = readRSSI();
  FEI = word(readReg(REG_FEIMSB), readReg(REG_FEILSB));
}

bool RFM69wsx::receiveDone() {
  if (_haveData) {
  	_haveData = false;
  	interruptHandler();
  }
  if (_mode == RF69_MODE_RX && PAYLOADLEN > 0)
  {
    setMode(RF69_MODE_STANDBY); // enables interrupts
    return true;
  }
  else if (_mode == RF69_MODE_RX) // already in RX no payload yet
  {
    return false;
  }
  receiveBegin();
  return false;
}
