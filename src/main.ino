#include <RFM69wsx.h>
#include <SPI.h>
#include <Wire.h>

#define SERIAL_BAUD 115200
#define DEBUG(input)   Serial.print(input);
#define DEBUGln(input) Serial.println(input);
#define SERIALFLUSH()  Serial.flush();

#define LISTEN_FREQUENCY 868350000L  // 868.35 MHz

RFM69wsx radio;
uint32_t lastRx = 0;

char buff[50];

void setup(void) {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);

  radio.initialize(LISTEN_FREQUENCY);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); // must include this only for RFM69HW/HCW!
#endif

  for (uint8_t i = 0; i <= A5; i++) {
    if (i == RF69_SPI_CS) continue;
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  sprintf(buff, "# listening at: %ld Hz", LISTEN_FREQUENCY);
  Serial.println(buff);
}

void loop() {
  uint32_t now = micros();
  if (radio.receiveDone()) {
    if (radio.DATA[0] != 0x90) return;

    uint8_t crc = crc8(radio.DATA, 31, 0x31, 0x00);
    uint8_t chk = add_bytes(radio.DATA, 31);
    if (crc != 0 || chk != radio.DATA[31]) {
      // Serial.println("crc:err");
      return;
    }

    Serial.print("raw:");
    for (byte i = 0; i < 32; i++) {
      if (radio.DATA[i] < 0x10) Serial.print('0');
      Serial.print(radio.DATA[i], HEX);
    }

    Serial.print(" rssi:");
    Serial.print(radio.RSSI);
    Serial.print(" fei:");
    Serial.print(round(radio.FEI * RF69_FSTEP / 1000));
    if (lastRx > 0) {
      Serial.print(" delay:");
      Serial.print(now - lastRx);
    }
    Serial.println(" crc:ok");

    lastRx = now;
    blink(3);
  }
}

int add_bytes(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i) {
        result += message[i];
    }
    return result;
}

uint8_t crc8(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    uint8_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 0x80) {
                remainder = (remainder << 1) ^ polynomial;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

void blink(int blinkDelay) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(blinkDelay);
  digitalWrite(LED_BUILTIN, LOW);
}
