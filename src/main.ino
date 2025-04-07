#include <RFM69wsx.h>
#include <SPI.h>
#include <Wire.h>

// #define SENSOR_TYPE_EMULATED
#define SENSOR_TYPE_BMP388
#define I2C_BMP388 0x76

#ifdef SENSOR_TYPE_BMP388
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#endif

#define NAME_VERSION F("wsx-receiver v2025040701")

#define SERIAL_BAUD 115200
#define DEBUG(input)   Serial.print(input);
#define DEBUGln(input) Serial.println(input);
#define SERIALFLUSH()  Serial.flush();

#define LISTEN_FREQUENCY 868350000L  // 868.35 MHz

RFM69wsx radio;

#ifdef SENSOR_TYPE_BMP388
Adafruit_BMP3XX bmp388;
#endif

uint32_t lastRx = 0, now = 0;
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

#ifdef SENSOR_TYPE_BMP388
  bmp388.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp388.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp388.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  delay(10); // safe sensor init
  bmp388.begin_I2C(I2C_BMP388);
  bmp388.performReading();
#endif

  printBanner();
  blink(750);
}

void loop() {
  if (radio.receiveDone()) {
    if (radio.DATA[0] != 0x90) return;

    uint8_t crc = crc8(radio.DATA, 31, 0x31, 0x00);
    uint8_t chk = add_bytes(radio.DATA, 31);
    if (crc != 0 || chk != radio.DATA[31]) {
      return;
    }

    // Serial.print("raw:");
    // for (byte i = 0; i < 32; i++) {
    //   if (radio.DATA[i] < 0x10) Serial.print('0');
    //   Serial.print(radio.DATA[i], HEX);
    // }

    printPacket(radio.DATA);
    printTHP();

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

// Print received, decoded json object
// Code fragments for decoding taken from: https://github.com/merbanan/rtl_433/blob/master/src/devices/fineoffset_ws90.c
void printPacket(uint8_t *b) {
  uint32_t id         = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | ((uint32_t)b[3]);
  uint16_t light_raw  = (b[4] << 8) | (b[5]);
  uint32_t light_lux  = light_raw * 10;  // Lux
  int battery_mv      = (b[6] * 20);  // mV
  int battery_lvl     = battery_mv < 1400 ? 0 : (battery_mv - 1400) / 16; // 1.4V-3.0V is 0-100
  int flags           = b[7]; // to find the wind msb
  int temp_raw        = ((flags & 0x03) << 8) | (b[8]);
  float temp_c        = (temp_raw - 400) * 0.1f;
  int humidity        = (b[9]);
  int wind_avg        = ((flags & 0x10) << 4) | (b[10]);
  int wind_dir        = ((flags & 0x20) << 3) | (b[11]);
  int wind_max        = ((flags & 0x40) << 2) | (b[12]);
  int uv_index        = (b[13]);
  int rain_raw        = (b[19] << 8 ) | (b[20]);
  int rain_start      = (b[16] & 0x10) >> 4;
  int supercap_V      = (b[21] & 0x3f);
  int firmware        = b[29];

  if (battery_lvl > 100) battery_lvl = 100;

  Serial.print(F("{\"model\":\"Fineoffset-WS90\",\"mic\":\"CRC\","));
  printJsonAttr(F("id"), id, true);
  printJsonAttr(F("battery_ok"), battery_lvl * 0.01f, true);
  printJsonAttr(F("battery_mV"), battery_mv, true);
  if (temp_raw != 0x3ff) printJsonAttr(F("temperature_C"), temp_c, true);
  if (humidity != 0xff) printJsonAttr(F("humidity"), humidity, true);
  if (wind_dir != 0x1ff) printJsonAttr(F("wind_dir_deg"), wind_dir, true);
  if (wind_avg != 0x1ff) printJsonAttr(F("wind_avg_m_s"), wind_avg * 0.1f, true);
  if (wind_max != 0x1ff) printJsonAttr(F("wind_max_m_s"), wind_max * 0.1f, true);
  if (uv_index != 0xff) printJsonAttr(F("uvi"), uv_index * 0.1f, true);
  if (light_raw != 0xffff) printJsonAttr(F("light_lux"), (float)light_lux, true);
  printJsonAttr(F("flags"), flags, true);
  printJsonAttr(F("rain_mm"), rain_raw * 0.1f, true);
  printJsonAttr(F("rain_start"), rain_start, true);
  if (supercap_V != 0xff) printJsonAttr(F("supercap_V"), supercap_V * 0.1f, true);
  printJsonAttr(F("firmware"), firmware, true);
  printJsonAttr(F("rssi"), radio.RSSI, true);
  printJsonAttr(F("fei"), round(radio.FEI * RF69_FSTEP / 1000), true);
  printJsonAttr(F("delay"), now - lastRx, false);
  Serial.println('}');
}

// Print T/H/P json object emulating a Fineoffset WH25 station
void printTHP() {
  float P = 0, T = 0;
  int H = -1;

#ifdef SENSOR_TYPE_BMP388
  bmp388.performReading();
  T = bmp388.temperature;
  P = bmp388.pressure;
#endif

#ifdef SENSOR_TYPE_EMULATED
  T = 25.0;
  P = 101320.0;
  H = 45;
#endif

  // {"time" : "2021-04-08 18:11:01", "model" : "Fineoffset-WH25", "id" : 121, 
  //  "battery_ok" : 1, "temperature_C" : 20.000, "humidity" : 48, "pressure_hPa" : 979.100, "mic" : "CRC"}

  // time is added by the host
  Serial.print(F("{\"model\":\"Fineoffset-WH25\",\"id\":555,\"battery_ok\":1,\"mic\":\"CRC\","));
  printJsonAttr(F("temperature_C"), T, true);
  printJsonAttr(F("pressure_hPa"), P, H != -1);
  if (H != -1) printJsonAttr(F("humidity"), H, false);
  Serial.println('}');
}

void printJsonPre(const __FlashStringHelper * name) {
  Serial.print('"');
  Serial.print(name);
  Serial.print(F("\":"));
}

void printJsonPost(bool addComma) {
  Serial.print('"');
  if (addComma) Serial.print(',');
}

void printJsonAttr(const __FlashStringHelper * name, float value, bool addComma) {
  printJsonPre(name);
  Serial.print(value, 1);
  printJsonPost(addComma);
}

void printJsonAttr(class __FlashStringHelper * name, char *value, bool addComma) {
  printJsonPre(name);
  Serial.print(value);
  printJsonPost(addComma);
}

void printBanner()
{
  Serial.print(F("# "));
  Serial.print(NAME_VERSION);
  sprintf(buff, " - listening at: %ld Hz", LISTEN_FREQUENCY);
  Serial.println(buff);
}

void blink(int blinkDelay) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(blinkDelay);
  digitalWrite(LED_BUILTIN, LOW);
}
