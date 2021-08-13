// Uncomment the next line if uploading to Pavel
//#define Pavel 1
// Comment out this lime if you need the silent version:
// only incoming packets will be displayed in the serial monitor.
// If something doesn't work you won't know it though, unless
// you plug in an OLED.
#define NEED_DEBUG 1
// Uncomment this next line if you want to use a BME680
// #define NEED_BME 1
// Uncomment this next line if you want to use pins 5 & 6 for Gnd/Vcc
// Particularly useful on a breadboard as they are next to SDA/SCL
// #define NEED_SIDE_I2C 1
// Uncomment this next line if you want to use a DHTxx
//#define NEED_DHT
// Uncomment this next line if you want to use an SSD1306 OLED
#define NEED_SSD1306 1
// Detect the presence of an OLED at 0x2c/0x3d
// Larry Bank has code for that
// Uncomment this next line if you want to use an HDC1080
//#define NEED_HDC1080 1
// Uncomment this next line if you want to use a CCS811
//#define NEED_CCS811 1
// Uncomment this next line if you want to use an SGP30
//#define NEED_SGP30 1
// Uncomment this next line if you want to use an EEPROM
// #define NEED_EEPROM
// #define NEED_SHATEST

#include <SPI.h>
#include <Wire.h>
#include <SX126x-RAK4630.h>
// Click here to get the library: http://librarymanager/All#LoRa
#include "Sx1262LoRandom.h"
#include "aes.c"
#include "sha2.c"

static RadioEvents_t RadioEvents;

/*
  NOTE!
  Add:
  namespace std _GLIBCXX_VISIBILITY(default) {
    _GLIBCXX_BEGIN_NAMESPACE_VERSION
    void __throw_length_error(char const*) {}
    void __throw_bad_alloc() {}
    void __throw_out_of_range(char const*) {}
    void __throw_logic_error(char const*) {}
  }
  to ~/Library/Arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/arm-none-eabi/include/c++/4.8.3/bits/basic_string.h

  Add:
   -D_GLIBCXX_USE_C99
  to compiler.c.flags & compiler.cpp.flags.
  Or the code won't compile.
*/

/*
  If you're planning to usee an EEPROM, you need
  to define the buffer lengths in SparkFun_External_EEPROM.h
  Around line 56:
  #elif defined(_VARIANT_ELECTRONICCATS_BASTWAN_)
  #define I2C_BUFFER_LENGTH_RX SERIAL_BUFFER_SIZE
  #define I2C_BUFFER_LENGTH_TX SERIAL_BUFFER_SIZE
*/
#include "ArduinoJson.h"
// Click here to get the library: http://librarymanager/All#ArduinoJson

#ifdef NEED_SSD1306
#include "SSD1306Ascii.h"
// Click here to get the library: http://librarymanager/All#SSD1306Ascii
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C
#define RST_PIN -1
#define OLED_FORMAT &Adafruit128x64
SSD1306AsciiWire oled;
#endif // NEED_SSD1306

#ifdef NEED_SGP30
#include "sensirion_common.h"
#include "sgp30.h"
// https://github.com/Seeed-Studio/SGP30_Gas_Sensor
uint16_t tvoc_co2[2] = {0};
void displaySGP30();
#endif // NEED_SGP30

#ifdef NEED_HDC1080
#include <ClosedCube_HDC1080.h>
// Click here to get the library: http://librarymanager/All#ClosedCube_HDC1080
ClosedCube_HDC1080 hdc1080;
#define hdc1080_waitout 30000
double lastReading = 0;
float temp_hum_val[2] = {0};
#define PING_DELAY 300000 // 5 minutes
#ifdef NEED_CCS811 // Linked to NEED_HDC1080
#include <SparkFunCCS811.h>
// Click here to get the library: http://librarymanager/All#SparkFunCCS811
#define CCS811_ADDR 0x5A // Alternate I2C Address
uint16_t tvoc_co2[2] = {0};
#define PIN_NOT_WAKE 5
#define PIN_NOT_INT 6
CCS811 myCCS811(CCS811_ADDR);
#endif // NEED_CCS811
#endif // NEED_HDC1080

#ifdef NEED_BME
#include "ClosedCube_BME680.h"
// Click here to get the library: http://librarymanager/All#ClosedCube_BME680
ClosedCube_BME680 bme680;
double lastReading = 0;
#define PING_DELAY 300000 // 5 minutes
#endif // NEED_BME

#ifdef NEED_EEPROM
#include "SparkFun_External_EEPROM.h"
// Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
ExternalEEPROM myMem;
#endif // NEED_EEPROM

#ifdef NEED_DHT
#include "DHT.h"
// Click here to get the library: http://librarymanager/All#DHT_sensor_library
#define DHTPIN 9 // what pin we're connected to
#define DHTTYPE DHT22 // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
#define PING_DELAY 300000 // 5 minutes
double lastReading = 0;
float temp_hum_val[2] = {0};
void displayDHT();
#endif // NEED_DHT

#include "helper.h"
#include "haversine.h"
#include "SerialCommands.h"

/*
  Welcome to role-assigned values: each machine will have a specific role,
  and code will be compiled and run depending on who it is for.

  Pavel:
   - Outdoors (WHEN IT'S NOT RAINING) device.
   - BME680 inside
   OR
   - DHT22 inside
   - Possibly an OV5208 camera soon
*/

void setup() {
  // ---- HOUSEKEEPING ----
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  Serial.begin(115200);
  for (uint8_t i = 5; i > 0; i--) {
    delay(1000);
    Serial.print(i); Serial.print(", ");
  }
  digitalWrite(LED_BLUE, LOW);
  delay(1000);
  Serial.println(0);
#ifdef NEED_DEBUG
  Serial.println("\n\nWisblock at your service!");
#endif // NEED_DEBUG
#ifdef NEED_SIDE_I2C
  // this has to happen first, if the I2C bus is powered by 5/6
#ifdef NEED_DEBUG
  Serial.println(" - Set up I2C");
#endif // NEED_DEBUG
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(5, LOW); // Keyboard Featherwing I2C GND
  digitalWrite(6, HIGH); // Keyboard Featherwing I2C VCC
  // And obviously we can't display on the OLED yet...
#endif // NEED_SIDE_I2C

  Wire.begin();
  //Wire.setClock(100000);
#ifdef NEED_SSD1306
  Serial.println("Setting up OLED");
  // Initialising the UI will init the display too.
  oled.begin(OLED_FORMAT, I2C_ADDRESS);
  oled.setFont(System5x7);
#if INCLUDE_SCROLLING == 0
#error INCLUDE_SCROLLING must be non-zero. Edit SSD1306Ascii.h
#endif // INCLUDE_SCROLLING
  // Set auto scrolling at end of window.
  oled.setScrollMode(SCROLL_MODE_AUTO);
  oled.println("BastWAN Minimal LoRa");
#endif // NEED_SSD1306

#ifdef NEED_EEPROM
#ifdef NEED_DEBUG
  Serial.println(" - Start EEPROM");
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
  oled.println(" . Start EEPROM");
#endif // NEED_SSD1306
  if (myMem.begin() == false) {
#ifdef NEED_DEBUG
    Serial.println("   No memory detected. Freezing.");
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
    oled.println("No memory detected.");
    oled.println("Freezing...");
#endif // NEED_SSD1306
    while (1)
      ;
  }
  uint32_t myLen = myMem.length(), index = 0;
#ifdef NEED_DEBUG
  Serial.println("Memory detected!");
  Serial.print("Mem size in bytes: ");
  Serial.println(myLen);
#ifdef NEED_SSD1306
  oled.println("Memory detected!");
  oled.println("Size in bytes: ");
  oled.println(myLen);
#endif // NEED_SSD1306
#endif // NEED_DEBUG
  memset(msgBuf, 0, BUFF_LENGTH);
  myMem.read(0, msgBuf, 32);
  myMem.read(32, msgBuf + 32, 32);
  myMem.read(64, msgBuf + 64, 32);
  // Let's limit the JSON string size to 96 for now.
#ifdef NEED_DEBUG
  hexDump(msgBuf, 96);
#endif // NEED_DEBUG
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, msgBuf);
  if (error) {
#ifdef NEED_DEBUG
    Serial.println(F("\ndeserializeJson() failed!"));
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
    oled.println("JSON prefs fail.");
#endif // NEED_SSD1306
    savePrefs();
  }
  myFreq = doc["myFreq"];
  mySF = doc["mySF"] = mySF;
  myBW = doc["myBW"];
  myCR = doc["myCR"];
  const char *x = doc["deviceName"];
  Serial.println("setDeviceName in NEED_EEPROM");
  setDeviceName(x);
#ifdef NEED_DEBUG
  Serial.print("FQ: "); Serial.println(myFreq / 1e6);
  Serial.print("SF: "); Serial.println(mySF);
  Serial.print("BW: "); Serial.println(myBW);
  Serial.print("Device Name: "); Serial.println(deviceName);
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
  oled.println("JSON prefs fail.");
  oled.print("SF: "); oled.println(SF);
  oled.print("BW: "); oled.println(myBW);
#endif // NEED_SSD1306
#endif // NEED_EEPROM

#ifdef NEED_BME
  // ---- BME STUFF ----
#ifdef NEED_DEBUG
  Serial.println(" - ClosedCube BME680 ([T]emperature, [P]ressure, [H]umidity)");
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
  oled.println("ClosedCube BME680");
#endif // NEED_SSD1306
  bme680.init(0x77); // I2C address: 0x76 or 0x77
  bme680.reset();
#ifdef NEED_DEBUG
  Serial.print("Chip ID=0x");
  Serial.println(bme680.getChipID(), HEX);
#endif // NEED_DEBUG
  // oversampling: humidity = x1, temperature = x2, pressure = x16
  bme680.setOversampling(BME680_OVERSAMPLING_X1, BME680_OVERSAMPLING_X2, BME680_OVERSAMPLING_X16);
  bme680.setIIRFilter(BME680_FILTER_3);
  bme680.setForcedMode();
#endif // NEED_BME

#ifdef NEED_SSD1306
  oled.println("LoRa Setup");
#endif // NEED_SSD1306
  lora_rak4630_init();
  // fillRandom(randomStock, 256);
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = NULL;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = NULL;
  // Initialize the Radio
  Radio.Init(&RadioEvents);
  // Set Radio channel
  Radio.SetChannel(myFreq);
  // Set Radio RX configuration
  Radio.SetRxConfig(MODEM_LORA, myBW, mySF, myCR, 0, 8, 0, false, 0, true, 0, 0, false, true);
  // Start LoRa
  Serial.println("Starting Radio.Rx");
  Radio.Rx(3000);
  SX126xSetTxParams(TxPower, RADIO_RAMP_40_US);

#ifdef Pavel
  setDeviceName("Pavel");
#else
  setDeviceName("RAK4631");
#endif // Pavel
  pingFrequency = 60000;
  needPing = true;

#ifdef NEED_SSD1306
  oled.println("Device name:"); oled.println(deviceName);
#endif // NEED_SSD1306

#ifdef NEED_DHT
#ifdef NEED_SSD1306
  oled.println("DHT");
#endif // NEED_SSD1306
  dht.begin();
#endif // NEED_DHT

#ifdef NEED_SGP30
#ifdef NEED_SSD1306
  oled.println("SGP30");
#endif // NEED_SSD1306
  s16 err;
  u16 scaled_ethanol_signal, scaled_h2_signal;
  while (sgp_probe() != STATUS_OK) {
    Serial.println("SGP failed");
    digitalWrite(LED_BUILTIN, 0);
    delay(800);
    digitalWrite(LED_BUILTIN, 1);
    delay(800);
  }
  /*Read H2 and Ethanol signal in the way of blocking*/
  err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal, &scaled_h2_signal);
  if (err == STATUS_OK) {
    Serial.println("get ram signal!");
  } else {
    Serial.println("Error reading signals");
#ifdef NEED_SSD1306
    oled.println("Error reading signals");
#endif // NEED_SSD1306
  }
  err = sgp_iaq_init();
#endif // NEED_SGP30

#ifdef NEED_HDC1080
#ifdef NEED_SSD1306
  oled.println("HDC1080");
#endif // NEED_SSD1306
  Serial.println("HDC1080:");
  hdc1080.begin(0x40); // I2C address
  Serial.print("Manufacturer ID=0x");
  Serial.println(hdc1080.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
  Serial.print("Device ID=0x");
  Serial.println(hdc1080.readDeviceId(), HEX); // 0x1050 ID of the device
  Serial.print("Device Serial Number=");
  HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
  char format[12];
  sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
  Serial.println(format);
  HDC1080_Registers reg = hdc1080.readRegister();
  Serial.print("Battery: 0x");
  Serial.println(reg.BatteryStatus, HEX);
  Serial.print("Heater: 0x");
  Serial.println(reg.Heater, HEX);
  Serial.print("HumidityMeasurementResolution: 0x");
  Serial.println(reg.HumidityMeasurementResolution, HEX);
  Serial.print("TemperatureMeasurementResolution: 0x");
  Serial.println(reg.TemperatureMeasurementResolution, HEX);

#ifdef NEED_CCS811 // Linked to NEED_HDC1080
#ifdef NEED_SSD1306
  oled.println("CCS811");
#endif // NEED_SSD1306
  Serial.println("CCS811:");
  // This begins the CCS811 sensor and prints error status of .beginWithStatus()
  CCS811Core::CCS811_Status_e returnCode = myCCS811.beginWithStatus();
  Serial.print("CCS811 begin exited with: ");
  // Pass the error code to a function to print the results
  Serial.println(myCCS811.statusString(returnCode));
  // This sets the mode to 60 second reads, and prints returned error status.
  returnCode = myCCS811.setDriveMode(2);
  Serial.print("Mode request exited with: ");
  Serial.println(myCCS811.statusString(returnCode));
  // Configure and enable the interrupt line,
  // then print error status
  pinMode(PIN_NOT_INT, INPUT_PULLUP);
  returnCode = myCCS811.enableInterrupts();
  Serial.print("Interrupt configuration exited with: ");
  Serial.println(myCCS811.statusString(returnCode));
  // Configure the wake line
  pinMode(PIN_NOT_WAKE, OUTPUT);
  digitalWrite(PIN_NOT_WAKE, 1); // Start asleep
#endif // NEED_CCS811

#endif // NEED_HDC1080

#ifdef NEED_SSD1306
  oled.println("Sets");
#endif // NEED_SSD1306
  DeserializationError error = deserializeJson(sets, "{\"freq\":[868,868.125,868.125],\"sf\":[12,9,9],\"bw\":[9,8,6]}");
  if (error) {
#ifdef NEED_SSD1306
    oled.println("\ndeserializeJson failed");
#endif // NEED_SSD1306
#ifdef NEED_DEBUG
    Serial.println(F("\ndeserializeJson() in Sets failed!"));
    hexDump(msgBuf, BUFF_LENGTH);
#endif // NEED_DEBUG
  } else {
    setsFQ = sets["freq"];
    setsSF = sets["sf"];
    setsBW = sets["bw"];
    uint8_t i, j = setsFQ.size();
#ifdef NEED_DEBUG
    Serial.println("\n\n" + String(j) + " Sets:");
#endif // NEED_DEBUG
    for (i = 0; i < j; i++) {
      float F = setsFQ[i];
      int S = setsSF[i];
      int B = setsBW[i];
#ifdef NEED_DEBUG
      sprintf((char*)msgBuf, " . Freq: %3.3f MHz, SF %d, BW %d: %3.2f", F, S, B, BWs[B]);
      Serial.println((char*)msgBuf);
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
      oled.print("Freq["); oled.print(i); oled.print("]: "); oled.println(String(F, 3) + " MHz");
      oled.print("SF["); oled.print(i); oled.print("]: "); oled.println(S);
      oled.print("BW["); oled.print(i); oled.print("]: "); oled.print(B); oled.print(" ie "); oled.println(BWs[B]);
#endif // NEED_SSD1306
    }
  }
#ifdef NEED_DEBUG
  Serial.println("Setup done...");
#endif // NEED_DEBUG
#ifdef NEED_BME
  displayBME680();
  lastReading = millis();
#endif // NEED_BME
#ifdef NEED_DHT
  displayDHT();
  lastReading = millis();
#endif // NEED_DHT
#ifdef NEED_SGP30
  displaySGP30();
#endif // NEED_SGP30
#ifdef NEED_HDC1080
  displayHDC1080();
  lastReading = millis();
#endif // NEED_HDC1080
#ifdef NEED_SHATEST
  shaTest();
#endif // NEED_SHATEST
  digitalWrite(LED_GREEN, HIGH); // Turn off Green LED
}

void loop() {
  double t0 = millis();
#ifdef NEED_BME
  if (t0 - lastReading >= PING_DELAY) {
    displayBME680();
    lastReading = millis();
  }
#endif // NEED_BME

#ifdef NEED_DHT
  if (t0 - lastReading >= PING_DELAY) {
    displayDHT();
    lastReading = millis();
  }
#endif // NEED_DHT

#ifdef NEED_SGP30
  if (t0 - lastReading >= PING_DELAY) {
    displaySGP30();
    lastReading = millis();
  }
#endif // NEED_SGP30

#ifdef NEED_HDC1080
  if (t0 - lastReading >= PING_DELAY) {
    displayHDC1080();
    lastReading = millis();
  }
#endif // NEED_HDC1080

  // Uncomment if you have a battery plugged in.
  // if (millis() - batteryUpdateDelay > 10000) {
  // getBattery();
  // batteryUpdateDelay = millis();
  // }
  if (Serial.available()) {
    // When the BastMobile is connected via USB to a computer,
    // you can make changes to settings via Serial,
    // like in BastWAN_Minimal_LoRa
    handleSerial();
  }
  if (needPing) {
    double t0 = millis();
    if (t0 - lastAutoPing > pingFrequency) {
#ifdef NEED_SSD1306
      oled.println("Auto PING!");
#endif // NEED_SSD1306
      sendPing();
      // lastAutoPing = millis();
      // done in sendPing
    }
  }
}

#ifdef NEED_BME
void displayBME680() {
#ifdef NEED_SSD1306
  oled.println("displayBME680");
#endif // NEED_SSD1306
#ifdef NEED_DEBUG
  Serial.println("BME680");
#endif // NEED_DEBUG
  ClosedCube_BME680_Status status = bme680.readStatus();
  if (status.newDataFlag) {
    double temp = bme680.readTemperature();
    double pres = bme680.readPressure();
    double hum = bme680.readHumidity();
    // save the values in the same global array, so that they can be sent in packets
    temp_hum_val[0] = (float)hum;
    temp_hum_val[1] = (float)temp;
#ifdef NEED_SSD1306
    displayHT();
#endif // NEED_SSD1306
#ifdef NEED_DEBUG
    sprintf((char*)msgBuf, "result: T = % f C, RH = % f % %, P = % d hPa\n", temp, hum, pres);
    Serial.println((char*)msgBuf);
#endif // NEED_DEBUG
    lastReading = millis();
  }
}
#endif // NEED_DEBUG

#ifdef NEED_DHT
void displayDHT() {
#ifdef NEED_SSD1306
  oled.println("displayDHT");
#endif // NEED_SSD1306
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  //  if (dht.readTempAndHumidity(temp_hum_val)) {
  dht.readTempAndHumidity(temp_hum_val);
  Serial.print("Humidity: ");
  Serial.print(temp_hum_val[0]);
  Serial.print(" % \t");
  Serial.print("Temperature: ");
  Serial.print(temp_hum_val[1]);
  Serial.println(" *C");
#ifdef NEED_SSD1306
  displayHT();
#endif // NEED_SSD1306
  //  } else {
  //    Serial.println("Failed to get temperature and humidity value.");
  //  }
}
#endif // NEED_DHT

#ifdef NEED_HDC1080
void displayHDC1080() {
  char buff[48];
  temp_hum_val[0] = hdc1080.readHumidity();
  temp_hum_val[1] = hdc1080.readTemperature();
  sprintf(buff, "Temp: %2.2f C, Humidity: %2.2f%%\n", temp_hum_val[1], temp_hum_val[0]);
  Serial.print(buff);
#ifdef NEED_CCS811 // Linked to NEED_HDC1080
  // Look for interrupt request from CCS811
  if (digitalRead(PIN_NOT_INT) == 0) {
    // Wake up the CCS811 logic engine
    digitalWrite(PIN_NOT_WAKE, 0);
    // Need to wait at least 50 µs
    delay(1);
    // Interrupt signal caught, so cause the CCS811 to run its algorithm
    myCCS811.readAlgorithmResults(); // Calling this function updates the global tVOC and CO2 variables
    tvoc_co2[0] = myCCS811.getTVOC();
    tvoc_co2[1] = myCCS811.getCO2();
    char buff[32];
    sprintf(buff, "tVOC: %d co2: %d\n", tvoc_co2[0], tvoc_co2[1]);
    Serial.print(buff);
    // Now put the CCS811's logic engine to sleep
    digitalWrite(PIN_NOT_WAKE, 1);
    // Need to be asleep for at least 20 µs
  }
#endif // NEED_CCS811
#ifdef NEED_SSD1306
  displayHT();
#endif // NEED_SSD1306
}
#endif // NEED_HDC1080

#ifdef NEED_SGP30
void displaySGP30() {
  s16 err = 0;
  u16 tvoc_ppb, co2_eq_ppm;
  err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
  if (err == STATUS_OK) {
    tvoc_co2[0] = tvoc_ppb;
    tvoc_co2[1] = co2_eq_ppm;
    char buff[32];
    sprintf(buff, "tVOC: %d co2: %d\n", tvoc_co2[0], tvoc_co2[1]);
    Serial.print(buff);
#ifdef NEED_SSD1306
    displayHT();
#endif // NEED_SSD1306
  } else {
    Serial.println("error reading IAQ values\n");
  }
}
#endif // NEED_SGP30

#ifdef NEED_SSD1306
#if defined(NEED_HDC1080) || defined(NEED_BME) || defined(NEED_DHT)
void displayHT() {
  char buff[32];
  sprintf(buff, "H: %2.2f%% T: %2.2f *C\n", temp_hum_val[0], temp_hum_val[1]);
  oled.println(buff);
#ifdef NEED_CCS811 // Linked to NEED_HDC1080
  sprintf(buff, "tVOC: %d co2: %d\n", tvoc_co2[0], tvoc_co2[1]);
  oled.println(buff);
#endif // NEED_CCS811
#ifdef NEED_SGP30
  sprintf(buff, "tVOC: %d co2: %d\n", tvoc_co2[0], tvoc_co2[1]);
  oled.println(buff);
#endif // NEED_SGP30
}
#endif // NEED_SSD1306
#endif // NEED_SSD1306

#ifdef NEED_SHATEST
void shaTest() {
  Serial.println("\n\n");
  const unsigned char message1[] = "{\"UUID\":\"00D60A54\",\"cmd\":\"ping\",\"from\":\"BastMobile\"}";
  const unsigned char message2[] = "Hi There";
  unsigned char hash0[] = {
    0x42, 0x81, 0x88, 0x40, 0xe4, 0x01, 0xca, 0x59,
    0xfe, 0x1a, 0x02, 0x79, 0x7b, 0x33, 0xe9, 0x66,
    0x7e, 0x4b, 0x8f, 0x9c, 0x43, 0xa6, 0x82, 0xc2,
    0xcf, 0xf6, 0x31, 0x88
  };
  unsigned char hash1[] = {
    0x4c, 0x1f, 0xe4, 0xd5, 0x6a, 0xd6, 0x13, 0xfe,
    0x29, 0x83, 0x3d, 0x3f, 0x2c, 0x10, 0x5e, 0x17,
    0xe6, 0x6b, 0x52, 0x24, 0xe8, 0xeb, 0x71, 0x46,
    0x91, 0xbb, 0xbc, 0x6a, 0x60, 0xc2, 0x2d, 0x62
  };
  unsigned char hash2[] = {
    0x89, 0x6f, 0xb1, 0x12, 0x8a, 0xbb, 0xdf, 0x19,
    0x68, 0x32, 0x10, 0x7c, 0xd4, 0x9d, 0xf3, 0x3f,
    0x47, 0xb4, 0xb1, 0x16, 0x99, 0x12, 0xba, 0x4f,
    0x53, 0x68, 0x4b, 0x22
  };
  unsigned char hash3[] = {
    0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53,
    0x5c, 0xa8, 0xaf, 0xce, 0xaf, 0x0b, 0xf1, 0x2b,
    0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83, 0x3d, 0xa7,
    0x26, 0xe9, 0x37, 0x6c, 0x2e, 0x32, 0xcf, 0xf7
  };
  unsigned char key[20];
  unsigned char digest[SHA512_DIGEST_SIZE];
  unsigned char mac[SHA512_DIGEST_SIZE];

  Serial.println("\nSHA-224 test");

  sha224(message1, strlen((char *)message1), digest);
  hexDump(digest, SHA224_DIGEST_SIZE);
  hexDump(hash0, SHA224_DIGEST_SIZE);
  if (memcmp(digest, hash0, SHA224_DIGEST_SIZE) == 0) Serial.println(" * test passed");
  else Serial.println(" * test failed");
  Serial.println("\nSHA-256 test");

  sha256(message1, strlen((char *)message1), digest);
  hexDump(digest, SHA256_DIGEST_SIZE);
  hexDump(hash1, SHA224_DIGEST_SIZE);
  if (memcmp(digest, hash1, SHA256_DIGEST_SIZE) == 0) Serial.println(" * test passed");
  else Serial.println(" * test failed");

  Serial.println("\nSHA-HMAC test");
  Serial.println("\nSHA-HMAC-224 test");
  memset(key, 0x0b, 20);
  hmac_sha224(key, 20, (unsigned char *) message2, strlen((char*)message2), mac, SHA224_DIGEST_SIZE);
  hexDump(mac, SHA224_DIGEST_SIZE);
  hexDump(hash2, SHA224_DIGEST_SIZE);
  if (memcmp(mac, hash2, SHA224_DIGEST_SIZE) == 0) Serial.println(" * test passed");
  else Serial.println(" * test failed");

  Serial.println("\nSHA-HMAC-256 test");
  memset(key, 0x0b, 20);
  hmac_sha256(key, 20, (unsigned char *) message2, strlen((char*)message2), mac, SHA256_DIGEST_SIZE);
  hexDump(mac, SHA256_DIGEST_SIZE);
  hexDump(hash3, SHA256_DIGEST_SIZE);
  if (memcmp(mac, hash3, SHA256_DIGEST_SIZE) == 0) Serial.println(" * test passed");
  else Serial.println(" * test failed");

  Serial.println("\n\nSpeed Test");
  double t0, t1;
  uint16_t i, j = 1000;
  t0 = millis();
  for (i = 0; i < j; i++) {
    memset(key, 0x0b, 20);
    hmac_sha256(key, 20, (unsigned char *) message2, strlen((char*)message2), mac, SHA256_DIGEST_SIZE);
  }
  t1 = millis() - t0;
  sprintf((char*)msgBuf, "%d iterations of SHA-HMAC-256: %3.1f ms\n", j, t1);
  Serial.print((char*)msgBuf);
}

#endif // NEED_SHATEST

void OnRxDone(uint8_t *payload, uint16_t ix, int16_t rssi, int8_t snr) {
  digitalWrite(LED_GREEN, HIGH); // Turn on Green LED
#ifdef NEED_SSD1306
  oled.print("Incoming! ");
#endif // NEED_SSD1306
#ifdef NEED_SSD1306
  oled.print("RSSI: ");
  oled.println(rssi);
  oled.print("SNR: ");
  oled.println(snr);
#endif // NEED_SSD1306
#ifdef NEED_DEBUG
  Serial.println("Received packet: ");
  if (NEED_DEBUG > 0) hexDump(payload, ix);
#endif // NEED_DEBUG
  if (needEncryption) {
#ifdef NEED_DEBUG
    Serial.println("\n . Decrypting...");
#endif // NEED_DEBUG
    int16_t packetSize = decryptECB(payload, ix);
    if (packetSize > -1) {
      memset(msgBuf, 0, BUFF_LENGTH);
      memcpy(msgBuf, encBuf, packetSize);
    } else {
      Serial.println("Error while decrypting");
      return;
    }
  }
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, msgBuf);
  if (error) {
#ifdef NEED_DEBUG
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
    oled.print("deserializeJson failed");
#endif // NEED_SSD1306
    return;
  }

  // DISPLAY HERE JSON PACKET
  // IF NEED_DEBUG IS NOT DEFINED
#ifndef NEED_DEBUG
  doc["rssi"] = rssi;
  doc["snr"] = snr;
  serializeJson(doc, Serial); Serial.println("");
#endif // NEED_DEBUG
  // Print 4-byte ID
  const char *myID = doc["UUID"];
  // Print sender
  const char *from = doc["from"];
  // Print command
  const char *cmd = doc["cmd"];
  // Do we have a message?
#ifdef NEED_SSD1306
  oled.print("from ");
  oled.print(from);
  oled.print(": ");
  oled.println(cmd);
#endif // NEED_SSD1306
  if (strcmp(cmd, "msg") == 0) {
    const char *msg = doc["msg"];
#ifdef NEED_SSD1306
    oled.println(msg);
#endif // NEED_SSD1306
  }
  JsonVariant mydata = doc["V"];
  if (!mydata.isNull()) {
    uint16_t tvoc = mydata.as<uint16_t>();
    mydata = doc["C"];
    uint16_t co2 = mydata.as<uint16_t>();
    char buff[32];
    sprintf(buff, "H: %2.2f%% T: %2.2f *C\n", tvoc, co2);
#ifdef NEED_DEBUG
    Serial.print(buff);
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
    oled.print(buff);
#endif // NEED_SSD1306
  }

  bool hasLatLong = true;
  float tLat, tLong, tDistance;
  mydata = doc["lat"];
  if (mydata.isNull()) {
    // we don't have
    hasLatLong = false;
  } else {
    tLat = mydata.as<float>();
    mydata = doc["long"];
    if (mydata.isNull()) {
      // we don't have
      hasLatLong = false;
    } else {
      tLong = mydata.as<float>();
      // we now have both values AND hasLatLong = true
      // Display distance
      tDistance = haversine(homeLatitude, homeLongitude, tLat, tLong);
    }
  }
#ifdef NEED_DEBUG
  Serial.print("ID: ");
  Serial.println(myID);
  Serial.print("Sender: ");
  Serial.println(from);
  Serial.print("Command: ");
  Serial.println(cmd);
  if (strcmp(cmd, "msg") == 0) {
    const char *msg = doc["msg"];
    Serial.print("Message: ");
    Serial.println(msg);
  }
  if (hasLatLong) {
    Serial.print("Distance: ");
    if (tDistance >= 1000.0) {
      Serial.print(tDistance / 1000.0);
      Serial.println(" km");
    } else {
      Serial.print(tDistance);
      Serial.println(" m");
    }
  }
  Serial.print("RSSI: ");
  Serial.println(rssi);
#endif // NEED_DEBUG
  if (strcmp(cmd, "ping") == 0 && pongBack) {
    // if it's a PING, and we are set to respond:
    // Radio.Standby();
#ifdef NEED_DEBUG
    Serial.println("Pong back:");
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
    oled.print("PONG back! ");
#endif // NEED_SSD1306
    // we cannot pong back right away – the message would be lost
    // if there are other devices on the same network
    uint16_t dl = getRamdom16() % 2800 + 3300;
#ifdef NEED_DEBUG
    Serial.println("Delaying " + String(dl) + " millis...");
#endif // NEED_DEBUG
    delay(dl);
    sendPong((char*)myID, rssi);
    Radio.Rx(3000);
  } else if (strcmp(cmd, "pong") == 0) {
    int rcvRSSI = doc["rcvRSSI"];
#ifdef NEED_DEBUG
    Serial.print("rcvRSSI: ");
    Serial.println(rcvRSSI);
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
    oled.print("rcvRSSI: ");
    oled.println(rcvRSSI);
#endif // NEED_SSD1306
    mydata = doc["T"];
    if (!mydata.isNull()) {
      float tp = mydata.as<float>();
      mydata = doc["H"];
      float hm = mydata.as<float>();
#ifdef NEED_DEBUG
      Serial.print("Humidity: ");
      Serial.print(hm);
      Serial.print("% Temperature: ");
      Serial.println(tp);
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
      oled.print("H: ");
      oled.print(hm);
      oled.print("% ");
      oled.print("T: ");
      oled.print(tp);
      oled.println(" C");
#endif // NEED_SSD1306
    }
    mydata = doc["V"];
    if (!mydata.isNull()) {
      float tVoc = mydata.as<float>();
      mydata = doc["C"];
      float cO2 = mydata.as<float>();
#ifdef NEED_DEBUG
      Serial.print("tVoc: ");
      Serial.println(tVoc);
      Serial.print("% CO2: ");
      Serial.println(cO2);
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
      oled.print("tVoc: ");
      oled.print(tVoc);
      oled.print(" ");
      oled.print("co2: ");
      oled.println(cO2);
#endif // NEED_SSD1306
    }
  } else if (strcmp(cmd, "freq") == 0) {
    // Do we have a frequency change request?
    if (strcmp(from, "BastMobile") != 0) return;
    // Not for you, brah
    mydata = doc["freq"];
    if (mydata.isNull()) {
#ifdef NEED_DEBUG
      Serial.println("mydata (doc['freq']) is null!");
#endif
    } else {
      uint32_t fq = mydata.as<float>() * 1e6;
#ifdef NEED_DEBUG
      Serial.println("mydata (doc['freq']) = " + String(fq, 3));
#endif
      if (fq < 862e6 || fq > 1020e6) {
#ifdef NEED_DEBUG
        Serial.println("Requested frequency (" + String(fq) + ") is invalid!");
#endif // NEED_DEBUG
      } else {
        myFreq = fq;
        Radio.Standby();
        Radio.SetChannel(myFreq);
        delay(100);
        Radio.Rx(3000);
#ifdef NEED_DEBUG
        Serial.println("Frequency set to " + String(myFreq / 1e6, 3) + " MHz");
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
        oled.print("New freq: ");
        oled.println(String(myFreq / 1e6, 3) + " MHz");
#endif // NEED_SSD1306
        savePrefs();
      }
    }
  } else if (strcmp(cmd, "bw") == 0) {
    // Do we have a bandwidth change request?
    /*
      Note on SF / BW pairs:
      Unless you are sending very small packets, all pairs might not work.
      Here is a table based on empirical results of pairs that work.
       BW|SF|Y/N
       --|--|---
         |9 | Y
       6 |10| N
         |11| N
         |12| N
       --|--|---
         |9 | Y
       7 |10| Y
         |11| N
         |12| N
       --|--|---
         |9 | Y
       8 |10| Y
         |11| Y
         |12| N
       --|--|---
         |9 | Y
       9 |10| Y
         |11| Y
         |12| Y
    */
    if (strcmp(from, "BastMobile") != 0) return;
    // Not for you, brah
    mydata = doc["bw"];
    if (mydata.isNull()) {
#ifdef NEED_DEBUG
      Serial.println("mydata (doc['bw']) is null!");
#endif
    } else {
      int bw = mydata.as<int>();
#ifdef NEED_DEBUG
      Serial.println("mydata (doc['bw']) = " + String(bw));
#endif
      if (bw < 0 || bw > 9) {
#ifdef NEED_DEBUG
        Serial.println("Requested bandwidth (" + String(bw) + ") is invalid!");
#endif // NEED_DEBUG
      } else {
        myBW = bw;
        Radio.Standby();
        Radio.SetRxConfig(MODEM_LORA, myBW, mySF, myCR, 0, 8, 0, false, 0, true, 0, 0, false, true);
        delay(100);
        Radio.Rx(3000);
#ifdef NEED_DEBUG
        Serial.println("Bandwidth set to " + String(BWs[myBW], 3) + " KHz");
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
        oled.print("New BW: ");
        oled.println(String(BWs[myBW], 3) + " KHz");
#endif // NEED_SSD1306
        savePrefs();
      }
    }
  } else if (strcmp(cmd, "sf") == 0) {
    // Do we have a spreading factor change request?
    if (strcmp(from, "BastMobile") != 0) return;
    // Not for you, brah
    mydata = doc["sf"];
    if (mydata.isNull()) {
#ifdef NEED_DEBUG
      Serial.println("mydata (doc['sf']) is null!");
#endif
    } else {
      int sf = mydata.as<int>();
#ifdef NEED_DEBUG
      Serial.println("mydata (doc['sf']) = " + String(sf));
#endif
      if (sf < 7 || sf > 12) {
#ifdef NEED_DEBUG
        Serial.println("Requested SF (" + String(sf) + ") is invalid!");
#endif // NEED_DEBUG
      } else {
        mySF = sf;
        Radio.SetRxConfig(MODEM_LORA, myBW, mySF, myCR, 0, 8, 0, false, 0, true, 0, 0, false, true);
#ifdef NEED_DEBUG
        Serial.println("SF set to " + String(sf));
#endif // NEED_DEBUG
#ifdef NEED_SSD1306
        oled.print("New SF: ");
        oled.println(mySF);
#endif // NEED_SSD1306
        savePrefs();
      }
    }
  } else if (strcmp(cmd, "switch") == 0) {
    // Do we have a spreading factor change request?
    if (strcmp(from, "BastMobile") != 0) return;
    // Not for you, brah
    mydata = doc["set"];
    if (mydata.isNull()) {
#ifdef NEED_DEBUG
      Serial.println("mydata (doc['set']) is null!");
#endif
    } else {
      int setNum = mydata.as<int>();
#ifdef NEED_DEBUG
      Serial.println("Switching to set #" + String(setNum));
#endif
      float F = setsFQ[0];
      int S = setsSF[0];
      int B = setsBW[0];
#ifdef NEED_DEBUG
      sprintf((char*)msgBuf, " . Freq: %3.3f MHz, SF %d, BW %d: %3.2f", F, S, B, BWs[B]);
      Serial.println((char*)msgBuf);
#endif // NEED_DEBUG
      myFreq = F * 1e6;
      mySF = S;
      myBW = B;
      Radio.SetRxConfig(MODEM_LORA, myBW, mySF, myCR, 0, 8, 0, false, 0, true, 0, 0, false, true);
      sendPong((char*)myID, rssi);
#ifdef NEED_SSD1306
      oled.print("New freq: ");
      oled.println(String(myFreq / 1e6, 3) + " MHz");
      oled.print("New SF: ");
      oled.println(mySF);
      oled.print("New BW: ");
      oled.println(String(BWs[myBW], 3) + " KHz");
#endif // NEED_SSD1306
    }
  }
}

void OnRxTimeout(void) {
  //  u8g2.clearBuffer(); // clear the internal memory
  //  u8g2.setFont(u8g2_font_courB08_tf); // choose a suitable font
  //  u8g2.drawStr(3, 15, "OnRxTimeout");
  //  u8g2.sendBuffer(); // transfer internal memory to the display
  Serial.println("OnRxTimeout");
  Radio.Rx(3000);
}

void OnRxError(void) {
  Serial.println("OnRxError");
  Radio.Rx(3000);
}
