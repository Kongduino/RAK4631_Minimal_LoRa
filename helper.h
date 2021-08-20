#undef max
#undef min
#include <string>
#include <vector>

using namespace std;
template class basic_string<char>; // https://github.com/esp8266/Arduino/issues/1136
// Required or the code won't compile!
namespace std _GLIBCXX_VISIBILITY(default) {
_GLIBCXX_BEGIN_NAMESPACE_VERSION
// void __throw_bad_alloc() {}
}

/*
  NOTE!
  Add:
  namespace std _GLIBCXX_VISIBILITY(default) {
    _GLIBCXX_BEGIN_NAMESPACE_VERSION
    void __throw_length_error(char const*) {
    }
    void __throw_out_of_range(char const*) {
    }
    void __throw_logic_error(char const*) {
    }
    void __throw_out_of_range_fmt(char const*, ...) {
    }
  }
  to ~/Library/Arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/arm-none-eabi/include/c++/4.8.3/bits/basic_string.h
  Or the code won't compile.
*/

#define REG_OCP 0x0B
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0c
#define REG_OP_MODE 0x01
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_MODEM_CONFIG_3 0x26
#define REG_PA_DAC 0x4D
#define PA_DAC_HIGH 0x87

#define CBC 0
#define CTR 1
#define ECB 2

// Define LoRa parameters
#define askForLORA_BW_125 0
#define askForLORA_BW_250 1
#define askForLORA_BW_500 2
#define askForLORA_BW_062 3
#define askForLORA_BW_041 4
#define askForLORA_BW_031 5
#define askForLORA_BW_020 6
#define askForLORA_BW_015 7
#define askForLORA_BW_010 8
#define askForLORA_BW_007 9

uint8_t myMode = ECB;
bool needEncryption = true;
bool needAuthentification = false;
bool pongBack = false;
bool OCP_ON = false, PA_BOOST = true;
uint8_t SecretKey[33] = "YELLOW SUBMARINEENIRAMBUS WOLLEY";
char deviceName[33];
#define BUFF_LENGTH 512
uint8_t encBuf[BUFF_LENGTH], hexBuf[BUFF_LENGTH], msgBuf[BUFF_LENGTH];
uint16_t encLen;
uint8_t randomIndex = 0;
float lastBattery = 0.0;
double batteryUpdateDelay;
uint32_t myFreq = 868.125e6;
int mySF = 9;
uint8_t myBW = 0;
uint8_t myCR = 5;
// See above
// double BWs[10 = {125, 250, 500, 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5};
// a=INDEX(natural order);b=(a+3);if(b>6)b=b-10;BWs[b];---> new order
double BWs[10] = {
  125.0, 250.0, 500.0, 62.5, 41.7, 31.25, 20.8, 15.6, 10.4, 7.8
};
uint16_t pingCounter = 0;
uint8_t randomStock[256];
time_t cadTime;

long g_latitude;
long g_longitude;
long g_altitude;
long g_speed;
long g_heading;
byte g_SIV;

#define RX_TIMEOUT_VALUE 30000

#ifdef Pavel
// enable autoPing for Pavel
double pingFrequency = 120000;
// 120,000 ms = 2 mn
bool needPing = true;
#else
double pingFrequency = 0;
bool needPing = false;
#endif

double lastAutoPing = 0;
uint8_t TxPower = 20;

// Sets of Freq / SF / BW settings
StaticJsonDocument<BUFF_LENGTH>sets;
JsonArray setsFQ, setsSF, setsBW;

uint16_t encryptECB(uint8_t*);
int16_t decryptECB(uint8_t*, uint8_t);
void array2hex(uint8_t *, size_t, uint8_t *, uint8_t);
void hex2array(uint8_t *, uint8_t *, size_t);
void sendPacket(char *);
void setPWD(char *);
void prepareJSONPacket(char *);
void setPongBack(bool);
void stockUpRandom();
void showHelp();
uint8_t getRandomByte();
uint16_t getRamdom16();
void getRandomBytes(uint8_t *, uint8_t);
void getBattery();
void setFQ(char*);
void setSF(char*);
void setTxPower(char* buff);
void setBW(char* buff);
void setCR(char* buff);
void setDeviceName(char *);
void sendJSONPacket();
void savePrefs();
void setAutoPing(char *);
void handleSerial();
uint8_t calcMaxPayload();
void hexDump(unsigned char *, uint16_t);
void fillRandom();

void hex2array(uint8_t *src, uint8_t *dst, size_t sLen) {
  size_t i, n = 0;
  for (i = 0; i < sLen; i += 2) {
    uint8_t x, c;
    c = src[i];
    if (c != '-') {
      if (c > 0x39) c -= 55;
      else c -= 0x30;
      x = c << 4;
      c = src[i + 1];
      if (c > 0x39) c -= 55;
      else c -= 0x30;
      dst[n++] = (x + c);
    }
  }
}

void array2hex(uint8_t *inBuf, size_t sLen, uint8_t *outBuf, uint8_t dashFreq = 0) {
  size_t i, len, n = 0;
  const char * hex = "0123456789ABCDEF";
  for (i = 0; i < sLen; ++i) {
    outBuf[n++] = hex[(inBuf[i] >> 4) & 0xF];
    outBuf[n++] = hex[inBuf[i] & 0xF];
    if (dashFreq > 0 && i != sLen - 1) {
      if ((i + 1) % dashFreq == 0) outBuf[n++] = '-';
    }
  }
  outBuf[n++] = 0;
}

void setPWD(char *buff) {
  // buff can be 32 or 64 bytes:
  // 32 bytes = plain text
  // 64 bytes = hex-encoded
  uint8_t len = strlen(buff), i;
  for (i = 0; i < len; i++) {
    if (buff[i] < 32) {
      buff[i] = 0;
      i = len + 1;
    }
  }
  len = strlen(buff);
#ifdef NEED_DEBUG
  Serial.print("setPWD: ");
  Serial.println(buff);
  Serial.print("len: ");
  Serial.println(len);
  if (NEED_DEBUG > 0) hexDump((uint8_t *)buff, len);
#endif
  if (len == 32) {
    // copy to the SecretKey buffer
    memcpy(SecretKey, buff, 32);
    needEncryption = true;
#ifdef NEED_DEBUG
    if (NEED_DEBUG > 0) hexDump((uint8_t *)SecretKey, 32);
#endif
    return;
  }
  if (len == 64) {
    // copy to the SecretKey buffer
    hex2array((uint8_t *)buff, SecretKey, 64);
    needEncryption = true;
#ifdef NEED_DEBUG
    if (NEED_DEBUG > 0) hexDump((uint8_t *)SecretKey, 32);
#endif
    return;
  }
}

void sendPacket(char *buff) {
  uint16_t olen = strlen(buff);
  memcpy(encBuf + 8, buff, olen);
  // prepend UUID
  // 4 bytes --> 8 bytes
  uint8_t ix = 0;
  getRandomBytes(encBuf, 4);
  // encBuf[ix++] = getRandomByte();
  // encBuf[ix++] = getRandomByte();
  // encBuf[ix++] = getRandomByte();
  // encBuf[ix++] = getRandomByte();
  array2hex(encBuf, 4, hexBuf);
  memcpy(encBuf, hexBuf, 8);
  olen += 8;
#ifdef NEED_DEBUG
  Serial.println("Before calling encryption. olen = " + String(olen));
#endif
  memcpy(msgBuf, encBuf, olen);
#ifdef NEED_DEBUG
  if (NEED_DEBUG > 0) hexDump(msgBuf, olen);
#endif
  if (needEncryption) {
    olen = encryptECB((uint8_t*)msgBuf);
    // encBuff = encrypted buffer
    // hexBuff = encBuf, hex encoded
    // olen = len(hexBuf)
  }
#ifdef NEED_DEBUG
  Serial.println("olen: " + String(olen));
  Serial.print("Sending packet...");
#endif
  // Now send a packet
  encLen = olen;
  Radio.Standby();
  Radio.SetCadParams(LORA_CAD_08_SYMBOL, myBW + 13, 10, LORA_CAD_ONLY, 0);
  cadTime = millis();
  Radio.StartCad();
}

int16_t decryptECB(uint8_t* myBuf, uint8_t olen) {
#ifdef NEED_DEBUG
  if (NEED_DEBUG > 0) hexDump(myBuf, olen);
#endif
  // Test the total len vs requirements:
  // AES: min 16 bytes
  // HMAC if needed: 28 bytes
  uint8_t reqLen = 16 + needAuthentification ? 28 : 0;
  printf("reqLen: %d\n", reqLen);
  if (olen < reqLen) return -1;
  uint8_t len;

  // or just copy over
  memcpy(encBuf, myBuf, olen);
  len = olen;
  if (needAuthentification) {
    // hmac the buffer minus SHA2xx_DIGEST_SIZE
    // compare with the last SHA2xx_DIGEST_SIZE bytes
    // if correct, reduce len by SHA2xx_DIGEST_SIZE
    unsigned char key[20];
    unsigned char digest[SHA224_DIGEST_SIZE];
    unsigned char mac[SHA224_DIGEST_SIZE];
    memset(key, 0x0b, 20); // set up key
#ifdef NEED_DEBUG
    if (NEED_DEBUG > 0) {
      Serial.println("Original HMAC:");
      hexDump((unsigned char *)encBuf + len - SHA224_DIGEST_SIZE, SHA224_DIGEST_SIZE);
    }
#endif // NEED_DEBUG
    hmac_sha224(key, 20, (unsigned char *)encBuf, len - SHA224_DIGEST_SIZE, mac, SHA224_DIGEST_SIZE);
#ifdef NEED_DEBUG
    if (NEED_DEBUG > 0) {
      Serial.println("HMAC:");
      hexDump(mac, SHA224_DIGEST_SIZE);
    }
#endif // NEED_DEBUG
    if (memcmp(mac, encBuf + len - SHA224_DIGEST_SIZE, SHA224_DIGEST_SIZE) == 0) {
#ifdef NEED_DEBUG
      Serial.println(" * test passed");
#endif // NEED_DEBUG
    } else {
#ifdef NEED_DEBUG
      Serial.println(" * test failed");
      // notify we failed
#endif // NEED_DEBUG
      return -1;
    }
    // deduct SHA224_DIGEST_SIZE from length
    len -= SHA224_DIGEST_SIZE;
  }
#ifdef NEED_DEBUG
  if (NEED_DEBUG > 0) hexDump(encBuf, len);
#endif // NEED_DEBUG

  struct AES_ctx ctx;
  AES_init_ctx(&ctx, SecretKey);
  uint8_t rounds = len / 16, steps = 0;
  for (uint8_t ix = 0; ix < rounds; ix++) {
    // void AES_ECB_encrypt(const struct AES_ctx* ctx, uint8_t* buf);
    AES_ECB_decrypt(&ctx, (uint8_t*)encBuf + steps);
    steps += 16;
    // encrypts in place, 16 bytes at a time
  } encBuf[steps] = 0;
#ifdef NEED_DEBUG
  if (NEED_DEBUG > 0) hexDump(encBuf, len);
#endif // NEED_DEBUG
  return len;
}

uint16_t encryptECB(uint8_t* myBuf) {
  // first ascertain length
  uint8_t len = strlen((char*)myBuf);
  uint16_t olen;
  struct AES_ctx ctx;
  olen = len;
  if (olen != 16) {
    if (olen % 16 > 0) {
      if (olen < 16) olen = 16;
      else olen += 16 - (olen % 16);
    }
  }
#ifdef NEED_DEBUG
  Serial.println("\nolen: " + String(olen) + ", len: " + String(len));
#endif // NEED_DEBUG
  memset(encBuf, (olen - len), olen);
  memcpy(encBuf, myBuf, len);

  encBuf[len] = 0;
  AES_init_ctx(&ctx, (const uint8_t*)SecretKey);
  uint8_t rounds = olen / 16, steps = 0;
  for (uint8_t ix = 0; ix < rounds; ix++) {
    AES_ECB_encrypt(&ctx, encBuf + steps);

    // void AES_ECB_decrypt(&ctx, encBuf + steps);
    steps += 16;
    // encrypts in place, 16 bytes at a time
  }
#ifdef NEED_DEBUG
  if (NEED_DEBUG > 0) {
    Serial.println("encBuf:");
    hexDump(encBuf, olen);
  }
#endif // NEED_DEBUG


  // Now do we have to add a MAC?
  if (needAuthentification) {
    // hmac the buffer
    // increase len by SHA2xx_DIGEST_SIZE
    unsigned char key[20];
    unsigned char digest[SHA224_DIGEST_SIZE];
    unsigned char mac[SHA224_DIGEST_SIZE];
    memset(key, 0x0b, 20);// set up key
    // authenticate in place
    // at offset olen
    // OF COURSE IT'D BE NICE TO CHECK THAT OLEN+SHA224_DIGEST_SIZE DOESN'T BLOW UP THE BUFFER

    hmac_sha224(key, 20, (unsigned char *)encBuf, olen, (unsigned char*)encBuf + olen, SHA224_DIGEST_SIZE);

#ifdef NEED_DEBUG
    if (NEED_DEBUG > 0) {
      Serial.println("MAC, plain [" + String(olen) + "]:");
      hexDump((unsigned char*)encBuf + olen, SHA224_DIGEST_SIZE);
    }
#endif // NEED_DEBUG
    olen += SHA224_DIGEST_SIZE;
#ifdef NEED_DEBUG
    if (NEED_DEBUG > 0) {
      Serial.println("encBuf with MAC:");
      hexDump(encBuf, olen);
    }
#endif // NEED_DEBUG
  }
  return olen;
}

void stockUpRandom() {
  fillRandom();
  randomIndex = 0;
#ifdef NEED_DEBUG
  if (NEED_DEBUG > 0) {
    hexDump(randomStock, 256);
  }
#endif // NEED_DEBUG
}

void setPongBack(bool x) {
  pongBack = x;
#ifdef NEED_DEBUG
  Serial.print("PONG back set to ");
  if (x) Serial.println("true");
  else Serial.println("false");
#endif
}

uint8_t getRandomByte() {
  uint8_t r = randomStock[randomIndex++];
  // reset random stock automatically if needed
  if (randomIndex > 254) stockUpRandom();
  return r;
}

uint16_t getRamdom16() {
  uint8_t r0 = randomStock[randomIndex++];
  // reset random stock automatically if needed
  if (randomIndex > 254) stockUpRandom();
  uint8_t r1 = randomStock[randomIndex++];
  // reset random stock automatically if needed
  if (randomIndex > 254) stockUpRandom();
  return r1 * 256 + r0;
}

void getRandomBytes(uint8_t *buff, uint8_t count) {
  uint8_t r;
  for (uint8_t i = 0; i < count; i++) {
    buff[i] = randomStock[randomIndex++];
    // reset random stock automatically if needed
    if (randomIndex > 254) stockUpRandom();
  }
}

void getBattery() {
  float battery = analogRead(A0);
  if (battery != lastBattery) {
    // update visually etc.
#ifdef NEED_DEBUG
    Serial.println("Last Battery: " + String(lastBattery) + " vs current: " + String(battery));
#endif
    lastBattery = battery;
  }
}

void setFQ(char* buff) {
  uint32_t fq = (uint32_t)(atof(buff) * 1e6);
  // RAK4260: 862 to 1020 MHz frequency coverage
  // clearFrame();
  if (fq < 862e6 || fq > 1020e6) {
#ifdef NEED_DEBUG
    Serial.println("Requested frequency (" + String(buff) + ") is invalid!");
#endif
  } else {
    myFreq = fq;
#ifdef NEED_DEBUG
    Serial.println("Frequency set to " + String(myFreq / 1e6, 3) + " MHz");
#endif
#ifdef NEED_SSD1306
    oled.print("New freq: ");
    oled.println(myFreq);
#endif // NEED_SSD1306
    savePrefs();
    Radio.Standby();
    Radio.SetChannel(myFreq);
    delay(100);
    Radio.Rx(RX_TIMEOUT_VALUE);
  }
}

void setSF(char* buff) {
  int sf = atoi(buff);
  // SF 7 to 12
  // clearFrame();
  if (sf < 7 || sf > 12) {
#ifdef NEED_DEBUG
    Serial.println("Requested SF (" + String(buff) + ") is invalid!");
#endif
  } else {
    mySF = sf;
    uint8_t mpl = calcMaxPayload();
#ifdef NEED_DEBUG
    Serial.println("SF set to " + String(mySF));
    sprintf((char*)msgBuf, "Maximum payload at SF %d / BW %3.2f: %d bytes", mySF, BWs[myBW], mpl);
    Serial.println((char*)msgBuf);
#endif
#ifdef NEED_SSD1306
    oled.print("New SF: ");
    oled.println(mySF);
    oled.print("Max payload: ");
    oled.println(mpl);
#endif // NEED_SSD1306
    savePrefs();
    // Radio.Standby();
    Radio.SetTxConfig(MODEM_LORA, TxPower, 0, myBW, mySF, myCR - 4, 8, false, true, 0, 0, false, 3000);
    Radio.SetRxConfig(MODEM_LORA, myBW, mySF, myCR - 4, 0, 8, 0, false, 0, true, 0, 0, false, false);
    delay(100);
    Radio.Rx(RX_TIMEOUT_VALUE);
  }
}

void setBW(char* buff) {
  int bw = atoi(buff);
  /*Signal bandwidth:
    0000  7.8 kHz
    0001  10.4 kHz
    0010  15.6 kHz
    0011  20.8kHz
    0100  31.25 kHz
    0101  41.7 kHz
    0110  62.5 kHz
    0111  125 kHz
    1000  250 kHz
    1001  500 kHz
  */
  // clearFrame();
  if (bw < 0 || bw > 9) {
#ifdef NEED_DEBUG
    Serial.println("Requested BW (" + String(bw) + ") is invalid!");
#endif
  } else {
    String s = "BW set to: " + String(bw);
    myBW = bw;
    uint8_t mpl = calcMaxPayload();
#ifdef NEED_DEBUG
    Serial.println("BW set to " + String(BWs[myBW])) + " KHz";
    sprintf((char*)msgBuf, "Maximum payload at SF %d / BW %3.2f: %d bytes", mySF, BWs[myBW], mpl);
    Serial.println((char*)msgBuf);
#endif
#ifdef NEED_SSD1306
    oled.print("New BW: ");
    oled.print(myBW);
    oled.print(" ie ");
    oled.print(BWs[myBW]);
    oled.println(" KHz");
    oled.print("Max payload: ");
    oled.println(mpl);
#endif // NEED_SSD1306
    savePrefs();
    // Radio.Standby();
    Radio.SetTxConfig(MODEM_LORA, TxPower, 0, myBW, mySF, myCR - 4, 8, false, true, 0, 0, false, 3000);
    Radio.SetRxConfig(MODEM_LORA, myBW, mySF, myCR - 4, 0, 8, 0, false, 0, true, 0, 0, false, false);
    delay(100);
    Radio.Rx(RX_TIMEOUT_VALUE);
  }
}

void setTxPower(char* buff) {
  int txp = atoi(buff);
  if (txp < 7 || txp > 23) {
#ifdef NEED_DEBUG
    Serial.println("Requested TxPower (" + String(txp) + ") is invalid!");
#endif
  } else {
    String s = "TxPower set to: " + String(txp);
    TxPower = txp;
    // Radio.Standby();
    Radio.SetTxConfig(MODEM_LORA, TxPower, 0, myBW, mySF, myCR - 4, 8, false, true, 0, 0, false, 3000);
    Radio.SetRxConfig(MODEM_LORA, myBW, mySF, myCR - 4, 0, 8, 0, false, 0, true, 0, 0, false, false);
    delay(100);
    Radio.Rx(RX_TIMEOUT_VALUE);
#ifdef NEED_DEBUG
    Serial.println("TxPower set to " + String(TxPower));
#endif
  }
}

void setCR(char* buff) {
  int cr = atoi(buff);
  // clearFrame();
  if (cr < 5 || cr > 8) {
#ifdef NEED_DEBUG
    Serial.println("Requested C/R (" + String(cr) + ") is invalid!");
#endif
  } else {
    String s = "C/R set to: " + String(cr);
    myCR = cr;
    // Radio.Standby();
    Radio.SetTxConfig(MODEM_LORA, TxPower, 0, myBW, mySF, myCR - 4, 8, false, true, 0, 0, false, 3000);
    Radio.SetRxConfig(MODEM_LORA, myBW, mySF, myCR - 4, 0, 8, 0, false, 0, true, 0, 0, false, false);
    delay(100);
    Radio.Rx(RX_TIMEOUT_VALUE);
#ifdef NEED_DEBUG
    Serial.println("C/R set to 4/" + String(cr));
#endif
#ifdef NEED_SSD1306
    oled.print("New C/R: 4/");
    oled.println(myCR);
#endif // NEED_SSD1306
    savePrefs();
  }
}

void setDeviceName(char *truc) {
  memset(deviceName, 0, 33);
  memcpy(deviceName, truc, strlen(truc));
#ifdef NEED_DEBUG
  Serial.print("Device Name set to: ");
  Serial.println(deviceName);
#endif
  savePrefs();
}

void prepareJSONPacket(char *buff) {
  StaticJsonDocument<BUFF_LENGTH> doc;
  memset(msgBuf, 0, BUFF_LENGTH);
  char myID[9];
  getRandomBytes(encBuf, 4);
  array2hex(encBuf, 4, (uint8_t*)myID);
  myID[8] = 0;
  doc["UUID"] = myID;
  doc["cmd"] = "msg";
  doc["msg"] = buff;
  char x[33];
  memcpy(x, deviceName, 33);
  doc["from"] = x;
  serializeJson(doc, (char*)msgBuf, BUFF_LENGTH);
}

void sendJSONPacket() {
#ifdef NEED_DEBUG
  Serial.println("Sending JSON Packet... ");
#endif
  uint16_t olen = strlen((char*)msgBuf);
#ifdef NEED_DEBUG
  if (NEED_DEBUG > 0) hexDump(msgBuf, olen);
#endif
  if (needEncryption) {
    olen = encryptECB((uint8_t*)msgBuf);
    // encBuff = encrypted buffer
    // hexBuff = encBuf, hex encoded
    // olen = len(hexBuf)
  } // Serial.println("olen: " + String(olen));
#ifdef NEED_DEBUG
  Serial.println("Sending packet...");
#endif
  // Now send a packet
  encLen = olen;
  Radio.Standby();
  Radio.SetCadParams(LORA_CAD_08_SYMBOL, myBW + 13, 10, LORA_CAD_ONLY, 0);
  cadTime = millis();
  Radio.StartCad();
}

void sendPing() {
  // PING!
  StaticJsonDocument<BUFF_LENGTH> doc;
  char myID[9];
  getRandomBytes(encBuf, 4);
  array2hex(encBuf, 4, (uint8_t*)myID);
  myID[8] = 0;
  doc["UUID"] = myID;
  doc["cmd"] = "ping";

  char x[33];
  memcpy(x, deviceName, 33);
  doc["from"] = x;
  char freq[8];
  snprintf( freq, 8, "%f", float(myFreq / 1e6) );
  doc["freq"] = freq;
#ifdef NEED_RAK12500
  // Lat/Long come from GNSS if available
  doc["lat"] = g_latitude;
  doc["long"] = g_longitude;
#endif // NEED_RAK12500
#if defined(NEED_DHT) || defined(NEED_BME) || defined(NEED_HDC1080) || defined(NEED_SHTC3)
  doc["H"] = temp_hum_val[0];
  doc["T"] = temp_hum_val[1];
#ifdef NEED_CCS811
  doc["V"] = tvoc_co2[0];
  doc["C"] = tvoc_co2[1];
#endif // NEED_CCS811
#ifdef NEED_SGP30
  doc["V"] = tvoc_co2[0];
  doc["C"] = tvoc_co2[1];
#endif // NEED_SGP30
#endif // NEED_DHT || NEED_BME || NEED_HDC1080
  serializeJson(doc, (char*)msgBuf, BUFF_LENGTH);
  sendJSONPacket();
  lastAutoPing = millis();
  // sending a ping, even manually, resets the clock
}

void sendPong(char *msgID, int rssi) {
  // PONG!
  StaticJsonDocument<BUFF_LENGTH> doc;
  char myID[9];
  getRandomBytes(encBuf, 4);
  array2hex(encBuf, 4, (uint8_t*)myID);
  myID[8] = 0;
  doc["UUID"] = msgID ;
  doc["cmd"] = "pong";
  char x[33];
  memcpy(x, deviceName, 33);
  doc["from"] = x;
  doc["rcvRSSI"] = rssi;
#if defined(NEED_DHT) || defined(NEED_BME) || defined(NEED_HDC1080) || defined(NEED_SHTC3)
  doc["H"] = temp_hum_val[0];
  doc["T"] = temp_hum_val[1];
#ifdef NEED_CCS811
  doc["V"] = tvoc_co2[0];
  doc["C"] = tvoc_co2[1];
#endif // NEED_CCS811
#endif // NEED_DHT || NEED_BME || NEED_HDC1080
  // char freq[8];
  // snprintf( freq, 8, "%f", float(myFreq / 1e6) );
  // doc["freq"] = freq;
  serializeJson(doc, (char*)msgBuf, BUFF_LENGTH);
  sendJSONPacket();
}

void savePrefs() {
  // Serial.println("Saving prefs:");
  // StaticJsonDocument<200> doc;
  // doc["myFreq"] = myFreq;
  // doc["mySF"] = mySF;
  // doc["myBW"] = myBW;
  // doc["deviceName"] = deviceName;
  // memset(msgBuf, 0, 97);
  // serializeJson(doc, (char*)msgBuf, 97);
  // hexDump(msgBuf, 96);
  // myMem.write(0, msgBuf, 32);
  // myMem.write(32, msgBuf + 32, 32);
  // myMem.write(64, msgBuf + 64, 32);
}

void setAutoPing(char* buff) {
  double fq = (double)(atof(buff) * 60e3);
#ifdef NEED_DEBUG
  Serial.print("PING frequency: ");
  Serial.print(buff);
  Serial.print(" --> ");
  Serial.println(fq);
#endif
  if (fq == 0) {
#ifdef NEED_DEBUG
    Serial.println("Turning auto PING off!");
#endif
#ifdef NEED_SSD1306
    oled.println("auto PING off!");
    oled.println((char*)msgBuf + 3);
#endif // NEED_SSD1306
    needPing = false;
    return;
  }
  if (fq < 60000) fq = 60000;
  if (fq > 600000) fq = 600000;
#ifdef NEED_DEBUG
  Serial.print("Turning auto PING on, every ");
  Serial.print((double)fq / 60e3);
  Serial.println(" mn");
#endif
#ifdef NEED_SSD1306
  oled.print("auto PING: ");
  oled.print((double)fq / 60e3);
  oled.println(" mn");
#endif // NEED_SSD1306
  pingFrequency = fq;
  needPing = true;
  lastAutoPing = millis();
}

uint8_t calcMaxPayload() {
  // https://www.thethingsnetwork.org/airtime-calculator
  if (myFreq > 900.0) {
    if (mySF == 7 && myBW == 0) return 242;
    if (mySF == 7 && myBW == 2) return 222;
    if (mySF == 8 && myBW == 2) return 222;
    if (mySF == 8 && myBW == 0) return 125;
    if (mySF == 9 && myBW == 2) return 222;
    if (mySF == 9 && myBW == 0) return 53;
    if (mySF == 10 && myBW == 2) return 222;
    if (mySF == 10 && myBW == 0) return 11;
    if (mySF == 11 && myBW == 2) return 109;
    if (mySF == 12 && myBW == 2) return 33;
  } else if (myFreq >= 860.9) {
    if (mySF == 7 && (myBW == 0 || myBW == 1)) return 222;
    if (mySF == 8 &&  myBW == 0) return 222;
    if (mySF == 9 &&  myBW == 1) return 115;
    if ((mySF == 10 || mySF == 11 || mySF == 12) & (myBW == 0)) return 51;
    return 0;
  } else return 0;
}
