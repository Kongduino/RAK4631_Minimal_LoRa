void hexDump(unsigned char *, uint16_t);
void hexDump(unsigned char *buf, uint16_t len) {
  String s = "|", t = "| |";
  Serial.println(F("  |.0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .a .b .c .d .e .f |"));
  Serial.println(F("  +------------------------------------------------+ +----------------+"));
  for (uint16_t i = 0; i < len; i += 16) {
    for (uint8_t j = 0; j < 16; j++) {
      if (i + j >= len) {
        s = s + "   "; t = t + " ";
      } else {
        char c = buf[i + j];
        if (c < 16) s = s + "0";
        s = s + String(c, HEX) + " ";
        if (c < 32 || c > 127) t = t + ".";
        else t = t + (String(c));
      }
    }
    uint8_t index = i / 16;
    Serial.print(index, HEX); Serial.write('.');
    Serial.println(s + t + "|");
    s = "|"; t = "| |";
  }
  Serial.println(F("  +------------------------------------------------+ +----------------+"));
}

/*
  uint32_t SX126xGetRandom(void) {
  uint8_t buf[] = {0, 0, 0, 0};
  SX126xReadRegisters(RANDOM_NUMBER_GENERATORBASEADDR, buf, 4);
  return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
  }
*/
#define RANDOM_NUMBER_GENERATORBASEADDR 0x0819
#define REG_ANA_LNA 0x08E2
#define REG_ANA_MIXER 0x08E5

void fillRandom() {
  // randomStock
  uint32_t number = 0;
  uint8_t regAnaLna = 0, regAnaMixer = 0, cnt = 0;
  regAnaLna = SX126xReadRegister(REG_ANA_LNA);
  SX126xWriteRegister(REG_ANA_LNA, regAnaLna & ~(1 << 0));
  regAnaMixer = SX126xReadRegister(REG_ANA_MIXER);
  SX126xWriteRegister(REG_ANA_MIXER, regAnaMixer & ~(1 << 7));
  // Set radio in continuous reception
  SX126xSetRx(0xFFFFFF); // Rx Continuous
  for (uint8_t i = 0; i < 32; i++) {
    SX126xReadRegisters(RANDOM_NUMBER_GENERATORBASEADDR, (uint8_t*)&number, 4);
    Serial.println("number 0x" + String(number, HEX));
    memcpy(randomStock + cnt, (uint8_t*)&number, 4);
    cnt += 4;
  }
  SX126xSetStandby(STDBY_RC);
  SX126xWriteRegister(REG_ANA_LNA, regAnaLna);
  SX126xWriteRegister(REG_ANA_MIXER, regAnaMixer);
}
