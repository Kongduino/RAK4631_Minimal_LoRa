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

void fillRandom(uint8_t *rb, uint16_t len) {
  uint16_t count = 0, times = len / 4;
  for (uint8_t i = 0; i < times; i++) {
    SX126xReadRegisters(RANDOM_NUMBER_GENERATORBASEADDR, rb + (i * 4), 4);
    count += 4;
  }
  if (count < len) {
    uint8_t buf[] = {0, 0, 0, 0};
    SX126xReadRegisters(RANDOM_NUMBER_GENERATORBASEADDR, buf, 4);
    uint8_t i = 0;
    while (count < len) {
      rb[count++] = buf[i++];
    }
  }
  hexDump(rb, len);
}
