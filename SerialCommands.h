void displayHDC1080();
void displayHT();
void displaySGP30();
void displayDHT();

void handleSerial() {
  memset(msgBuf, 0, BUFF_LENGTH);
  int ix = 0;
  while (Serial.available() && ix < 255) {
    char c = Serial.read();
    delay(10);
    if (c > 31) msgBuf[ix++] = c;
  } msgBuf[ix] = 0;
  char c = msgBuf[0]; // Command
  if (c == '/') {
    c = msgBuf[1];
    char c1 = msgBuf[2];
    if (c == '>') {
      char buff[256];
      strcpy(buff, (char*)msgBuf + 2);
#ifdef NEED_SSD1306
      oled.print("Sending msg...");
#endif // NEED_SSD1306
      prepareJSONPacket(buff);
      sendJSONPacket();
#ifdef NEED_SSD1306
      oled.println(" done");
#endif // NEED_SSD1306
      return;
    } else if (c == 'D' && c1 == 'N') {
      setDeviceName((char*)msgBuf + 3);
      Serial.println("setDeviceName in /DN");
#ifdef NEED_SSD1306
      oled.println("New device name:");
      oled.println(deviceName);
#endif // NEED_SSD1306
      return;
    } else if (c == 'A' && c1 == 'P') {
      setAutoPing((char*)msgBuf + 3);
      return;
    } else if (c == 'E' || c == 'e') {
      if (c1 == '1') needEncryption = true;
      if (c1 == '0') needEncryption = false;
#ifdef NEED_SSD1306
      oled.print("Encryption ");
      oled.println(c1 == '1' ? "ON" : "OFF");
#endif // NEED_SSD1306
      return;
    } else if (strcmp((char*)msgBuf, "/HM1") == 0) {
      needAuthentification = true;
      char buff[64];
      Serial.println("needAuthentification set to: ON");
#ifdef NEED_SSD1306
      oled.println("HMAC: ON");
#endif // NEED_SSD1306
      return;
    } else if (strcmp((char*)msgBuf, "/HM0") == 0) {
      needAuthentification = false;
      Serial.println("needAuthentification set to: OFF");
#ifdef NEED_SSD1306
      oled.println("HMAC: OFF");
#endif // NEED_SSD1306
      return;
    } else if (c == 'R' || c == 'r') {
      if (c1 == '1') setPongBack(true);
      if (c1 == '0') setPongBack(false);
#ifdef NEED_SSD1306
      oled.print("PONG back ");
      oled.println(c1 == '1' ? "ON" : "OFF");
#endif // NEED_SSD1306
      return;
    } else if (c == 'F' && c1 == 'Q') {
      setFQ((char*)msgBuf + 3);
      return;
    } else if (c == 'S' && c1 == 'F') {
      setSF((char*)msgBuf + 3);
      return;
    } else if (c == 'B' && c1 == 'W') {
      setBW((char*)msgBuf + 3);
      return;
    } else if (c == 'C' && c1 == 'R') {
      setCR((char*)msgBuf + 3);
      return;
    } else if (c == 'T' && c1 == '0') {
      setTxPower((char*)msgBuf + 3);
      return;
    } else if (c == 'p' || c == 'P') {
      if (c1 < 32) {
        sendPing();
        return;
      } else if (c1 == 'w' || c1 == 'W') {
        setPWD((char*)msgBuf + 2);
        return;
      }
    } else if (c == 'O' && c1 == 'C') {
      if (msgBuf[3] == '1') {
        // OCP ON
        OCP_ON = true;
        Radio.Standby();
        Radio.Write(REG_OCP, 0x18); // MAX OCP
        Radio.Rx(RX_TIMEOUT_VALUE);
        Serial.println("--> OCP Trim on, Max OCP");
        return;
      } else if (msgBuf[3] == '0') {
        // OCP Trim OFF
        OCP_ON = false;
        Radio.Standby();
        Radio.Write(REG_OCP, 0); // MAX OCP
        Radio.Rx(RX_TIMEOUT_VALUE);
        Serial.println("--> OCP Trim off");
        return;
      }
    } else if (c == 'D' && c1 == 'H') {
#ifdef NEED_BME
      displayBME680();
#endif
#ifdef NEED_DHT
      displayDHT();
#endif
#ifdef NEED_HDC1080
      displayHDC1080();
#endif
#ifdef NEED_SGP30
      displaySGP30();
#endif
    }
  } else {
    // Serial.println((char*)msgBuf);
    showHelp();
  }
}

void showHelp() {
  char buff[256];
  Serial.println(" +==================+================================+");
  sprintf(buff, " |%-18s|%32s|\n", "     Command", "Explanation            ");
  Serial.print(buff);
  Serial.println(" +==================+================================+");
  sprintf(buff, " |%-18s|%32s|\n", "/DN<max 32 chars>", "Set device name");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", " -> right now", deviceName);
  Serial.print(buff);
  Serial.println(" +==================+================================+");
  sprintf(buff, " |%-18s|%32s|\n", "/>xxxxxxxxxxx", "send string xxxxxxxxxxx");
  Serial.print(buff);
  Serial.println(" +==================+================================+");
  Serial.println(" |                      OPTIONS                      |");
  Serial.println(" +==================+================================+");
  sprintf(buff, " |%-18s|%32s|\n", " /E1 or /e1", "turn on encryption");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", " /E0 or /e0", "turn off encryption");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", "  -> right now", needEncryption ? "on" : "off");
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /HM1", "turn on authentication");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", " /HM0", "turn off authentication");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", "  -> right now", needAuthentification ? "on" : "off");
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /PW<32 chars>", "set password [32 chars]");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", " /pw", "[exactly 32] (Uses AES256)");
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /R1 or /r1", "turn on PONG back [Reply on]");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", " /R0 or /r0", "turn off PONG back [Reply off]");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", "  -> right now", pongBack ? "on" : "off");
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /FQ<float>", "Set a new LoRa frequency");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", " Frequency:", "Between 862 and 1020 MHz (HF)");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%28.3f MHz|\n", "  -> right now", (myFreq / 1e6));
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /SF[7-12]", "Set a new LoRa Spreading Factor");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32u|\n", "  -> right now", mySF);
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /BW[0-9]", "Set a new LoRa Bandwidth");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", "", "From 0: 7.8 KHz to 9: 500 KHz");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32.3f|\n", "  -> right now", BWs[myBW]);
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /CR[5-8]", "Set a new Coding Rate");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32u|\n", "  -> right now", myCR);
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /TX[7-17]", "Set a new Tx Power");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32u|\n", "  -> right now", TxPower);
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /OC1 or /oc1", "turn on OCP Trim");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", " /OC0 or /oc0", "turn off OCP Trim");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", "  -> right now", OCP_ON ? "on" : "off");
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /PB1 or /pb1", "turn on PA_BOOST");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", " /pb0 or /pb0", "turn off PA_BOOST");
  Serial.print(buff);
  sprintf(buff, " |%-18s|%32s|\n", "  -> right now", PA_BOOST ? "on" : "off");
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /AP[0-X]", "Set autoPING OFF (0) or ON");
  Serial.print(buff);
  if (!needPing) sprintf(buff, " |%-18s|%32s|\n", "  -> right now", "OFF");
  else {
    String np;
    np = "ON: " + String(pingFrequency / 60e3, 1) + " mn";
    sprintf(buff, " |%-18s|%32s|\n", "  -> right now", np.c_str());
  }
  Serial.print(buff);
  Serial.println(" +---------------------------------------------------+");
  sprintf(buff, " |%-18s|%32s|\n", " /P or /p", "Send PING packet");
  Serial.print(buff);
  Serial.println(" +==================+================================+");
  Serial.println(" | Anything else    | show this help message.        |");
  Serial.println(" +==================+================================+");
  Serial.println(" | NEED_* OPTIONS   |                                |");
  Serial.println(" +---------------------------------------------------+");
#ifdef NEED_DEBUG
  sprintf(buff, " |%-18s|%32s|\n", " NEED_DEBUG", "ON");
#else
  sprintf(buff, " |%-18s|%32s|\n", " NEED_DEBUG", "OFF");
#endif
  Serial.print(buff);

#ifdef NEED_EEPROM
  sprintf(buff, " |%-18s|%32s|\n", " NEED_EEPROM", "ON");
#else
  sprintf(buff, " |%-18s|%32s|\n", " NEED_EEPROM", "OFF");
#endif
  Serial.print(buff);

#ifdef NEED_SIDE_I2C
  sprintf(buff, " |%-18s|%32s|\n", " NEED_SIDE_I2C", "ON");
#else
  sprintf(buff, " |%-18s|%32s|\n", " NEED_SIDE_I2C", "OFF");
#endif
  Serial.print(buff);

#ifdef NEED_SSD1306
  sprintf(buff, " |%-18s|%32s|\n", " NEED_SSD1306", "ON");
#else
  sprintf(buff, " |%-18s|%32s|\n", " NEED_SSD1306", "OFF");
#endif
  Serial.print(buff);

#ifdef NEED_DHT
  sprintf(buff, " |%-18s|%32s|\n", " NEED_DHT", "ON");
#else
  sprintf(buff, " |%-18s|%32s|\n", " NEED_DHT", "OFF");
#endif
  Serial.print(buff);

#ifdef NEED_BME
  sprintf(buff, " |%-18s|%32s|\n", " NEED_BME", "ON");
#else
  sprintf(buff, " |%-18s|%32s|\n", " NEED_BME", "OFF");
#endif
  Serial.print(buff);

#ifdef NEED_HDC1080
  sprintf(buff, " |%-18s|%32s|\n", " NEED_HDC1080", "ON");
#else
  sprintf(buff, " |%-18s|%32s|\n", " NEED_HDC1080", "OFF");
#endif
  Serial.print(buff);

#ifdef NEED_CCS811
  sprintf(buff, " |%-18s|%32s|\n", " NEED_CCS811", "ON");
#else
  sprintf(buff, " |%-18s|%32s|\n", " NEED_CCS811", "OFF");
#endif
  Serial.print(buff);
  Serial.println(" +==================+================================+");
}
