//TPMS sensor IDs
#define ID_FL 0x67f40001
#define ID_FR 0x67f40002
#define ID_RL 0x67f40003
#define ID_RR 0x67f40004
//use OLED to display the received values
#define USE_OLED


//RadioLib------------------------------------------------------------------------
#include <RadioLib.h>
// CC1101 has the following connections:
// CS pin:    15/D8(ESP8266)
// GDO0 pin:  5/D1(ESP8266)
// RST pin:   unused
// GDO2 pin:  3 (optional)
CC1101 radio = new Module(15, 5, RADIOLIB_NC, 3);  // cs, gdo0, lib, gdo2

#ifdef USE_OOK_GOD
#define CC1101_AGCCTRL2     0x1B        // AGC control
#define CC1101_AGCCTRL1     0x1C        // AGC control
#define CC1101_AGCCTRL0     0x1D        // AGC control
#endif
//--------------------------------------------------------------------------------


//Display-------------------------------------------------------------------------
#ifdef USE_OLED
#include <U8g2lib.h>
#define OLED_RESET     U8X8_PIN_NONE // Reset pin
#define OLED_SDA 14                  // D6
#define OLED_SCL 12                  // D5

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_RESET, OLED_SCL, OLED_SDA);
#endif
//--------------------------------------------------------------------------------

//Button--------------------------------------------------------------------------
#include <EasyButton.h>
#define BUTTON_PIN 0

EasyButton button(BUTTON_PIN);
//--------------------------------------------------------------------------------

//global cycle time
unsigned long time_last; //last lifetime in ms

void setup() {
  //init cycle time measurement
  time_last = millis();

  Serial.begin(115200);
  Serial.println();
  Serial.println();

  //RadioLib------------------------------------------------------------------------
  // initialize CC1101 with default settings
  Serial.print(F("[CC1101] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
  Serial.print(F("[CC1101] Version ")); Serial.println(radio.getChipVersion(), HEX);
#ifdef USE_OOK_GOD
  radio.setOOK(true);
#endif
  radio.setFrequency(433.92);
  radio.setRxBandwidth(162.0);
  radio.setBitRate(19.2);
  radio.setEncoding(RADIOLIB_ENCODING_NRZ);
  
  radio.setSyncWord(0x69, 0x6a); //machester encoded 67
  //radio.setSyncWord(0x55, 0x56);
  radio.fixedPacketLengthMode(14);
  //radio.fixedPacketLengthMode(16);
  radio.setCrcFiltering(false);

  // set the function that will be called
  // when new packet is received
  radio.setPacketReceivedAction(setFlag);

#ifdef USE_OOK_GOD
  //Design Note DN022   swra215e.pdf
  //The optimum AGC settings change with RX filter bandwidth and data rate, but for OOK/ASK the following has been found to give good results:
  radio.SPIwriteRegister(CC1101_AGCCTRL2, 0x03); //AGCCTRL2 0x03 to 0x07
  radio.SPIwriteRegister(CC1101_AGCCTRL1, 0x00); //AGCCTRL1 0x00
  radio.SPIwriteRegister(CC1101_AGCCTRL0, 0x91); //AGCCTRL0 0x91 or 0x92
#endif

  // start listening for packets
  Serial.print(F("[CC1101] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // radio.standby()
  // radio.sleep()
  // radio.transmit();
  // radio.receive();
  // radio.readData();
  //--------------------------------------------------------------------------------

  //Display-------------------------------------------------------------------------
#ifdef USE_OLED
  //I2C and SPI is shard on GPIO14(D6) and GPIO12(D5), so we need to reconfigure
  //each time we want to refresh the display. Configure for I2C usage
  pinMode(OLED_SDA, INPUT_PULLUP);
  pinMode(OLED_SCL, INPUT_PULLUP);
  u8g2.begin();
  handle_oled();
  //Configure for SPI usage
  pinMode(OLED_SDA, SPECIAL);
  pinMode(OLED_SCL, SPECIAL);
#endif
  //--------------------------------------------------------------------------------
  
  //Button--------------------------------------------------------------------------
  // Initialize the button.
  button.begin();
  // Add the callback function to be called when the button is pressed.
  button.onPressed(onPressed);
  //--------------------------------------------------------------------------------
}

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

byte decode_manchester_nibble(byte b, bool& error) {
  switch(b) {
    case 0x55: return 0x0;
    case 0x56: return 0x1;
    case 0x59: return 0x2;
    case 0x5A: return 0x3;
    case 0x65: return 0x4;
    case 0x66: return 0x5;
    case 0x69: return 0x6;
    case 0x6A: return 0x7;
    case 0x95: return 0x8;
    case 0x96: return 0x9;
    case 0x99: return 0xA;
    case 0x9A: return 0xB;
    case 0xA5: return 0xC;
    case 0xA6: return 0xD;
    case 0xA9: return 0xE;
    case 0xAA: return 0xF;
    default: 
      error=true; 
      return 0x00;
  }
}

byte decode_manchester_byte(byte bnh, byte bnl, bool& error) {
  byte ret = 0;
  ret = (decode_manchester_nibble(bnh, error) << 4);
  ret = ret | decode_manchester_nibble(bnl, error);
  return ret;
}

float pressures[4] = {0,0,0,0};
int temperatures[4] = {-273,-273,-273,-273};
unsigned int alive[4] = {20000,15000,10000,5000};
char codes1[4] = {0,0,0,0};
char codes2[4] = {0,0,0,0};
char rcvind[4] = {0,0,0,0};
float rssi[4] = {0,0,0,0};
char lqi[4] = {0,0,0,0};
unsigned long last_tpms_id;
byte last_checksum;


#ifdef USE_OLED
unsigned long last_refresh;

void handle_oled() {
  char buffer[40];
  //I2C and SPI is shard on GPIO14(D6) and GPIO12(D5), so we need to reconfigure
  //each time we want to refresh the display. Configure for I2C usage
  pinMode(OLED_SDA, INPUT_PULLUP);
  pinMode(OLED_SCL, INPUT_PULLUP);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvR12_tr);
  u8g2.drawStr(0, 12, "unverbuggtRDKS");

  //draw a "car"
  u8g2.drawFrame(54,20,20,42);
  u8g2.drawFrame(57,32,14,26);
  u8g2.drawFrame(57,32,14,4);

  u8g2.drawLine(52, 23, 52, 32);
  u8g2.drawLine(53, 23, 53, 32);

  u8g2.drawLine(74, 23, 74, 32);
  u8g2.drawLine(75, 23, 75, 32);

  u8g2.drawLine(52, 50, 52, 59);
  u8g2.drawLine(53, 50, 53, 59);

  u8g2.drawLine(74, 50, 74, 59);
  u8g2.drawLine(75, 50, 75, 59);

  //draw pressures
  u8g2.setFont(u8g2_font_helvR18_tr);
  if (alive[0] > 0) {
    snprintf(buffer, sizeof(buffer), "%.2f", pressures[0]);
    u8g2.drawStr(0, 37, buffer);
  }
  if (alive[1] > 0) {
    snprintf(buffer, sizeof(buffer), "%.2f", pressures[1]);
    u8g2.drawStr(128-u8g2.getStrWidth(buffer), 37, buffer);
  }
  if (alive[2] > 0) {
    snprintf(buffer, sizeof(buffer), "%.2f", pressures[2]);
    u8g2.drawStr(0, 63, buffer);
  }
  if (alive[3] > 0) {
    snprintf(buffer, sizeof(buffer), "%.2f", pressures[3]);
    u8g2.drawStr(128-u8g2.getStrWidth(buffer), 63, buffer);
  }
  u8g2.sendBuffer();
  
  //Configure for SPI usage
  pinMode(OLED_SDA, SPECIAL);
  pinMode(OLED_SCL, SPECIAL);
}
#endif

// Callback function to be called when the button is pressed.
void onPressed() {
    Serial.println("Button has been pressed!");
}


void loop() {
  unsigned long time_cur; //current lifetime in [ms]
  unsigned int cycle; //cycle time in [ms]
  
  int i, k;
  char c;
  String str;
  bool error;
  byte data[8];
  byte checksum;

  unsigned long tpms_id;
  float pressure;
  int temperature;
  char code1;
  char code2;

  button.read();

  //generate cycle time  
  time_cur = millis();
  if (time_cur >= time_last) {
    cycle = time_cur - time_last;
  }
  else { //millis() overflows approximately every 50 days
    cycle = 0xFFFFFF - time_last + time_cur;
  }
  time_last = time_cur;

  // check if the flag is set
  if(receivedFlag) {
    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    int state = radio.readData(str);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int numBytes = radio.getPacketLength();
      int state = radio.readData(byteArr, numBytes);
    */

    if (state == RADIOLIB_ERR_NONE && str.length() == 14) { //16
      //Serial.print(F("."));
      k = 1;
      data[0] = 0x67; //Sync Word
      //k = 0;
      error = false;
      for (i=0; i < str.length(); i+=2) {
        data[k] = decode_manchester_byte(str.charAt(i), str.charAt(i+1), error);
        k++;
      }
      if (error) {
        Serial.println(F("Manchester ERROR!"));
      } else {
        checksum = 0;
        for (k=0; k < 7; k++) {
          checksum = checksum + data[k];
        }
        if (checksum == data[7]) {
          //https://github.com/merbanan/rtl_433/blob/master/src/devices/tpms_ford.c
          //ID, pressure and temperature decoding seems to work. Flags seem to be different for F2GT
          tpms_id = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
          pressure = data[4] * 0.25 * 0.0689476;
          temperature = -273; //decoding still unknown for F2GT
          code1 = data[5];
          code2 = data[6];
          if (tpms_id == ID_FL) {
            i = 0;
          } else if (tpms_id == ID_FR) {
            i = 1;
          } else if (tpms_id == ID_RL) {
            i = 2;
          } else if (tpms_id == ID_RR) {
            i = 3;
          } else {
            i = -1;
          }
          if (i != -1) {
            alive[i] = 60000;
            pressures[i] = pressure;
            if (temperature != -273) {
              temperatures[i] = temperature;
            }            
            codes1[i] = code1;
            codes2[i] = code2;
            rcvind[i] = rcvind[i] + 1;
            rssi[i] = radio.getRSSI();
            lqi[i] = radio.getLQI();
          }
          if (tpms_id != last_tpms_id || checksum != last_checksum) {
            //Serial.println();
            if (i == -1) {
              Serial.print(F("?"));
            }
            Serial.print(F("ID "));
            Serial.print(tpms_id,HEX);
            Serial.print(F(", "));
            Serial.print(pressure);
            Serial.print(F(" bar, "));
            if (temperature != -273) {
              Serial.print(temperature);
            } else {
              Serial.print(F("?"));
            }
              Serial.print(F(" °C, "));
            Serial.print(code1,HEX);
            Serial.print(F(", "));
            Serial.print(code2,HEX);
            Serial.print(F(", "));
            Serial.print(F("RSSI: "));
            Serial.print(radio.getRSSI());
            Serial.print(F(" dBm, "));
            Serial.print(F("LQI: "));
            Serial.print(radio.getLQI());
            Serial.println();
            last_tpms_id = tpms_id;
            last_checksum = checksum;
          }
        } else {
          Serial.println(F("Checksum ERROR!"));
        }
      }
    } else {
      // some other error occurred
      Serial.print(str.length());
      Serial.print(F(" failed, code "));
      Serial.println(state);
    }

    // put module back to listen mode
    radio.startReceive();
  }
#ifdef USE_OLED
  if ( (millis() - last_refresh) > 5000) {
    last_refresh = millis();
    handle_oled();
  }
#endif
  for (i=0; i < 4; i++) {
    if (cycle < alive[i]) {
      alive[i] = alive[i] - cycle;
    } else {
      alive[i] = 0;
    }
  }
}
