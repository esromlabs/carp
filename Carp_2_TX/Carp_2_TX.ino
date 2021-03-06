// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>

#define VBATPIN A7

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
  
#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
  // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
  
#elif defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13

#elif defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  #define RFM69_INT     9  // "A"
  #define RFM69_CS      10  // "B"
  #define RFM69_RST     11  // "C"
  #define LED           13

#elif defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13

#elif defined(ARDUINO_NRF52832_FEATHER)
  /* nRF52832 feather w/wing */
  #define RFM69_RST     7   // "A"
  #define RFM69_CS      11   // "B"
  #define RFM69_INT     31   // "C"
  #define LED           17

#endif


/* Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/
 
/* WICED Feather w/wing 
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

#define SLIDER_PIN A0 
void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(SLIDER_PIN, INPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(16, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the receiver
  // uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
  //                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  uint8_t key[] = { 0x06, 0xb2, 0xaf, 0x41, 0x06, 0x6c, 0xd9, 0xaa,
                    0xea, 0x42, 0x42, 0x48, 0xc1, 0xba, 0x68, 0x13};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  pinMode(VBATPIN, INPUT);
}

// measuredvbat *= 2;    // resistors divided by 2, so multiply back
// measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
// measuredvbat /= 1024.0; // convert to voltage
// measuredvbat *= 1000.0; // prepare to send millivolts (integer)
#define VBAT_CONV2_MV 6.4453125 

int pos = 0;
float slider = 0;
int last_pos = 0;
float slider_scale = 180.0 / 1024.0;
int power_blink_interrval = 2000;
unsigned long previous_power_blink = 0;

void loop() {
  // indicate that the power is on
  unsigned long currentMillis = millis();
  if (currentMillis - previous_power_blink > power_blink_interrval) {
    // save the last time you blinked the LED
    previous_power_blink = currentMillis;
    Blink(LED, 2, 1);
  }
  else {
    delay(2);
  }
  
  slider = analogRead(SLIDER_PIN);
  // Serial.println(slider);
  slider = slider * slider_scale;
  // Serial.println(slider);
  
  pos = (int)slider;
  if (last_pos + 1 < pos || last_pos - 1 > pos) {
    last_pos = pos;
    
    char request[20] = "";
    char pos_label[8] = " x move";
    itoa(pos, request, 10);
    strcat(request, pos_label);
    Serial.print("Sending "); Serial.println(request);
    
    // Send a message!
    rf69.send((uint8_t *)request, strlen(request));
    rf69.waitPacketSent();
  
    // Now wait for a reply
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN] = {0};
    uint8_t len = sizeof(buf);
  
    if (rf69.waitAvailableTimeout(500))  { 
      // Should be a reply message for us now   
      if (rf69.recv(buf, &len)) {
        Serial.print("Got a reply: ");
        Serial.println((char*)buf);
        Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
      } else {
        Serial.println("Receive failed");
      }
    } else {
      Serial.println("No reply, no other RFM69 responded!");
    }

    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= VBAT_CONV2_MV; // to millivolts 
    Serial.print("local BAT_MV: " ); Serial.println(measuredvbat);
  }
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}