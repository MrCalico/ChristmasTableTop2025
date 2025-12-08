#include <Arduino.h>
#include <FastLED.h>
#include <DFRobotDFPlayerMini.h>
#include <HardwareSerial.h>



const uint8_t NUM_CARS = 10;
const uint16_t START_OFFSET = 0;

// initial speed (ms between frames). We'll copy this into a mutable variable so a pot can control it.
const uint16_t TRAIN_SPEED = 220;  // default speed (ms between frames)
const uint8_t  TRAIN_VOLUME = 10;  // Volume level for DFPlayer Mini (0-30)

//DFRobot DFPlayer Mini setup
// We'll use UART2 (index 2). You can use 1 for UART1 if you prefer.
HardwareSerial ExtSerial(2);
// Pick valid pins for your wiring:
constexpr int RX_PIN = 16;  // GPIO16 (safe for RX)
constexpr int TX_PIN = 17;  // GPIO17 (safe for TX)
constexpr unsigned long BAUD = 9600;
DFRobotDFPlayerMini myDFPlayer;
// Potentiometer pins (ADC1) - using GPIO34/35 (far from UART2 on GPIO16/17) to minimize interference
constexpr int VOLUME_POT_PIN = 34; // ADC1_CH6 (GPIO34) - far from UART2, less noisy
// NOTE: Two 10k pots in parallel on the same 3.3V rail create loading that limits ADC range to ~0–450.
// SOLUTION: Add a 10–100 nF capacitor from wiper to GND (reduces noise) and map 0–450 instead of 0–4095.
// FUTURE: Upgrade to 100k pots to eliminate parallel loading, then change mapping back to 0–4095.
constexpr int VOLUME_POT_READ_INTERVAL = 250; // ms between pot reads
constexpr uint8_t VOLUME_HYST = 1; // minimum volume change to apply
// Second pot for train speed control
constexpr int SPEED_POT_PIN = 35; // ADC1_CH7 (GPIO35) - far from UART2, less noisy
constexpr uint8_t SPEED_HYST = 5; // ms delta before applying new speed
constexpr int SPEED_MIN_MS = 90;  // fastest animation delay
constexpr int SPEED_MAX_MS = 500; // slowest animation delay

constexpr uint8_t LED_PIN = 4;
constexpr uint16_t NUM_LEDS = 800;

CRGB leds[NUM_LEDS];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  // Initialize DFPlayer Mini
  ExtSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN ); // Initialize Serial for debug output

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(ExtSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));

  /*
  */
  // configure ADC pin
  analogReadResolution(12); // 12-bit ADC (0-4095)
  // Ensure full-scale range up to 3.3V on the ADC pin
  analogSetPinAttenuation(VOLUME_POT_PIN, ADC_11db);
  // Configure attenuation for the speed pot as well
  analogSetPinAttenuation(SPEED_POT_PIN, ADC_11db);
  Serial.println(F("ADC configured: 12-bit resolution, 11dB attenuation"));
  // nothing else required for ADC1 pins on ESP32

  myDFPlayer.volume(TRAIN_VOLUME);  //Set volume value (0~30).
  /*  
  myDFPlayer.play(11);  //Play the first mp3 - Merry Christmas
  Serial.println(F("Playing Merry Christmas"));
  delay(5000);
  */
}


void loop() {
  static uint32_t lastPotRead = 0;
  static uint8_t currentVolume = TRAIN_VOLUME;
  static uint32_t lastSpeedRead = 0;
  static int trainSpeedMs = TRAIN_SPEED; // mutable copy controlled by pot

  // Read potentiometer periodically and map to volume 0-30
  uint32_t now = millis();
  if (!lastPotRead || (now - lastPotRead >= VOLUME_POT_READ_INTERVAL)) {
    lastPotRead = now;
    int raw = analogRead(VOLUME_POT_PIN); // 0..4095
  // map to 0..30 (12-bit ADC range is 0..4095)
  uint8_t vol = map(raw, 0, 4095, 0, 30);
    if (vol > 30) vol = 30;
    if (vol < 0) vol = 0;
    if (vol + VOLUME_HYST < currentVolume || vol > currentVolume + VOLUME_HYST) {
      currentVolume = vol;
      myDFPlayer.volume(currentVolume);
      Serial.print(F("Raw ADC: "));
      Serial.print(raw);
      Serial.print(F(" -> Mapped volume: "));
      Serial.println(currentVolume);
    }
  }

  // Read speed pot periodically and map to speed range
  if (now - lastSpeedRead >= VOLUME_POT_READ_INTERVAL) {
    lastSpeedRead = now;
    int raw2 = analogRead(SPEED_POT_PIN);
    int mapped = map(raw2, 0, 4095, SPEED_MIN_MS, SPEED_MAX_MS);
    if (abs(mapped - trainSpeedMs) > SPEED_HYST) {
      trainSpeedMs = mapped;
      Serial.print(F("Raw speed ADC: "));
      Serial.print(raw2);
      Serial.print(F(" -> trainSpeedMs: "));
      Serial.println(trainSpeedMs);
    }
  }
 
  static uint16_t i = START_OFFSET;

  // Clear all LEDs 
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Simple train: white headlight 2 positions ahead, red body at current position
  if (i + 1 < NUM_LEDS) {
    leds[i + 1] = CRGB(100, 100, 100);  // white headlight ahead
  }
  leds[i] = CRGB(200, 0, 0);  // red body at head position

  FastLED.show();
  delay(trainSpeedMs);

  // Move to next position
  i = START_OFFSET + (i + 1) % (NUM_LEDS - START_OFFSET);

}

