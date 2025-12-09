#include <Arduino.h>
#include <FastLED.h>
#include <DFRobotDFPlayerMini.h>
#include <HardwareSerial.h>

const uint8_t NUM_CARS = 11;  // number of train cars (not including locomotive and caboose) Odd # for green before cabose.
const uint16_t START_OFFSET = 0;

// initial speed (ms between frames). We'll copy this into a mutable variable so a pot can control it.
const uint16_t TRAIN_SPEED = 220;  // default speed (ms between frames)
const uint8_t  TRAIN_VOLUME = 30;  // Volume level for DFPlayer Mini (0-30)

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
constexpr int SPEED_MIN_MS = 100;  // fastest animation delay
constexpr int SPEED_MAX_MS = 500; // slowest animation delay

constexpr int BUSY_PIN = 27; // optional pin to monitor DFPlayer busy status

constexpr uint8_t LED_PIN = 4;
constexpr uint16_t NUM_LEDS = 800;

uint8_t playerState = 0;
  
CRGB leds[NUM_LEDS];

void QueueTrack(int track, bool waitForCompletion = true, uint8_t volume = TRAIN_VOLUME)
{
  myDFPlayer.volume(volume); // Set volume value (0~30).
  // Play an initial sound on startup (uncomment or change track as desired)
  myDFPlayer.play(track); // play track 11 on the SD card - Ho Ho Ho Merry Christmas
  Serial.print("Playing track: ");
  Serial.println(track);
  if(waitForCompletion) {
    uint32_t now = millis();
    uint32_t lastPlayCheck = now;
    Serial.print(F("Wating, track state: "));
    delay(1000);
    while(waitForCompletion) {
      now = millis();
      if (now - lastPlayCheck >= 500) {
        waitForCompletion = !digitalRead(BUSY_PIN); // simulate readState() using BUSY pin
        Serial.print(playerState);
        lastPlayCheck = now;
      }
      delay(10); // small delay to avoid busy loop
    }
    Serial.println(F(" Track complete."));
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  // Initialize DFPlayer Mini
  ExtSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN ); // Initialize Serial for debug output

  pinMode(BUSY_PIN, INPUT);
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

  QueueTrack(11, true, TRAIN_VOLUME); // Play initial track 11 and wait for completion
}

void loop() {
  static uint32_t lastPotRead = 0;
  static uint8_t currentVolume = TRAIN_VOLUME;
  static uint32_t lastSpeedRead = 0;
  static int trainSpeedMs = TRAIN_SPEED; // mutable copy controlled by pot
  static uint32_t lastPlayCheck = millis();
  static bool playbackInitialized = false;  // flag to start chugging sounds on first loop
  static uint16_t i = START_OFFSET;
  static uint8_t lastPlayTrack = 0;

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
 
  // Clear all LEDs 
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Simple train: one white headlight ahead + multi-LED red locomotive + trailing cars
  const uint8_t LOCO_REDS = 5; // number of red LEDs for the locomotive body
  if (i + 1 < NUM_LEDS) {
    leds[i + 1] = CRGB(150, 150, 150);  // white headlight ahead
  }

  // Draw locomotive body as LOCO_REDS red LEDs behind the headlight
  for (uint8_t r = 0; r < LOCO_REDS; r++) {
    if (i >= r) leds[i - r] = CRGB::Red;
  }

  // Draw trailing cars (4 LEDs each, alternating green and red), starting after the locomotive
  for (uint8_t car = 0; car < NUM_CARS; car++) {
    uint16_t offset = LOCO_REDS + (car * 4);  // Each car starts after the locomotive
    CRGB color = (car % 2 == 0) ? CRGB(0, 200, 0) : CRGB(200, 0, 0);  // green, red, green, red...
    if (i > offset)     leds[i - offset - 1] = color;
    if (i > offset + 1) leds[i - offset - 2] = CRGB(color.r * 0.75, color.g * 0.75, color.b * 0.75);  // 75% brightness
    if (i > offset + 2) leds[i - offset - 3] = CRGB(color.r * 0.5, color.g * 0.5, color.b * 0.5);   // 50% brightness
    if (i > offset + 3) leds[i - offset - 4] = CRGB(color.r * 0.25, color.g * 0.25, color.b * 0.25); // 25% brightness (tail light)
  }

  // Then add a dedicated caboose after the cars
  uint16_t caboseStart = LOCO_REDS + (NUM_CARS * 4) + 1;
  if (i > caboseStart)     leds[i - caboseStart - 1] = CRGB(255, 0, 0);
  if (i > caboseStart + 1) leds[i - caboseStart - 2] = CRGB(200, 0, 0);
  if (i > caboseStart + 2) leds[i - caboseStart - 3] = CRGB(100, 80, 0);

  FastLED.show();
  delay(trainSpeedMs);

  // If we've reached the end of the line, play the shutdown mp3 and pause
  if (i == 0) {
    // At start position
    playbackInitialized = false;  // Reset so chugging sounds restart each lap
    QueueTrack(1, true, currentVolume);  // Play track #1 (station) at start
    QueueTrack(10, true, currentVolume);  // Play track #10 (all aboard) after shutdown
    lastPlayCheck = now;  // Reset timer after playing startup tracks
    lastPlayTrack = 0;   // Reset track counter for chugging sounds
  } else {
    if (i == NUM_LEDS - 1) {
      QueueTrack(11, true, currentVolume);  // Play track #11 at end of line
      Serial.println(F("Reached end of line - playing track 11"));
      lastPlayCheck = now;  // Reset timer after playing end-of-line track
    } else {   // Not at the end yet.
      if(now - lastPlayCheck >= 300) {
        if(digitalRead(BUSY_PIN)) { // simulate readState() using BUSY pin
          // Previous track finished, queue next chugging sound if any
             QueueTrack(++lastPlayTrack%9, false, currentVolume);  // play chugging sound (track 1)
            lastPlayCheck = now;  // Reset timer after queuing new track
        } 
      }
    }
  } 

  // Move to next position
  i = START_OFFSET + (i + 1) % (NUM_LEDS - START_OFFSET);
  if(i%100==0) {
    Serial.print(F("Train position i: "));
    Serial.println(i);
  }
}

