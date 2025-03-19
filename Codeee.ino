#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>

// WiFi credentials
const char* ssid = "OnePlus Nord CE 2 Lite 5G";
const char* password = "88888888";

// Pin definitions
#define FSR_PIN 35          // Force-sensitive resistor pin (GPIO 35)
#define SOUND_ANALOG_PIN 34 // Sound sensor pin (GPIO 34)
#define PULSE_PIN 36        // Heart rate sensor pin (VP - GPIO 36)
#define SPIKE_THRESHOLD 10.0
#define CF 19.5
#define INITIAL_THRESHOLD 550
#define SAMPLE_SIZE 10
#define HUMAN_MODE 1
#define DOG_MODE 2

// FSR constants
#define FSR_SAMPLES 10
#define FSR_MAX_VALUE 4095

// WebSocket server
WebSocketsServer webSocket = WebSocketsServer(81);

// Variables for sensors
float lastForce = 0;
int signalSamples[SAMPLE_SIZE];
int sampleIndex = 0;
unsigned long lastBeat = 0;
unsigned int BPM;
unsigned long beatIntervals[5] = {0};
int beatIntervalIndex = 0;
int beatCount = 0;
unsigned long startTime;
int threshold = INITIAL_THRESHOLD;
bool fallingEdgeDetected = false;
int mode = HUMAN_MODE;

// Sound sensor smoothing
int soundSamples[10];
int soundSampleIndex = 0;

// FSR smoothing
int fsrSamples[FSR_SAMPLES];
int fsrSampleIndex = 0;
int lastValidFSRValue = 0;

// Scaling function for Dog Mode (ensures BPM is between 100-140)
unsigned int scaleBPMForDog(unsigned int rawBPM) {
    int minRawBPM = 50;  // Minimum detected BPM
    int maxRawBPM = 180; // Maximum detected BPM
    int minScaledBPM = 100; // Target minimum BPM
    int maxScaledBPM = 140; // Target maximum BPM
    
    if (rawBPM < minRawBPM) return minScaledBPM;
    if (rawBPM > maxRawBPM) return maxScaledBPM;
    
    return minScaledBPM + ((rawBPM - minRawBPM) * (maxScaledBPM - minScaledBPM)) / (maxRawBPM - minRawBPM);
}

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  
  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  pinMode(PULSE_PIN, INPUT);
  pinMode(SOUND_ANALOG_PIN, INPUT);
  pinMode(FSR_PIN, INPUT);
  startTime = millis();
  calibrateThreshold();

  // Initialize smoothing array
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    signalSamples[i] = analogRead(PULSE_PIN);
  }

  // Mode selection
  Serial.println("Select mode: 1 for Human, 2 for Dog");
  while (!Serial.available()); // Wait for user input
  mode = Serial.parseInt();
  if (mode == HUMAN_MODE) {
    Serial.println("Human mode selected.");
  } else if (mode == DOG_MODE) {
    Serial.println("Dog mode selected.");
  } else {
    Serial.println("Invalid mode. Defaulting to Human mode.");
    mode = HUMAN_MODE;
  }
}

void loop() {
  webSocket.loop();
  
  // Check sensors and send data
  checkHeartbeat();
  delay(10);
  checkSound();
  checkFSR();
  sendSensorData();
  
  delay(100);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      break;
    case WStype_CONNECTED:
      break;
  }
}

void sendSensorData() {
  StaticJsonDocument<200> doc;
  doc["heartRate"] = BPM;
  
  // Sound processing
  int totalSound = 0;
  for(int i = 0; i < 10; i++) {
    totalSound += soundSamples[i];
  }
  int soundLevel = totalSound / 10;
  doc["soundLevel"] = soundLevel;
  
  // Send FSR value
  doc["fsrValue"] = lastValidFSRValue;
  
  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer);
  webSocket.broadcastTXT(jsonBuffer);
}

void checkSound() {
  soundSamples[soundSampleIndex] = analogRead(SOUND_ANALOG_PIN);
  soundSampleIndex = (soundSampleIndex + 1) % 10;
}

void checkFSR() {
    int rawValue = analogRead(FSR_PIN);
    if (rawValue > FSR_MAX_VALUE) {
        rawValue = FSR_MAX_VALUE;
    }

    fsrSamples[fsrSampleIndex] = rawValue;
    fsrSampleIndex = (fsrSampleIndex + 1) % FSR_SAMPLES;

    long sum = 0;
    for (int i = 0; i < FSR_SAMPLES; i++) {
        sum += fsrSamples[i];
    }

    float newAverage = sum / FSR_SAMPLES;
    
    // If force is detected, update immediately
    if (newAverage > 5) { 
        lastValidFSRValue = map(newAverage, 0, FSR_MAX_VALUE, 0, 100);
    } 
    // If force is removed, decrease value rapidly
    else if (lastValidFSRValue > 0) {
        lastValidFSRValue = max(0, lastValidFSRValue - 10);  // Decrease by 10 each cycle
    }
}


void checkHeartbeat() {
  int rawSignal = analogRead(PULSE_PIN);
  addSample(rawSignal);
  int pulseSignal = getSmoothedSignal();
  
  if (pulseSignal > threshold) {
    fallingEdgeDetected = true;
  } else if (fallingEdgeDetected && (millis() - lastBeat) > getMinBeatInterval()) {
    unsigned long currentTime = millis();
    unsigned long beatInterval = currentTime - lastBeat;
    lastBeat = currentTime;
    fallingEdgeDetected = false;
    beatIntervals[beatIntervalIndex] = beatInterval;
    beatIntervalIndex = (beatIntervalIndex + 1) % 5;
    BPM = calculateRollingAverageBPM();
    beatCount++;

    // Apply scaling for Dog Mode
    if (mode == DOG_MODE) {
      BPM = scaleBPMForDog(BPM);
    }

    Serial.print("Heartbeat detected. BPM: ");
    Serial.println(BPM);
  }
  
  threshold = getSmoothedSignal() + 50;

  // Print average BPM every 10 seconds
  if ((millis() - startTime) >= 10000) {
    if (beatCount > 0) {
      unsigned int avgBPM = calculateRollingAverageBPM(); // Use rolling average
      if (mode == DOG_MODE) {
        avgBPM = scaleBPMForDog(avgBPM); // Apply scaling in Dog Mode
      }
      Serial.print("Average BPM (last 10s): ");
      Serial.println(avgBPM);
    } else {
      Serial.println("No beats detected. Check threshold or sensor connection.");
    }
    beatCount = 0; // Reset counter
    startTime = millis(); // Reset timer
  }
}

void addSample(int sample) {
  signalSamples[sampleIndex] = sample;
  sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
}

int getSmoothedSignal() {
  int sum = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    sum += signalSamples[i];
  }
  return sum / SAMPLE_SIZE;
}

void calibrateThreshold() {
  int sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(PULSE_PIN);
    delay(10);
  }
  threshold = sum / 100 + 50;
}

unsigned int calculateRollingAverageBPM() {
  unsigned long sumIntervals = 0;
  int validIntervals = 0;
  for (int i = 0; i < 5; i++) {
    if (beatIntervals[i] > 0) {
      sumIntervals += beatIntervals[i];
      validIntervals++;
    }
  }
  return validIntervals > 0 ? 60000 / (sumIntervals / validIntervals) : 0;
}

unsigned long getMinBeatInterval() {
  return (mode == HUMAN_MODE) ? 400 : 200;
}