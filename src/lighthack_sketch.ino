/*
 * EOS Wheel Controller
 * For Arduino UNO R4 WiFi
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

// Network configuration
const char* ssid = "ssid";
const char* password = "password";

const IPAddress destinationIP(192, 168, 1, 246);  // Console Address
const IPAddress staticIP(192, 168, 1, 200);       // Local static ip
const unsigned int destinationPort = 8000;        // ETC EOS OSC port (default)
const unsigned int localPort = 8001;              // Local port to listen on

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Encoder pins
const int panEncoderClkPin = 2;   // Pan encoder CLK
const int panEncoderDtPin = 3;    // Pan encoder DT
const int tiltEncoderClkPin = 8;  // Tilt encoder CLK
const int tiltEncoderDtPin = 12;  // Tilt encoder DT
const int shiftButtonPin = 7;     // Optional: Shift button for fine control

// Encoder multipliers
const float COARSE_MULTIPLIER = 5.0;  // Multiplier for coarse movement
const float FINE_MULTIPLIER = 20.0;   // Multiplier for fine movement

// Variables for tracking encoders
volatile int panDelta = 0;
volatile int tiltDelta = 0;
bool displayNeedsUpdate = true;

// Encoder state tracking variables
volatile uint8_t prevPanState = 0;
volatile uint8_t prevTiltState = 0;

// Smoothing variables
const int SMOOTH_SAMPLES = 3;
int panSamples[SMOOTH_SAMPLES] = {0};
int tiltSamples[SMOOTH_SAMPLES] = {0};
int sampleIndex = 0;

// Adaptive speed variables
unsigned long lastPanMoveTime = 0;
unsigned long lastTiltMoveTime = 0;

// Current parameter values from EOS
float currentPanValue = 0.0;
float currentTiltValue = 0.0;

// Network objects
WiFiUDP Udp;
int status = WL_IDLE_STATUS;

// OSC message buffer
const int OSC_BUFFER_SIZE = 256;
char oscBuffer[OSC_BUFFER_SIZE];

void setup() {
  Serial.begin(9600);
  Serial.println("EOS Pan/Tilt Controller Starting...");
  
  // Initialize pins
  pinMode(panEncoderClkPin, INPUT_PULLUP);
  pinMode(panEncoderDtPin, INPUT_PULLUP);
  pinMode(tiltEncoderClkPin, INPUT_PULLUP);
  pinMode(tiltEncoderDtPin, INPUT_PULLUP);
  pinMode(shiftButtonPin, INPUT_PULLUP);
  
  // Pre-read encoder pins to initialize state
  prevPanState = (digitalRead(panEncoderClkPin) << 1) | digitalRead(panEncoderDtPin);
  prevTiltState = (digitalRead(tiltEncoderClkPin) << 1) | digitalRead(tiltEncoderDtPin);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Connecting WiFi..");
  
  // Initialize WiFi and UDP
  setupWiFi();
  Udp.begin(localPort);
  
  // Subscribe to EOS parameters
  issueEosSubscribes();
  
  // Display initial values
  updateDisplay();
  
  // Initialize timing variables
  lastPanMoveTime = millis();
  lastTiltMoveTime = millis();
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(panEncoderClkPin), panEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(panEncoderDtPin), panEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(tiltEncoderClkPin), tiltEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(tiltEncoderDtPin), tiltEncoderInterrupt, CHANGE);
}

void loop() {
  // Handle incoming OSC messages
  receiveOSC();
  
  // Get current multiplier based on shift button
  float multiplier = (digitalRead(shiftButtonPin) == LOW) ? FINE_MULTIPLIER : COARSE_MULTIPLIER;
  
  // Handle Pan encoder changes with smoothing
  if (panDelta != 0) {
    // Add to samples array
    panSamples[sampleIndex] = panDelta;
    panDelta = 0; // Reset delta after capturing
    
    // Calculate average of samples
    int smoothedPanDelta = 0;
    for (int i = 0; i < SMOOTH_SAMPLES; i++) {
      smoothedPanDelta += panSamples[i];
    }
    smoothedPanDelta = smoothedPanDelta / SMOOTH_SAMPLES;
    
    // Only send if the averaged value is non-zero
    if (smoothedPanDelta != 0) {
      // Calculate adaptive multiplier based on speed
      unsigned long currentTime = millis();
      unsigned long timeDiff = currentTime - lastPanMoveTime;
      lastPanMoveTime = currentTime;
      
      // Adjust multiplier - faster movements get higher multiplier
      float speedFactor = constrain(map(timeDiff, 5, 100, 150, 50), 50, 150) / 100.0;
      float adjustedMultiplier = multiplier * speedFactor;
      
      float adjustedPanDelta = smoothedPanDelta * adjustedMultiplier;
      sendEosWheelMove(true, adjustedPanDelta);
      displayNeedsUpdate = true;
      
      // Clear the sample that was just used
      panSamples[sampleIndex] = 0;
    }
  }
  
  // Handle Tilt encoder changes with smoothing
  if (tiltDelta != 0) {
    // Add to samples array
    tiltSamples[sampleIndex] = tiltDelta;
    tiltDelta = 0; // Reset delta after capturing
    
    // Calculate average of samples
    int smoothedTiltDelta = 0;
    for (int i = 0; i < SMOOTH_SAMPLES; i++) {
      smoothedTiltDelta += tiltSamples[i];
    }
    smoothedTiltDelta = smoothedTiltDelta / SMOOTH_SAMPLES;
    
    // Only send if the averaged value is non-zero
    if (smoothedTiltDelta != 0) {
      // Calculate adaptive multiplier based on speed
      unsigned long currentTime = millis();
      unsigned long timeDiff = currentTime - lastTiltMoveTime;
      lastTiltMoveTime = currentTime;
      
      // Adjust multiplier - faster movements get higher multiplier
      float speedFactor = constrain(map(timeDiff, 5, 100, 150, 50), 50, 150) / 100.0;
      float adjustedMultiplier = multiplier * speedFactor;
      
      float adjustedTiltDelta = smoothedTiltDelta * adjustedMultiplier;
      sendEosWheelMove(false, adjustedTiltDelta);
      displayNeedsUpdate = true;
      
      // Clear the sample that was just used
      tiltSamples[sampleIndex] = 0;
    }
  }
  
  // Increment sample index for next time
  sampleIndex = (sampleIndex + 1) % SMOOTH_SAMPLES;
  
  // Update display if needed
  if (displayNeedsUpdate) {
    updateDisplay();
    displayNeedsUpdate = false;
  }
  
  // Small delay to prevent CPU hogging
  delay(5);
}

void issueEosSubscribes() {
  // Add filter for parameter updates
  OSCMessage filter("/eos/filter/add");
  filter.add("/eos/out/param/*");
  Udp.beginPacket(destinationIP, destinationPort);
  filter.send(Udp);
  Udp.endPacket();
  filter.empty();
  
  // Subscribe to pan updates
  OSCMessage subPan("/eos/subscribe/param/pan");
  subPan.add(1); // 1 = subscribe
  Udp.beginPacket(destinationIP, destinationPort);
  subPan.send(Udp);
  Udp.endPacket();
  subPan.empty();
  
  // Subscribe to tilt updates
  OSCMessage subTilt("/eos/subscribe/param/tilt");
  subTilt.add(1); // 1 = subscribe
  Udp.beginPacket(destinationIP, destinationPort);
  subTilt.send(Udp);
  Udp.endPacket();
  subTilt.empty();
  
  Serial.println("Subscribed to EOS parameters");
}

void receiveOSC() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    OSCMessage msg;
    while (packetSize--) {
      msg.fill(Udp.read());
    }
    
    if (!msg.hasError()) {
      msg.dispatch("/eos/out/param/pan", handlePanValue);
      msg.dispatch("/eos/out/param/tilt", handleTiltValue);
    }
  }
}

void handlePanValue(OSCMessage &msg) {
  currentPanValue = msg.getFloat(0);
  displayNeedsUpdate = true;
}

void handleTiltValue(OSCMessage &msg) {
  currentTiltValue = msg.getFloat(0);
  displayNeedsUpdate = true;
}

void sendEosWheelMove(bool isPan, float ticks) {
  String wheelMsg = "/eos/wheel";
  
  // Add coarse/fine mode
  if (digitalRead(shiftButtonPin) == LOW)
    wheelMsg += "/fine";
  else
    wheelMsg += "/coarse";
    
  // Add parameter type
  wheelMsg += isPan ? "/pan" : "/tilt";

  // Create and send OSC message
  OSCMessage msg(wheelMsg.c_str());
  msg.add(ticks);
  
  Udp.beginPacket(destinationIP, destinationPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  
  // Debug output
  Serial.print("OSC message sent: ");
  Serial.print(wheelMsg);
  Serial.print(" ");
  Serial.println(ticks);
}

// Encoder interrupt handler for Pan
void panEncoderInterrupt() {
  // Get current state
  uint8_t currentState = (digitalRead(panEncoderClkPin) << 1) | digitalRead(panEncoderDtPin);
  
  // Gray code state transition for quadrature encoder
  // This pattern detects valid state transitions and filters out noise
  if ((prevPanState == 0 && currentState == 1) ||
      (prevPanState == 1 && currentState == 3) ||
      (prevPanState == 3 && currentState == 2) ||
      (prevPanState == 2 && currentState == 0)) {
    panDelta++;
  } else if ((prevPanState == 0 && currentState == 2) ||
             (prevPanState == 2 && currentState == 3) ||
             (prevPanState == 3 && currentState == 1) ||
             (prevPanState == 1 && currentState == 0)) {
    panDelta--;
  }
  
  // Update the state for next time
  prevPanState = currentState;
}

// Encoder interrupt handler for Tilt
void tiltEncoderInterrupt() {
  // Get current state
  uint8_t currentState = (digitalRead(tiltEncoderClkPin) << 1) | digitalRead(tiltEncoderDtPin);
  
  // Gray code state transition for quadrature encoder
  // This pattern detects valid state transitions and filters out noise
  if ((prevTiltState == 0 && currentState == 1) ||
      (prevTiltState == 1 && currentState == 3) ||
      (prevTiltState == 3 && currentState == 2) ||
      (prevTiltState == 2 && currentState == 0)) {
    tiltDelta++;
  } else if ((prevTiltState == 0 && currentState == 2) ||
             (prevTiltState == 2 && currentState == 3) ||
             (prevTiltState == 3 && currentState == 1) ||
             (prevTiltState == 1 && currentState == 0)) {
    tiltDelta--;
  }
  
  // Update the state for next time
  prevTiltState = currentState;
}

void updateDisplay() {
  lcd.clear();
  
  // First line: Pan value and mode
  lcd.setCursor(0, 0);
  lcd.print("Pan:");
  lcd.print(currentPanValue, 1);
  lcd.print(" ");
  lcd.print(digitalRead(shiftButtonPin) == LOW ? "FINE" : "COARSE");
  
  // Second line: Tilt value
  lcd.setCursor(0, 1);
  lcd.print("Tilt:");
  lcd.print(currentTiltValue, 1);
}

void setupWiFi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    lcd.clear();
    lcd.print("WiFi Failed!");
    while (true);
  }

  WiFi.config(staticIP);

  while (status != WL_CONNECTED) {
    Serial.print("Connecting to: ");
    Serial.println(ssid);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connecting to:");
    lcd.setCursor(0, 1);
    lcd.print(ssid);
    
    status = WiFi.begin(ssid, password);
    delay(2000);
  }

  IPAddress currentIP = WiFi.localIP();
  if (currentIP != staticIP) {
    Serial.println("Warning: Got different IP than requested");
    Serial.print("Requested: ");
    Serial.println(staticIP);
    Serial.print("Received: ");
    Serial.println(currentIP);
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connected!");
  lcd.setCursor(0, 1);
  lcd.print(currentIP);
  
  Serial.println("\nConnected!");
  Serial.print("Actual IP: "); Serial.println(currentIP);
  Serial.print("Actual Gateway: "); Serial.println(WiFi.gatewayIP());
  Serial.print("Actual Subnet: "); Serial.println(WiFi.subnetMask());
  
  delay(2500);
}