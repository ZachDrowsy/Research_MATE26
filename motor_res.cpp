#include <HardwareSerial.h>  // Core Arduino class that exposes additional UART ports beyond Serial
#include <Arduino.h>         // Base Arduino API for pin I/O, timing, and serial primitives
#include <ArduinoJson.h>     // Lightweight JSON builder/parser for microcontrollers (by Benoit Blanchon)
#include <Wire.h>            // Arduino I2C/TWI bus support used to talk to sensors
#include "MS5837.h"         // Blue Robotics driver for the MS5837 pressure/depth sensor over I2C
#include <PID.h>             // Simple PID helper library to compute control corrections
#include <TMCStepper.h>      // Trinamic stepper-driver configuration library (here for TMC2209)

#define LED 2        // Onboard status LED pin for run/stop indication
MS5837 sensor;       // Sensor object representing the MS5837 pressure/depth module

// defines pins
// LoRa
String received;                    // Buffer to hold incoming LoRa UART payloads
HardwareSerial LoRaSerial(2);       // Instantiate UART2 as the LoRa interface (ESP32 supports multiple)
#define RX_PIN 16                   // ESP32 GPIO used for LoRa module RX
#define TX_PIN 17                   // ESP32 GPIO used for LoRa module TX
#define LORA_BAUD 115200            // Serial baud rate for the RYLR897 module
// Motor
int motor_cycles = 0;               // Counter placeholder for future cycle tracking
#define stepPin 18                  // Step pulse output to TMC2209
#define dirPin 5                    // Direction control pin to TMC2209
#define enPin 19                    // Enable pin to bring TMC2209 out of disable state
#define DRIVER_ADDRESS 0            // TMC2209 slave address when MS1/MS2 tied low
#define R_SENSE 0.11f               // Sense resistor value for current calibration

// Use hardware Serial1 (TX1/RX1) on boards that support it
TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);  // Driver object bound to hardware Serial1 UART


struct SensorData {  // Compact POD struct for timestamped pressure/depth samples
  int time;          // Seconds since boot when measurement was taken
  float pressure;    // Pressure reading from MS5837 (mbar)
  float depth;       // Depth computed using configured fluid density (meters)
};

SensorData current[200];  // Ring buffer for logged sensor samples
String jsonString[200];   // Parallel buffer holding serialized JSON payloads

float M3_error = 0;           // Placeholder for PID error (not updated elsewhere yet)
float M3_setpoint = 0;        // Placeholder for target depth (kept default 0)
float M3_previous = 0;        // Stores previous error input for PID call
float M3_corrective_val = 0;  // Output from PID computation
int curr_json = 0;            // Index of next free JSON slot in jsonString buffer

int first_run = 0;  // Tracks whether initial purge/startup has already executed
int motor = 0;      // Flag representing motor active state controlled by LoRa command

String writeSensorData(const SensorData& data) {
  // set capacity of JsonDocument
  const size_t capacity = JSON_OBJECT_SIZE(4);  // Helper macro computes object size for 4 keys
  // StaticJsonDocument is a JsonDocument that allocates its memory pool in-place
  // It doesn't rely on dynamic memory allocation (faster than DynamicJsonDocument)
  StaticJsonDocument<capacity> doc;             // Stack-allocated JSON container to avoid heap use

  //putting data into doc
  doc["c"] = "EX05";        // Tag identifying mission/sample block
  doc["t"] = data.time;      // Timestamp field
  doc["p"] = data.pressure;  // Pressure field
  doc["d"] = data.depth;     // Depth field

  // convert doc to string
  char jsonString[50];             // Fixed-size buffer sized for the small JSON
  serializeJson(doc, jsonString);  // Serialize JSON into char buffer
  return jsonString;
}

String writeData(String name, const float data) {
  // set capacity of JsonDocument
  const size_t capacity = JSON_OBJECT_SIZE(1);  // Single key/value object
  // StaticJsonDocument is a JsonDocument that allocates its memory pool in-place
  // It doesn't rely on dynamic memory allocation (faster than DynamicJsonDocument)
  StaticJsonDocument<capacity> doc;             // Minimal doc used for ad-hoc values

  //putting data into doc
  doc[name] = data;  // Insert the dynamic field name/value pair

  // convert doc to string
  char jsonString[20];             // Tight buffer because object is tiny
  serializeJson(doc, jsonString);  // Render JSON into buffer
  return jsonString;
}

void check_json() {
  if (curr_json >= 198) {
    digitalWrite(dirPin, HIGH);         // Reverse direction to clear water if buffer overflows
    step(14725 * 4.36, 1000);           // Extended purge move; multiplier compensates missed steps
    motor = 0;                          // Force stop to avoid overruns
  }
}

void setup() {
  // LORA
  Serial.begin(115200);                                     // USB serial for debugging
  LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);  // Configure UART2 with pins/baud/parity

  delay(1000);  // in case the lora module takes long to initialize
  Serial.println("\nRYLR897 Test");

  Serial.println("Try AT commands now");

  LoRaSerial.print("AT\r\n");  // Basic attention command to verify module presence
  delay(1000);

  LoRaSerial.print("AT+ADDRESS=116\r\n");  // Assign node address (unique per radio)
  delay(1000);                             //wait for module to respond

  LoRaSerial.print("AT+NETWORKID=10\r\n");  // Join radio network ID shared with peer
  delay(1000);                              //wait for module to respond

  LoRaSerial.print("AT+BAND=915000000\r\n");  // Select RF frequency band (915 MHz region)
  delay(1000);                                //wait for module to respond

  LoRaSerial.print("AT+PARAMETER=10,7,1,7\r\n");  // Modem settings: BW/SF/CR/Preamble
  delay(1000);                                    //wait for module to respond

  LoRaSerial.print("AT+PARAMETER?\r\n");  // Query current modem settings
  //Serial.print("AT+PARAMETER=10,7,1,7\r\n");    //For More than 3Kms
  delay(500);  //wait for module to respond

  LoRaSerial.print("AT+BAND?\r\n");  // Query band to confirm configuration
  delay(500);                        //wait for module to respond

  LoRaSerial.print("AT+NETWORKID?\r\n");  // Confirm network ID
  delay(500);                             //wait for module to respond

  LoRaSerial.print("AT+ADDRESS?\r\n");  // Confirm node address
  delay(500);                           //wait for module to respond

  pinMode(LED, OUTPUT);   // Prepare LED for status signaling
  digitalWrite(LED, LOW); // Start with LED off
  Wire.begin();           // Initialize I2C bus for the MS5837 sensor

  // MOTOR

  // Sets the two pins as Outputs
  pinMode(stepPin, OUTPUT);  // Configure step output pin
  pinMode(enPin, OUTPUT);    // Configure enable pin
  digitalWrite(enPin, HIGH); // Hold driver disabled until commanded
  pinMode(dirPin, OUTPUT);   // Configure direction pin
  pinMode(LED, OUTPUT);      // Redundant but ensures LED configured after setup
  //delay(300000);

  driver.begin();            // Initialize TMC2209 over UART
  driver.pdn_disable(true);  // disable PDN_UART pin function

  // Configure full-step mode:
  driver.microsteps(1);  // 1 step per full step
  driver.intpol(false);  // disable MicroPlyer interpolation

  driver.en_spreadCycle(true);  // enable SpreadCycle for better torque dynamics
  driver.rms_current(675);      // set run current to ~675 mA RMS (about 800 mA peak)
  driver.TCOOLTHRS(0);          // disable CoolStep
  driver.SGTHRS(0);             // disable StallGuard

  /*
  digitalWrite(dirPin, HIGH);
  for (int x = 0; x < 14725; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  } 
    digitalWrite(dirPin, LOW);
  for (int x = 0; x < 7363; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(300);
  }
  */

  while (!sensor.init()) {  // Retry until MS5837 responds on I2C
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar2: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_02BA);  // Specify 2-bar model so library scales correctly
  sensor.setFluidDensity(997);           // Density for fresh water to compute depth
}

void loop() {
  digitalWrite(enPin, HIGH);  // Keep motor disabled until a start command arrives
  // LORA
  // Forward Serial Monitor input to LoRa module sends the serial monitor input to the lora module
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // Capture full line from USB serial
    LoRaSerial.print(command + "\r\n");          // Relay host command to LoRa (AT passthrough)
  }

  // reads in LoRa module response to Serial Monitor
  while (LoRaSerial.available()) {
    received = LoRaSerial.readString();   // Pull any LoRa response
    Serial.print(received);               // Echo to USB for visibility
    if (received.startsWith("+RCV")) {   // RYLR897 prefixes inbound packets with +RCV
      if (received[11] == '1') {          // Inspect command byte at payload start
        Serial.println("begin");
        motor = 1;
        while (motor == 1) {
          received = LoRaSerial.readString();  // Keep consuming live commands
          Serial.print(received);
          digitalWrite(enPin, LOW);            // Enable TMC2209 to drive motor
          digitalWrite(LED, HIGH);             // Visual indicator motor is active
          digitalWrite(dirPin, HIGH);          // Intake water
          first_run++;
          if (received[11] == '0') {           // Stop command from remote
            digitalWrite(LED, LOW);
            digitalWrite(enPin, HIGH);
            break;
          }


          // Read from Sensor
          // sensor.read();

          /*
          SensorData begin;
          float Depth = sensor.depth(); 
          Serial.print(Depth);
          */


          /*
          begin.time = millis() / 1000;
          begin.pressure = sensor.pressure();
          begin.depth = sensor.depth();
          writeSensorData(begin);
          */

          sensor.read();                                      // Trigger I2C measurement update
          M3_previous = M3_error;                             // Save error history for PID call
          jsonString[curr_json] = writeData("Curr_depth", sensor.depth());
          curr_json++;
          check_json();
          M3_corrective_val = pid(sensor.depth(), M3_previous);  // Compute PID output based on depth
          jsonString[curr_json] = writeData("PID_Val", M3_corrective_val);
          curr_json++;
          check_json();

          if (first_run == 1) {                         // Only once per boot
            LoRaSerial.print("AT+SEND=115,15,{CONNECTED}\r\n");  // Announce to base
            // Push everything out
            digitalWrite(dirPin, HIGH);
            step(14725, 1000);                          // Long purge stroke to empty ballast
            // digitalWrite(dirPin, LOW); // Neutrally Bouyant
            // step(3681.25, 1500);
          }
          first_run++;




          // Begin Run
          digitalWrite(dirPin, LOW);
          // Currently intakes water to start, sinks
          step(7362.5, 1000);  // 200 pulses at 800 µs total period

          sensor.read();

          M3_previous = M3_error;                             // Refresh PID input snapshot
          jsonString[curr_json] = writeData("Curr_depth", sensor.depth());
          curr_json++;
          check_json();
          M3_corrective_val = pid(sensor.depth(), M3_previous);  // Another PID sample post-dive
          jsonString[curr_json] = writeData("PID_Val", M3_corrective_val);
          curr_json++;
          check_json();

          delay(5000);
          //Serial.println("LED is on");
          //delay(10000);  // One second delay

          // Read from Sensor

          for (int i = 1; i < 11; i++) {        // Collect 10 samples over 10 seconds
            sensor.read();                      // Update sensor registers
            current[i].time = millis() / 1000;  // Log current time in seconds
            current[i].pressure = sensor.pressure();
            current[i].depth = sensor.depth();

            jsonString[i + curr_json] = writeSensorData(current[i]);  // Serialize and store reading
            delay(1000);
            Serial.println(jsonString[i + curr_json]);               // Print JSON for debugging
          }
          curr_json += 10;
          check_json();


          delay(1000);

          // Begin going back up
          digitalWrite(dirPin, HIGH);  //Changes the rotations direction
                                       // Makes 400 pulses for making two full cycle rotation
          step(7362.5, 1000);   // 200 pulses at 800 µs total period

          sensor.read();
          M3_previous = M3_error;                             // Update PID history before ascent log
          jsonString[curr_json] = writeData("Curr_depth", sensor.depth());
          curr_json++;
          check_json();
          M3_corrective_val = pid(sensor.depth(), M3_previous);  // PID after ascent move
          jsonString[curr_json] = writeData("PID_Val", M3_corrective_val);
          curr_json++;
          check_json();

          delay(15000);

          // Send JSON
          if (received[11] == 2) {          // Transmit stored data when instructed
            Serial.println("sending");
            for (int i = 0; i < curr_json; i++) {
              LoRaSerial.print("AT+SEND=115,");
              LoRaSerial.print(50);
              LoRaSerial.print(",");
              LoRaSerial.println(jsonString[i]);
              delay(1000);
            }
            digitalWrite(LED, LOW);
            digitalWrite(enPin, HIGH);
            motor = 0;
          }



          //Serial.println("LED is off");
          curr_json = 0;  // Reset buffer index for next mission
          if (received[11] == '0') {  //in this case our single received byte would always be at the 11th position
            digitalWrite(LED, LOW);
            digitalWrite(enPin, HIGH);
            motor = 0;
          }
        }
      }
    }
  }
}
// Helper: toggle STEP pin N times at the given pulse_period (µs)
void step(uint16_t steps, uint16_t pulse_period) {
  for (uint16_t i = 0; i < steps; ++i) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulse_period / 2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulse_period / 2);
  }
}