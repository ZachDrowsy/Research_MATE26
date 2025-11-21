#include <HardwareSerial.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "MS5837.h"
#include <PID.h>
#include <TMCStepper.h>

#define LED 2
MS5837 sensor;

// defines pins
// LoRa
String received;
HardwareSerial LoRaSerial(2);  // accessing Uart
#define RX_PIN 16
#define TX_PIN 17
#define LORA_BAUD 115200
// Motor
int motor_cycles = 0;
#define stepPin 18
#define dirPin 5
#define enPin 19
#define DRIVER_ADDRESS 0  // MS1/MS2 = GND/GND → address 0
#define R_SENSE 0.11f     // Rsense value in ohms on your board

// Use hardware Serial1 (TX1/RX1) on boards that support it
TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);


struct SensorData {
  int time;
  float pressure;
  float depth;
};

SensorData current[200];
String jsonString[200];

float M3_error = 0;
float M3_setpoint = 0;
float M3_previous = 0;
float M3_corrective_val = 0;
int curr_json = 0;

int first_run = 0;
int motor = 0;

String writeSensorData(const SensorData& data) {
  // set capacity of JsonDocument
  const size_t capacity = JSON_OBJECT_SIZE(4);
  // StaticJsonDocument is a JsonDocument that allocates its memory pool in-place
  // It doesn't rely on dynamic memory allocation (faster than DynamicJsonDocument)
  StaticJsonDocument<capacity> doc;

  //putting data into doc
  doc["c"] = "EX05";
  doc["t"] = data.time;
  doc["p"] = data.pressure;
  doc["d"] = data.depth;

  // convert doc to string
  char jsonString[50];
  serializeJson(doc, jsonString);
  return jsonString;
}

String writeData(String name, const float data) {
  // set capacity of JsonDocument
  const size_t capacity = JSON_OBJECT_SIZE(1);
  // StaticJsonDocument is a JsonDocument that allocates its memory pool in-place
  // It doesn't rely on dynamic memory allocation (faster than DynamicJsonDocument)
  StaticJsonDocument<capacity> doc;

  //putting data into doc
  doc[name] = data;

  // convert doc to string
  char jsonString[20];
  serializeJson(doc, jsonString);
  return jsonString;
}

void check_json() {
  if (curr_json >= 198) {
    digitalWrite(dirPin, HIGH);
    step(14725 * 4.36, 1000);
    motor = 0;
  }
}

void setup() {
  // LORA
  Serial.begin(115200);                                     // beginning baud rate
  LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);  // LoRa module UART initialization

  delay(1000);  // in case the lora module takes long to initialize
  Serial.println("\nRYLR897 Test");

  Serial.println("Try AT commands now");

  LoRaSerial.print("AT\r\n");
  delay(1000);

  LoRaSerial.print("AT+ADDRESS=116\r\n");  //needs to be unique
  delay(1000);                             //wait for module to respond

  LoRaSerial.print("AT+NETWORKID=10\r\n");  //needs to be same for receiver and transmitter
  delay(1000);                              //wait for module to respond

  LoRaSerial.print("AT+BAND=915000000\r\n");  //Bandwidth set to 868.5MHz
  delay(1000);                                //wait for module to respond

  LoRaSerial.print("AT+PARAMETER=10,7,1,7\r\n");  //For Less than 3Kms
  delay(1000);                                    //wait for module to respond

  LoRaSerial.print("AT+PARAMETER?\r\n");  //For Less than 3Kms
  //Serial.print("AT+PARAMETER=10,7,1,7\r\n");    //For More than 3Kms
  delay(500);  //wait for module to respond

  LoRaSerial.print("AT+BAND?\r\n");  //Bandwidth set to 868.5MHz
  delay(500);                        //wait for module to respond

  LoRaSerial.print("AT+NETWORKID?\r\n");  //needs to be same for receiver and transmitter
  delay(500);                             //wait for module to respond

  LoRaSerial.print("AT+ADDRESS?\r\n");  //needs to be unique
  delay(500);                           //wait for module to respond

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Wire.begin();

  // MOTOR

  // Sets the two pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH);
  pinMode(dirPin, OUTPUT);
  pinMode(LED, OUTPUT);
  //delay(300000);

  driver.begin();
  driver.pdn_disable(true);  // disable PDN_UART pin function

  // Configure full-step mode:
  driver.microsteps(1);  // 1 step per full step
  driver.intpol(false);  // disable MicroPlyer interpolation

  driver.en_spreadCycle(true);  // enable SpreadCycle for better torque dynamics
  driver.rms_current(675);      // set run current to 800 mA
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

  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar2: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997);
}

void loop() {
  digitalWrite(enPin, HIGH);
  // LORA
  // Forward Serial Monitor input to LoRa module sends the serial monitor input to the lora module
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    LoRaSerial.print(command + "\r\n");  // Send to LoRa module with CRLF
  }

  // reads in LoRa module response to Serial Monitor
  while (LoRaSerial.available()) {
    received = LoRaSerial.readString();
    Serial.print(received);
    if (received.startsWith("+RCV")) {
      if (received[11] == '1') {  //in this case our single received byte would always be at the 11th position
        Serial.println("begin");
        motor = 1;
        while (motor == 1) {
          received = LoRaSerial.readString();
          Serial.print(received);
          digitalWrite(enPin, LOW);
          digitalWrite(LED, HIGH);
          digitalWrite(dirPin, HIGH);  // Intake water
          first_run++;
          if (received[11] == '0') {
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

          sensor.read();
          M3_previous = M3_error;
          jsonString[curr_json] = writeData("Curr_depth", sensor.depth());
          curr_json++;
          check_json();
          M3_corrective_val = pid(sensor.depth(), M3_previous);
          jsonString[curr_json] = writeData("PID_Val", M3_corrective_val);
          curr_json++;
          check_json();

          if (first_run == 1) {
            LoRaSerial.print("AT+SEND=115,15,{CONNECTED}\r\n");
            // Push everything out
            digitalWrite(dirPin, HIGH);
            step(14725, 1000);
            // digitalWrite(dirPin, LOW); // Neutrally Bouyant
            // step(3681.25, 1500);
          }
          first_run++;




          // Begin Run
          digitalWrite(dirPin, LOW);
          // Currently intakes water to start, sinks
          step(7362.5, 1000);  // 200 pulses at 800 µs total period

          sensor.read();

          M3_previous = M3_error;
          jsonString[curr_json] = writeData("Curr_depth", sensor.depth());
          curr_json++;
          check_json();
          M3_corrective_val = pid(sensor.depth(), M3_previous);
          jsonString[curr_json] = writeData("PID_Val", M3_corrective_val);
          curr_json++;
          check_json();

          delay(5000);
          //Serial.println("LED is on");
          //delay(10000);  // One second delay

          // Read from Sensor

          for (int i = 1; i < 11; i++) {
            sensor.read();
            current[i].time = millis() / 1000;
            current[i].pressure = sensor.pressure();
            current[i].depth = sensor.depth();

            jsonString[i + curr_json] = writeSensorData(current[i]);
            delay(1000);
            Serial.println(jsonString[i + curr_json]);
          }
          curr_json += 10;
          check_json();


          delay(1000);

          // Begin going back up
          digitalWrite(dirPin, HIGH);  //Changes the rotations direction
                                       // Makes 400 pulses for making two full cycle rotation
          step(7362.5, 1000);   // 200 pulses at 800 µs total period

          sensor.read();
          M3_previous = M3_error;
          jsonString[curr_json] = writeData("Curr_depth", sensor.depth());
          curr_json++;
          check_json();
          M3_corrective_val = pid(sensor.depth(), M3_previous);
          jsonString[curr_json] = writeData("PID_Val", M3_corrective_val);
          curr_json++;
          check_json();

          delay(15000);

          // Send JSON
          if (received[11] == 2) {
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
          curr_json = 0;
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