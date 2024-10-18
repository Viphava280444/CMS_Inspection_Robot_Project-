#include <Servo.h>

const int magneticSensorPin = 8; // Digital pin for the DFR0033 magnetic sensor


void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Set the pin modes for the sensors
  pinMode(A1, INPUT); // LM35 temperature sensor
  pinMode(A2, INPUT); // SEN0134 gas sensor
  pinMode(A3, INPUT); // DFR0034 sound sensor
  pinMode(magneticSensorPin, INPUT); // DFR0033 magnetic sensor
  

}

void loop() {
  // Read the temperature sensor value
  float tempSensorValue = analogRead(A1);
  float tempVoltage = tempSensorValue * (5.0 / 1024.0);
  float temperature = tempVoltage * 100.0;

  // Read the gas sensor value
  float gasSensorValue = analogRead(A2);
  float conversionFactor = 10.0; // Hypothetical calibration factor
  float gasConcentration = gasSensorValue * conversionFactor / 1024.0;

  // Read the sound sensor value
  float soundSensorValue = analogRead(A3);
  float soundDb = map(soundSensorValue, 0, 1023, 30, 120); // Example mapping

  // Read the magnetic sensor value
  int magneticSensorValue = digitalRead(magneticSensorPin);
  int magneticField = (magneticSensorValue == HIGH) ? 1 : 0; // Use 1 for Detected and 0 for Not Detected

  // Create JSON formatted string
  String jsonData = "{";
  jsonData += "\"temperature\":" + String(temperature) + ",";
  jsonData += "\"gas\":" + String(gasConcentration) + ",";
  jsonData += "\"sound\":" + String(soundDb) + ",";
  jsonData += "\"magnetic\":" + String(magneticField);
  jsonData += "}";

  // Print JSON data to serial
  Serial.println(jsonData);


  delay(10);
 
}
