const int analogPin = 34; // The analog input pin to read from

void setup() {
  Serial.begin(115200); // Initialize serial communication at 9600 bps
}

void loop() {
  int analogValue = analogRead(analogPin); // Read analog value from the specified pin
  Serial.println(analogValue); // Print the analog value to the serial monitor

  delay(1000); // Add a small delay to avoid flooding the serial monitor
}
