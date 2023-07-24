
const int analogPin = 34; // The analog input pin to read from

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
int analogValue = analogRead(analogPin); // Read analog value from the specified pin
  Serial.println(analogValue); // Print the analog value to the serial monitor

  delay(100); 
}
