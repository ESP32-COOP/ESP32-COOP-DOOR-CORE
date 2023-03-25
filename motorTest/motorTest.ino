#define moteurA_1 27
#define moteurA_2 26
int vitesse = 255;  // 0 à 255
void setup() {
    // Configuration des ports en mode "sortie"
    pinMode(moteurA_1, OUTPUT);
    pinMode(moteurA_2, OUTPUT);
}
void loop() {
  digitalWrite(moteurA_1, LOW);
  digitalWrite(moteurA_2, HIGH);
  delay(5000);
  
  digitalWrite(moteurA_1, HIGH);
  digitalWrite(moteurA_2, LOW);
 
  delay(500);
  digitalWrite(moteurA_1, LOW);
  digitalWrite(moteurA_2, LOW);
  
  delay(200);
}
