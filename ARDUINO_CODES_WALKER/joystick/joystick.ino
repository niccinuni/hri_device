/*
 *  CODICE SEMPLIFICATO PER TESTARE I PULSANTI DI UN JOYSTICK
 *  
 *  Obiettivo: Verificare che i pulsanti siano collegati correttamente.
 *  
 *  Funzionamento:
 *  - Legge lo stato dei pin a cui sono collegati i pulsanti.
 *  - Stampa un messaggio sul Monitor Seriale solo quando un pulsante viene premuto.
 *  - Non sono necessarie librerie esterne.
 */

// --- Configurazione dei Pin ---
// Assicurati che questi pin corrispondano a come hai collegato fisicamente i pulsanti.
const int deadmanSwitchPin = 13;
const int motorForwardPin  = 25;
const int moveLeftPin      = 12;
const int moveRightPin     = 14;

bool deadmanSwitchState = false;
bool leftSwitchState=false;
bool rightSwitchState=false;
bool forwardSwitchState=false;
bool resetSwitchState=false;

void setup() {
  // Inizializza la comunicazione Seriale per poter vedere i messaggi sul computer.
  // Assicurati che il Monitor Seriale sia impostato sulla stessa velocità (115200).
  Serial.begin(115200);
  
  // Attendi un istante per stabilizzare la Seriale.
  delay(1000); 
  
  Serial.println("--- Tester Pulsanti Joystick ---");
  Serial.println("Pronto a rilevare le pressioni.");
  Serial.println("Apri il Monitor Seriale per vedere l'output.");

  // Configura ogni pin del pulsante come INPUT con una resistenza di PULLUP interna.
  // Questo significa che non hai bisogno di resistenze esterne.
  // Il pin sarà HIGH (alto) quando il pulsante non è premuto.
  // Il pin sarà LOW (basso) quando il pulsante VIENE PREMUTO.
  pinMode(deadmanSwitchPin, INPUT_PULLUP);
  pinMode(motorForwardPin, INPUT_PULLUP);
  pinMode(moveLeftPin, INPUT_PULLUP);
  pinMode(moveRightPin, INPUT_PULLUP);
}

void loop() {
  // Legge lo stato di ogni pulsante.
  // La variabile sarà 'true' se il pulsante è premuto (pin a livello LOW).
  bool deadmanPressed = (digitalRead(deadmanSwitchPin) == LOW);
  bool forwardPressed = (digitalRead(motorForwardPin) == LOW);
  bool leftPressed    = (digitalRead(moveLeftPin) == LOW);
  bool rightPressed   = (digitalRead(moveRightPin) == LOW);

  // Controlla se ciascun pulsante è stato premuto e stampa un messaggio.
  // Usiamo 'if' separati per poter rilevare anche più pressioni contemporaneamente.
  if (deadmanPressed) {
    deadmanSwitchState = true;
    Serial.println("Pulsante PREMUTO: Deadman Switch (Pin 13)");
    }


  if (forwardPressed) {
    Serial.println("Pulsante PREMUTO: Avanti (Pin 25)");
    forwardSwitchState=true;
  }

  if (leftPressed && !rightPressed) {
    Serial.println("Pulsante PREMUTO: Sinistra (Pin 12)");
    leftSwitchState=true;
  }

  if (rightPressed && !leftPressed) {
    Serial.println("Pulsante PREMUTO: Destra (Pin 14)");
    rightSwitchState=true;
  }

  if (rightPressed && leftPressed){
    Serial.println("reset");
    resetSwitchState=true;
  }

  // Aggiungiamo un piccolo ritardo per evitare di inondare il Monitor Seriale 
  // di messaggi se un pulsante viene tenuto premuto.
  delay(150); 
}