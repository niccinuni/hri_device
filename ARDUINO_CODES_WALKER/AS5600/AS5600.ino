#include "AS5600.h"

// --- Selezione del Sensore ---
AS5600 as5600;

// --- CALIBRAZIONE ---
// NUOVO: Definiamo il valore grezzo che corrisponde alla nostra posizione di zero.
// Modifica questo valore in base alla tua calibrazione.
const int ZERO_OFFSET = 641;

// Variabili per memorizzare i valori letti
double rawAngle = 0.0;
double correctedRawAngle = 0.0;
double angleInDegrees = 0.0;

void setup()
{
  Serial.begin(115200);
  while(!Serial)
  {
    ; // attesa
  }
  Serial.println("Avvio dell'esempio AS5600 con calibrazione dello zero...");

  Wire.begin(21, 22); // Pin SDA e SCL per l'ESP32 

  as5600.begin();
  as5600.setDirection(AS5600_CLOCK_WISE);

  if (as5600.isConnected())
  {
    Serial.print("Sensore AS5600 connesso. Offset di zero impostato a: ");
    Serial.println(ZERO_OFFSET);
  }
  else
  {
    Serial.println("Errore: sensore AS5600 non trovato. Controlla i collegamenti.");
    while(1);
  }

  delay(1000);
}
void loop()
{
  // 1. Legge il valore grezzo dell'angolo (0-4095)
  rawAngle = as5600.readAngle();

  // 2. NUOVA FORMULA: Applica l'offset per correggere il punto zero
  // Sottrae l'offset e gestisce il "wrap-around" (il passaggio da 359° a 0°)
  correctedRawAngle = rawAngle - ZERO_OFFSET;
  if (correctedRawAngle < 0)
  {
    correctedRawAngle += 4096; // Se il risultato è negativo, aggiungi il giro completo
  }

  // 3. FORMULA DI CONVERSIONE: Converte il valore grezzo CORRETTO in gradi (0-360)
  angleInDegrees = (correctedRawAngle / 4096.0) * 360.0;

  // Stampa i risultati sul monitor seriale
  Serial.print("Grezzo: ");
  Serial.print(rawAngle);
  
  Serial.print("\t Gradi Corretti: ");
  Serial.print(angleInDegrees, 2); // Stampa con 2 cifre decimali
  Serial.print("°");

  Serial.print("\t Velocità (ω): ");
  Serial.print(as5600.getAngularSpeed(), 2);
  Serial.println(" °/s");
  
  delay(100);
}

