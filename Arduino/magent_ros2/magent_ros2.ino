#define HALL_PIN 4

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT);
}

void loop() {
  int raw = analogRead(HALL_PIN);     // returns 0..1023
  float voltage = raw * (3.3 / 4095.0);  // convert to voltage
    Serial.print("raw: ");
    Serial.print(raw);
    Serial.print("   voltage: ");
    Serial.print(voltage);
  if (raw > 2050){
    Serial.print("   magnet detected  ");    
    Serial.print("   Range: ");
    if(raw>2300){

      Serial.println("CLOSE");
    }
    else if(raw>2100){

      Serial.println("MID");
    }
    else {
      Serial.println("FAR");
    }
  }
  else{
    Serial.println("no magnet detected");
  }


  delay(500);
}
