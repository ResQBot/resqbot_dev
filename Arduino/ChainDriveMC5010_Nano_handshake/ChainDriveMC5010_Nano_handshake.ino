//refresh management
int refresh_rate = 20;   //[Hz]
int DELAY = (1000 / refresh_rate);
int dropped_rx = 0;

//motor pins
//left motor
const byte pinL1 = 16;
const byte pinL2 = 15;
const byte pinL3 = 14;
//right motor
const byte pinR1 = 9;
const byte pinR2 = 8;
const byte pinR3 = 7;
//variables for motor controll
int motorStateLeft  = 0;
int motorStateRight = 0;

//variables for movement definition
int Velocity = 0;
int Angle = 0;

//degrees for Joystick positions
int A = 15;
int B = 45;
int C = 75;
int D = 105;
int E = 135;
int F = 165;

void setup() {
  //pins for left chain drive
  pinMode(pinL1, OUTPUT);
  pinMode(pinL2, OUTPUT);
  pinMode(pinL3, OUTPUT);

  //pins for right chain drive
  pinMode(pinR1, OUTPUT);
  pinMode(pinR2, OUTPUT);
  pinMode(pinR3, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0){
    dropped_rx = 0;
  }

  if (dropped_rx < 4){
    //receive message via serial connection
    String rx_msg = Serial.readStringUntil('\n');
    if (rx_msg == "Who are you?"){
      handshake();
    }
    else{
      sscanf(rx_msg.c_str(), "V%iA%i\n", &Velocity, &Angle);

      //calculate movement state
      if(Velocity == 0) {
        motorStateRight = 0;
        motorStateLeft  = 0;
      }
      else if(Velocity < 30){
        //left semi sircle
        if (Angle > ((-1)*A) && Angle < A){
          motorStateRight = 2;
          motorStateLeft  = 2;
        }
        else if(Angle >= A && Angle < B){
          motorStateRight = 4;
          motorStateLeft  = 2;
        }
        else if(Angle >= B && Angle < C){
          motorStateRight = 4;
          motorStateLeft  = 0;
        }
        else if(Angle >= C && Angle < D){
          motorStateRight = 2;
          motorStateLeft  = 3;
        }
        else if(Angle >= D && Angle < E){
          motorStateRight = 3;
          motorStateLeft  = 0;
        }
        else if(Angle >= E && Angle < F){
          motorStateRight = 5;
          motorStateLeft  = 3;
        }
        else if(Angle >= F && Angle <= 180){
          motorStateRight = 3;
          motorStateLeft  = 3;
        }
        //right semi circle
        else if(Angle <= ((-1)*A) && Angle > ((-1)*B)){
          motorStateRight = 2;
          motorStateLeft  = 4;
        }
        else if(Angle <= ((-1)*B) && Angle > ((-1)*C)){
          motorStateRight = 0;
          motorStateLeft  = 4;
        }
        else if(Angle <= ((-1)*C) && Angle > ((-1)*D)){
          motorStateRight = 3;
          motorStateLeft  = 2;
        }
        else if(Angle <= ((-1)*D) && Angle > ((-1)*E)){
          motorStateRight = 0;
          motorStateLeft  = 5;
        }
        else if(Angle <= ((-1)*E) && Angle > ((-1)*F)){
          motorStateRight = 3;
          motorStateLeft  = 5;
        }
        else if(Angle <= ((-1)*F) && Angle >= -180){
          motorStateRight = 3;
          motorStateLeft  = 3;
        }
      }
      else if (Velocity < 70){
        //left semi sircle
        if (Angle > ((-1)*A) && Angle < A){
          motorStateRight = 4;
          motorStateLeft  = 4;
        }
        else if(Angle >= A && Angle < B){
          motorStateRight = 4;
          motorStateLeft  = 2;
        }
        else if(Angle >= B && Angle < C){
          motorStateRight = 4;
          motorStateLeft  = 0;
        }
        else if(Angle >= C && Angle < D){
          motorStateRight = 4;
          motorStateLeft  = 5;
        }
        else if(Angle >= D && Angle < E){
          motorStateRight = 3;
          motorStateLeft  = 0;
        }
        else if(Angle >= E && Angle < F){
          motorStateRight = 5;
          motorStateLeft  = 3;
        }
        else if(Angle >= F && Angle <= 180){
          motorStateRight = 5;
          motorStateLeft  = 5;
        }
        //right semi circle
        else if(Angle <= ((-1)*A) && Angle > ((-1)*B)){
          motorStateRight = 2;
          motorStateLeft  = 4;
        }
        else if(Angle <= ((-1)*B) && Angle > ((-1)*C)){
          motorStateRight = 0;
          motorStateLeft  = 4;
        }
        else if(Angle <= ((-1)*C) && Angle > ((-1)*D)){
          motorStateRight = 5;
          motorStateLeft  = 4;
        }
        else if(Angle <= ((-1)*D) && Angle > ((-1)*E)){
          motorStateRight = 0;
          motorStateLeft  = 5;
        }
        else if(Angle <= ((-1)*E) && Angle > ((-1)*F)){
          motorStateRight = 3;
          motorStateLeft  = 5;
        }
        else if(Angle <= ((-1)*F) && Angle >= -180){
          motorStateRight = 5;
          motorStateLeft  = 5;
        }
      }
      else if (Velocity < 120){
        //left semi sircle
        if (Angle > ((-1)*A) && Angle < A){
          motorStateRight = 6;
          motorStateLeft  = 6;
        }
        else if(Angle >= A && Angle < B){
          motorStateRight = 6;
          motorStateLeft  = 4;
        }
        else if(Angle >= B && Angle < C){
          motorStateRight = 6;
          motorStateLeft  = 2;
        }
        else if(Angle >= C && Angle < D){
          motorStateRight = 6;
          motorStateLeft  = 7;
        }
        else if(Angle >= D && Angle < E){
          motorStateRight = 5;
          motorStateLeft  = 3;
        }
        else if(Angle >= E && Angle < F){
          motorStateRight = 7;
          motorStateLeft  = 5;
        }
        else if(Angle >= F && Angle <= 180){
          motorStateRight = 7;
          motorStateLeft  = 7;
        }
        //right semi circle
        else if(Angle <= ((-1)*A) && Angle > ((-1)*B)){
          motorStateRight = 4;
          motorStateLeft  = 6;
        }
        else if(Angle <= ((-1)*B) && Angle > ((-1)*C)){
          motorStateRight = 2;
          motorStateLeft  = 6;
        }
        else if(Angle <= ((-1)*C) && Angle > ((-1)*D)){
          motorStateRight = 7;
          motorStateLeft  = 6;
        }
        else if(Angle <= ((-1)*D) && Angle > ((-1)*E)){
          motorStateRight = 3;
          motorStateLeft  = 7;
        }
        else if(Angle <= ((-1)*E) && Angle > ((-1)*F)){
          motorStateRight = 5;
          motorStateLeft  = 7;
        }
        else if(Angle <= ((-1)*F) && Angle >= -180){
          motorStateRight = 7;
          motorStateLeft  = 7;
        }
      }
      else {                    //ERROR if velocity >120
        motorStateRight = 0;
        motorStateLeft = 0;
      }

    setRightMode(motorStateRight);
    setLeftMode(motorStateLeft);
    //Serial.print(motorStateRight);
    //Serial.print(motorStateLeft);
    delay(DELAY);
    }
  }
}

//setting the pins to tell the MC what to do
//left Motor then right Motor
void setLeftMode(int L){
  switch (L){              
    case 0:                  //stop
      digitalWrite(pinL1, LOW);
      digitalWrite(pinL2, LOW);
      digitalWrite(pinL3, LOW);
      break;
     
    case 2:                  //forward slow
      digitalWrite(pinL1, LOW);
      digitalWrite(pinL2, HIGH);
      digitalWrite(pinL3, LOW);
      break;
    
    case 3:                  //backwards slow
      digitalWrite(pinL1, HIGH);
      digitalWrite(pinL2, HIGH);
      digitalWrite(pinL3, LOW);
      break;
    
    case 4:                  //forward medium
      digitalWrite(pinL1, LOW);
      digitalWrite(pinL2, LOW);
      digitalWrite(pinL3, HIGH);
      break;
    
    case 5:                  //backwards medium
      digitalWrite(pinL1, HIGH);
      digitalWrite(pinL2, LOW);
      digitalWrite(pinL3, HIGH);
      break;
    
    case 6:                  //forward fast
      digitalWrite(pinL1, LOW);
      digitalWrite(pinL2, HIGH);
      digitalWrite(pinL3, HIGH);
      break;
    
    case 7:                  //backwards fast
      digitalWrite(pinL1, HIGH);
      digitalWrite(pinL2, HIGH);
      digitalWrite(pinL3, HIGH);
      break;
      
    default:                 //ERROR -> stop
    digitalWrite(pinL1, LOW);
    digitalWrite(pinL2, LOW);
    digitalWrite(pinL3, LOW);
  } 
}

void setRightMode(int R){
  switch (R){
    case 0:                   //stop
      digitalWrite(pinR1, LOW);
      digitalWrite(pinR2, LOW);
      digitalWrite(pinR3, LOW);
      break;
      
    case 2:                   //forward slow
      digitalWrite(pinR1, LOW);
      digitalWrite(pinR2, HIGH);
      digitalWrite(pinR3, LOW);
      break;
      
    case 3:                   //backwards slow
      digitalWrite(pinR1, HIGH);
      digitalWrite(pinR2, HIGH);
      digitalWrite(pinR3, LOW);
      break;
      
    case 4:                   //forward medium
      digitalWrite(pinR1, LOW);
      digitalWrite(pinR2, LOW);
      digitalWrite(pinR3, HIGH);
      break;
    
    case 5:                   //backwards medium
      digitalWrite(pinR1, HIGH);
      digitalWrite(pinR2, LOW);
      digitalWrite(pinR3, HIGH);
      break;
    
    case 6:                   //forward fast
      digitalWrite(pinR1, LOW);
      digitalWrite(pinR2, HIGH);
      digitalWrite(pinR3, HIGH);
      break;
    
    case 7:                   //backwards fast
      digitalWrite(pinR1, HIGH);
      digitalWrite(pinR2, HIGH);
      digitalWrite(pinR3, HIGH);
      break;
    
    default:                    //ERROR -> stop
      digitalWrite(pinR1, LOW);
      digitalWrite(pinR2, LOW);
      digitalWrite(pinR3, LOW);
  } 
}

void handshake(){ 
  //set marker to show handshake is ongoing
  bool confirmation = 0;
  //answer handshake
  Serial.println("driveInterface");
  //wait for confirmation#
  int start = millis();
  while(!confirmation){
    //abort if handshake takes too long
    if (millis() - start > 1500){
      return;
    }
    //wait for answer, then read
    //delay(50);
    String rx_msg = Serial.readStringUntil('\n');

    //check for correct answer and confirm handshake
    if(rx_msg == "Hello driveInterface"){
      Serial.println("confirmed");
      confirmation = 1;
      return;
    }
    //if answer is wrong, retry
    else {
      confirmation = 0;
    }
  }
}
