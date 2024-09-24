//refresh management
int refresh_rate = 20;   //[Hz]
int DELAY = (1000 / refresh_rate);

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


//variables for motor controll
int motorStateLeft  = 0;
int motorStateRight = 0;

void setup() {

//pins for left chain drive
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);

//pins for right chain drive
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);

  Serial.begin(115200);
}

void loop() {
//check if Serial connection is good. else stop motors.
  if (Serial) {
//receive message via serial connection
    String rx_msg = Serial.readStringUntil('\n');
    sscanf(rx_msg.c_str(), "V%iA%i\n", &Velocity, &Angle);
  }
  else {
    Velocity  = 0;
    Angle     = 0;
  }
  
//--------------------------------------------------------------------------
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

  Serial.print(motorStateRight);
  Serial.print(motorStateLeft);
  
  delay(DELAY);

}

//setting the pins to tell the MC what to do
//left Motor then right Motor
void setLeftMode(int L){
  switch (L){              
    case 0:                  //stop
      digitalWrite(13, LOW);
      digitalWrite(12, LOW);
      digitalWrite(11, LOW);
      break;
     
    case 2:                  //forward slow
      digitalWrite(13, LOW);
      digitalWrite(12, HIGH);
      digitalWrite(11, LOW);
      break;
    
    case 3:                  //backwards slow
      digitalWrite(13, HIGH);
      digitalWrite(12, HIGH);
      digitalWrite(11, LOW);
      break;
    
    case 4:                  //forward medium
      digitalWrite(13, LOW);
      digitalWrite(12, LOW);
      digitalWrite(11, HIGH);
      break;
    
    case 5:                  //backwards medium
      digitalWrite(13, HIGH);
      digitalWrite(12, LOW);
      digitalWrite(11, HIGH);
      break;
    
    case 6:                  //forward fast
      digitalWrite(13, LOW);
      digitalWrite(12, HIGH);
      digitalWrite(11, HIGH);
      break;
    
    case 7:                  //backwards fast
      digitalWrite(13, HIGH);
      digitalWrite(12, HIGH);
      digitalWrite(11, HIGH);
      break;
      
    default:                 //ERROR -> stop
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
  } 
}

void setRightMode(int R){
  switch (R){
    case 0:                   //stop
      digitalWrite(4, LOW);
      digitalWrite(3, LOW);
      digitalWrite(2, LOW);
      break;
      
    case 2:                   //forward slow
      digitalWrite(4, LOW);
      digitalWrite(3, HIGH);
      digitalWrite(2, LOW);
      break;
      
    case 3:                   //backwards slow
      digitalWrite(4, HIGH);
      digitalWrite(3, HIGH);
      digitalWrite(2, LOW);
      break;
      
    case 4:                   //forward medium
      digitalWrite(4, LOW);
      digitalWrite(3, LOW);
      digitalWrite(2, HIGH);
      break;
    
    case 5:                   //backwards medium
      digitalWrite(4, HIGH);
      digitalWrite(3, LOW);
      digitalWrite(2, HIGH);
      break;
    
    case 6:                   //forward fast
      digitalWrite(4, LOW);
      digitalWrite(3, HIGH);
      digitalWrite(2, HIGH);
      break;
    
    case 7:                   //backwards fast
      digitalWrite(4, HIGH);
      digitalWrite(3, HIGH);
      digitalWrite(2, HIGH);
      break;
    
    default:                    //ERROR -> stop
      digitalWrite(4, LOW);
      digitalWrite(3, LOW);
      digitalWrite(2, LOW);
  } 
}
