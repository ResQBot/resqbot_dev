const int ledPin = 13;
bool ledMode = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  
  //establish connection
  Serial.begin(115200);
}

void loop() {

  String rx_msg = Serial.readStringUntil('\n');
  
  if (rx_msg =="Who are you?"){
    handshake(rx_msg);
  } 
  else{
  //normal operation
    sscanf(rx_msg.c_str(), "P%i\n", &ledMode);
    digitalWrite(ledPin, ledMode);
  }
 
  delay(50);

}

void handshake(String rx_msg){ 

  //set confirmation to 0
  bool confirmation = 0;
  
  //answer Handshake
  Serial.println("ChainController");

  //wait for confirmation
  while(!confirmation){

    //wait for answer, then read
    delay(50);
    String rx_msg = Serial.readStringUntil('\n');

    //check for correct answer and confirm handshake
    if(rx_msg == "Hello ChainController"){
      confirmation = 1;
      Serial.println("confirmed");
    }
    
    //if answer is wrong, retry
    else {
      confirmation = 0;
    }
  }
}
