const int analogInPin = A0;  // Analog input pin that the capactors is attached to
const int viPin = 11; 


int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

int i, Status,pwm;
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
   pinMode(viPin, OUTPUT); 

}

void loop() {
  if (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    Status = Serial.parseInt();
    if(Status == 2){ 
     
       while (Serial.available() <= 1){}
       pwm = Serial.parseInt();
       analogWrite(viPin, pwm);
      //Serial.print("5\n");   
    }
    else if(Status == 1){ 
      // read the analog in value:
      outputValue = 0;
     for( i = 1 ; i <= 3 ; i++){
      outputValue = outputValue+analogRead(analogInPin);            
      //outputValue =outputValue + sensorValue; 
      } 
     outputValue= outputValue/3;
     outputValue=map(outputValue, 0, 1023, 0, 255);
           
      Serial.println(outputValue); 
         
           
  
    }
  }  
}
