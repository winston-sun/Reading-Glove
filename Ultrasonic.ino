

#define TrigPin 6
#define EchoPin 5

int Value_cm;

void setup()
{
 Serial.begin(9600);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
}
int oldval;
void loop()
{

  Value_cm=getddistance(TrigPin,EchoPin); 
 //Serial.println(Value_cm);

  if(Value_cm/10!=oldval)
  {
     oldval=Value_cm/10;
     Serial.println("photograph");
  }
 delay(100);
}

float getddistance(int Trig,int Echo)
{
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  digitalWrite(Trig, LOW); 
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
    
  return float( pulseIn(Echo, HIGH) * 17 )/1000;  
  
}
