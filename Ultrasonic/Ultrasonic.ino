

#define TrigPin 6
#define EchoPin 5

int value_cm;

void setup()
{
 Serial.begin(9600);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
}
int oldval = 0;
void loop()
{

  value_cm = getdistance(TrigPin,EchoPin); 

  
}

float getdistance(int Trig,int Echo)
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
