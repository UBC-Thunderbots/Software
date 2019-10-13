const int output = 12;
const int buzzerOut = 10;
int max_time = 370000; //6 min 10 sec

//oven is started immediately after usb is connected. No button is necessary. 
void setup()
{
  pinMode(output,OUTPUT);
  pinMode(buzzerOut, OUTPUT);
  
  digitalWrite(output, HIGH);
  delay(130000);
  digitalWrite(output, LOW);
  delay(30000);
  digitalWrite(output, HIGH);
  delay(20000);
  digitalWrite(output, LOW);
  delay(20000);
  //oven won't turn off until power is manually removed. Go ahead and make
  //sure that the solder is turning shiny. 
  digitalWrite(output, HIGH);
  delay(150000);
}
void loop()
{
  //turns off oven at time limit to avoid overheating the board in case 
  //ppl forgot to chech
  if ( millis() >= max_time)
  {
    digitalWrite(output, LOW);
  }
  
  analogWrite(buzzerOut, 500);
  delay(500);
  analogWrite(buzzerOut, 0);
  delay(200);
  
}


