const int output = 8;
const int buzzerOut = 9;
const int SSB_PIN = 10;

//int max_time = 370000; //6 min 10 sec

const int stage1End = 120;
const int stage2End = 170;
const int stage3End = 245;

const unsigned long stage1Time = 50000UL;
const unsigned long stage2Time = stage1Time + 90000UL;
const unsigned long stage3Time = stage2Time + 105000UL;
const unsigned long max_stage2_time = 120000;
const unsigned long max_stage3_time = 100000;
double roomTemp = 20; //assumption
  
const int period = 1000;

//oven is started immediately after usb is connected. No button is necessary. 
double readTemp (int port);

#include <SPI.h>
void setup()
{
  pinMode(output, OUTPUT);
  pinMode(buzzerOut, OUTPUT);
  //pinMode(thermoCouple, INPUT);
  Serial.begin(9600);
  SPI.begin();
  SPI.setBitOrder ( MSBFIRST );
}

void loop()
{ 
  float currentTemp = readTemp(  );

  unsigned long stage_1_endtime;
  unsigned long stage_2_endtime;
  boolean first_time = true;
  float stage_duty_cycles[3] = { 1, 0.38, 1 };
  
  while ( currentTemp < stage1End )
  {
    digitalWrite ( output, HIGH );
    delay ( period * stage_duty_cycles[0] );
    digitalWrite ( output, LOW );
    delay ( period * (1 - stage_duty_cycles[0]) );
    currentTemp = readTemp( );
    Serial.println( currentTemp );
  }
  stage_1_endtime = millis();
  Serial.println ( stage_1_endtime);
  while ( currentTemp < stage2End )
  {
    digitalWrite ( output, HIGH );
    delay ( period * stage_duty_cycles[1] );
    digitalWrite ( output, LOW );
    delay ( period * (1 - stage_duty_cycles[1]) );
    currentTemp = readTemp( );
    Serial.println( currentTemp );
    if ( millis() > (stage_1_endtime + max_stage2_time) ) break;
  }

  while ( currentTemp < stage3End )
  {
    digitalWrite ( output, HIGH );
    delay ( period * stage_duty_cycles[2] );
    digitalWrite ( output, LOW );
    delay ( period * (1 - stage_duty_cycles[2]) );
    currentTemp = readTemp(  );
    Serial.println( currentTemp );
    if ( currentTemp > 217 && first_time ) 
    {
        stage_2_endtime = millis();
        first_time = false;
    }
    if ( currentTemp > 217 && millis() > stage_2_endtime + max_stage3_time )
      break;
  }
   
  //turns off oven at time limit to avoid overheating the board in case 
  //ppl forgot to check
  /*
  if ( millis() >= max_time)
  {
    digitalWrite(output, LOW);
  }
  */
  while (1 )
  {
    analogWrite(buzzerOut, 500);
    delay(500);
    analogWrite(buzzerOut, 0);
    delay(200);
  }
  
}
/*
double readTemp()
{ 
  double temp;
  
  if ( millis() < stage1Time )
  {
    temp = (double) millis()/1000.0*8.0/5.0 + roomTemp;
  }
  else if ( millis() < stage2Time )
  {
    temp = ((double) (millis() - stage1Time))/1000.0*(5.0/9.0) + stage1End;
  }
  else
  {
    temp = ((double) (millis() - stage2Time))/1000*45/50 + stage2End;
  }  
  return temp;  
}*/


float readTemp ( void )
{
  digitalWrite (SSB_PIN,LOW);
  delayMicroseconds(1);
  byte firstHalf = SPI.transfer(0);
  byte lastHalf = SPI.transfer(0);
  digitalWrite (SSB_PIN, HIGH);
    
  unsigned int value = (firstHalf << 8) + lastHalf;
  value >>= 3;
  
  return value/4.0;

}


  
  




