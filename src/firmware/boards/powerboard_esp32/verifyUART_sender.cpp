#include<SoftwareSerial.h> 
SoftwareSerial SUART(2, 3); //SRX = DPin-2; STX = DPin-3  //SoftwareSerial is an Arduino package to 
                                                          //allow serial communication on other digital pins
                                                          //Theortically, if SoftwareSerial is not viable, we can 
                                                          //adjust the code to use directly to the UART ports
                                                          //Reference:
                                                          //https://forum.arduino.cc/t/how-to-display-text-sent-via-rx-tx-serial-line/593434/8

char myMsg[] = "Verifying Connection";         //Message we would like to be sent and received
void setup() 
{
  Serial.begin(9600);
  SUART.begin(9600);
}

void loop() 
{
  Serial.print("Sending to ______ this string: ");
  Serial.println(myMsg);   //display what you are sending
  //--------------------
  SUART.println(myMsg);   //send ASCII coded string via SUART Port; goes 1 char at a time
  delay(1000);          //1-sec test interval
}