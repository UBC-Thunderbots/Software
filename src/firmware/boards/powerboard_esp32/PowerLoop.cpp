#include <SoftwareSerial.h> // include library to enable software serial ports
#include <EEPROM.h> // include library to read and write from flash memory

#define EEPROM_SIZE 4

SoftwareSerial SUART(2,3); //SRX = DPin-2; STX = DPin-3

//Message Storage
char INITIAL_MESSAGE;
char MESSAGE;

//Action Messages Library
char HV_MEASUREMENT;
char GENEVA_ENCODER;
char GENEVA_MOTOR_ANGLE;
char GENEVA_MOTOR_SPEED;
char BREAKBEAM;
char CHICKER_AUTO_KICK;
char CHICKER_AUTO_CHIP;
char CHICKER_KICK_DISTANCE_SPEED;
char CHICKER_CHIP_DISTANCE_SPEED;
char PULSE_WIDTH;
char FLYBACK;

//PINS
const int HV_MEASUREMENT_PIN;
const int G_ENCODER_PIN;
const int G_MOTOR_ANGLE_PIN;
const int G_MOTOR_SPEED_PIN;
const int BREAKBEAM_INTER_PIN;
const int BREAKBEAM_DIFF_PIN;
const int CHICKER_AUTOKICK_PIN;
const int CHICKER_AUTOCHIP_PIN;
const int KICK_DISTANCE_SPEED_PIN;
const int KICK_DISTANCE_SPEED_VAL_PIN;
const int CHIP_DISTANCE_SPEED_PIN;
const int CHIP_DISTANCE_SPEED_VAL_PIN;
const int PULSE_WIDTH_PIN;
const int FLYBACK_FAULT_PIN;

//Flash Address
int HV_MEASUREMENT_ADDRESS;
int G_ENCODER_ADDRESS;
int G_MOTOR_ANGLE_ADDRESS;
int G_MOTOR_SPEED_ADDRESS;
int BREAKBEAM_INTER_ADDRESS;
int BREAKBEAM_DIFF_ADDRESS;
int CHICKER_AUTOKICK_ADDRESS;
int CHICKER_AUTOCHIP_ADDRESS;
int KICK_DISTANCE_SPEED_ADDRESS;
int CHIP_DISTANCE_SPEED_ADDRESS;
int PULSE_WIDTH_ADDRESS;
int FLYBACK_FAULT_ADDRESS;

//Others
int COUNTER;
float checker;

int hv_data;
int g_encoder_angle;
float g_motor_angle;
float g_motor_speed;
int breakbeam_diff;
int breakbeam_inter;
int autokick;
int autochip;
float kick_distance_speed;
float chip_distance_speed;
int flyback_fault;



void setup(){

    //Initializing serial ports 
    Serial.begin(9600);
    SUART.begin(9600);

    //Initializing GPIO pins
    pinMode(HV_MEASUREMENT_PIN, INPUT);
    pinMode(G_ENCODER_PIN, INPUT);
    pinMode(G_MOTOR_ANGLE_PIN, INPUT);
    pinMode(G_MOTOR_SPEED_PIN, INPUT);
    pinMode(BREAKBEAM_DIFF_PIN, INPUT);
    pinMode(BREAKBEAM_INTER_PIN, INPUT);
    pinMode(CHICKER_AUTOCHIP_PIN, INPUT);
    pinMode(CHICKER_AUTOKICK_PIN, INPUT);
    pinMode(KICK_DISTANCE_SPEED_PIN, INPUT);
    pinMode(CHIP_DISTANCE_SPEED_PIN, INPUT);
    pinMode(PULSE_WIDTH_PIN, INPUT);
    pinMode(FLYBACK_FAULT_PIN, INPUT);

    
    // initialize EEPROM with predefined size
    for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
    }

}


void loop(){
    INITIAL_MESSAGE = SUART.available(); //Checks if initial message is avaliable (If a message is recieved)

    if (INITIAL_MESSAGE != 0) //If message avaliable, continue
    {
        
        MESSAGE = SUART.read(); //Store entire message

        if (MESSAGE == HV_MEASUREMENT){

            //Action
            for(COUNTER = 0; COUNTER < 5; COUNTER++){  //Total 2.5s of reading in 0.5 intervals

                hv_data = analogRead(HV_MEASUREMENT_PIN);

                Serial.println(hv_data);

                delay(500); //wait 0.5s
             
            }

            EEPROM.write(HV_MEASUREMENT_ADDRESS, hv_data);

        }

        if(MESSAGE == GENEVA_ENCODER){

            //Action
            g_encoder_angle = analogRead(G_ENCODER_PIN);

            Serial.println(g_encoder_angle);

            EEPROM.write(G_ENCODER_ADDRESS, g_encoder_angle);
        }

        if(MESSAGE == GENEVA_MOTOR_ANGLE){

            //Action
            byte checker = SUART.available(); //check if a character has arrived via SUART Port
            if (checker != 0) //a charctaer has arrived; it has been auto saved in FIFO; say 1 as 0x31
            {
                g_motor_angle = SUART.read(); //read arrived character from FIFO (say 1) and put into x as 0x31 

                analogWrite(G_MOTOR_ANGLE_PIN, g_motor_angle); //Set motor angle
            }
            else{
                Serial.println("Failed Retrieve Geneva Motor Angle");
            }


            EEPROM.write(G_MOTOR_ANGLE_ADDRESS, g_motor_angle);
        }

        if(MESSAGE == GENEVA_MOTOR_SPEED){

            //Action
            checker = SUART.available(); //check if a character has arrived via SUART Port
            if (checker != 0) //a charctaer has arrived; it has been auto saved in FIFO; say 1 as 0x31
            {
                g_motor_speed = SUART.read(); //read arrived character from FIFO (say 1) and put into x as 0x31 

                analogWrite(G_MOTOR_SPEED_PIN, g_motor_speed); //Set motor angle
            }
            else{
                Serial.println("Failed Retrieve Geneva Motor Speed");
            }

            EEPROM.write(G_MOTOR_SPEED_ADDRESS ,g_motor_speed);
        }

        if(MESSAGE == BREAKBEAM){

            //Action
            breakbeam_inter = analogRead(BREAKBEAM_DIFF_PIN);
            Serial.println(breakbeam_inter);

            breakbeam_diff = analogRead(BREAKBEAM_INTER_PIN);
            Serial.println(breakbeam_diff);

            EEPROM.write(BREAKBEAM_INTER_ADDRESS, breakbeam_inter);
            EEPROM.write(BREAKBEAM_DIFF_ADDRESS, breakbeam_diff);
        }

        if(MESSAGE == CHICKER_AUTO_KICK){

            //Action
            checker = SUART.available(); //check if a character has arrived via SUART Port
            if (checker != 0) //a charctaer has arrived; it has been auto saved in FIFO; say 1 as 0x31
            {
                autokick = SUART.read(); //read arrived character from FIFO (say 1) and put into x as 0x31 

                analogWrite(CHICKER_AUTOKICK_PIN, autokick); //Set motor angle
            }
            else{
                Serial.println("Failed Retrieve Chicker Auto Kick Data");
            }

            EEPROM.write(CHICKER_AUTOKICK_ADDRESS, autokick);
        }

        if(MESSAGE == CHICKER_AUTO_CHIP){

            //Action
            checker = SUART.available(); //check if a character has arrived via SUART Port
            if (checker != 0) //a charctaer has arrived; it has been auto saved in FIFO; say 1 as 0x31
            {
                autochip = SUART.read(); //read arrived character from FIFO (say 1) and put into x as 0x31 

                analogWrite(CHICKER_AUTOCHIP_PIN, autochip); //Set motor angle
            }
            else{
                Serial.println("Failed Retrieve Chicker Auto Kick Data");
            }

            EEPROM.write(CHICKER_AUTOCHIP_ADDRESS, autochip);
        }
        
        if(MESSAGE == CHICKER_KICK_DISTANCE_SPEED){

            //Action
            checker = SUART.available(); //check if a character has arrived via SUART Port
            if (checker != 0) //a charctaer has arrived; it has been auto saved in FIFO; say 1 as 0x31
            {
                kick_distance_speed = SUART.read(); //read arrived character from FIFO (say 1) and put into x as 0x31 

                analogWrite(KICK_DISTANCE_SPEED_VAL_PIN, kick_distance_speed); //Set motor angle
            }
            else{
                Serial.println("Failed Retrieve Chicker Auto Kick Data");
            }

            EEPROM.write(KICK_DISTANCE_SPEED_ADDRESS, kick_distance_speed);
        }

        if(MESSAGE == CHICKER_CHIP_DISTANCE_SPEED){

            //Action
            checker = SUART.available(); //check if a character has arrived via SUART Port
            if (checker != 0) //a charctaer has arrived; it has been auto saved in FIFO; say 1 as 0x31
            {
                chip_distance_speed = SUART.read(); //read arrived character from FIFO (say 1) and put into x as 0x31 

                analogWrite(CHIP_DISTANCE_SPEED_VAL_PIN, chip_distance_speed); //Set motor angle
            }
            else{
                Serial.println("Failed Retrieve Chicker Auto Kick Data");
            }

            EEPROM.write(CHIP_DISTANCE_SPEED_ADDRESS, chip_distance_speed);
        }

        if(MESSAGE == PULSE_WIDTH){

            //Action
            //?????????????????????

        }

        if(MESSAGE == FLYBACK){

            //Action
            flyback_fault = analogRead(FLYBACK_FAULT_PIN);

            Serial.println(flyback_fault);

            EEPROM.write(FLYBACK_FAULT_ADDRESS, flyback_fault);
        }


    }



}
