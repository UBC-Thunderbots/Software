#include <SoftwareSerial.h> // include library to enable software serial ports

SoftwareSerial SUART(2,3); //SRX = DPin-2; STX = DPin-3

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

//Active Low & High
int LOW = 0;
int HIGH = 1;

//PINS
const int HV_MEASUREMENT_PIN = 5;
const int G_ENCODER_A_PIN = 35;
const int G_ENCODER_B_PIN = 34;
const int G_MOTOR_ANGLE_PIN = 22;  //
const int G_MOTOR_SPEED_PIN = 21;
const int BREAKBEAM_INTER_PIN = 6;
const int PULSE_WIDTH_KICKER_PIN = 10; //
const int PUSLE_WIDTH_CHIPPER_PIN = 11;
const int FLYBACK_FAULT_PIN = 17;
const int PWM_SDA_PIN = 18;
const int PWM_SCL_PIN = 20;


//Current Values
int current_geneva_angle;


//Values
float hv_data;
float g_encoder_angle;
float g_motor_angle;
float g_encoder_a;
float g_encoder_b;
float breakbeam_inter;
bool autokick_sw;
bool autochip_sw;
float kick_distance_speed;
float chip_distance_speed;
float flyback_fault;
float PWM_SDA;
float PWM_SCL;

// Request 
struct PowerLoopRequest request;

// Status Response
struct PowerLoopStatus status;


void setup(){

    //Initializing serial ports 
    Serial.begin(9600);
    SUART.begin(9600);

    //Initializing GPIO pins
    pinMode(HV_MEASUREMENT_PIN, INPUT);
    pinMode(G_ENCODER_A_PIN, INPUT);
    pinMode(G_ENCODER_B_PIN, INPUT);
    pinMode(G_MOTOR_ANGLE_PIN, INPUT);
    pinMode(G_MOTOR_SPEED_PIN, INPUT);
    pinMode(BREAKBEAM_INTER_PIN, INPUT);
    pinMode(PULSE_WIDTH_KICKER_PIN, OUTPUT);
    pinMode(PUSLE_WIDTH_CHIPPER_PIN, OUTPUT);
    pinMode(FLYBACK_FAULT_PIN, INPUT);

}

//Action Functions
void moveGeneva(float g_motor_angle){
    analogWrite(G_MOTOR_ANGLE_PIN, g_motor_angle); 
}

//Get Functions
void getGenevaAngle(){
    g_encoder_angle = analogRead(G_MOTOR_ANGLE_PIN);
    return(g_encoder_angle);
}

void getGenevaEncoderA(){
    g_encoder_a = analogRead(G_ENCODER_A_PIN);
    return(g_encoder_a);
}

void getGenevaEncoderB(){
    g_encoder_b = analogRead(G_ENCODER_B_PIN);
    return(g_encoder_b);
}

void getHVMeasurement(){
    hv_data = analogRead(HV_MEASUREMENT_PIN);
    return(hv_data);
}

void getBreakBeam(){
    breakbeam_inter = analogRead(BREAKBEAM_INTER_PIN);
    return(breakbeam_inter);
}

void getPowerMonitorSDA(){
    PWM_SDA = analogRead(PWM_SDA_PIN);
    return(PWM_SDA);
}

void getPowerMonitorSCL(){
    PWM_SCL = analogRead(PWM_SCL_PIN);
    return(PWM_SCL);
}

void getFlybackFault(){
    flyback_fault = analogRead(FLYBACK_FAULT_PIN);
    return(flyback_fault);
}



//Main State Machine Loop
void loop(){

    // Request Handling
    request = getIncomingJetsonNanoRequestMessageHereCOBS();

        if(request.kick_angle != current_geneva_angle){
            moveGeneva(request.kick_angle);
        }


        if(request.kick_speed != 0 && request.autokick == false){
            kick(request.kick_speed);


        if(request.kick_distance != 0 && request.autokick == false){
            kick(request.kick_distance);
        }


        if(request.autokick == true){
            if(analogRead(BREAKBEAM_INTER_PIN) == LOW){
                kick(request.kick_speed);
            }
        }
        else{
            auto_kick(false);
        }

        
        if(request.chip_speed != 0 && request.autochip == false){
            kick(request.chip_speed);
        }


        if(request.chip_distance != 0 && request.autochip == false){
            kick(request.chip_distance); 
        }

        if(request.autochip == true){
            if(analogRead(BREAKBEAM_INTER_PIN) == LOW){
                kick(request.chip_speed);
            }
        }
        else{
            auto_chip(false);
        }


    //Updating Status
    status.high_voltage_measurement_volts = getHVMeasurement(); 
    status.breakbeam_tripped = getBreakBeam();
    status.geneva_angle = getGenevaAngle();
    status.battery_voltage = getBatteryVoltage();
    status.current_draw = getCurrentDraw();
    status.geneva_encoder_a = getGenevaEncoderA();
    status.geneva_encoder_a = getGenevaEncoderB();
    status.PWM_SDA = getPowerMonitorSDA();
    status.PWM_SCL = getPowerMonitorSCL();
    status.flyback_fault = getFlybackFault();


    //Sending Status to JetsonNano
    sendPowerLoopStatusToJetsonNanoOverCOBS(status); 

}
