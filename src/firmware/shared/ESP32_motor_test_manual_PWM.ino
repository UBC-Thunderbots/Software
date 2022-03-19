const int DIR = 14;
const int PWM = 33;
const int encoderA = 9;
const int encoderB = 10;

const int push1 = 19;
const int push2 = 22;

volatile unsigned long encoderValue = 0;

void updateEncoderAB() {
  if (digitalRead(encoderA) == HIGH) {
    if (digitalRead(encoderB) == LOW) {
      encoderValue++;
    } else {
      encoderValue--;
    }
  } else {
    if (digitalRead(encoderB) == LOW) {
      encoderValue--;
    } else {
      encoderValue++;
    }
  }
}

void updateEncoder() {
  if(digitalRead(encoderA) == HIGH){
    encoderValue++;
  }else{
    encoderValue--;
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(encoderA, INPUT_PULLDOWN);
  pinMode(encoderB, INPUT_PULLDOWN);

  pinMode(push1, OUTPUT);
  pinMode(push2, OUTPUT);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(encoderA), updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), updateEncoder, RISING);
}

void loop() {
  if (digitalRead(push1) == HIGH && digitalRead(push2) == LOW) {
    digitalWrite(DIR, HIGH); // forward
    digitalWrite(PWM, HIGH);
  }

  else if (digitalRead(push1) == LOW && digitalRead(push2) == HIGH) {
    digitalWrite(DIR, LOW); // backward
    digitalWrite(PWM, HIGH);
  }

  else {
    digitalWrite(PWM, LOW); // stop
  }

  Serial.print("PULSES: ");
  Serial.println(encoderValue);
  delay(100);

  encoderValue = 0;
}
