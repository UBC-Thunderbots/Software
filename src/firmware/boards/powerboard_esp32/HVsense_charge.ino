int HV_SENSE = 5;
int FAULT = 23;
int DONE = 18;
int CHRG = 15;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(HV_SENSE, INPUT);
  pinMode(DONE, INPUT);
  pinMode(FAULT, INPUT);
  pinMode(CHRG, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(HVmeasure(HV_SENSE));
  digitalWrite(CHRG,LOW);
  delay(5000);
  Serial.println("Charging Cap");
  bool success = ChargeCap(CHRG, FAULT, DONE, HV_SENSE);
  Serial.println("exit");
}

int HVmeasure(int HV_SENSE){
  return analogRead(HV_SENSE);
}

bool ChargeCap(int CHRG, int FAULT, int DONE, int HV_SENSE){
  digitalWrite(CHRG, HIGH);
  Serial.println("ChargeHigh");
  while (digitalRead(DONE) == HIGH){
    //Serial.println(HVmeasure(HV_SENSE));
    if (digitalRead(FAULT) == LOW){
      Serial.println("FAIL");
      digitalWrite(CHRG,LOW);
      return false;
    }
  }
  digitalWrite(CHRG, LOW);
  return true;
}
