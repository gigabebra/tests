#define L_ctrl 4
#define L_PWM  5
#define R_ctrl 2
#define R_PWM  9
int Rsens=8;
int Lsens=6;
void setup() {
  Serial.begin(9600);
  pinMode(L_ctrl, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_ctrl, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(Rsens, INPUT);
  pinMode(Lsens, INPUT);
}
int Rval;
int Lval;
void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(digitalRead(Rsens));
  Serial.println(digitalRead(Lsens));
  Rval=digitalRead(Rsens);
  Lval=digitalRead(Lsens);
  if ((Rval==1)&&(Lval==1)){
    front();
  }
  else if (Rval>Lval){
    left();
  }
  else if (Lval>Rval){
    right();
  }
  else{
    stop();
  }
}
void front(){
  digitalWrite(L_ctrl,HIGH);  
  digitalWrite(R_ctrl,HIGH); 
  analogWrite(L_PWM, 70); 
  analogWrite(R_PWM, 70); 
}
void left(){
  digitalWrite(L_ctrl,HIGH);  
  digitalWrite(R_ctrl,HIGH); 
  analogWrite(L_PWM, 200); 
  analogWrite(R_PWM, 70); 
}
void right(){
  digitalWrite(L_ctrl,HIGH);  
  digitalWrite(R_ctrl,HIGH); 
  analogWrite(L_PWM, 70); 
  analogWrite(R_PWM, 200); 
}
void stop(){
  analogWrite(L_PWM, 0); 
  analogWrite(R_PWM, 0); 
}
