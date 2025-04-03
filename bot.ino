// Left Motor connections
int enLeft = 3;
int in3    = 5;
int in4    = 4;

// Right Motor connections
int enRight = 9;
int in2     = 7;
int in1     = 8;

// These will contain the speed of the motors
int left_motor_speed;
int right_motor_speed;
int delay_time;

// Delay for turns
//Change it according to the delay you find in physical model
const int u_turn_delay = 5000; 
const int turn_delay = 5000;

// as our speed are from 0-75 we need a speed factor as voltage range is from 0-255
const int speed_factor = 2;

void runnigMotor(int left_motor_speed,int right_motor_speed, int delay_time){
  // This case is for right turn 
  if (left_motor_speed > 0 && right_motor_speed == 0){
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enLeft, left_motor_speed*speed_factor);

      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enRight, right_motor_speed*speed_factor);
      delay(turn_delay);
      
      analogWrite(enLeft, left_motor_speed*speed_factor);
      analogWrite(enRight, left_motor_speed*speed_factor);
      delay(delay_time - turn_delay);
    }
  // This case is for left turn
  else if (right_motor_speed > 0 && left_motor_speed == 0){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enRight, right_motor_speed*speed_factor);

      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enLeft, left_motor_speed*speed_factor);
      delay(turn_delay);
      
      analogWrite(enLeft, right_motor_speed*speed_factor);
      analogWrite(enRight, right_motor_speed*speed_factor);
      delay(delay_time - turn_delay);
    }
  // This case is for u turn
  else if (left_motor_speed > 0 && right_motor_speed < 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enRight, right_motor_speed*speed_factor);

      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enLeft, left_motor_speed*speed_factor);
      delay(u_turn_delay);
      analogWrite(enLeft, left_motor_speed*speed_factor);
      analogWrite(enRight, left_motor_speed*speed_factor);
      delay(delay_time - u_turn_delay);
  }
  // This is the general case where motor have varying speeds
  else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enRight, right_motor_speed*speed_factor);

      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enLeft, left_motor_speed*speed_factor);
      delay(delay_time);
  }
  }

  
void setup() {
  // Beginning serial control
  Serial.begin(19200);
  Serial.setTimeout(3);
  
  //Setting all pins to Output
  pinMode(enLeft, OUTPUT);  
  pinMode(enRight, OUTPUT);  
  pinMode(in1, OUTPUT);  
  pinMode(in2, OUTPUT);  
  pinMode(in3, OUTPUT);  
  pinMode(in4, OUTPUT);  
  pinMode(LED_BUILTIN, OUTPUT);

   // Intial state of motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

void loop() {
  if (Serial.available() > 0){
    left_motor_speed = Serial.readStringUntil(' ').toInt();
    Serial.print("Left motor: ");
    Serial.print(left_motor_speed);
    Serial.print(" ");
    right_motor_speed = Serial.readStringUntil(' ').toInt();
    Serial.print("Right motor: ");
    Serial.print(right_motor_speed);
    Serial.print(" ");
    delay_time = Serial.readStringUntil(' ').toInt();
    Serial.print("Delay time: ");
    Serial.println(delay_time);
  }
  runnigMotor(left_motor_speed, right_motor_speed, delay_time);
  Serial.flush();
}