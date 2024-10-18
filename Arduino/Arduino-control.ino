#include <Servo.h>
Servo myServo;

#define motor1_1 12
#define motor1_2 13
#define encoder1_A 2
#define encoder1_B 4

#define motor2_1 10
#define motor2_2 11
#define encoder2_A 3
#define encoder2_B 5

#define motor3_1 8
#define motor3_2 9
#define encoder3_A 18
#define encoder3_B 22

#define motor4_1 6
#define motor4_2 7
#define encoder4_A 19
#define encoder4_B 23

// for encoder
long count1 = 0; // this motor is connected to pin 13 which blinks whenever you connect serial
long count2 = 0;
long count3 = 0;
long count4 = 0;

const int resolution = 960; // considering we take phase A rising edge as interrupt (resolution multiplier 1)

// for looping
const int DELTA_T = 100; //milliseconds
unsigned long current_time = 0l;
unsigned long previous_time = 0l; 

// for serial
String temp = "";
char current;
int temp_values[2] = {0,0};
int index = 0;
bool endline_reached = false;

int motor_combinations[11][4] = {
  { 1,  1,  1,  1},
  {-1, -1, -1, -1},
  {-1,  1, -1,  1},
  { 1, -1,  1, -1},
  {-1,  1,  1, -1},
  { 1, -1, -1,  1},
  { 0,  1,  0,  1},
  { 1,  0,  1,  0},
  {-1,  0, -1,  0},
  { 0, -1,  0, -1},
  { 0,  0,  0, 0}
};

void setup() {
  Serial.begin(115200);

  pinMode(motor1_1, OUTPUT);
  pinMode(motor1_2, OUTPUT);
  pinMode(encoder1_A, INPUT);
  pinMode(encoder1_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_A), motor1, RISING);

  pinMode(motor2_1, OUTPUT);
  pinMode(motor2_2, OUTPUT);
  pinMode(encoder2_A,  INPUT);
  pinMode(encoder2_B,  INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder2_A), motor2, RISING);

  pinMode(motor3_1, OUTPUT);
  pinMode(motor3_2, OUTPUT);
  pinMode(encoder3_A,  INPUT);
  pinMode(encoder3_B,  INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder3_A), motor3, RISING);

  pinMode(motor4_1, OUTPUT);
  pinMode(motor4_2, OUTPUT);
  pinMode(encoder4_A,  INPUT);
  pinMode(encoder4_B,  INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder4_A), motor4, RISING);
  myServo.attach(45); 
}

void loop() {
  current_time = millis();
  
  reset_m1();

  if((current_time - previous_time) > DELTA_T){
    if (Serial.available()) {
      read_line();
      if(endline_reached){
        endline_reached = false;
      }
      else{
        // Serial.println(count1);
        print_encoder_data();
      }  
    } else {
      move(10, 0); // fefervedvervsdrbhtyfcjy
    }
    previous_time = current_time;
  }
}

void set_Speed(byte motor, int speed){ // because both numbers are between 0 and 255
  switch (motor) {
    case 1:
      set_Speed_1(speed);
      break;
    case 2:
      set_Speed_2(speed);
      break;
    case 3:
      set_Speed_3(speed);
      break;
    case 4:
      set_Speed_4(speed);
      break;
  }
}

void set_Speed_1(int speed){
  if(speed == 0){
    analogWrite(motor1_1, 255); //brakes
    analogWrite(motor1_2, 255);
  } else if (speed < 0){
    analogWrite(motor1_1, 0);
    analogWrite(motor1_2, -speed);
  } else{
    analogWrite(motor1_1, speed);
    analogWrite(motor1_2, 0);
  }
}

void set_Speed_2(int speed){
  if(speed == 0){
    analogWrite(motor2_1, 255); //brakes
    analogWrite(motor2_2, 255);
  } else if (speed < 0){
    analogWrite(motor2_1, 0);
    analogWrite(motor2_2, -speed);
  } else{
    analogWrite(motor2_1, speed);
    analogWrite(motor2_2, 0);
  }
}

void set_Speed_3(int speed){
  if(speed == 0){
    analogWrite(motor3_1, 255); //brakes
    analogWrite(motor3_2, 255);
  } else if (speed < 0){
    analogWrite(motor3_1, 0);
    analogWrite(motor3_2, -speed);
  } else{
    analogWrite(motor3_1, speed);
    analogWrite(motor3_2, 0);
  }
}

void set_Speed_4(int speed){
  if(speed == 0){
    analogWrite(motor4_1, 255); //brakes
    analogWrite(motor4_2, 255);
  } else if (speed < 0){
    analogWrite(motor4_1, 0);
    analogWrite(motor4_2, -speed);
  } else{
    analogWrite(motor4_1, speed);
    analogWrite(motor4_2, 0);
  }
}

void motor1(){
  if(digitalRead(encoder1_B)){ count1--; }
  else{ count1++; }
}

void motor2(){ // this is flipped because it is reversed as a signal
  if(digitalRead(encoder2_B)){ count2++; }
  else{ count2--; }
}

void motor3(){ // this is flipped because it is reversed as a signal
  if(digitalRead(encoder3_B)){ count3++; }
  else{ count3--; }
}

void motor4(){
  if(digitalRead(encoder4_B)){ count4--; }
  else{ count4++; }
}
void read_line(){
  while(Serial.available()){
    current = Serial.read();
    
    if(isDigit(current) || current == '-'){
      temp += current;  // Append character directly to temp
    } 
    else if(current == ','){
      temp_values[index] = temp.toInt();
      index++;
      temp = "";
    }
    else if (current == ';'){
      endline_reached = true;
      temp_values[index] = temp.toInt();  // Store last value before ';'
      
      // Check for the specific format '-1, x, -1;'
      if (index == 2 && temp_values[0] == -1 && temp_values[2] == -1) {
        int x = temp_values[1];
        Serial.print("Extracted x: ");
        Serial.println(x);
        myServo.write(x);   // Set servo to 90 degrees
        delay(10); // Process or use x as needed
      } else {
        // Your existing condition or actions
        if(((temp_values[0] + temp_values[1]) == temp.toInt())){
          move(temp_values[0], temp_values[1]);
        }
        print_encoder_data();
      }
      
      temp = "";
      index = 0;
    }
  }
}


void move(byte dir, int speed){
  set_Speed(1, (speed*motor_combinations[dir][0]));
  set_Speed(2, (speed*motor_combinations[dir][1]));
  set_Speed(3, (speed*motor_combinations[dir][2]));
  set_Speed(4, (-speed*motor_combinations[dir][3]));
}

void print_encoder_data(){
    Serial.print("m1:");
    Serial.print(count1);
    Serial.print(",\t");
    Serial.print("m2:");
    Serial.print(count2);
    Serial.print(",\t");
    Serial.print("m3:");
    Serial.print(count3);
    Serial.print(",\t");
    Serial.print("m4:");
    Serial.print(count4);
    Serial.println();
}

void reset_m1(){
  if(!count2 && !count3 && !count4 && (count1!=0)) {count1 = 0;}
}
