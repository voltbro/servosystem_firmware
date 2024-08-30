#include <VBCoreG4_arduino_system.h>
#include <AS5047P.h>

#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define USR_BTN PC13

#define MOSI PC12
#define MISO PC11
#define SCK PC10
#define AS5047P_CHIP_SELECT_PORT PA_15_ALT1

#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

SPIClass SPI_3(MOSI, MISO, SCK);
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

HardwareTimer *timer_read_request = new HardwareTimer(TIM3);
HardwareTimer *timer_move = new HardwareTimer(TIM5);
HardwareTimer *timer_send_response = new HardwareTimer(TIM7);


float kx = 0;
float kv = 0;

float target_angle, current_angle, prev_current_angle;
float vel_model, vel_current;

unsigned long t, time_dot;

int step_flag = 0, sin_flag = 0, stop_flag = 0, sin_phi_flag = 0, square_flag = 0, triangle_flag = 0;

String answer, request;

float u = 0;
float u_send = 0;
float amplitude = 1;
float freq = 1;

float offset;
float angle_tmp;

float pwm_gain = 1;
int pwm_offset = 0;



void setup() {

  pinMode(SLEEPn, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LED2, OUTPUT);

  digitalWrite(SLEEPn, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  Serial.begin(500000);

  while (!as5047p.initSPI(& SPI_3)) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(3000);
  }

  vel_current = 0;
  offset = as5047p.readAngleDegree()*PI/180;
  current_angle = 0;

  delay(1000);

  timer_read_request->pause();
  timer_read_request->setOverflow(200, HERTZ_FORMAT); 
  timer_read_request->attachInterrupt(read_request);
  timer_read_request->refresh();
  timer_read_request->resume();

  timer_move->pause();
  timer_move->setOverflow(1000, HERTZ_FORMAT); 
  timer_move->attachInterrupt(move);
  timer_move->refresh();
  timer_move->resume();

  timer_send_response->pause();
  timer_send_response->setOverflow(200, HERTZ_FORMAT); 
  timer_send_response->attachInterrupt(send_response);
  timer_send_response->refresh();
  timer_send_response->resume();
  
}


void read_request(){
  if(Serial.available()>0){
    request = Serial.readStringUntil('\n');
    int index = request.indexOf(' ');

    if (index != -1){

      String tmp_sub_req = request.substring(0, index);
      request = request.substring(index+1);
      index = request.indexOf(' ');

      if (tmp_sub_req == "#SETK"){        
        kx = request.substring(0, index).toFloat();        
        kv = request.substring(index+1).toFloat();    
        Serial.println("#SETK OK");
      }

      else if (tmp_sub_req == "#STEP"){
        target_angle = request.toFloat();
        Serial.println("#STEP OK");
        step_flag = 1;
        sin_flag = 0;
        stop_flag = 0;
        sin_phi_flag = 0;
        square_flag = 0;
        triangle_flag = 0;
        t = millis();
      }

      else if (tmp_sub_req == "#SIN_U"){
        amplitude = request.substring(0, index).toFloat(); 
        freq = request.substring(index+1).toFloat();
        Serial.println("#SIN_U OK");
        sin_flag = 1;
        step_flag = 0;
        stop_flag = 0;
        sin_phi_flag = 0;
        square_flag = 0;
        triangle_flag = 0;
        t = millis();
      }

      else if (tmp_sub_req == "#SETFRIC"){
        pwm_gain = request.substring(0, index).toFloat();
        pwm_offset = request.substring(index+1).toInt();
        Serial.println("#SETFRIC OK");;
      }

      else if (tmp_sub_req == "#SIN_PHI"){
        amplitude = request.substring(0, index).toFloat(); 
        freq = request.substring(index+1).toFloat();
        Serial.println("#SIN_PHI OK");
        sin_phi_flag = 1;
        sin_flag = 0;
        step_flag = 0;
        stop_flag = 0;
        square_flag = 0;
        triangle_flag = 0;
        t = millis();
      }

      else if (tmp_sub_req == "#SQUARE"){
        amplitude = request.substring(0, index).toFloat(); 
        freq = request.substring(index+1).toFloat();
        Serial.println("#SQUARE OK");
        sin_phi_flag = 0;
        sin_flag = 0;
        step_flag = 0;
        stop_flag = 0;
        square_flag = 1;
        triangle_flag = 0;
        t = millis();
      }

      else if (tmp_sub_req == "#TRI"){
        amplitude = request.substring(0, index).toFloat(); 
        freq = request.substring(index+1).toFloat();
        Serial.println("#TRI OK");
        sin_phi_flag = 0;
        sin_flag = 0;
        step_flag = 0;
        stop_flag = 0;
        square_flag = 0;
        triangle_flag = 1;
        t = millis();
      }
    }

    else {
      if (request == "#STOP"){
        stop();
      }
      else if (request == "#GETK"){
        get_k();
      }
    }

  }
}
int sign(float val){
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

void calc_fric_comp(){
  u = int(sign(u) * (pwm_gain * abs(u) + pwm_offset));
}

void get_k(){
  answer = String("#kx: ")+kx +" kv: "+ kv;
  Serial.println("#GETK OK");
  Serial.println(answer);
}

void calc_angle_vel(){
  angle_tmp = fmod((2*PI - (-(as5047p.readAngleDegree()*PI/180)+offset) - prev_current_angle), 2*PI);
  if (angle_tmp > PI){current_angle += (angle_tmp - 2*PI);}  
  else if (angle_tmp < -PI){current_angle += (angle_tmp + 2*PI);} 
  else {current_angle += angle_tmp;}
  vel_current = (current_angle - prev_current_angle)*1000;
  prev_current_angle = current_angle;

  if (sin_phi_flag) {
    target_angle = amplitude*sin((2*PI*freq*(float(time_dot)/1000.0)));
    vel_model = amplitude*2*PI*freq*cos((2*PI*freq*(float(time_dot)/1000.0)));
    }
  if(square_flag){
    target_angle = amplitude*sign(sin((2*PI*freq*(float(time_dot)/1000.0))));
    vel_model = 0;
    }
  if(triangle_flag){
    float t_tmp = float((time_dot/1000.0));
    target_angle = amplitude - (2*amplitude/PI)*acos(cos(2*PI*freq*t_tmp-PI/2));   
    vel_model = (-4*amplitude*freq)*sin(2*PI*freq*t_tmp-PI/2)/sqrt(1-sq(cos(2*PI*freq*t_tmp-PI/2))); 
    }
}

void send_response(){
  if (step_flag){
    answer = String("#OUT ")+time_dot + " " + target_angle +" "+ current_angle;
    Serial.println(answer);
  }
  else if (sin_flag){
    answer = String("#OUT ")+time_dot + " "+ u_send +" "+ current_angle ;
    Serial.println(answer);
  }
  else if (sin_phi_flag || square_flag || triangle_flag){
    answer = String("#OUT ")+time_dot + " "+ target_angle +" "+ current_angle+" "+ vel_model ;
    Serial.println(answer);
  }
}

void stop(){
  analogWrite(IN1, LOW);
  analogWrite(IN2, LOW);
  u = 0;
  step_flag = 0;
  sin_flag = 0;
  sin_phi_flag = 0;
  square_flag = 0;
  triangle_flag = 0;
  stop_flag = 1;
  Serial.println("#STOP OK");
  delay(100);
}

void move(){
  if (!stop_flag){
  calc_angle_vel();
  if (step_flag ){
    time_dot = millis() - t;
    u = kx* (target_angle - current_angle ) - kv * vel_current; 
  }
  else if (sin_flag){
    time_dot = millis() - t;
    u_send =amplitude*sin((2*PI*freq*(float(time_dot)/1000.0)));
    u = -kx * current_angle + u_send;
  }
  else if(sin_phi_flag || square_flag || triangle_flag){
    time_dot = millis() - t;
    u = kx* (target_angle - current_angle ) + kv * (vel_model - vel_current); 
  }
 
  
  if (u >= 12)
    u = 12;
  else if (u <= -12)
    u = -12;

  u = int(u*255/12);
  calc_fric_comp();
  if (u>=0) {
    analogWrite(IN1, 255);
    analogWrite(IN2, 255-u);
    }
  else {
    analogWrite(IN2, 255);
    analogWrite(IN1, 255-abs(u));
  }
  }
}

void loop() {
}
