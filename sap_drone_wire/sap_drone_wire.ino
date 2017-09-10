#include <CurieIMU.h>
#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3);
int input;
int output = 0;
float pre_msecs = 0;
int count = 0;
int i = 0;
byte data;
double motorFSpeed;
double motorBSpeed;
double motorLSpeed;
double motorRSpeed;
double stabilizePgain = 0.0;
double RKp = 0.00;
double RKi = 0.00;
double RKd = 0.00;


double PKp = 0.12;
double PKi = 0.02;
double PKd = 0.02;

double P_error;
double R_error;
double P_RateError;
double R_RateError;
double P_Pre_RateError;
double R_Pre_RateError;
double error_previous;

float rate_cmd = 0;
float rate_error = 0;
float total_intg = 0;
int throttle = 0;

double outMin = -8, outMax = 8;
double ITerm, lastInput;

int calibrateOffsets = 1;
double desired_angle_y = 0;
double desired_angle_x = 0;
double Pitch_P, Pitch_I, Pitch_D, LPF_prev_Pitch_D, LPF_Pitch_D;
double Pitch_PID;

double ROLL_RateError;
double PITCH_RateError;
double Roll_P, Roll_I, Roll_D;
double Roll_PID;

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

#define pi 3.141592
#define RADIANS_TO_DEGREES 180/3.14159
#define fs 131.0;

float dt = 0;

Servo bldc_a;
Servo bldc_b;
Servo bldc_c;
Servo bldc_d;

double LPF_angle_x , LPF_angle_y , LPF_prevangle_x, LPF_prevangle_y =0 ;
double LPF_gyro_x , LPF_gyro_y , LPF_prev_gyro_x, LPF_prev_gyro_y =0 ;
double tau=0.1;

//자이로(각속도)
float gyro_x, gyro_y;
 
//최종 가속도,자이로 각도
float accel_x, accel_y;
float gyro_angle_x=0, gyro_angle_y=0;
 
//상보필터 거친 각도
double angle_x=0,angle_y=0,angle_z=0;
double lastangle_x=0,lastangle_y=0,lastangle_z=0;
 
//자이로센서 바이어스값
float base_gx=0, base_gy=0, base_gz=0;
 
unsigned long pre_msec=0;
void calibrate(){  
  /*
   Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //CurieIMU.setAccelerometerOffset(X_AXIS,495.3);
    //CurieIMU.setAccelerometerOffset(Y_AXIS,-15.6);
    //CurieIMU.setAccelerometerOffset(Z_AXIS,491.4);
    //CurieIMU.setGyroOffset(X_AXIS,7.869);
    //CurieIMU.setGyroOffset(Y_AXIS,-0.061);
    //CurieIMU.setGyroOffset(Z_AXIS,15.494);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(1000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
    */
  int loop =50;
  for (int i=0;i<loop;i++)
  {
    CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
    base_gx += gx;
    base_gy += gy;
    base_gz += gz;
    delay(80);
  }
  
  base_gx /=loop;
  base_gy /=loop;
  base_gz /=loop;

  
}
void calculateAngle()
{
  dt = (millis()-pre_msec)/1000.0;
  pre_msec = millis();
  
 CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
 
  //가속도값 아크탄젠트->각도변환
  accel_x = atan(ay/sqrt(pow(ax,2) + pow(az,2)))*RADIANS_TO_DEGREES;
  accel_y = atan(-1*ax/sqrt(pow(ay,2) + pow(az,2)))*RADIANS_TO_DEGREES;
  
 
  
  //자이로 -32766~+32766을 실제 250degree/s로 변환
  //앞에서 계산한 오차의 평균값을 빼줌  
  gyro_x = (gx-base_gx)/fs;  
  gyro_y = (gy-base_gy)/fs;
 
  //변화량을 적분 
  gyro_angle_x = angle_x + dt*gyro_x;
  gyro_angle_y = angle_y + dt*gyro_y;
 
  //상보필터
  angle_x = 0.95*gyro_angle_x + 0.05*accel_x ;
  angle_y = 0.95*gyro_angle_y + 0.05*accel_y ;

  

  LPF_angle_x = (tau*LPF_prevangle_x + dt*angle_x)/(tau+dt) ;
  LPF_angle_y = (tau*LPF_prevangle_y + dt*angle_y)/(tau+dt) ;
  LPF_prevangle_x = LPF_angle_x;
  LPF_prevangle_y = LPF_angle_y ;

  LPF_gyro_x = (tau*LPF_prev_gyro_x + dt*gyro_x)/(tau+dt) ;
  LPF_gyro_y = (tau*LPF_prev_gyro_y + dt*gyro_y)/(tau+dt) ;
  LPF_prev_gyro_x = LPF_gyro_x;
  LPF_prev_gyro_y = LPF_gyro_y ;

}

void pidControl() {
   dt = (millis()-pre_msec)/1000.0;
  pre_msec = millis();

  /*
P_error = desired_angle_x - LPF_angle_x; //각도에러 = 목표각도 - 현재각도
P_RateError = (P_error*stabilizePgain) - LPF_gyro_x; // 각속도에러 = (각도에러*Pgain) - 현재각속도
 
 
Pitch_P = P_RateError*Kp;
Pitch_I += (P_RateError*dt)*Ki;
Pitch_D = ((P_RateError - P_Pre_RateError)/dt)*Kd;
P_Pre_RateError = P_RateError;

//LPF_Pitch_D = (tau*LPF_prev_Pitch_D + dt*Pitch_D)/(tau*dt);
//LPF_prev_Pitch_D = LPF_Pitch_D;
 

Pitch_PID = Pitch_P + Pitch_I +Pitch_D;
Pitch_PID = constrain(Pitch_PID, outMin, outMax);
*/
/////// //Pitch PID
  P_error = desired_angle_x - LPF_angle_x;
  Pitch_P = PKp * P_error;
  Pitch_I += PKi * P_error * dt;
  Pitch_D = PKd * (LPF_angle_x - lastangle_x)*10;

  Pitch_PID = Pitch_P + Pitch_I + Pitch_D;
  Pitch_PID = constrain(Pitch_PID, outMin, outMax);


  R_error = desired_angle_y - LPF_angle_y;
  Roll_P = RKp * R_error;
  Roll_I += RKi * R_error * dt;
  Roll_D = RKd * (LPF_angle_y - lastangle_y);

  Roll_PID = Roll_P + Roll_I + Roll_D;
  Roll_PID = constrain(Roll_PID, outMin, outMax);

  lastangle_x = LPF_angle_x;
  lastangle_y = LPF_angle_y;
/*
R_error = desired_angle_y - LPF_angle_y; //각도에러 = 목표각도 - 현재각도
R_RateError = (R_error*stabilizePgain) - LPF_gyro_y; // 각속도에러 = (각도에러*Pgain) - 현재각속도
 
 
Roll_P = R_RateError*Kp;
Roll_I += (R_RateError*dt)*Ki;
Roll_D = ((R_RateError - R_Pre_RateError)/dt)*Kd;
R_Pre_RateError = R_RateError;

//LPF_Pitch_D = (tau*LPF_prev_Pitch_D + dt*Pitch_D)/(tau*dt);
//LPF_prev_Pitch_D = LPF_Pitch_D;
 
Roll_PID = Roll_P + Roll_I + Roll_D;
Roll_PID = constrain(Roll_PID, outMin, outMax);
*/
   /*
  P_error = desired_angle_x - angle_x;
  rate_cmd = P_error * Kp + total_intg;
  rate_error = rate_cmd - gyro_x;
  Pitch_PID = rate_error*Kd;
  Pitch_PID = constrain(Pitch_PID, outMin, outMax);
  total_intg = P_error* dt * Ki;



 
  error = anlge_cmd(명령할 각도) - sesor_angle(가속도 센서서 받아온 실제 각도) // 각도 오차
rate_cmd(명령할 각속도) = error * Kp + total_intg(전체 적분값)    // PI 제어
rate_error = rate_cmd - sesor_rate(자이로 센서서 받아온 실제 각속도) // 각속도 오차
moment = rate_error * Kd  // D 제어
total_intg = error * step * Ki // 전체 적분값 업데이트

  //Pitch PID
   P_error = desired_angle_x - angle_x;
  Pitch_P = Kp * P_error;
  Pitch_I += Ki * P_error * dt;
  Pitch_D = Kd * (angle_x - lastangle_x)/dt;

  Pitch_PID = Pitch_P + Pitch_I + Pitch_D;
  Pitch_PID = constrain(Pitch_PID, outMin, outMax);
 

  //Roll PID
  R_error = desired_angle_y - angle_y;
 // ROLL_RateError = (R_error * kpp) + gyro_y;
  Roll_P = Kp * R_error;
  Roll_I += Ki * R_error * dt;
  Roll_D = Kd * (angle_y - lastangle_y)/dt;

  Roll_PID = Roll_P + Roll_I + Roll_D;
  Roll_PID = constrain(Roll_PID, outMin, outMax);

  lastangle_x = angle_x;
  lastangle_y = angle_y;
*/
/*
P_error = desired_angle_x - angle_x; //각도에러 = 목표각도 - 현재각도
RateError = (P_error*stabilizePgain) - gyro_x; // 각속도에러 = (각도에러*Pgain) - 현재각속도
 
 
Pitch_P = RateError*Kp;
Pitch_I += (RateError*dt)*Ki;
Pitch_D = ((RateError - Pre_RateError)/dt)*Kd;
 
Pre_RateError = RateError;
Pitch_PID = Pitch_P + Pitch_I + Pitch_D;
Pitch_PID = constrain(Pitch_PID, outMin, outMax);
*/
}


void motorControl(){
  motorFSpeed = output + Roll_PID;
  motorBSpeed = output - Roll_PID;
  motorLSpeed = (output + Pitch_PID);
  motorRSpeed = (output - Pitch_PID);
  //bldc_a.write(output);


  if(motorLSpeed > 46){
    if(motorRSpeed > 46){
  
  bldc_b.write(motorFSpeed);
  bldc_c.write(motorBSpeed);
  bldc_a.write(motorRSpeed);
  bldc_d.write(motorLSpeed);
   } else {
    bldc_a.write(0);
    bldc_b.write(0);
    bldc_c.write(0);
    bldc_d.write(0);
  }
  }else {
    bldc_a.write(0);
    bldc_b.write(0);
    bldc_c.write(0);
    bldc_d.write(0);
  }
}
void setup() {

  
   CurieIMU.begin();
  Serial.begin(57600);
   BTSerial.begin(57600);
  bldc_a.attach(10, 1000, 2000); //ccw ,R, GreenPin
//  bldc_b.attach(11, 1000, 2000); //cw, F, WhitePin
//  bldc_c.attach(4, 1000, 2000);  //cw, B, YellowPin
  bldc_d.attach(6, 1000, 2000);  //ccw ,L, OrangePin
  calibrate();

//  Serial.setTimeout(50); 
  bldc_a.write(0);
  bldc_b.write(0);
  bldc_c.write(0);
  bldc_d.write(0);
  
 Serial.println("wait. press 'b'");
 while(1){
  if (Serial.available()) {
    if(Serial.read() == 'b'){
      break;
  }
  }
 }
  Serial.print("output:");
  Serial.println(output);


}



void loop() {
  //f가더 쌤

float times = (millis()-pre_msec);
  pre_msecs = millis();
  calculateAngle();
  pidControl();
  motorControl();
  /*
  Serial.println(angle_x);
  Serial.println(LPF_angle_x);
    Serial.println("###############");
    */

  if (Serial.available()) {
    data= Serial.read();
    switch((char)data){
      case 's':
      output = 55;
      Serial.print("output = ");
      Serial.println(output);
      break;
      case 'q':
      output = 0;
      Serial.print("output = ");
      Serial.println(output);
      break;
      case 't':
      output += 5;
      Serial.print("output = ");
      Serial.println(output);
      break;
      case 'v':
      output -= 5;
      Serial.print("output = ");
      Serial.println(output);
      break;
      case 'a':
      desired_angle_x -= 10;
      Serial.print("desired_angle_x = ");
      Serial.println(desired_angle_x);
      break;
      case 'l':
      desired_angle_x += 10;
      Serial.print("desired_angle_x = ");
      Serial.println(desired_angle_x);
      break;
      case 'p':
      PKp += 0.01;
      Serial.print("PKp = ");
      Serial.println(PKp);
      break;
      case 'b':
      PKp -= 0.01;
      Serial.print("PKp = ");
      Serial.println(PKp);
      break;
      case 'z':
      PKd += 0.01;
      Serial.print("PKd = ");
      Serial.println(PKd);
      break;
      case 'c':
      PKi += 0.01;
      Serial.print("PKi = ");
      Serial.println(PKi);
      break;
      default:
      break;
    }
  }
  
  
/*
  Serial.print(LPF_angle_x);
  Serial.print("####");
  Serial.print(times);
  Serial.print("####");
  Serial.print(PKp);
  Serial.print("####");
  Serial.print(PKd);
  Serial.print("####");
  Serial.print(PKi);
  Serial.print("####");
  Serial.println(Pitch_PID);
*/
}



/*     while(1)
      {
        char a = BTSerial.read();
        if(a == ','){
            if(count == 0){
            pitch =Stemp.toInt(); 
            count = 1;
          } else if(count == 1){
          roll =Stemp.toInt();
           count = 0;
          }
          Stemp = "";
        }else if(a == '*'){
          throttle = Stemp.toInt();
          //Serial.println(pitch);
          //Serial.println(roll);
          //Serial.println(throttle);
          Stemp = "";
          break;
        } else{
          Stemp.concat(a);
        }
      }
      */
