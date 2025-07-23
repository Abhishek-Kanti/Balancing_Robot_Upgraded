#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro

int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
float acc_calibration_value_x, acc_calibration_value_y, acc_calibration_value_z;        //Enter the accelerometer calibration value
float acc_raw_x, acc_raw_y, acc_raw_z;
float acc_x, acc_y, acc_z;
float acc_angle, calibration_angle;

//Various settings
float pid_p_gain = 11.0;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 0.7;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 4.0;                                       //Gain setting for the D-controller (30)
float turning_speed = 30;                                    //Turning speed (20)
float max_target_speed = 150;                                //Max target speed (100)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

#define m0x 4                       //motor driver 1 step pins. will be setting them in void setup for quater stepping.
#define m1x 3
#define m2x 2

#define m0y 7                      //motor driver 2 step pins. will be setting them in void setup for quater stepping.
#define m1y 8
#define m2y 9

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(9600);                                                       //Start the serial port at 9600 kbps
  Wire.begin();                                                             //Start the I2C bus as master
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode
  
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 

  pinMode(5, OUTPUT);                                                       //Configure digital port 5 as output for dir pin
  pinMode(6, OUTPUT);                                                       //Configure digital port 6 as output for step pin
  pinMode(10, OUTPUT);                                                       //Configure digital port 10 as output for dir pin
  pinMode(11, OUTPUT);                                                       //Configure digital port 11 as output for step pin
  pinMode(13, OUTPUT);                                                      //Configure digital port 6 as output

  pinMode(m0x, OUTPUT);
  pinMode(m1x, OUTPUT);
  pinMode(m2x, OUTPUT);

  pinMode(m0y, OUTPUT);
  pinMode(m1y, OUTPUT);
  pinMode(m2y, OUTPUT);

  digitalWrite(m0x, 0);
  digitalWrite(m1x, 1);
  digitalWrite(m2x, 0);

  digitalWrite(m0y, 0);
  digitalWrite(m1y, 1);
  digitalWrite(m2y, 0);
//  Serial.print("Calibrating gyro...");

  for(receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    if(receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));        //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer
    delayMicroseconds(1850);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset

//  Serial.println("");
//  Serial.print("Calibrating Accel...");
  
for(receive_counter = 0; receive_counter < 500; receive_counter++){         //Create 500 loops
    Wire.beginTransmission(gyro_address);                                   //Start communication with the Accel
    Wire.write(0x3B);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 6);                                      //Request 2 bytes from the Accel
    acc_calibration_value_x += Wire.read()<<8|Wire.read();                  //Combine the two bytes to make one integer
    acc_calibration_value_y += Wire.read()<<8|Wire.read();                  //Combine the two bytes to make one integer
    acc_calibration_value_z += Wire.read()<<8|Wire.read(); 
    delayMicroseconds(1850);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  acc_calibration_value_x /= 500;                                           //Divide the total value by 500 to get the avarage gyro offset
  acc_calibration_value_y /= 500;    
  acc_calibration_value_z /= 500; 

  acc_calibration_value_z = acc_calibration_value_z/8200.0;
  calibration_angle = acos(acc_calibration_value_z)*57.29;

  loop_timer = micros() + 4000;                                           //Set the loop_timer variable at the next end loop time
}

void loop() {
  if(Serial.available()){
    received_byte = Serial.read();
    receive_counter = 0;
  }
  if(receive_counter <= 25)receive_counter++;
  else received_byte = 0x00;
  
  //put your main code here, to run repeatedly:
  Wire.beginTransmission(gyro_address);                                   //Start communication with the Accel
  Wire.write(0x3B);                                                       //Start reading the Who_am_I register 75h
  Wire.endTransmission();                                                 //End the transmission
  Wire.requestFrom(gyro_address, 6);                                      //Request 2 bytes from the Accel
  acc_raw_x = Wire.read()<<8|Wire.read();                                 //Combine the two bytes to make one integer
  acc_raw_y = Wire.read()<<8|Wire.read(); 
  acc_raw_z = Wire.read()<<8|Wire.read(); 
  
  acc_x = acc_raw_x/8192.0;
  acc_y = acc_raw_y/8192.0;
  acc_z = acc_raw_z/8192.0;
    
  if(acc_z > 1) acc_z=1;
  if(acc_z < -1) acc_z=-1;
  acc_angle = (atan2(acc_x, acc_z)*57.29) - calibration_angle;

  if(start == 0 && acc_angle > -0.5 && acc_angle < 0.5){
    angle_gyro = acc_angle;
    start = 1;
  }

//  Wire.beginTransmission(gyro_address);                                   //Start communication with the Accel
//  Wire.write(0x3F);                                                       //Start reading the Who_am_I register 75h
//  Wire.endTransmission();                                                 //End the transmission
//  Wire.requestFrom(gyro_address, 2);                                      //Request 2 bytes from the Accel
//  acc_raw_z = Wire.read()<<8|Wire.read();                                 //Combine the two bytes to make one integer
//  acc_raw_z += acc_calibration_value_z;
//  
//  if(acc_raw_z > 8200) acc_raw_z = 8200;                                  // acc_z = acc_raw_z/8192.0; (approximating 8192 to 8200)
//  if(acc_raw_z < -8200) acc_raw_z = -8200;
//
//  acc_angle = asin((float)acc_raw_z/8200.0)*57.296;


  Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
  Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
  Wire.endTransmission();                                                 //End the transmission
  Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
  gyro_yaw_data_raw = Wire.read()<<8|Wire.read();                         //Combine the two bytes to make one integer
  gyro_pitch_data_raw = Wire.read()<<8|Wire.read();                       //Combine the two bytes to make one integer

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;
  angle_gyro += -(gyro_pitch_data_raw * 0.000121);

  angle_gyro =  angle_gyro*0.9996 + acc_angle*0.0004;
  //angle_gyro =  angle_gyro*0.998 + acc_angle*0.002;

  ////////////////////////////////////////////////////////////////////////////////////////
  //PID-Controller calculations
  ////////////////////////////////////////////////////////////////////////////////////////
  
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015;

  pid_i_mem += pid_error_temp*pid_i_gain;
  if(pid_i_mem > 400)pid_i_mem = 400;
  else if(pid_i_mem < -400)pid_i_mem = -400;
  
  pid_output = pid_p_gain*(pid_error_temp) + pid_i_mem + pid_d_gain*(pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = 400;
  else if(pid_output < -400)pid_output = -400;
  
  pid_last_d_error = pid_error_temp;

  //Handle the nervous behavior of the robot
  if(pid_output < 5 && pid_output > -5) pid_output=0;

  //Handle if the robot is falling
  if(angle_gyro > 30 || angle_gyro < -30 || start == 0){
    pid_output = 0;
    pid_i_mem = 0;
    start = 0;
    self_balance_pid_setpoint = 0;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  //RC-Control
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  pid_output_left = pid_output;
  pid_output_right = pid_output;

  if(received_byte & B00000001){                        //right
    pid_output_left += turning_speed;
    pid_output_right -= turning_speed;
  }

  if(received_byte & B00000010){                        //left
    pid_output_left -= turning_speed;
    pid_output_right += turning_speed;
  }

  if(received_byte & B00000100){                        //forward
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;
  }

  if(received_byte & B00001000){                        //backward
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;
    if(pid_output < max_target_speed)pid_setpoint += 0.005;
  }  

  if(received_byte & B00001100){                        // slowly reduce the setpoint to zero if no forward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -= 0.05;
    else if(pid_setpoint < -0.5)pid_setpoint += 0.05;
    else pid_setpoint = 0;
  }  
  
  //slowly come to the 0 setpoint if not already
  if(pid_setpoint == 0){
    if(pid_output < 0) self_balance_pid_setpoint += 0.0015; 
    if(pid_output > 0) self_balance_pid_setpoint -= 0.0015; 
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  //compensating for the non linear behavior of the stepper motor
  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //calculating the needed pulse time for the left and right motor
  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  //Assigning the calculated pulse time to the variable in ISR
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;
    
  Serial.print(95);
  Serial.print(",");
  Serial.print(-95);
  Serial.print(",");
  Serial.println(angle_gyro);
  //Serial.println(acc_angle);

  while(loop_timer > micros());
  loop_timer += 4000;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////TIMER2_COMPA_vect////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

ISR(TIMER2_COMPA_vect) {
  //Left motor pulses 
  throttle_counter_left_motor ++;                                       //Increase CC_Speed_Left_Motor by 1 every time this routine is executed
  if (throttle_counter_left_motor > throttle_left_motor_memory) {       //If the number of loops is larger then the Left_Motor_Speed_Prev variable
    throttle_counter_left_motor = 0;                                    //Reset the CC_Speed_Left_Motor variable
    throttle_left_motor_memory = throttle_left_motor;                   //Load the next Left_Motor_Speed variable
    if (throttle_left_motor_memory < 0) {                               //If the Left_Motor_Speed_Prev is negative
      PORTD &= 0b11011111;                                              //Set D5 low. Reverse  direction
      throttle_left_motor_memory *= -1;                                 //Invert the Left_Motor_Speed_Prev variable
    }
    else PORTD |= 0b00100000;                                           //Set output D5 high. Forward direction.
  }
  else if (throttle_counter_left_motor == 1)PORTD |= 0b01000000;        //Set output D6 high to create a pulse for the stepper
  else if (throttle_counter_left_motor == 2)PORTD &= 0b10111111;        //Set output D6 low because the pulse only has to last for 20us


  //Right motor pulses
 throttle_counter_right_motor ++;                                       //Increase CC_Speed_Right_Motor by 1 every time the routine is executed
  if (throttle_counter_right_motor > throttle_right_motor_memory) {     //If the number of loops is larger then the Left_Motor_Speed_Prev variable
    throttle_counter_right_motor = 0;                                   //Reset the CC_Speed_Left_Motor variable
    throttle_right_motor_memory = throttle_right_motor;                 //Load the next Left_Motor_Speed variable
    if (throttle_right_motor_memory < 0) {                              //If the Left_Motor_Speed_Prev is negative
      PORTB &= 0b11111011;                                              //Set D10 low. Reverse  direction
      throttle_right_motor_memory *= -1;                                //Invert the Left_Motor_Speed_Prev variable
    }
    else PORTB |= 0b00000100;                                           //Set output D10 high. Forward direction.
  }
  else if (throttle_counter_right_motor == 1)PORTB |= 0b00001000;        //Set output D11 high to create a pulse for the stepper
  else if (throttle_counter_right_motor == 2)PORTB &= 0b11110111;        //Set output D11 low because the pulse only has to last for 20us
}//End of timer routine
