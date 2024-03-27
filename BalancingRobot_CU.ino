#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro

int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
float acc_calibration_value_x, acc_calibration_value_y, acc_calibration_value_z;        //Enter the accelerometer calibration value
float acc_raw_x, acc_raw_y, acc_raw_z;
float acc_x, acc_y, acc_z;
float acc_angle, calibration_angle;

//Various settings
float pid_p_gain = 15;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                       //Gain setting for the D-controller (30)
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(9600);                                                       //Start the serial port at 9600 kbps
  Wire.begin();                                                             //Start the I2C bus as master
//  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz
//
//  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
//  //This subroutine is called TIMER2_COMPA_vect
//  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
//  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
//  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
//  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
//  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
//  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode
  
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

  pinMode(2, OUTPUT);                                                       //Configure digital poort 2 as output
  pinMode(3, OUTPUT);                                                       //Configure digital poort 3 as output
  pinMode(4, OUTPUT);                                                       //Configure digital poort 4 as output
  pinMode(5, OUTPUT);                                                       //Configure digital poort 5 as output
  pinMode(13, OUTPUT);                                                      //Configure digital poort 6 as output

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
//    Serial.println(acc_calibration_value_z);
  }
  acc_calibration_value_x /= 500;                                           //Divide the total value by 500 to get the avarage gyro offset
  acc_calibration_value_y /= 500;    
  acc_calibration_value_z /= 500; 

  acc_calibration_value_z = acc_calibration_value_z/8200.0;
  calibration_angle = acos(acc_calibration_value_z)*57.29;
//  Serial.print(calibration_angle);
  
//  Serial.println("");
//  Serial.print("Calibration gyro_pitch = ");
//  Serial.print(gyro_pitch_calibration_value);
//  Serial.print(" | ");
//  Serial.print("acc_calibration_value_z = ");
//  Serial.println(acc_calibration_value_z);

  loop_timer = micros() + 4000;                                           //Set the loop_timer variable at the next end loop time
}

void loop() {
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
  acc_z = acc_raw_z/8200.0;
    
  if(acc_z > 1) acc_z=1;
  if(acc_z < -1) acc_z=-1;
  acc_angle = (acos(acc_z)*57.29) - calibration_angle;

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

  angle_gyro =  angle_gyro*0.998 + acc_angle*0.002;

  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015;

  pid_i_mem += pid_error_temp*pid_i_gain;

  pid_output = pid_p_gain*(pid_error_temp) + pid_i_mem + pid_d_gain*(pid_error_temp - pid_last_d_error) 

  pid_last_d_error = pid_error_temp;
  
//  Serial.print("accel_angle = ");
//  Serial.println(acc_angle);

  Serial.print(95);
  Serial.print(",");
  Serial.print(-95);
  Serial.print(",");
  Serial.println(angle_gyro);

//  Serial.print("accel_x = ");
//  Serial.print(acc_x);
//  Serial.print(" | ");
//  Serial.print("accel_y = ");
//  Serial.print(acc_y);
//  Serial.print(" | ");
//  Serial.print("accel_z = ");
//  Serial.println(acc_z);

  while(loop_timer > micros());
  loop_timer += 4000;
}
