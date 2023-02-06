

///////////////////////////////////////////////////////////////////////////////////////

// reference: http://www.brokking.net/ymfc-al_main.html?fbclid=IwAR34dg2kMHGb4VirsuUn6qFi5bB4ugDmev0Pu3L5xdhLqDfhqhbdbI7lJcI
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM


////////////// biến thay đổi PID từ winform ////////////////
boolean stringComplete = false;  // whether the string is complete
String mySt = "";
uint8_t inChar;
String check_balance;
//==========

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.4;               // Gain setting for the roll P-controller 
float pid_i_gain_roll = 0.06;              // Gain setting for the roll I-controller   
float pid_d_gain_roll = 13.8;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. 
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. 
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
int  correction_angle=15;

 float pid_p_gain_altitude = 1.4;           //Gain setting for the altitude P-controller (default = 1.4). (changing for later on the winform c#)
 float pid_i_gain_altitude = 0.3;           //Gain setting for the altitude I-controller (default = 0.2).(changing for later on the winform c#)
 float pid_d_gain_altitude = 1.35;          //Gain setting for the altitude D-controller (default = 0.75).(changing for later on the winform c#)
float stable_altitude = 3.0;               // set fixed altitude for holding function at the power up.(changing for later on the winform c#)
 int throttle_altitude=1630;

int pid_max_altitude = 400;                  //Maximum output of the PID-controller (+/-).
// we should init ad_roll/pitch = 0 at the first time to see the error of angle_roll/pithc_acc when it run
float add_roll = 0;                          // to calibrate the angle_roll_acc 
float add_pitch =3.5;                        // to calibrate the angle_pitch_acc 

int pid_max_GPS = 250;                     // it is related to the tilt of Quadcopter when it fly to waypoints in the autoflight function 
float gps_p_gain = 3.1;                    //Gain setting for the GPS P-controller (default = 2.7).
float gps_d_gain = 7.5;                    //Gain setting for the GPS D-controller (default = 6.5).
double  gps_i_gain = 0.001;
boolean auto_level = true;                 //Auto level on (true) or off (false)
//===============TELEMETRY==========================
 float kp_set,ki_set,kd_set,altitude_set;
 String readData1,data_buffer1,data_buffer2,data_buffer3,data_buffer4,data_buffer5,data_buffer6,data_buffer7,data_buffer8;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
byte eeprom_data[52];
uint8_t error,telemetry_loop_counter,telemetry_send_byte,check_byte;
uint32_t telemetry_buffer_byte;
int debug_function=0;
byte highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter, timer_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage,raw_battery_total,raw_baterry_total,raw_batterty_rotating_memory[6],battery_mem_location;
int cal_int, start, gyro_address;
int receiver_input[5];
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;
int temperature;
long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, roll_setpoint_input, pitch_setpoint_input, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
boolean gyro_angles_set;


//=========Pressure variables==============

float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P, T;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total, temper;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;

//==========Altitude PID variables============

float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_average;
uint8_t pressure_rotating_mem_location;
uint8_t manual_altitude_change;
uint8_t flight_mode;
int16_t manual_throttle;
float pressure_rotating_mem_actual,altitude_quad;

//=====Adress of MS5611=====
uint8_t MS5611_address = 0x77;

//===========GPS variables==============
uint32_t raw_lat_total,raw_lat_rotating_memory[4],lat_mem_location;
uint8_t read_serial_byte, incomming_message[100], number_used_sats,previous_sats, fix_type;
uint8_t waypoint_set, latitude_north, longtitude_east ;
uint16_t message_counter;
int16_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add, pid_i_mem_lat,pid_i_mem_lon;
uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
uint8_t gps_rotating_mem_location;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
uint32_t gps_watchdog_timer;
uint8_t check_waypoints;
uint32_t lat_list[6];
uint32_t lon_list[6];
uint32_t lat_home_return;
uint32_t lon_home_return;
uint8_t wp_list,waypoint_mode, completed_wp, emergency_return;
uint8_t time_to_change_wp=10;

//===========compass variables============
        
uint8_t QMC_address = 0x0D; 
uint8_t compass_calibration_on, heading_lock;
int16_t compass_x, compass_y, compass_z;
int32_t compass_x_total_average,compass_x_rotating_mem[30],compass_y_total_average,compass_y_rotating_mem[30],compass_z_total_average,compass_z_rotating_mem[30];
int8_t compass_x_rotating_mem_location,compass_y_rotating_mem_location,compass_z_rotating_mem_location;
int16_t compass_cal_values[6],compass_x_max,compass_x_min,compass_y_max,compass_y_min,compass_z_max,compass_z_min;
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading,heading_horizontal;
float compass_scale_y, compass_scale_x;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
float course_lock_heading, heading_lock_course_deviation;
float declination = -0.73;                   //Set the declination between the magnetic and geographic north.
String direction;
//================auto signal
String read_command;
int readsignal,takeoff_detected;
int time_to_landing, landing_detected, flytowp_detected, time_to_flywp;
//===============
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial3.begin(57600);
  Serial1.begin(38400); // telemetry

  //======Copy the EEPROM data for fast access data.===========
  for (start = 0; start <= 51; start++)eeprom_data[start] = EEPROM.read(start);
  start = 0;                                                                //Set start back to zero.
  gyro_address = eeprom_data[32];
 
 
  //==========================================
  Wire.begin();                                                             //Start the I2C as master.
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz.

  //===== ARDUINO MEGA
  DDRB |= B11110000;                                                         //Configure digital port as output:10,11,12,13: PB4-PB7 
  DDRH |= B01111000;                                                         //Configure digital port as output:(PORDH:PH3,PH4,PH5) 6,7,8 : used for Led
  testled();


  digitalWrite(6, HIGH);                                                   //Turn on the warning led.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while (eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program
  if (eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  set_gyro_registers();                                                     //Set the specific gyro registers.
  setup_compass();
  read_compass();
  angle_yaw =heading_horizontal; 

  for (cal_int = 0; cal_int < 1250 ; cal_int ++) {                          //Wait 5 seconds before continuing.
    PORTB |= B11110000;                                                     //Set digital port 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTB &= B00001111;                                                     //Set digital port 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
    
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                          //Take 2000 readings for calibration.
    if (cal_int % 15 == 0)digitalWrite(6, !digitalRead(6));               //Change the led status to indicate calibration.
    gyro_signalen();                                                        //Read the gyro output.
     gyro_roll_cal  += gyro_roll;                                             //Ad roll value to gyro_roll_cal.
     gyro_pitch_cal += gyro_pitch;                                           //Ad pitch value to gyro_pitch_cal.
     gyro_yaw_cal   += gyro_yaw;                                             //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTB |= B11110000;                                                     //Set digital port 10, 11, 12 and 13 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTB &= B00001111;                                                     //Set digital poort 10, 11, 12 and 13 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_roll_cal  /= 2000;                                                 //Dvide the roll total by 2000.
  gyro_pitch_cal /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_yaw_cal   /= 2000;                                                 //Divide the yaw total by 2000.

  PCICR |= (1 << PCIE2);                                                    //Set PCIE2 to enable PCMSK2 scan.
  PCMSK2 |= (1 << PCINT18);                                                  //Set PCINT18 (A10) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT19);                                                  //Set PCINT19 (A11)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT20);                                                  //Set PCINT20 (A12)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT21);                                                  //Set PCINT21 (A13)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT22);                                                  //Set PCINT22 (A14)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT23);                                                  //Set PCINT23 (A15)to trigger an interrupt on state change.

                                              //Set PCINT4 (digital input 10(uno 12)to trigger an interrupt on state change.

  //Wait until the receiver is active and the throtle is set to the lower position.
  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400) {
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTB |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTB &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if (start == 125) {                                                     //Every 125 loops (500ms).
      digitalWrite(6, !digitalRead(6));                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  start = 0;                                                                //Set start back to 0.
  //===========
  //The voltage divider (R2 and R3 on the schematic) lower the voltage for the analog input.
  //This analog input can handle a maximum input voltage of 5V.
  //((12.6V - 0.6V) / (1000Ω + 1500Ω)) * 1000Ω = 4.8V
  //0,6 is the voltage drop of diode
  //=======================================================================

  //Load the battery voltage to the battery_voltage variable.
  //121 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  // used for telemetry
     battery_voltage = (analogRead(0)+121)*1.2317;
  
  //======================================================================
  //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
  //These 2 byte values are stored in the memory location 0xA2 and up.
  delay(10);

  for (start = 1; start <= 6; start++) {
    Wire.beginTransmission(MS5611_address);                    //Start communication with the MPU-6050.
    Wire.write(0xA0 + start * 2);                              //Send the address that we want to read.
    Wire.endTransmission();                                    //End the transmission.

    Wire.requestFrom(MS5611_address, 2);                       //Request 2 bytes from the MS5611.
    C[start] = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the C[x] calibration variable.
  }

  OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
  SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

  //The MS5611 needs a few readings to stabilize.
  for (start = 1; start <= 100; start++) {                       //This loop runs 100 times.
    read_barometer();                                           //Read and calculate the barometer data.
    // ground_pressure=actual_pressure;
    temper = (float)T / 100.00;

    PORTB |= B11110000;                                                     //Set digital poort 10, 11, 12 and 13 high
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTB &= B00001111;                                                     //Set digital poort 10, 11, 12 and 13 low
    delayMicroseconds(1000);
    delayMicroseconds(6400);                                                
  }
   actual_pressure = 0;              
  
  //Set the timer for the next loop.
  start = 0;
  //When everything is done, turn off the led.  */
  digitalWrite(6, LOW);                                                    //Turn off the warning led.
  loop_timer = micros();
}
/*void checkSettings()
  {
  Serial.print("Oversampling: ");
  Serial.println(ms5611.getOversampling());
  } */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() { 
      
   //============ set theo kp,ki,kd,altitude nhận từ telemetry ==================
  if (start == 0) 
  {
      if (receiver_input_channel_1 > 1900 && receiver_input_channel_2 < 1100 && receiver_input_channel_3 > 1900 && receiver_input_channel_4 > 1900)
    {
      calibrate_compass();
    }
    if(receiver_input_channel_2 >=1600){
  

    uint8_t counter = 0;
    while(Serial1.available()) 
      {

      String readData;
      
      //if(debug_function==0)
          readData = Serial1.readStringUntil('\n'); // đọc giá trị gửi đến cho đến khi gặp kí tự xuống dòng \n
          debug_function = readData.toInt();
            

      }
    }
    if(receiver_input_channel_2 <=1300){
        while(Serial1.available()) 
         
       { 
           readData1 = Serial1.readStringUntil('\n');
           data_buffer1   =  splitString(readData1, "/", 0);
           data_buffer2   =  splitString(readData1, "/", 1);
           data_buffer3   =  splitString(readData1, "/", 2);
           data_buffer4   =  splitString(readData1, "/", 3);
           data_buffer5   =  splitString(readData1, "/", 4);
           data_buffer6   =  splitString(readData1, "/", 5);
           data_buffer7   =  splitString(readData1, "/", 6);
           data_buffer8   =  splitString(readData1, "/", 7);
           

        
        if(debug_function == 2)
         {
                   
          stable_altitude   = data_buffer1.toFloat();
          time_to_change_wp = data_buffer2.toInt();
          
          }
         if(debug_function == 3)
         {
        
                
         lat_list[1] = data_buffer1.toInt();    
         lat_list[2] = data_buffer2.toInt(); 
         lat_list[3] = data_buffer3.toInt(); 
         lat_list[4] = data_buffer4.toInt();
         //==== lONGTITUDE 
         lon_list[1] = data_buffer5.toInt();    
         lon_list[2] = data_buffer6.toInt(); 
         lon_list[3] = data_buffer7.toInt(); 
         lon_list[4] = data_buffer8.toInt();
         
         }  
          
    }
    }

         
 }

 while(Serial1.available())
 {
  read_command = Serial1.readStringUntil('\n'); 
  readsignal = read_command.toInt();
  if(readsignal<4)
  {
    debug_function = readsignal;
  }
 }
 /*
  if(loop_counter == 0)Serial.print("readsignal:");
  if(loop_counter == 1)Serial.println(readsignal);
  if(loop_counter == 2)Serial.print("flight_mode:");
  if(loop_counter == 3)Serial.println(flight_mode);
  if(loop_counter == 4)Serial.print("Throttle:");
  if(loop_counter == 5)Serial.println(throttle);
   if(loop_counter == 6)Serial.print("ground_pressure");
  if(loop_counter == 7)Serial.println(ground_pressure);
  if(loop_counter == 8)Serial.print("actual_pressure");
  if(loop_counter == 9)Serial.println(actual_pressure);
  if(loop_counter == 10)Serial.print("throttle_altitude");
  if(loop_counter == 11)Serial.println(throttle_altitude);
  if(loop_counter == 12)Serial.print("altitude:");
  if(loop_counter == 13)Serial.println(altitude_quad);
  if(loop_counter == 14)Serial.print("start:");
  if(loop_counter == 15)Serial.println(start);
  if(loop_counter == 16)Serial.print("takeoff_detected:");
  if(loop_counter == 17)Serial.println(takeoff_detected);
  if(loop_counter == 18)Serial.print("manual_throttle");
  if(loop_counter == 19)Serial.println(manual_throttle);
  if(loop_counter == 20)Serial.print("flytowp_detected:");
  if(loop_counter == 21)Serial.println(flytowp_detected);
 
  if(loop_counter == 22)Serial.print("landing_detected:");
  if(loop_counter == 23)Serial.println(landing_detected);
  
 if(loop_counter == 24)Serial.println("=====================");
 */
 
  
       
       
 
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  read_barometer();
  read_compass();
  if (gps_add_counter >= 0)gps_add_counter --;
  read_gps();

  //=========== flight mode ====================
   // heading_lock = 0;
  if(receiver_input_channel_6>=1600)heading_lock =1;
  if (receiver_input_channel_6 >= 1350 ) flight_mode = 3;
  else if(receiver_input_channel_6<=1200 && start == 0) flight_mode=1;
  
 
  if (flight_mode == 3)digitalWrite(7, 1);
  else(digitalWrite(7, 0));
 //  if (flight_mode == 3 && receiver_input_channel_5>=1600)digitalWrite(7, 0);
  
  //=======Gyro angle calculations(150hz)================= 
  angle_pitch += gyro_pitch * 0.0001018;                                           //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0001018;
  angle_yaw += gyro_yaw * 0.0001018;
  
  //0.000001708 = 0.0001053 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001776);                         //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001776);                         //If the IMU has yawed transfer the pitch angle to the roll angel.
  angle_yaw -= course_deviation(angle_yaw, heading_horizontal) / 1200.0;           //Calculate the difference between the gyro and compass heading and make a small correction.
  if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (angle_yaw >= 360) angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x / acc_total_vector) *  57.296;      //Calculate the roll angle.
  }
 // angle_pitch_acc -= 3.4;   //updated 21/7/2022                                                //Accelerometer calibration value for pitch.
 // angle_roll_acc -= 1.5;
 
  angle_pitch_acc -= add_pitch;   
  angle_roll_acc -= add_roll;

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
// angle_pitch = angle_pitch * 0.9980 + angle_pitch_acc * 0.0020;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
// angle_roll = angle_roll * 0.9980 + angle_roll_acc * 0.0020;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  //======================= */

   pitch_level_adjust = angle_pitch * correction_angle;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * correction_angle;                                      //Calculate the roll angle correction
  roll_setpoint_input = receiver_input_channel_1;                            //Normally channel_1 is the pid_roll_setpoint input.
  pitch_setpoint_input = receiver_input_channel_2;                           //Normally channel_2 is the pid_pitch_setpoint input.

  
 
//===============================================================
  if (!auto_level) 
 {   //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }

 // 
  //For starting the motors: throttle low and yaw left (step 1).


//======================= set heading lock================
if(heading_lock ==1 ) {
    heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
    roll_setpoint_input = 1500 + ((float)(receiver_input_channel_1 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(receiver_input_channel_2 - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
    pitch_setpoint_input = 1500 + ((float)(receiver_input_channel_2 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(receiver_input_channel_1 - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
}
  if (flight_mode >= 3 && waypoint_set == 1) {
    
    roll_setpoint_input  += gps_roll_adjust;
    pitch_setpoint_input += gps_pitch_adjust;
  }
  if (roll_setpoint_input > 2000)roll_setpoint_input = 2000;
  if (roll_setpoint_input < 1000)roll_setpoint_input = 1000;
  if (pitch_setpoint_input > 2000)pitch_setpoint_input = 2000;
  if (pitch_setpoint_input < 1000)pitch_setpoint_input = 1000;
  
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
 

  calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.
  start_stop_takeoff(); 
  
  //The battery voltage is needed for compensation.+
  //A complementary filter is used to reduce noise.
 
     battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 121) * 0.09853;
     raw_baterry_total -= raw_batterty_rotating_memory[battery_mem_location];    
     raw_batterty_rotating_memory[battery_mem_location]  =   battery_voltage ;  
     raw_baterry_total += raw_batterty_rotating_memory[battery_mem_location];
     battery_mem_location++;    
     if (battery_mem_location == 5)battery_mem_location = 0;
     battery_voltage = raw_baterry_total / 5;                      //Calculate the avarage temperature of the last 5 measurements. */
 
  //Turn on the led if battery voltage is to low.
  if (battery_voltage <= 1050 && battery_voltage >= 700)
  {
    digitalWrite(9, 1); // còi báo bật
  }else digitalWrite(9, 0);
  //
  if (flight_mode == 1) {
    throttle ++;                                      //We need the throttle signal as a base signal.
    if (throttle > 1630)throttle = 1630; 
  }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
  // hold altitude mode
  else if (flight_mode >= 2) { 
       
      throttle = throttle_altitude + pid_output_altitude + manual_throttle; 
    //  if(loop_counter == 20)Serial.println("im hereeeeeeeeeeeeeeeeeeeeeeeeeee");
  }

  if (start == 2) {                                                         //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    //================= initial code==========
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
    //============*/                                                          

    if (battery_voltage < 1240 && battery_voltage > 800) {                  //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-4 pulse for voltage drop.
    }

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                          //Limit the esc-4 pulse to 2000us.
  }

  else {
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating the pulses for the ESC's is explained in this video:
  //https://youtu.be/fqEkVcqxtU8
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
  //that the loop time is still 4000us and no longer! More information can be found on
  //the Q&A page:
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //==============DEBUGGGGGGGGGGGGGGGGGGGGGG===================================

 // print_signals();
  //delay(100);
 send_telemetry_data();
  loop_counter ++;
  
  if (loop_counter == 75)loop_counter = 0;
 
  //================================== */

  if (micros() - loop_timer > 6717)digitalWrite(8, HIGH);                  //Turn on the LED if the loop time exceeds 4050us.
  else digitalWrite(8, LOW);
   //  if(loop_counter == 10) Serial.println(micros() - loop_timer);
     // Serial.print(",");
     // Serial.print(6800);
     // Serial.print(",");
     // Serial.println(6900); 
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while (micros() - loop_timer < 6667);                                     //We wait until 5000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTB |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
 


  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  gyro_signalen();

  while (PORTB >= 16) {                                                     //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if (timer_channel_1 <= esc_loop_timer)PORTB &= B11101111;               //Set digital output 4 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer)PORTB &= B11011111;               //Set digital output 5 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer)PORTB &= B10111111;               //Set digital output 6 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer)PORTB &= B01111111;               //Set digital output 7 to low if the time is expired.

  }
 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state. This is used to read the receiver signals.
//More information about this subroutine can be found in this video:
//https://youtu.be/bENjl1KQbvo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT2_vect) {
  current_time = micros();
  //Channel 1=========================================
  if (PINK & B00000100) {                                                   //Is input A10 high?
    if (last_channel_1 == 0) {                                              //Input A10 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if (last_channel_1 == 1) {                                           //Input A10 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if (PINK & B00001000 ) {                                                  //Is input A11 high?
    if (last_channel_2 == 0) {                                              //Input A11 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if (last_channel_2 == 1) {                                           //Input A11 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if (PINK & B00010000 ) {                                                  //Is input A12 high?
    if (last_channel_3 == 0) {                                              //Input A12 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if (last_channel_3 == 1) {                                           //Input 12 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if (PINK & B00100000 ) {                                                  //Is input A13 high?
    if (last_channel_4 == 0) {                                              //Input 13 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if (last_channel_4 == 1) {                                           //Input A13 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
  //Channel 5=========================================
  if (PINK & B01000000 ) {                                                  //Is input A14 high?
    if (last_channel_5 == 0) {                                              //Input A14 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_5 to current_time.
    }
  }
  else if (last_channel_5 == 1) {                                           //Input 14 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    receiver_input[5] = current_time - timer_5;                             //Channel 5 is current_time - timer_5.
  }
  //Channel 6=========================================
  if (PINK & B10000000 ) {                                                  //Is input A15 high?
    if (last_channel_6 == 0) {                                              //Input A15 changed from 0 to 1.
      last_channel_6 = 1;                                                   //Remember current input state.
      timer_6 = current_time;                                               //Set timer_6 to current_time.
    }
  }
  else if (last_channel_6 == 1) {                                           //Input A15 is not high and changed from 1 to 0.
    last_channel_6 = 0;                                                     //Remember current input state.
    receiver_input[6] = current_time - timer_6;                             //Channel 6 is current_time - timer_6.
  }
}
void testled()
{
  // Tắt toàn bộ các led - cái này dễ mà ha

  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);


  delay(1000);


  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);

  delay(1000);



  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);

  delay(1000);


  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);
  delay(1000);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  delay(1000);
  digitalWrite(9,LOW);
}
void print_signals() {
  Serial.print("flight_mode:");
  Serial.print(flight_mode);

  Serial.print("  Roll:");
  if (receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
  else if (receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Pitch:");
  if (receiver_input_channel_2 - 1480 < 0)Serial.print("^^^");
  else if (receiver_input_channel_2 - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Throttle:");
  if (receiver_input_channel_3 - 1480 < 0)Serial.print("vvv");
  else if (receiver_input_channel_3 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Yaw:");
  if (receiver_input_channel_4 - 1480 < 0)Serial.print("<<<");
  else if (receiver_input_channel_4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-"); 
  Serial.println(receiver_input_channel_4);
     Serial.print("receiver_input_channel_5:");
   Serial.println(receiver_input_channel_5);
     Serial.print("receiver_input_channel_6:");
    Serial.println(receiver_input_channel_6);
}

String splitString(String str, String delim, uint16_t pos) {
  String tmp = str;
  for (int i = 0; i < pos; i++) {
    tmp = tmp.substring(tmp.indexOf(delim) + 1);
    if (tmp.indexOf(delim) == -1
        && i != pos - 1 )
      return "";
  }
  return tmp.substring(0, tmp.indexOf(delim));
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
