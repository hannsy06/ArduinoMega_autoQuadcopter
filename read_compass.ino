void setup_compass(){
  Wire.beginTransmission(QMC_address);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(QMC_address);
  Wire.write(0x09);
  Wire.write(0x1D);
  Wire.endTransmission();
  compass_x_min =  eeprom_data[41]<<8 | eeprom_data[40]; 
  compass_x_max =  eeprom_data[43]<<8 | eeprom_data[42];
  compass_y_min =  eeprom_data[45]<<8 | eeprom_data[43];
  compass_y_max =  eeprom_data[47]<<8 | eeprom_data[46];
  compass_z_min =  eeprom_data[49]<<8 | eeprom_data[48];
  compass_z_max =  eeprom_data[51]<<8 | eeprom_data[50];
  compass_scale_x = ((float)compass_y_max - compass_y_min) / (compass_x_max - compass_x_min);
  if(compass_scale_x < 1)compass_scale_x = 1;
  compass_scale_y = ((float)compass_x_max - compass_x_min) / (compass_y_max - compass_y_min);
  if(compass_scale_y < 1)compass_scale_y = 1;
  compass_offset_x = (((float)compass_x_max - compass_x_min) / 2 - compass_x_max) * compass_scale_x;
  compass_offset_y = (((float)compass_y_max - compass_y_min) / 2 - compass_y_max) * compass_scale_y;
}
void read_compass(){
  Wire.beginTransmission(QMC_address);
  Wire.write(0x00);                                        
  Wire.endTransmission();                                     

  Wire.requestFrom(QMC_address, 6);                        //Request 6 bytes from the compass.
  compass_x = Wire.read() | Wire.read()<< 8;                 //Add the low and high byte to the compass_y variable.                                           //Invert the direction of the axis.
  compass_y = Wire.read() | Wire.read()<< 8;                 //Add the low and high byte to the compass_z variable.;
  compass_y*=-1;
  compass_z = Wire.read() | Wire.read()<< 8;                 //Add the low and high byte to the compass_x variable.;
    if (compass_calibration_on == 0) {                            //When the compass is not beeing calibrated.
    compass_x = compass_x*compass_scale_x + compass_offset_x;                              //Add the x-offset, scale factor to the raw value.
    compass_y = compass_y*compass_scale_y + compass_offset_y;                            //Add the y-offset, scale factor to the raw value.  
   //===== median filter compass x ========
/*    compass_x_total_average -= compass_x_rotating_mem[compass_x_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    compass_x_rotating_mem[compass_x_rotating_mem_location] = compass_x;                                                //Calculate the new change between the actual pressure and the previous measurement.
    compass_x_total_average += compass_x_rotating_mem[compass_x_rotating_mem_location];                          //Add the new value to the long term avarage value.
    compass_x_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if(compass_x_rotating_mem_location == 20)compass_x_rotating_mem_location = 0;                              //Start at 0 whe last 20 pressure readings.n the memory location 20 is reached.
    compass_x = compass_x_total_average / 20;                                             //Calculate the average pressure of the
     //===== median filter compass y ========
    compass_y_total_average -= compass_y_rotating_mem[compass_y_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    compass_y_rotating_mem[compass_y_rotating_mem_location] = compass_y;                                         //Calculate the new change between the actual pressure and the previous measurement.
    compass_y_total_average += compass_y_rotating_mem[compass_y_rotating_mem_location];                          //Add the new value to the long term avarage value.
    compass_y_rotating_mem_location++;                                                                           //Increase the rotating memory location.
    if(compass_y_rotating_mem_location == 20)compass_y_rotating_mem_location = 0;                                //Start at 0 whe last 20 pressure readings.n the memory location 20 is reached.
    compass_y = compass_y_total_average / 20;           */                                                         //Calculate the average pressure of the                                      //Calculate the average pressure of the 
    }                             
   
  compass_x_horizontal = (float)compass_x * cos(angle_pitch * 0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * 0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * 0.0174533);
  compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);
  heading_horizontal =(atan2(compass_y_horizontal, compass_x_horizontal)) * (-180 / 3.14);
  heading_horizontal += declination;                                      //Add the declination to the magnetic compass heading to get the geographic north.
  if (heading_horizontal < 0) heading_horizontal+= 360;                   //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (heading_horizontal >= 360) heading_horizontal -= 360;          //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees r
  // xác định hướng la bàn gồm 4 hướng chính, 4 hướng phụ 
  //4 hướng chính
   if(heading_horizontal < 112.5 && heading_horizontal > 67.5)direction ="E";
   if(heading_horizontal < 292.5 && heading_horizontal > 247.5)direction ="W";
   if(heading_horizontal < 202.5 && heading_horizontal > 157.5)direction ="S";
   if(heading_horizontal < 22.5 || heading_horizontal > 337.5)direction ="N";
  //4 hướng phụ
  if(heading_horizontal < 67.5  && heading_horizontal > 22.5)direction ="NE";
  if(heading_horizontal < 157.5 && heading_horizontal > 112.5)direction ="SE";
  if(heading_horizontal < 247.5 && heading_horizontal > 202.5)direction ="SW";
  if(heading_horizontal < 337.5 && heading_horizontal > 292.5)direction ="NW";
    
    
}
void calibrate_compass(void) {
  compass_calibration_on = 1;                                                //Set the compass_calibration_on variable to disable the adjustment of the raw compass values.
  digitalWrite(8,HIGH);                                                             //The red led will indicate that the compass calibration is active.
  digitalWrite(7,LOW);                                                            //Turn off the green led as we don't need it.
  //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  while (receiver_input_channel_2 < 1800) {                                                 //Stay in this loop until the pilot lowers the pitch stick of the transmitter.                                              //Send telemetry data to the ground station.
    delayMicroseconds(5000);                                                 //Simulate a 250Hz program loop.
    read_compass();                                                          //Read the raw compass values.
    //In the following lines the maximum and minimum compass values are detected and stored.
    if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
    if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
     receiver_input_channel_2 = convert_receiver_channel(2);
  }
  compass_calibration_on = 0;                                                //Reset the compass_calibration_on variable.
  
  //The maximum and minimum values are needed for the next startup and are stored
  for (error = 0; error < 6; error ++)
  { int i=0;
    EEPROM.write(40 + error*2, compass_cal_values[error] & 0b11111111);
    EEPROM.write(41 + error*2, compass_cal_values[error] >> 8);  
  }

  setup_compass();                                                           //Initiallize the compass and set the correct registers.
  read_compass();                                                            //Read and calculate the compass data.
  angle_yaw = heading_horizontal;                                        //Set the initial compass heading.
  
  digitalWrite(8,LOW);
   for (error = 0; error < 6; error ++)
   {
    Serial.println(compass_cal_values[error]);
   }
  for (error = 0; error < 15; error ++) {
     digitalWrite(7,HIGH);
    delay(50);
     digitalWrite(7,LOW);
    delay(50);
  }

  error = 0;

  loop_timer = micros();                                                     //Set the timer for the next loop.
}
float course_deviation(float course_b, float course_c) {
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}
