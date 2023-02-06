///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part sends the telemetry data to the ground station.
//The output for the Serial1 monitor is PB0. Protocol is 1 start bit, 8 data bits, no parity, 1 stop bit.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void send_telemetry_data(void) {
  //======== display monitoring data============
  if(debug_function == 0)
 {

  if(loop_counter == 0)Serial1.print("check");
  if(loop_counter == 1)Serial1.print(F("|"));
  if(loop_counter == 2)Serial1.print((float)getaltitude(actual_pressure, ground_pressure, (float)T/100.0));
  if(loop_counter == 3)Serial1.print(F("|"));
  if(loop_counter == 4)Serial1.print(throttle);
  if(loop_counter == 5)Serial1.print(F("|"));
  if(loop_counter == 6)Serial1.print(check_waypoints);
  if(loop_counter == 7)Serial1.print(F("|"));
  if(loop_counter == 8)Serial1.print(fix_type);
  if(loop_counter == 9)Serial1.print(F("|"));
  if(loop_counter == 10)Serial1.print(l_lon_gps);
  if(loop_counter == 11)Serial1.print(F("|"));
  if(loop_counter == 12)Serial1.print(l_lat_gps);  
  if(loop_counter == 13)Serial1.print(F("|"));
  if(loop_counter == 14)Serial1.print(flight_mode);
  if(loop_counter == 15)Serial1.print(F("|"));
  if(loop_counter == 16)Serial1.print(number_used_sats); 
  if(loop_counter == 17)Serial1.print(F("|")); 
  if(loop_counter == 18)Serial1.print(heading_lock);
  if(loop_counter == 19)Serial1.print(F("|")); 
  if(loop_counter == 20)Serial1.print(wp_list);
  if(loop_counter == 21)Serial1.print(F("|")); 
  if(loop_counter == 22)Serial1.print(completed_wp);
  if(loop_counter == 23)Serial1.print(F("|")); 
  if(loop_counter == 24)Serial1.print(waypoint_mode);
  if(loop_counter == 25)Serial1.print(F("|")); 
  if(loop_counter == 26)Serial1.print(emergency_return);
  if(loop_counter == 27)Serial1.print(F("|")); 
  if(loop_counter == 28)Serial1.print((int)heading_horizontal); 
  if(loop_counter == 29)Serial1.println(direction);
  //=============

  //===================================
   /*  if(loop_counter ==57)Serial.print("angle_roll_acc:");
      if(loop_counter ==58)Serial.print(angle_roll_acc);
      if(loop_counter ==59)Serial.print("-----angle_pitch_acc:");
      if(loop_counter ==60)Serial.println(angle_pitch_acc);
      if(loop_counter ==61)Serial.print("gyro_roll:");
      if(loop_counter ==62)Serial.println(gyro_roll);
      if(loop_counter ==63)Serial.println(gyro_roll_input);
      if(loop_counter ==64)Serial.print("gyro_pitch:");
      if(loop_counter ==65)Serial.println(gyro_pitch);
      if(loop_counter ==66)Serial.println(gyro_pitch_input);
      if(loop_counter ==67)Serial.print("angle_roll:");
      if(loop_counter ==68)Serial.print(angle_roll);
      if(loop_counter ==69)Serial.print("-----angle_pitch:");
      if(loop_counter ==70)Serial.print(angle_pitch);
      if(loop_counter ==71)Serial.println("======================="); */
      
      

  }
  //======== diplay data for debugging angle =============
   if(debug_function == 1)
  {
  if(loop_counter ==0) Serial1.print("debug1");
  if(loop_counter == 1) Serial1.print(F("|"));
  if(loop_counter == 2) Serial1.print(add_roll );
  if(loop_counter == 3) Serial1.print(F("|"));
  if(loop_counter == 4) Serial1.print (add_pitch);
  if(loop_counter == 5) Serial1.print(F("|"));
  if(loop_counter == 6) Serial1.print(correction_angle);
  if(loop_counter == 7) Serial1.print(F("|"));
  if(loop_counter == 8) Serial1.print (battery_voltage/100.00);
  if(loop_counter == 9) Serial1.print(F("|"));
  if(loop_counter == 10)Serial1.print(angle_roll);
  if(loop_counter == 11)Serial1.print(F("|"));
  if(loop_counter == 12)Serial1.print(angle_pitch);
  if(loop_counter == 13)Serial1.print(F("|"));
  if(loop_counter == 14)Serial1.print(flight_mode);
  if(loop_counter == 15)Serial1.print(F("|"));
  if(loop_counter == 16)Serial1.print(heading_lock);
  if(loop_counter == 17)Serial1.print(F("|"));
  if(loop_counter == 18)Serial1.print(heading_horizontal);
   if(loop_counter ==19)Serial1.print(F("|"));
  if(loop_counter == 20)Serial1.println(throttle_altitude);}
  //=========== display data for debugging Holding altitude fuction =====
  if(debug_function == 2)
  {
  if(loop_counter == 0) Serial1.print("debug2");
  if(loop_counter == 1) Serial1.print(F("|"));
  if(loop_counter == 2)Serial1.print(stable_altitude); 
  if(loop_counter == 3) Serial1.print(F("|"));
  if(loop_counter == 4) Serial1.println((int)time_to_change_wp);
 /* 
  if(loop_counter == 2) Serial1.print(pid_p_gain_altitude);
  if(loop_counter == 3) Serial1.print(F("|"));
  if(loop_counter == 4) Serial1.print (pid_i_gain_altitude);
  if(loop_counter == 5) Serial1.print(F("|"));
  if(loop_counter == 6) Serial1.print(pid_d_gain_altitude);
  if(loop_counter == 7) Serial1.print(F("|"));
  if(loop_counter == 8) Serial1.print(battery_voltage/100.00);
  if(loop_counter == 9) Serial1.print(F("|"));
  if(loop_counter == 10)Serial1.print((float)getaltitude(actual_pressure, ground_pressure, (float)temper/100.0));
  if(loop_counter == 11)Serial1.print(F("|"));
  if(loop_counter == 12)Serial1.print(stable_altitude);
  if(loop_counter == 13)Serial1.print(F("|"));
  if(loop_counter == 14)Serial1.print(flight_mode);
  if(loop_counter == 15)Serial1.print(F("|"));
  if(loop_counter == 16)Serial1.print(heading_lock);
  if(loop_counter == 17)Serial1.print(F("|"));
  if(loop_counter == 18)Serial1.print(heading_horizontal); 
  if(loop_counter == 19)Serial1.print(F("|"));
  if(loop_counter == 20)Serial1.print(((float)getaltitude(actual_pressure, ground_pressure, (float)temper/100.0))-stable_altitude);
  if(loop_counter == 21)Serial1.print(F("|"));
*/
  }
 // ========== display data for debugging GPS fuction ========
  if(debug_function == 3)
  {
  if(loop_counter == 0) Serial1.print("debug3");
  if(loop_counter == 1) Serial1.print(F("|"));
  if(loop_counter == 2) Serial1.print(lat_list[1]);
  if(loop_counter == 3) Serial1.print(F("|"));
  if(loop_counter == 4) Serial1.print (lat_list[2]);
  if(loop_counter == 5) Serial1.print(F("|"));
  if(loop_counter == 6) Serial1.print(lat_list[3]);
  if(loop_counter == 7) Serial1.print(F("|"));
  if(loop_counter == 8) Serial1.print(lat_list[4]);
   if(loop_counter == 9)Serial1.print(F("|"));
  if(loop_counter == 10)Serial1.print(lon_list[1]);
  if(loop_counter == 11)Serial1.print(F("|"));
  if(loop_counter == 12)Serial1.print(lon_list[2]);
   if(loop_counter ==13)Serial1.print(F("|"));
  if(loop_counter == 14)Serial1.print(lon_list[3]);
  if(loop_counter == 15)Serial1.print(F("|"));
  if(loop_counter == 16)Serial1.println(lon_list[4]);
 
  }
}
/*  if(debug_function==4)
  {
    if(flight_mode==3)
    {
      if(loop_counter == 0)Serial1.print((float)(gps_lat_error));
      if(loop_counter == 1)Serial1.print(F(","));
      if(loop_counter == 2)Serial1.print((float)(gps_lon_error));
      if(loop_counter == 3)Serial1.print(F(","));
      if(loop_counter == 4)Serial1.println((float)getaltitude(actual_pressure, ground_pressure, (float)temper/100.0));
      
    }
  }
   if(debug_function == 5)
  {
   
  if(flight_mode >= 3 && waypoint_set == 1 && receiver_input_channel_5 <1600)
  {
    if(loop_counter==18)Serial1.print("--lat_HƠME:");
    if(loop_counter==19)Serial1.print(lat_list[0]/1000000.0,6);
    if(loop_counter==20)Serial1.print("--lon_HOME:");
    if(loop_counter==21)Serial1.println(lon_list[0]/1000000.0,6);
    if(loop_counter==22)Serial1.print("--lat_wp1:");
    if(loop_counter==23)Serial1.print(lat_list[1]/1000000.0,6);
    if(loop_counter==24)Serial1.print("--lon_wp1:");
    if(loop_counter==25)Serial1.println(lon_list[1]/1000000.0,6);
    if(loop_counter==26)Serial1.print("--lat_wp2:");
    if(loop_counter==27)Serial1.print(lat_list[2]/1000000.0,6);
    if(loop_counter==28)Serial1.print("--lon_wp2:");
    if(loop_counter==29)Serial1.println(lon_list[2]/1000000.0,6);
    if(loop_counter==30)Serial1.print("--lat_wp3:");
    if(loop_counter==31)Serial1.print(lat_list[3]/1000000.0,6);
    if(loop_counter==32)Serial1.print("--lon_wp3:");
    if(loop_counter==33)Serial1.println(lon_list[3]/1000000.0,6);
    if(loop_counter==34)Serial1.print("--lat_wp4:");
    if(loop_counter==35)Serial1.print(lat_list[4]/1000000.0,6);
    if(loop_counter==36)Serial1.print("--lon_wp4:");
    if(loop_counter==37)Serial1.println(lon_list[4]/1000000.0,6);
    if(loop_counter==38)Serial1.println("============================"); 
    }
    if((flight_mode >= 3) && (waypoint_set == 1)&& (receiver_input_channel_5>=1600) && (check_waypoints == 1))//&& start==2)
   {
    if(loop_counter == 0)Serial1.print(l_lat_gps/1000000.0,6);
    if(loop_counter == 1)Serial1.print(F(","));
    if(loop_counter == 2)Serial1.print(l_lon_gps/1000000.0,6);
    if(loop_counter == 3)Serial1.print(F(","));
    if(loop_counter == 4)Serial1.println((float)getaltitude(actual_pressure, ground_pressure, (float)temper/100.0));
   }   
    }  
  } */
/*  if(loop_counter == 9) Serial1.print(F("|")); 
  if(loop_counter == 10)Serial1.print(wp_list);
  if(loop_counter == 11)Serial1.print(F("|")); 
  if(loop_counter == 12)Serial1.print(completed_wp);
  if(loop_counter == 13)Serial1.print(F("|")); 
  if(loop_counter == 14)Serial1.print(waypoint_mode);
  if(loop_counter == 15)Serial1.print(F("|")); 
  if(loop_counter == 16)Serial1.print(emergency_return);
  if(loop_counter == 17)Serial1.print(F("|"));
  if(loop_counter == 18)Serial1.print(flight_mode);
  if(loop_counter == 19)Serial1.print(F("|"));
  if(loop_counter == 20)Serial1.print(heading_lock);
  if(loop_counter == 21)Serial1.print(F("|"));
  if(loop_counter == 22)Serial1.print(heading_horizontal);
  if(loop_counter == 23)Serial1.print(F("|"));
  if(loop_counter == 24)Serial1.print((float)(gps_lon_error));
  if(loop_counter == 25)Serial1.print(F("|"));
  if(loop_counter == 26)Serial1.print((float)(gps_lat_error));
  if(loop_counter == 27)Serial1.print(F("|"));
  if(loop_counter == 28)Serial1.print(throttle_altitude);
  if(loop_counter == 29)Serial1.print(F("|"));
  if(loop_counter == 30)Serial1.print((float)(l_lon_gps/1000000.0),6);
  if(loop_counter == 31)Serial1.print(F("|"));
  if(loop_counter == 32)Serial1.println((float)(l_lat_gps/1000000.0),6);  */
  
 
