///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the starting, stopping and take-off detection is managed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_stop_takeoff(void)
{

  //======== Auto take off ===========
  if(readsignal==4 && takeoff_detected == 0)
  { start=1;
    ground_pressure= actual_pressure; 
    takeoff_detected = 1;     
    readsignal=5;                         
  }
  if ((loop_counter==74)&&(actual_pressure-ground_pressure<=1)&& start == 1&& check_waypoints ==1) 
  {                        
    start = 2; 
    throttle = 1000;                                                   //Set the base throttle to the motor_idle_speed variable.  
    flight_mode=1; 
    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.
    course_lock_heading = angle_yaw;
    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
       
   // takeoff_detected = 1;                                                        //Set the auto take-off detection to 1, indicated that the quadcopter is flying.
  }

  //======= Thêm biến để nhận biết khi nào nên bay theo waypoints 
   if(altitude_quad >= 1.0 && start == 2 && check_waypoints ==1)
   {                                      
   flight_mode=3; 
   heading_lock =1;  
   }
   
   //========== Flight to wp ============ 
   if(flytowp_detected==0 && flight_mode>=2 && check_waypoints ==1 && time_to_flywp<725)
   {
    time_to_flywp++;
    if(time_to_flywp==725)
    {
      flytowp_detected=1;
    }
   }
   //check biến landing_detected để tự động hạ cánh sau 10s khi bay hết waypoints
   if(completed_wp==1 && time_to_landing<1500 && landing_detected==0)
   {
    time_to_landing++;
    if(time_to_landing==1500)
    {
      landing_detected=1;
    }
   }
  //============Auto Landing==============

  if (start == 2 && landing_detected==1 )
  {  
        
   pid_altitude_setpoint = actual_pressure;  
   manual_altitude_change = 1;   //Set the start variable to 0 to disable the motors.
     digitalWrite(6,0);
   if(altitude_quad >= 1.0)
   {
    manual_throttle = -5 ;   
  
   }
   if(altitude_quad<1.0)
   {
    throttle_altitude--;
    if(throttle_altitude<=1250)throttle_altitude=1250;
   }
    if(throttle_altitude==1250) 
    { 
     throttle_altitude = 1530;          
     start=0;
     takeoff_detected=0;
     flight_mode=1;
     landing_detected=0;
     flytowp_detected=0;
 //====== reset waypoints =========          
      completed_wp=0;
      waypoint_mode=0;
      emergency_return=0;
      wp_list=5;
      timer_counter=0;
      time_to_landing=0;
      time_to_flywp=0;
      heading_lock =0;
    }
  
  }
  if(readsignal==0 && takeoff_detected ==1)
  {
     throttle_altitude = 1530;          
     start=0;
     takeoff_detected=0;
     flight_mode=1;
     landing_detected=0;
     flytowp_detected=0;
 //====== reset waypoints =========          
      completed_wp=0;
      waypoint_mode=0;
      emergency_return=0;
      wp_list=5;
      timer_counter=0;
      time_to_landing=0;
      time_to_flywp=0;
      heading_lock =0;
  }
                                                       
  //=============



 }
