#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM
//============= redeclare eeprom's value 12/5/2022===========
 int battery_voltage;
 int center_channel = 1504;
 int high_channel = 1998;
 int low_channel = 1004;
 byte channel_1_assign=1;
 byte channel_2_assign=2;
 byte channel_3_assign=3;
 byte channel_4_assign=4;
 byte gyro_address=0x68,error;
  byte data;
void setup() {
  Serial.begin(57600);
 
  // put your setup code here, to run once:
 
/* //============= UPDATED EEPROM (12/5/2022)=========
    EEPROM.write(0, center_channel & 0b11111111);   //1500=0b00000101 11011100 > 250 nên ta phải chia thành 2 byte, kiểu dữ liệu int =16bit
    // 00000101 11011100 & 11111111= 11011100(giá trị lưu trong địa chỉ 0 của eeprom)
    EEPROM.write(1, center_channel >> 8); //00000101 1101110 >>8= 00000101 (giá trị lưu trong địa chỉ 1 của eeprom)     
    EEPROM.write(2, center_channel & 0b11111111);   //1500
    EEPROM.write(3, center_channel >> 8);
    EEPROM.write(4, center_channel & 0b11111111);   //1500
    EEPROM.write(5, center_channel >> 8);
    EEPROM.write(6, center_channel & 0b11111111);   //1500
    EEPROM.write(7, center_channel >> 8);
    EEPROM.write(8, high_channel & 0b11111111);     //2000
    EEPROM.write(9, high_channel >> 8);
    EEPROM.write(10, high_channel & 0b11111111);    //2000
    EEPROM.write(11, high_channel >> 8);
    EEPROM.write(12, high_channel & 0b11111111);    //2000
    EEPROM.write(13, high_channel >> 8);
    EEPROM.write(14, high_channel & 0b11111111);    //2000
    EEPROM.write(15, high_channel >> 8);
    EEPROM.write(16, low_channel & 0b11111111);     //1000
    EEPROM.write(17, low_channel >> 8);
    EEPROM.write(18, low_channel & 0b11111111);     //1000
    EEPROM.write(19, low_channel >> 8);
    EEPROM.write(20, low_channel & 0b11111111);     //1000 
    EEPROM.write(21, low_channel >> 8);
    EEPROM.write(22, low_channel & 0b11111111);     //1000
    EEPROM.write(23, low_channel >> 8);
    EEPROM.write(24, channel_1_assign);   //1:Chân 8
    EEPROM.write(25, channel_2_assign);   //2:Chân 9   
    EEPROM.write(26, channel_3_assign);   //3:Chân 10
    EEPROM.write(27, channel_4_assign);   //4:Chân 11*/
//=============================================================================
   EEPROM.write(0, center_channel & 0b11111111);   //1500=0b00000101 11011100 > 250 nên ta phải chia thành 2 byte, kiểu dữ liệu int =16bit
    // 00000101 11011100 & 11111111= 11011100(giá trị lưu trong địa chỉ 0 của eeprom)
    EEPROM.write(1, center_channel >> 8); //00000101 1101110 >>8= 00000101 (giá trị lưu trong địa chỉ 1 của eeprom)     
    EEPROM.write(2, center_channel & 0b11111111);   //1500
    EEPROM.write(3, center_channel >> 8);
    EEPROM.write(4, center_channel & 0b11111111);   //1500
    EEPROM.write(5, center_channel >> 8);
    EEPROM.write(6, center_channel & 0b11111111);   //1500
    EEPROM.write(7, center_channel >> 8);
    EEPROM.write(8, high_channel & 0b11111111);     //2000
    EEPROM.write(9, high_channel >> 8);
    EEPROM.write(10, high_channel & 0b11111111);    //2000
    EEPROM.write(11, high_channel >> 8);
    EEPROM.write(12, high_channel & 0b11111111);    //2000
    EEPROM.write(13, high_channel >> 8);
    EEPROM.write(14, high_channel & 0b11111111);    //2000
    EEPROM.write(15, high_channel >> 8);
    EEPROM.write(16, low_channel & 0b11111111);     //1000
    EEPROM.write(17, low_channel >> 8);
    EEPROM.write(18, low_channel & 0b11111111);     //1000
    EEPROM.write(19, low_channel >> 8);
    EEPROM.write(20, low_channel & 0b11111111);     //1000 
    EEPROM.write(21, low_channel >> 8);
    EEPROM.write(22, low_channel & 0b11111111);     //1000
    EEPROM.write(23, low_channel >> 8);
    EEPROM.write(24, channel_1_assign);   //Chân 8
    EEPROM.write(25, channel_2_assign);   //Chân 9   
    EEPROM.write(26, channel_3_assign);   //Chân 10
    EEPROM.write(27, channel_4_assign);   //Chân 11
    EEPROM.write(28, 1);    //if(movement == 1)roll_axis = trigger_axis =1                                                          
    EEPROM.write(29, 130);   //if(movement == 2)pitch_axis = trigger_axis=2
    EEPROM.write(30, 131);     //if(movement == 3)yaw_axis = trigger_axis=3
    EEPROM.write(31, 1);         //type=1, MPU6050
    EEPROM.write(32, gyro_address); // 0x68, địa chỉ của MPU6050
    //Write the EEPROM signature
    EEPROM.write(33, 'J'); 
    EEPROM.write(34, 'M');
    EEPROM.write(35, 'B');
    //Store the gyro address in the variable. 

/*  void esc_pulse_output(){                                         // Tạo một độ rộng xung trên các chân từ 4-7 bằng với xung từ receiver
 //================UNO R3===================
  zero_timer = micros();                                            
  PORTD |= B11110000;                                            //Set port 4, 5, 6 and 7 high at once
  timer_channel_1 = esc_1 + zero_timer;                          //Calculate the time when digital port 4 is set low.
  timer_channel_2 = esc_2 + zero_timer;                          //Calculate the time when digital port 5 is set low.
  timer_channel_3 = esc_3 + zero_timer;                          //Calculate the time when digital port 6 is set low.
  timer_channel_4 = esc_4 + zero_timer;                          //Calculate the time when digital port 7 is set low.

  while(PORTD >= 16){                                            //Execute the loop until digital port 4 to 7 is low.
    esc_loop_timer = micros();                                   //Check the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;     //When the delay time is expired, digital port 4 is set low.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;     //When the delay time is expired, digital port 5 is set low.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;     //When the delay time is expired, digital port 6 is set low.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;     //When the delay time is expired, digital port 7 is set low.
  }
 //============MEGA 256========  
   zero_timer = micros();                                            
  PORTH |= B01111000;                                            //Set port 4, 5, 6 and 7 high at once
  timer_channel_1 = esc_1 + zero_timer;                          //Calculate the time when digital port 4 is set low.
  timer_channel_2 = esc_2 + zero_timer;                          //Calculate the time when digital port 5 is set low.
  timer_channel_3 = esc_3 + zero_timer;                          //Calculate the time when digital port 6 is set low.
  timer_channel_4 = esc_4 + zero_timer;                          //Calculate the time when digital port 7 is set low.

   while(PORTD >= 16){                                            //Execute the loop until digital port 4 to 7 is low.
    esc_loop_timer = micros();                                   //Check the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;     //When the delay time is expired, digital port 4 is set low.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;     //When the delay time is expired, digital port 5 is set low.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;     //When the delay time is expired, digital port 6 is set low.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;     //When the delay time is expired, digital port 7 is set low.
 //====================================*/

}
void loop() {
  // put your main code here, to run repeatedly:
   // Serial.println(F("Verify EEPROM data"));
    //Kiểm tra lại dữ liệu chứa trong eeprom
     battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 75) * 0.09853;
 /*   delay(1000);
    if(center_channel != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;// 00000101(EEPROM.read(1))<<8 = 00000101 00000000 | 11011100(EEPROM.read(0)) =101 11011100 = 1500(center_channel_1)
    if(center_channel != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;// Cách tính tương tự
    if(center_channel != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
    if(center_channel != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;
    
    if(high_channel != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
    if(high_channel != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
    if(high_channel != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
    if(high_channel != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;
    
    if(low_channel != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
    if(low_channel != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
    if(low_channel != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
    if(low_channel != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;
    
    if(channel_1_assign != EEPROM.read(24))error = 1;
    if(channel_2_assign != EEPROM.read(25))error = 1;
    if(channel_3_assign != EEPROM.read(26))error = 1;
    if(channel_4_assign != EEPROM.read(27))error = 1;
    
    if(1 != EEPROM.read(28))error = 1;
    if(2 != EEPROM.read(29))error = 1;
    if(3 != EEPROM.read(30))error = 1;
    if(1 != EEPROM.read(31))error = 1;
    if(gyro_address != EEPROM.read(32))error = 1;
    
    if('J' != EEPROM.read(33))error = 1;
    if('M' != EEPROM.read(34))error = 1;
    if('B' != EEPROM.read(35))error = 1;
  
    if(error == 1)Serial.println(F("EEPROM verification failed!!! (ERROR 5)"));
    else Serial.println(F("Verification done")); */
    //Serial1.println("okokok");
    if(Serial.available() > 0){
     data = Serial.read(); 
      if(data=='t')
      {     
    Serial.print("center_channel_1:");
    Serial.println(((EEPROM.read(1) << 8) | EEPROM.read(0)));
    Serial.print("center_channel_2:");}
    Serial.println(((EEPROM.read(3) << 8) | EEPROM.read(2)));
     Serial.print("center_channel_3:");
    Serial.println(((EEPROM.read(5) << 8) | EEPROM.read(4)));
     Serial.print("center_channel_4:");
    Serial.println(((EEPROM.read(7) << 8) | EEPROM.read(6)));
    Serial.println("=============================================");
    Serial.print("High_channel_1:");
    Serial.println(((EEPROM.read(9) << 8) | EEPROM.read(8)));
    Serial.print("High_channel_2:");
    Serial.println(((EEPROM.read(11) << 8) | EEPROM.read(10)));
    Serial.print("High_channel_3:");
    Serial.println(((EEPROM.read(13) << 8) | EEPROM.read(12)));
    Serial.print("High_channel_4:");
    Serial.println(((EEPROM.read(15) << 8) | EEPROM.read(14)));
    Serial.println("=============================================");
    Serial.print("Low_channel_1:");
    Serial.println(((EEPROM.read(17) << 8) | EEPROM.read(16)));
    Serial.print("Low_channel_2:");
    Serial.println(((EEPROM.read(19) << 8) | EEPROM.read(18)));
    Serial.print("Low_channel_3:");
    Serial.println(((EEPROM.read(21) << 8) | EEPROM.read(20)));
    Serial.print("Low_channel_4:");
    Serial.println(((EEPROM.read(23) << 8) | EEPROM.read(22)));
    Serial.println("=============================================");
    Serial.print("channel_1_assign:");
    Serial.println((EEPROM.read(24)));
    Serial.print("channel_2_assign:");
    Serial.println((EEPROM.read(25)));
    Serial.print("channel_3_assign:");
    Serial.println((EEPROM.read(26)));
    Serial.print("channel_4_assign:");
    Serial.println((EEPROM.read(27)));
    Serial.println("EEPROM 28-31 = 1,2,3,1: ");
    Serial.println((EEPROM.read(28)));
    Serial.println((EEPROM.read(29)));
    Serial.println((EEPROM.read(30)));
    Serial.println((EEPROM.read(31)));
    Serial.print("gyro_address:");
    Serial.println((EEPROM.read(32)));
    Serial.print("check_completed__char:");   
    Serial.print((EEPROM.read(33)));
    Serial.print((EEPROM.read(34)));
    Serial.println((EEPROM.read(35)));
     Serial.print("baterry:");
     Serial.println(battery_voltage); 
     delay(1000);
    }

}
