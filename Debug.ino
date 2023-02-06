/*//============= redeclare eeprom's value 12/5/2022===========
 int center_channel = 1504;
 int high_channel = 1998;
 int low_channel = 1004;
 byte channel_1_assign=1;
 byte channel_2_assign=130;
 byte channel_3_assign=3;
 byte channel_4_assign=4;
*/
/*  //============= UPDATED EEPROM (12/5/2022)=========
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
   /* //Store the gyro address in the variable.
  Serial.println("Center_value 1-4");
  Serial.println((eeprom_data[1]<<8)|(eeprom_data[0]));
  Serial.println((eeprom_data[3]<<8)|(eeprom_data[2]));
  Serial.println((eeprom_data[5]<<8)|(eeprom_data[4]));
  Serial.println((eeprom_data[7]<<8)|(eeprom_data[6]));
  Serial.println("High_value 1-4");
  Serial.println((eeprom_data[9]<<8)|(eeprom_data[8]));
  Serial.println((eeprom_data[11]<<8)|(eeprom_data[10]));
  Serial.println((eeprom_data[13]<<8)|(eeprom_data[12]));
  Serial.println((eeprom_data[15]<<8)|(eeprom_data[14]));
  Serial.println("Low_value 1-4");
  Serial.println((eeprom_data[17]<<8)|(eeprom_data[16]));
  Serial.println((eeprom_data[19]<<8)|(eeprom_data[18]));
  Serial.println((eeprom_data[21]<<8)|(eeprom_data[20]));
  Serial.println((eeprom_data[23]<<8)|(eeprom_data[22]));
  
  Serial.println("channel_assign 1-4");
  Serial.println(eeprom_data[24]);
  Serial.println(eeprom_data[25]);
  Serial.println(eeprom_data[26]);
  Serial.println(eeprom_data[27]);
 //====================================*/

 //================reading MS5611========================
 /*uint32_t read_temp()
{     uint32_t value;
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write(0x58);                                                        //Send a 0x58 to indicate that we want to request the temperature data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
      delay(10);
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      Wire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();   
      Wire.beginTransmission(MS5611_address); 
      Wire.requestFrom(MS5611_address, 3);
      if(Wire.available()!=3)
      return 1;
      Wire.endTransmission();
      for (int8_t k=3-1; k>=0; k--)
       value|=  (uint32_t) Wire.read() << (8*k) ;  // concantenate bytes      
      return value;
  
}
uint32_t get_rawtemp()
{
   uint32_t D2 = read_temp();
    int32_t dT = D2 - (uint32_t)C[5] * 256;

     int32_t TEMP = 2000 + ((int64_t) dT * C[6]) / 8388608;
     return  TEMP;
}

uint32_t read_rawpres()
{     uint32_t value;
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write(0x48);                                                        //Send a 0x58 to indicate that we want to request the temperature data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
      delay(9);
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      Wire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();    
      Wire.requestFrom((uint8_t)MS5611_address, 3);
      // raw_pressure=0;
      if(Wire.available()!=3)return 1;
      for (int8_t k=3-1; k>=0; k--)
      value |=  (uint32_t) Wire.read() << (8*k) ;  // concantenate bytes
      return value;
      } */
