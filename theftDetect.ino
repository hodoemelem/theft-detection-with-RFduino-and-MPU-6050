/*
 * Author: Henry Ugochukwwu Odoemelem
 * Description: Bicycle theft detection code using MPU-6050 IMU activity threshold, altitude detection and RFduinoBLE
 * DAte: 9-Jan-2019
 */

#include <RFduinoBLE.h>
#include<Wire.h>
#include <math.h>

const uint8_t adxl_addr = 0x53; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ; // declare accellerometer
//void read_adxl_data();
float accx, accy, accz;
float last_accy = 100;
float curr_accy = 100;


int32_t adc_T;//raw temp value
int32_t adc_P;//raw pressure value



//temperature compensation/calib values
uint16_t digT1;
int16_t digT2;
int16_t digT3;

//Pressure compensation/calib values
uint16_t digP1;
int16_t digP2;
int16_t digP3;
int16_t digP4;
int16_t digP5;
int16_t digP6;
int16_t digP7;
int16_t digP8;
int16_t digP9;


float To = 273.15; //K, in standard temp.
float Po = 1.0; //(atmopheric pressure at sea level) 1 atm =1.01325 bar.
float Pz;//Pressure above sea level at Height z
float R = 287.0; //J/KgK, specific gas constant of air
float g = 9.8; //m/s^2 acceleration due to gravity
float Height;// height above sea level

int last_millis = 0;
int alarm = 3; //alarm is on GPIO3
int flag = 1;// to check for BLE connection
unsigned long alarmtime = 0;//time before alarm beeps

void CompensationValues();// get compensation values for temp and pressure
void Calctemp();//calculate the temp
void Calcpressure();//calculate the pressure

char tempt[5] = {' ', '0', '0', '°', 'C'};
char pressureble[7] = {'0', '0', '0', '0', 'h', 'P', 'a'};
char altitudeble[5] = {'0', '0', '0', '0', 'm',};

float temperature;//converted temp
double t_fine;//raw temp2
double pressure;//converted pressure
uint8_t numi[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};//digits to send to ble app in byte
char numc[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};//digits to send to ble app in char
char chat[6] = {'A', 'C', 'C', 'E', 'S', 'S'};//to show ACCESS on Mobile connect


void accValue();
uint32_t interruptPin = 2;//adxl INT1 connected to GPIO2 of rfduino

void bleTemp();
void blePressure();

void setup() {

  Serial.begin(9600);

  Wire.begin(); // initiate i2c system

  //map to INT1
  Wire.beginTransmission(adxl_addr);
  Wire.write(0x2F);//INT_MAP reg
  Wire.write(0xEF);  //map only activity interrupt to INT1
  Wire.endTransmission();

  //threshold accelereation
  Wire.beginTransmission(adxl_addr);
  Wire.write(0x24);//threshold reg( 8 bits, 62.5mg/LSB(0xFF=16g), 1g=9.81mls^2
  Wire.write(0x04);  //2.45m/s^2 threshold value
  Wire.endTransmission();


  //axis to check for threshold accele value
  Wire.beginTransmission(adxl_addr);
  Wire.write(0x27);//axis reg
  Wire.write(0x60);  //check x,y axis
  Wire.endTransmission();

  Wire.beginTransmission(0x76);//BME280 12c address
  // Select control measurement register
  Wire.write(0xF4);//ctrl_meas reg for temp and press
  // Normal mode, temp and pressure over sampling rate = 1
  Wire.write(0b10110111);//oversampling register pressure(osr_p)=101(*16),osr_t=101(ie *16),mode=11(ie normal)
  // Stop I2C Transmission
  Wire.endTransmission();


  Wire.beginTransmission(adxl_addr); // be sure we talk to our MPU vs some other device
  Wire.write(0x2D);  //power reg
  Wire.write(0x0D);     // measurement mode
  Wire.endTransmission(true); // done talking over to adxl device, for the moment

  Wire.beginTransmission(adxl_addr); // be sure we talk to our MPU vs some other device
  Wire.write(0x2C);  //BW-reg
  Wire.write(0x0F);     // 3200hz output datarate, best for 16g range according to datasheet
  Wire.endTransmission(true); // done talking over to adxl device, for the moment

  //Configure the accelerometer (+/-16g)
  Wire.beginTransmission(adxl_addr);
  Wire.write(0x31);//data format reg
  Wire.write(0x03);  //msb right justified, range 16g
  Wire.endTransmission();


  //INT_ENAbLe, has to be done last according to datasheet
  Wire.beginTransmission(adxl_addr);
  Wire.write(0x2E);//int_enable reg
  Wire.write(0x10);  //enable activity interrupt
  Wire.endTransmission();



  pinMode(interruptPin, INPUT);

  attachPinInterrupt(interruptPin, Alert, HIGH);//external interrupt handler for rfduino
  

  pinMode(alarm, OUTPUT);// set alarm as alarm pin 3 as output pin

  RFduinoBLE.begin();//initiate BLE
  RFduinoBLE.advertisementData = "TDS";//BLE name




}


void loop() {

  //call functions to get BME280 sensor values
  
  CompensationValues();
  Calctemp();
  Calcpressure();

  //pz, is preesure at height z
  Pz = (pressure / 100000) / 1.01325; //pressure in Pa,Pz in atm

  ///#####################CalculateAltitude
  Height = R * To * log(Pz / Po) / (-g);//calculate height above sea level

  //display#######################################################################################
  Serial.print("Temp.= ");  Serial.print(temperature); Serial.println(" °C");
  Serial.print("Pressure= ");  Serial.print(Pz); Serial.println(" atm");
  Serial.print("Altitude= ");  Serial.print(Height); Serial.println(" m"); Serial.println(" ");

  accValue();
  
  bleTemp();
  blePressure();
  bleAltitude();

  if (flag == 0)//if flag is equal to 0(ie ble connected), send data to ble app
  {
    for (int i = 0; i < 2; i++)
    {


      RFduinoBLE.send(tempt, 5);
      RFduinoBLE.send('\n');
      RFduinoBLE.send(pressureble, 7);//pressreble in hPa
      RFduinoBLE.send('\n');
      RFduinoBLE.send(altitudeble, 5);
      RFduinoBLE.send('\n');
      RFduinoBLE.send('\n');
      delay(1000);
    }
    
  }

  //if (flag==1)//read source of interrupt only when user is disconnected
  //{
  Wire.beginTransmission(adxl_addr); // get ready to talk to MPU again
  Wire.write(0x30);  // interrupt source register
  Wire.endTransmission(false); // done talking to adxl for the time being
  Wire.requestFrom(adxl_addr, 1, true); 
  byte a = Wire.read();//we need to read the interrupt source register to clear existing activity interrupt to begin check for next activity interrupt
  Serial.print("source = "); Serial.println(a, BIN);
  
  delay(3000);
  //}

}

void RFduinoBLE_onConnect()
{
  
  digitalWrite(alarm, LOW);//turn off alarm
 
  flag = 0;//turn off flag to allow print out of data to Ble app

  for (int i = 0; i < 2; i++)//done twice to make sure it is sent
  {
    RFduinoBLE.send(chat, 6);
    RFduinoBLE.send('\n');
    RFduinoBLE.send('\n');
    delay(1000);
  }

  //to disable adxl interrupts, i.e do not check for motion, i am the bike owner
  Wire.beginTransmission(adxl_addr);
  Wire.write(0x2E);//INT_ENABLE reg
  Wire.write(0x00);  //disable all interrupt
  Wire.endTransmission();

  //map all interrupt(if any!!!) to INT2 which is not in use in our case
  Wire.beginTransmission(adxl_addr);
  Wire.write(0x2F);//INT_MAP reg
  Wire.write(0xFF);  //
  Wire.endTransmission();




  digitalWrite(alarm, LOW);//turn off alarm

}

void RFduinoBLE_onDisconnect() {
//restart the system when i am disconnect?not in range to ensure activity interrupt becomes enabled.
  RFduino_systemReset();
}

void CompensationValues()
{
  uint16_t value;

  //for temp##############################################################################
  Wire.beginTransmission(0x76); // get ready to talk to BME again
  Wire.write(0x88);  // starting with register 0x88 (calib dit register for temp)
  Wire.endTransmission(); // done talking to BME for the time being
  Wire.requestFrom(0x76, (byte)6); // request a total of 6 registers



  value = Wire.read() << 8 | Wire.read();
  digT1 = (value >> 8) | (value << 8); //digT[7:0] / [15:8] -- > digT[0:7:8:15]


  value = Wire.read() << 8 | Wire.read();
  digT2 = (value >> 8) | (value << 8);

  value = Wire.read() << 8 | Wire.read();
  digT3 = (value >> 8) | (value << 8);

  //for pressure###########################################################################
  Wire.beginTransmission(0x76); // get ready to talk to BME again
  Wire.write(0x8E);  // starting with register 0x8E (calib dit register for temp)
  Wire.endTransmission(); // done talking to BME for the time being
  Wire.requestFrom(0x76, (byte)18); // request a total of 6 registers



  value = Wire.read() << 8 | Wire.read();
  digP1 = (value >> 8) | (value << 8); //digP[7:0] / [15:8] -- > digP[0:7:8:15]


  value = Wire.read() << 8 | Wire.read();
  digP2 = (value >> 8) | (value << 8);

  value = Wire.read() << 8 | Wire.read();
  digP3 = (value >> 8) | (value << 8);


  value = Wire.read() << 8 | Wire.read();
  digP4 = (value >> 8) | (value << 8); //digP[7:0] / [15:8] -- > digP[0:7:8:15]


  value = Wire.read() << 8 | Wire.read();
  digP5 = (value >> 8) | (value << 8);

  value = Wire.read() << 8 | Wire.read();
  digP6 = (value >> 8) | (value << 8);

  value = Wire.read() << 8 | Wire.read();
  digP7 = (value >> 8) | (value << 8); //digP[7:0] / [15:8] -- > digP[0:7:8:15]


  value = Wire.read() << 8 | Wire.read();
  digP8 = (value >> 8) | (value << 8);

  value = Wire.read() << 8 | Wire.read();
  digP9 = (value >> 8) | (value << 8);



}

void Calctemp()
{
  uint32_t value;
  double var1;
  double var2;
  double temperature_min = -40;
  double temperature_max = 85;

  
  Wire.beginTransmission(0x76); // get ready to talk to BME again
  Wire.write(0xFA);  // starting with register 0xFA (temp_H)
  Wire.endTransmission(); // done talking to BME for the time being
  Wire.requestFrom(0x76, (byte)3); // request a total of 3 registers


  //receive 24 bits from the 3 temp. registers(8 bits type)
  value = Wire.read();//read 8 bits
  value <<= 8;//then shift to the right 8 times
  value |= Wire.read();//now and more 8bits
  value <<= 8;
  value |= Wire.read();

  adc_T = value;
  adc_T >>= 4;//remove last four bits, we need just 20 bits

  //TEMPERATURE COMPENSATION AND CONVERSION FORMULA FROM BME280 DATASHEET
  var1 = ((double)adc_T) / 16384.0 - ((double)digT1) / 1024.0;
  var1 = var1 * ((double)digT2);
  var2 = (((double)adc_T) / 131072.0 - ((double)digT1) / 8192.0);
  var2 = (var2 * var2) * ((double)digT3);
  t_fine = (int32_t)(var1 + var2);
  temperature = (var1 + var2) / 5120.0;

  if (temperature < temperature_min)
    temperature = temperature_min;
  else if (temperature > temperature_max)
    temperature = temperature_max;

}

void Calcpressure()
{
  uint32_t value;
  double var1;
  double var2;
  double var3;
  double pressure_min = 30000.0;
  double pressure_max = 110000.0;


  Wire.beginTransmission(0x76); // get ready to talk to BME again
  Wire.write(0xF7);  // starting with register 0xF7 (press_H)
  Wire.endTransmission(); // done talking to BME for the time being
  Wire.requestFrom(0x76, (byte)3); // request a total of 3 registers


  //receive 24 bits from the 3 pressure. registers(8 bits type)
  value = Wire.read();
  value <<= 8;
  value |= Wire.read();
  value <<= 8;
  value |= Wire.read();

  adc_P = value;
  adc_P >>= 4;//remove last four bits, we need just 20 bits




  //PRESSURE COMPENSATION AND CONVERSION FORMULA FROM BME280 DATASHEET
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)digP6) / 32768.0;
  var2 = var2 + var1 * ((double)digP5) * 2.0;
  var2 = (var2 / 4.0) + (((double)digP4) * 65536.0);
  var3 = ((double)digP3) * var1 * var1 / 524288.0;
  var1 = (var3 + ((double)digP2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)digP1);
  /* avoid exception caused by division by zero */
  if (var1) {
    pressure = 1048576.0 - (double) adc_P;
    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)digP9) * pressure * pressure / 2147483648.0;
    var2 = pressure * ((double)digP8) / 32768.0;
    pressure = pressure + (var1 + var2 + ((double)digP7)) / 16.0;

    if (pressure < pressure_min)
      pressure = pressure_min;
    else if (pressure > pressure_max)
      pressure = pressure_max;
  } else { /* Invalid case */
    pressure = pressure_min;
  }
}

void bleTemp()
{
  for (int i = 0; i <= 9; i++)
  {
    //code block to convert float to character
    uint8_t temptm = (uint8_t)(abs((temperature / 10)));//abs , to consider on magnitude
    uint8_t temptL =  (uint8_t)(abs((((temperature / 10) - temptm) * 10)));

    if (numi[i] == temptm)//comparing digits of temp
    {
      tempt[1] = numc[i];//assign the corresponding char value.
    }
    if (numi[i] == temptL)
    {
      tempt[2] = numc[i];
    }
  }


  //takes care of the negative if any by padding with "-"
  if (temperature < 0)
  {
    tempt[0] = '-';
  }
  if (temperature > 0)
  {
    tempt[0] = ' ';
  }

}

void blePressure()
{

  for (int i = 0; i <= 9; i++)
  {
    //code block to convert float to character
    float spress = pressure / 100;//spress in hPa, 1hPa=100Pa
    uint8_t a = uint8_t(spress / 1000); //digit1
    float b = (spress / 1000) - a;
    uint8_t c = uint8_t(b * 10); //digit2
    float d = (b * 10) - c;
    uint8_t e = uint8_t(d * 10); //digit3
    float f = (d * 10) - e;
    uint8_t g = uint8_t(f * 10); //digit4

    if (numi[i] == a)//comparing digits of pressure
    {
      pressureble[0]  = numc[i];//assign the corresponding char value.
    }
    if (numi[i] == c)
    {
      pressureble[1] = numc[i];
    }
    if (numi[i] == e)
    {
      pressureble[2]  = numc[i];
    }
    if (numi[i] == g)
    {
      pressureble[3] = numc[i];
    }
  }


}

void bleAltitude()
{

  for (int i = 0; i <= 9; i++)
  {

    //code block to convert flaot to character
    uint8_t a = uint8_t(Height / 1000); //digit1
    float b = (Height / 1000) - a;
    uint8_t c = uint8_t(b * 10); //digit2
    float d = (b * 10) - c;
    uint8_t e = uint8_t(d * 10); //digit3
    float f = (d * 10) - e;
    uint8_t g = uint8_t(f * 10); //digit4

    if (numi[i] == a)//comparing digits of altitude
    {
      altitudeble[0]  = numc[i];//assign the corresponding char value.
    }
    if (numi[i] == c)
    {
      altitudeble[1] = numc[i];
    }
    if (numi[i] == e)
    {
      altitudeble[2]  = numc[i];
    }
    if (numi[i] == g)
    {
      altitudeble[3] = numc[i];
    }
  }


}

int Alert(uint32_t ulPin)//rfduino interrupt callback function format
{
  Wire.beginTransmission(adxl_addr); // get ready to talk to MPU again
  Wire.write(0x32);  // starting with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false); // done talking to MPU for the time being
  Wire.requestFrom(adxl_addr, 6, true); //i need to read 6 registers

  //data is collected this way because it is MSB right justified
  AcX = Wire.read() | Wire.read() << 8; // 0x32 (ACCEL_XOUT_H) & 0x33 (ACCEL_XOUT_L)
  AcY = Wire.read() | Wire.read() << 8; // 0x34(ACCEL_YOUT_H) & 0x35 (ACCEL_YOUT_L)
  AcZ = Wire.read() | Wire.read() << 8; // 0x36 (ACCEL_ZOUT_H) & 0x37 (ACCEL_ZOUT_L)



  accx = (float)AcX / 31.0; //+_16g
  accy = (float)AcY / 31.0;
  accz = (float)AcZ / 31.0;

  accx = (float)accx * 9.81; //+_16g
  accy = (float)accy * 9.81;
  accz = (float)accz * 9.81;

  Serial.print("intaccx= ");  Serial.print(accx); Serial.println(" m/s^2");
  Serial.print("intaccy= ");  Serial.print(accy); Serial.println(" m/s^2");
  Serial.print("intaccz= ");  Serial.print(accz); Serial.println(" m/s^2");
  


  digitalWrite(alarm, HIGH);//turn on alarm


  return 0;

}

void accValue()
{

  Wire.beginTransmission(adxl_addr); // get ready to talk to MPU again
  Wire.write(0x32);  // starting with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false); // done talking to adxl for the time being
  Wire.requestFrom(adxl_addr, 6, true); // i need to read 6 registers

  //data is collected this way because it is MSB right justified
  AcX = Wire.read() | Wire.read() << 8; // 0x32 (ACCEL_XOUT_H) & 0x33 (ACCEL_XOUT_L)
  AcY = Wire.read() | Wire.read() << 8; // 0x34(ACCEL_YOUT_H) & 0x35 (ACCEL_YOUT_L)
  AcZ = Wire.read() | Wire.read() << 8; // 0x36 (ACCEL_ZOUT_H) & 0x37 (ACCEL_ZOUT_L)

  accx = (float)AcX / 31.0; //+_16g
  accy = (float)AcY / 31.0;
  accz = (float)AcZ / 31.0;

  accx = (float)accx * 9.81; //+_16g
  accy = (float)accy * 9.81;
  accz = (float)accz * 9.81;

  Serial.print("accx= ");  Serial.print(accx); Serial.println(" m/s^2");
  Serial.print("accy= ");  Serial.print(accy); Serial.println(" m/s^2");
  Serial.print("accz= ");  Serial.print(accz); Serial.println(" m/s^2");
}
