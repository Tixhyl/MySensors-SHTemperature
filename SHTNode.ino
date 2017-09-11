#define MY_NODE_ID 53
//#define MY_PARENT_NODE_ID 100
//#define MY_PARENT_NODE_IS_STATIC
// Enable debug prints

#define MY_RADIO_NRF24

#define MY_RF24_PA_LEVEL RF24_PA_MAX

#include <SPI.h>
#include <Wire.h>
#include <MySensors.h>

static const uint64_t UPDATE_INTERVAL = 30000;

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_VOLT 2

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgVolt(CHILD_ID_VOLT, V_TEMP);

#define HTDU21D_ADDRESS             0x40  //Unshifted 7-bit I2C address for the sensor

#define TRIGGER_TEMP_MEASURE_HOLD   0xE3
#define TRIGGER_HUMD_MEASURE_HOLD   0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD 0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD 0xF5

#define WRITE_USER_REG              0xE6
#define READ_USER_REG               0xE7
#define SOFT_RESET                  0xFE

#define ERROR_CRC                   998
#define ERROR_TIMEOUT               999

float GetHumidity(void)
{
    return (-6.0 + 125.0 / 65536.0 * (float)(readSensor(TRIGGER_HUMD_MEASURE_HOLD)));
}

float GetTemperature(void)
{
    return (-46.85 + 175.72 / 65536.0 * (float)(readSensor(TRIGGER_TEMP_MEASURE_HOLD)));
}

 void setResolution(uint8_t resolution)
{
  uint8_t userRegister = read_user_register(); //Go get the current register state
  userRegister &= B01111110; //Turn off the resolution bits
  resolution &= B10000001; //Turn off all other bits but resolution bits
  userRegister |= resolution; //Mask in the requested resolution bits
  
  //Request a write to user register
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(WRITE_USER_REG); //Write to the user register
  Wire.write(userRegister); //Write the new resolution bits
  Wire.endTransmission();
}

uint8_t read_user_register(void)
{
  uint8_t userRegister;
  
  //Request the user register
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(READ_USER_REG); //Read the user register
  Wire.endTransmission();
  
  //Read result
  Wire.requestFrom(HTDU21D_ADDRESS, 1);
  
  userRegister = Wire.read();

  return(userRegister);  
}

uint16_t readSensor(uint8_t command)
{
    uint16_t result;

    Wire.beginTransmission(HTDU21D_ADDRESS); //begin
    Wire.write(command);                     //send the pointer location
    Wire.endTransmission();                  //end
    
    //Hang out while measurement is taken. 50mS max, page 4 of datasheet.
    sleep(55);

    //Comes back in three bytes, data(MSB) / data(LSB) / Checksum
    Wire.requestFrom(HTDU21D_ADDRESS, 3);
    
    //Wait for data to become available
    int counter = 0;
    while(Wire.available() < 3)
    {
        counter++;
        sleep(1);
        if(counter > 100) return ERROR_TIMEOUT; //Error timout
    }

    //Store the result
    result = ((Wire.read()) << 8);
    result |= Wire.read();
    //Check validity
    uint8_t checksum = Wire.read();
    if(check_crc(result, checksum) != 0) return(ERROR_CRC); //Error checksum
    //sensorStatus = rawTemperature & 0x0003; //get status bits
    result &= ~0x0003;   // clear two low bits (status bits)
    return result;
}

#define SHIFTED_DIVISOR 0x988000 //This is the 0x0131 polynomial shifted to farthest left of three bytes

uint8_t check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
  uint32_t remainder = (uint32_t)message_from_sensor << 8; // Pad with 8 bits because we have to add in the check value
  remainder |= check_value_from_sensor;                    // Add on the check value
  uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;

  for (int i = 0 ; i < 16 ; i++) //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
  {
    if( remainder & (uint32_t)1<<(23 - i) ) //Check if there is a one in the left position
      remainder ^= divsor;
    divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
  }
  return (uint8_t)remainder;
}

void presentation()
{
  // Send the sketch version information to the gateway
  sendSketchInfo("Woonkamer Sensor", "3.0");
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_VOLT, S_TEMP);
}


void setup()
{
  Wire.begin();
  setResolution(B00000001);
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0) ;
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
}

long readVcc() {

  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

float lastTemp, lastHum, avg_battery;
byte voltsend;

void loop()
{
  // Get temperature from DHT library
  send(msgTemp.set(GetTemperature(), 1));
  sleep(UPDATE_INTERVAL);

  // Get humidity from DHT library
  send(msgHum.set(GetHumidity(), 1));
  sleep(UPDATE_INTERVAL);

  voltsend++;
  avg_battery += readVcc() / 1000.0;

  if (voltsend >= 5) {
    send(msgVolt.set(avg_battery / 5.0, 2));
    sleep(UPDATE_INTERVAL);
    voltsend = 0;
    avg_battery = 0.0;
  }

  // Sleep for a while to save energy
  
}
