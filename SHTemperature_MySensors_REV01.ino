#define MY_NODE_ID 20
#define MY_RADIO_NRF24
#define AVG_COUNT 5
/**
 * RF24_PA_LOW = -18dBm
 * RF24_PA_MID = -12dBm
 * RF24_PA_HIGH = -6dBm
 * RF24_PA_MAX = 0dBm
 */
#define MY_RF24_PA_LEVEL RF24_PA_HIGH

#include <SPI.h>
#include <Wire.h>
#include <MySensors.h>
#include <SHT2x.h>

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_VOLT 2

static const uint64_t UPDATE_INTERVAL = 60000;
float lastTemp, lastHum, avgBattery;
int batteryMeasureIndex;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgVolt(CHILD_ID_VOLT, V_TEMP);

void presentation()
{
  sendSketchInfo("MySensors SHT20 Temperature", "2.0");
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_VOLT, S_TEMP);
}

void setup()
{
  Wire.begin();
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
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

void loop()
{
  float temperature = SHT2x.GetTemperature();
  if (temperature != lastTemp) {
    send(msgTemp.set(temperature, 2));
    lastTemp = temperature;
  }

  float humidity = SHT2x.GetHumidity();
  if (humidity != lastHum) {
    send(msgHum.set(humidity, 2));
    lastHum = humidity;
  }

  sleep(UPDATE_INTERVAL);

  batteryMeasureIndex++;
  avgBattery += readVcc() / 1000.0;
  if(batteryMeasureIndex >= AVG_COUNT) {
    send(msgVolt.set(avgBattery / AVG_COUNT, 2));
    batteryMeasureIndex = 0;
    avgBattery = 0;
  }
}

