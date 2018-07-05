/**2.20/**
   REVISION HISTORY
   Version 1.0 - Nathan Metcalf

   Battery Relay with Temp.

*/
#define MY_NODE_ID 210

// Enable debug prints to serial monitor
#define MY_DEBUG

// Set blinking period (in milliseconds)
#define MY_DEFAULT_LED_BLINK_PERIOD 300

#define MY_DEFAULT_TX_LED_PIN 3
#define MY_DEFAULT_RX_LED_PIN 2

// Enable and select radio type attached
#define MY_RADIO_NRF24

#define MY_RF24_CE_PIN 9
#define MY_RF24_CS_PIN 10

#define MY_RF24_PA_LEVEL RF24_PA_HIGH

// Enable repeater functionality for this node
#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>
#include <DHT.h>

#define RELAY_PIN 5  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 3 // Total number of attached relays
#define RELAY_ON 0  // GPIO value to write to turn on attached relay
#define RELAY_OFF 1 // GPIO value to write to turn off attached relay

// DHT11 STUFF
// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 4

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 60000;

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define CHILD_ID_HUM 5
#define CHILD_ID_TEMP 6
#define CHILD_ID_VOLT 7

float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = true;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgBatteryVolt(CHILD_ID_VOLT, V_VOLTAGE);
DHT dht;

// END DHT11

// Voltage Devider
int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point
int oldBatteryPcnt = 0;
float oldBatteryVolt = 0;
// End Voltage

void before()
{
  for (int sensor = 1, pin = RELAY_PIN; sensor <= NUMBER_OF_RELAYS; sensor++, pin++) {
    // Then set relay pins in output mode
    pinMode(pin, OUTPUT);
    // Set relay to last known state (using eeprom storage)
    digitalWrite(pin, loadState(sensor) ? RELAY_ON : RELAY_OFF);
  }
}

void setup()
{
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL <= dht.getMinimumSamplingPeriod()) {
    Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
  }
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht.getMinimumSamplingPeriod());
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("NDM.Guru Office", "1.5");

  for (int sensor = 1, pin = RELAY_PIN; sensor <= NUMBER_OF_RELAYS; sensor++, pin++) {
    // Register all sensors to gw (they will be created as child devices)
    present(sensor, S_BINARY);
  }
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present( CHILD_ID_VOLT, S_MULTIMETER );
  metric = getControllerConfig().isMetric;
  
}


void loop()
{

  // 1M, 100K divider across battery and using internal ADC ref of 1.1V
  // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
  // ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
  // 3.44/1023 = Volts per bit = 0.003363075

  float batteryVolt = getBatteryVoltage();
  int batteryPcnt = getBatteryPercentage();

  #ifdef MY_DEBUG
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVolt);
  Serial.println(" V");

  Serial.print("Battery percent: ");
  Serial.print(batteryPcnt);
  Serial.println(" %");
  #endif

  if (oldBatteryVolt != batteryVolt) {
      // Power up radio after sleep
      send(msgBatteryVolt.set(batteryVolt, 3));      
      oldBatteryVolt = batteryVolt;
  }

  delay(10);

  if (oldBatteryPcnt != batteryPcnt) {
      // Power up radio after sleep
      sendBatteryLevel(batteryPcnt);
      
      oldBatteryPcnt = batteryPcnt;
  }
    
  // Force reading sensor, so it works also after sleep()
  dht.readSensor(true);

  // Get temperature from DHT library
  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT!");
  } else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {
    // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
    lastTemp = temperature;
    if (!metric) {
      temperature = dht.toFahrenheit(temperature);
    }
    // Reset no updates counter
    nNoUpdatesTemp = 0;
    temperature += SENSOR_TEMP_OFFSET;
    send(msgTemp.set(temperature, 1));

    #ifdef MY_DEBUG
    Serial.print("Temperature: ");
    Serial.println(temperature);
    #endif
  } else {
    // Increase no update counter if the temperature stayed the same
    nNoUpdatesTemp++;
  }

delay(10);

  // Get humidity from DHT library
  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity from DHT");
  } else if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS) {
    // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
    lastHum = humidity;
    // Reset no updates counter
    nNoUpdatesHum = 0;
    send(msgHum.set(humidity, 1));

    #ifdef MY_DEBUG
    Serial.print("Humidity: ");
    Serial.println(humidity);
    #endif
  } else {
    // Increase no update counter if the humidity stayed the same
    nNoUpdatesHum++;
  }

  // Sleep for a while to save energy
  sleep(UPDATE_INTERVAL); 
}

void receive(const MyMessage &message)
{
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type == V_STATUS) {
    // Change relay state
    digitalWrite(message.sensor - 1 + RELAY_PIN, message.getBool() ? RELAY_ON : RELAY_OFF);
    // Store state in eeprom
    saveState(message.sensor, message.getBool());
    // Write some debug info
    Serial.print("Incoming change for sensor:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
  }
}

float getBatteryVoltage() {

  
  // get the battery Voltage
  float sensorValue = analogRead(BATTERY_SENSE_PIN);
 
  float batteryVoltage = (((sensorValue * 5.00) / 1024) * 11.252);
 
  return batteryVoltage;
}

int getBatteryPercentage() {
  float batteryVoltage = getBatteryVoltage();
  int VccMin = 2.80;
  int VccMax = 4.25;
  
  int batteryPcnt = constrain(map(batteryVoltage, VccMin, VccMax, 0, 100),0,100); // and map to the 0-100% range
  return batteryPcnt;
}

