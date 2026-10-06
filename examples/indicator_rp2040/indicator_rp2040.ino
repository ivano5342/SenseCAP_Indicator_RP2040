#include <Arduino.h>
#include "Adafruit_SHT4x.h" //#include <SensirionI2cSht4x.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2cScd4x.h>
#include <SensirionI2CSen5x.h>
#include <MiCS6814-I2C.h>
#include <VOCGasIndexAlgorithm.h>
#include "SparkFun_BMV080_Arduino_Library.h"
#include "bme68xLibrary.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PacketSerial.h>
//#include "AHT20.h"
#include "TCA9548A.h"   // grove 8 channel I2C expander

#define DEBUG 0

#define VERSION "v1.0.1"

#define SENSECAP "\n\
   _____                      _________    ____         \n\
  / ___/___  ____  ________  / ____/   |  / __ \\       \n\
  \\__ \\/ _ \\/ __ \\/ ___/ _ \\/ /   / /| | / /_/ /   \n\
 ___/ /  __/ / / (__  )  __/ /___/ ___ |/ ____/         \n\
/____/\\___/_/ /_/____/\\___/\\____/_/  |_/_/           \n\
--------------------------------------------------------\n\
 Version: %s \n\
--------------------------------------------------------\n\
"

Adafruit_SHT4x  sht4x = Adafruit_SHT4x(); //AHT20 AHT;
SensirionI2CSgp40 sgp40;
SensirionI2cScd4x scd4x;
VOCGasIndexAlgorithm voc_algorithm;
MiCS6814 mics;
SparkFunBMV080 bmv080; // Create an instance of the BMV080 class
TCA9548A<TwoWire> TCA;  // i2c expander
Bme68x bme688;

PacketSerial myPacketSerial;

String SDDataString = "";

#define BME688_ADR  0x76  // Adafruit BME688 Breakout defualts to 0x77 (0x76 with pull-down)
#define BMV080_ADDR 0x57  // SparkFun BMV080 Breakout defaults to 0x57


//Type of transfer packet
#define PKT_TYPE_SENSOR_BMV080_PM1      0xAF
#define PKT_TYPE_SENSOR_BMV080_PM25     0xB0
#define PKT_TYPE_SENSOR_BMV080_P10      0xB1
#define PKT_TYPE_SENSOR_SCD41_CO2       0XB2
#define PKT_TYPE_SENSOR_SHT41_TEMP      0XB3
#define PKT_TYPE_SENSOR_SHT41_HUMIDITY  0XB4
#define PKT_TYPE_SENSOR_TVOC_INDEX      0XB5
#define PKT_TYPE_SENSOR_SEN55_PM1       0xB6
#define PKT_TYPE_SENSOR_SEN55_PM2_5     0xB7
#define PKT_TYPE_SENSOR_SEN55_PM4       0xB8
#define PKT_TYPE_SENSOR_SEN55_PM10      0xB9
#define PKT_TYPE_SENSOR_SEN55_VOCINDEX  0xBA
#define PKT_TYPE_SENSOR_SEN55_NOX       0xBB
#define PKT_TYPE_SENSOR_SEN55_TEMP      0xBC
#define PKT_TYPE_SENSOR_SEN55_HUMIDITY  0xBD

#define PKT_TYPE_CMD_COLLECT_INTERVAL   0xA0
#define PKT_TYPE_CMD_BEEP_ON            0xA1
#define PKT_TYPE_CMD_SHUTDOWN           0xA3

//BME688 defines
#define MEAS_DUR 140


// sensor data send to  esp32
void sensor_data_send(uint8_t type, float data) {
  uint8_t data_buf[32] = { 0 };
  int index = 0;

  data_buf[0] = type;
  index++;

  memcpy(&data_buf[1], &data, sizeof(float));
  index += sizeof(float);

  myPacketSerial.send(data_buf, index);

#if DEBUG
  Serial.printf("---> send len:%d, data: ", index);
  for (int i = 0; i < index; i++) {
    Serial.printf("0x%x ", data_buf[i]);
  }
  Serial.println("");
#endif
}


void printUint16Hex(uint16_t value) {
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
  Serial.print("Serial: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}

void sensor_power_on(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
}

void sensor_power_off(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
}

float bmv_pm10 = 0.0;
float bmv_pm25 = 0.0;
float bmv_pm1 = 0.0;

float temperature = 0.0;
float humidity = 0.0;

uint16_t defaultCompenstaionRh = 0x8000;
uint16_t defaultCompenstaionT = 0x6666;

uint16_t compensationRh = defaultCompenstaionRh;
uint16_t compensationT = defaultCompenstaionT;

bool mics_connected = false;
bool bme688_connected = false;
bool bmv080_connected = false;



/************************ aht  temp & humidity ****************************/

void sensor_sht4x_init(void) {
  sht4x.begin();//, SHT41_I2C_ADDR_44);
  sht4x.setPrecision(SHT4X_HIGH_PRECISION);
  sht4x.setHeater(SHT4X_NO_HEATER);
  //sht4x.softReset();
}

void sensor_sht4x_get(void) {

  sensors_event_t temp, humi;

  int ret = sht4x.getEvent(&humi, &temp);
  if (ret)  // GET DATA OK
  {
    temperature = temp.temperature;
    humidity = humi.relative_humidity;// * 100;
    Serial.print("humidity: ");
    Serial.print(humidity);
    Serial.print("%\t temerature: ");
    Serial.println((temperature*1.8) +32);
    compensationT = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
    compensationRh = static_cast<uint16_t>(humidity * 65535 / 100);
  } 
  else  // GET DATA FAIL
  {
    Serial.println("GET DATA FROM SHT41 FAIL");
    compensationRh = defaultCompenstaionRh;
    compensationT = defaultCompenstaionT;
  }

  SDDataString += "sht41,";
  if (ret) {
    SDDataString += String(temperature);
    SDDataString += ',';
    SDDataString += String(humidity);
    SDDataString += ',';

    sensor_data_send(PKT_TYPE_SENSOR_SHT41_TEMP, temperature);
    sensor_data_send(PKT_TYPE_SENSOR_SHT41_HUMIDITY, humidity);
  } else {
    SDDataString += "-,-,";
  }
}

/************************ sgp40 tvoc  ****************************/

void sensor_sgp40_init(void) {
  uint16_t error;
  char errorMessage[256];

  sgp40.begin(Wire);

  uint16_t serialNumber[3];
  uint8_t serialNumberSize = 3;

  error = sgp40.getSerialNumber(serialNumber, serialNumberSize);

  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("SerialNumber:");
    Serial.print("0x");
    for (size_t i = 0; i < serialNumberSize; i++) {
      uint16_t value = serialNumber[i];
      Serial.print(value < 4096 ? "0" : "");
      Serial.print(value < 256 ? "0" : "");
      Serial.print(value < 16 ? "0" : "");
      Serial.print(value, HEX);
    }
    Serial.println();
  }

  uint16_t testResult;
  error = sgp40.executeSelfTest(testResult);
  if (error) {
    Serial.print("Error trying to execute executeSelfTest(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (testResult != 0xD400) {
    Serial.print("executeSelfTest failed with error: ");
    Serial.println(testResult);
  }
}

void sensor_sgp40_get(void) {
  uint16_t error;
  char errorMessage[256];
  uint16_t defaultRh = 0x8000;
  uint16_t defaultT = 0x6666;
  uint16_t srawVoc = 0;

  Serial.print("sensor sgp40: ");

  error = sgp40.measureRawSignal(compensationRh, compensationT, srawVoc);
  if (error) {
    Serial.print("Error trying to execute measureRawSignal(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("SRAW_VOC:");
    Serial.println(srawVoc);
  }

  SDDataString += "sgp40,";
  if (error) {
    SDDataString += "-,";
  } else {
    SDDataString += String(srawVoc);
    SDDataString += ',';

    int32_t voc_index = voc_algorithm.process(srawVoc);
    Serial.print("VOC Index: ");
    Serial.println(voc_index);

    sensor_data_send(PKT_TYPE_SENSOR_TVOC_INDEX, (float)voc_index);
  }
}


/************************ scd4x  co2 ****************************/

void sensor_scd4x_init(void) {
  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire, 0x62);

  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  uint64_t serialNumber;
  error = scd4x.getSerialNumber(serialNumber);
  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("Serial number: ");
    Serial.println((uint32_t)(serialNumber >> 32), HEX);
    Serial.println((uint32_t)(serialNumber & 0xFFFFFFFF), HEX);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  // scd4x.powerDown();
}

void sensor_scd4x_get(void) {
  uint16_t error;
  char errorMessage[256];

  Serial.print("sensor scd4x: ");
  // Read Measurement
  uint16_t co2;
  float temperature;
  float humidity;
  error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error) {
    Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (co2 == 0) {
    Serial.println("Invalid sample detected, skipping.");
  } else {
    Serial.print("Co2:");
    Serial.print(co2);
    Serial.print("\t");
    Serial.print("Temperature:");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print("Humidity:");
    Serial.println(humidity);
  }

  SDDataString += "scd4x,";
  if (error) {
    SDDataString += "-,-,-,";
  } else {
    SDDataString += String(co2);
    SDDataString += ',';
    SDDataString += String(temperature);
    SDDataString += ',';
    SDDataString += String(humidity);
    SDDataString += ',';


    sensor_data_send(PKT_TYPE_SENSOR_SCD41_CO2, (float)co2);  //todo
  }
}

/************************ beep ****************************/

#define Buzzer 19  //Buzzer GPIO

void beep_init(void) {
  pinMode(Buzzer, OUTPUT);
}
void beep_off(void) {
  digitalWrite(19, LOW);
}
void beep_on(void) {
  analogWrite(Buzzer, 127);
  delay(50);
  analogWrite(Buzzer, 0);
}



/************************ grove  ****************************/

void grove_adc_get(void) {
  String dataString = "";
  int adc0 = analogRead(26);
  dataString += String(adc0);
  dataString += ',';
  int adc1 = analogRead(27);
  dataString += String(adc1);
  Serial.print("grove adc: ");
  Serial.println(dataString);
}


/************************ recv cmd from esp32  ****************************/

static bool shutdown_flag = false;

void onPacketReceived(const uint8_t *buffer, size_t size) {

#if DEBUG
  Serial.printf("<--- recv len:%d, data: ", size);
  for (int i = 0; i < size; i++) {
    Serial.printf("0x%x ", buffer[i]);
  }
  Serial.println("");
#endif
  if (size < 1) {
    return;
  }
  switch (buffer[0]) {
    case PKT_TYPE_CMD_SHUTDOWN:
      {
        Serial.println("cmd shutdown");
        shutdown_flag = true;
        sensor_power_off();
        break;
      }
    default:
      break;
  }
}

/************************ setuo & loop ****************************/

int cnt = 0;
int i = 0;
bool sd_init_flag = 0;

void setup() {
  Serial.begin(115200);

  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);
  myPacketSerial.setStream(&Serial1);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  sensor_power_on();

  Wire.setSDA(20);
  Wire.setSCL(21);
  TCA.begin(Wire);//Wire.begin();
  TCA.openChannel(TCA_CHANNEL_0); //TCA.closeChannel(TCA_CHANNEL_0);
  TCA.openChannel(TCA_CHANNEL_1); //TCA.closeChannel(TCA_CHANNEL_1);
  TCA.openChannel(TCA_CHANNEL_2); //TCA.closeChannel(TCA_CHANNEL_2);
  TCA.openChannel(TCA_CHANNEL_3); //TCA.closeChannel(TCA_CHANNEL_3);
  TCA.openChannel(TCA_CHANNEL_4); //TCA.closeChannel(TCA_CHANNEL_4);
  TCA.openChannel(TCA_CHANNEL_5); //TCA.closeChannel(TCA_CHANNEL_5);
  TCA.openChannel(TCA_CHANNEL_6); //TCA.closeChannel(TCA_CHANNEL_6);
  TCA.openChannel(TCA_CHANNEL_7); //TCA.closeChannel(TCA_CHANNEL_7); 

  const int chipSelect = 13;
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  if (!SD.begin(chipSelect, 1000000, SPI1)) {
    Serial.println("Card failed, or not present");
    sd_init_flag = 0;
  } else {
    Serial.println("card initialized.");
    sd_init_flag = 1;
  }

  sensor_sht4x_init();
  sensor_sgp40_init();
  sensor_scd4x_init();
  bme688.begin(BME688_ADR, Wire);
  {
    bme688_connected = true;
    /* Setting the default heater profile configuration */
    bme688.setTPH();
    /* Heater temperature in degree Celsius as per the suggested heater profile
    */
    uint16_t tempProf[10] = {320, 100, 100, 100, 200, 200, 200, 320, 320, 320};
    /* Multiplier to the shared heater duration */
    uint16_t mulProf[10] = {5, 2, 10, 30, 5, 5, 5, 5, 5, 5};
    /* Shared heating duration in milliseconds */
    uint16_t sharedHeatrDur =
        MEAS_DUR - (bme688.getMeasDur(BME68X_PARALLEL_MODE) / INT64_C(1000));

    bme688.setHeaterProf(tempProf, mulProf, sharedHeatrDur, 10);

    /* Parallel mode of sensor operation */
    bme688.setOpMode(BME68X_PARALLEL_MODE);
  }
  if (bmv080.begin(BMV080_ADDR, Wire) == true)
  {
    bmv080_connected = true;
    Serial.println("BMV080 found!");
    bmv080.init();
    /* Set the sensor mode to continuous mode */
    if (bmv080.setMode(SF_BMV080_MODE_CONTINUOUS) == true)
    {
        Serial.println("BMV080 set to continuous mode");
    }
    else
    {
        Serial.println("Error setting BMV080 mode");
    }
  }

  mics_connected = 0;//mics.begin(0x08);//the default I2C address of the slave is 0x04
  if (mics_connected)
  {
    mics.powerOn();
  }

  int32_t index_offset;
  int32_t learning_time_offset_hours;
  int32_t learning_time_gain_hours;
  int32_t gating_max_duration_minutes;
  int32_t std_initial;
  int32_t gain_factor;
  voc_algorithm.get_tuning_parameters(
    index_offset, learning_time_offset_hours, learning_time_gain_hours,
    gating_max_duration_minutes, std_initial, gain_factor);

  Serial.println("\nVOC Gas Index Algorithm parameters");
  Serial.print("Index offset:\t");
  Serial.println(index_offset);
  Serial.print("Learing time offset hours:\t");
  Serial.println(learning_time_offset_hours);
  Serial.print("Learing time gain hours:\t");
  Serial.println(learning_time_gain_hours);
  Serial.print("Gating max duration minutes:\t");
  Serial.println(gating_max_duration_minutes);
  Serial.print("Std inital:\t");
  Serial.println(std_initial);
  Serial.print("Gain factor:\t");
  Serial.println(gain_factor);


  beep_init();
  delay(500);
  beep_on();

  Serial.printf(SENSECAP, VERSION);
}

int val = 0;
String logHeader;
uint8_t lastMeasindex = 0;
bme68xData sensorData;
uint32_t lastLogged = 0;


void loop() {
  if (i > 500) {
    i = 0;

    SDDataString = "";
    Serial.printf("\r\n\r\n--------- start measure %d-------\r\n", cnt);

    SDDataString += String(cnt);
    SDDataString += ',';

    cnt++;
    sensor_sht4x_get();
    sensor_sgp40_get();
    sensor_scd4x_get();

    /* Control loop for data acquisition - checks if the data is available */
    uint8_t nFieldsLeft = 0;
    int16_t indexDiff;
    bool newLogdata = false;
    if ((millis() - lastLogged) >= MEAS_DUR) {

      lastLogged = millis();
      if (bme688.fetchData()) {
        do {
          nFieldsLeft = bme688.getData(sensorData);
          /* Check if new data is received */
          if (sensorData.status & BME68X_NEW_DATA_MSK) {
            ///* Inspect miss of data index */
            //indexDiff =
            //    (int16_t)sensorData.meas_index - (int16_t)lastMeasindex;
            //if (indexDiff > 1) {
//
            //  Serial.println("Skip nfield:" + String(nFieldsLeft) +
            //                 ", DIFF:" + String(indexDiff) +
            //                 ", MI:" + String(sensorData.meas_index) +
            //                 ", LMI:" + String(lastMeasindex) +
            //                 ", S:" + String(sensorData.status, HEX));
            //  continue;//panicLeds();
            //}
            //lastMeasindex = sensorData.meas_index;
            
            logHeader = "bme688  ";
            //logHeader += millis();
            //logHeader += ":";
            logHeader += nFieldsLeft;
            logHeader += ":";
            logHeader += ((sensorData.temperature*1.8)+32);
            logHeader += ",";
            logHeader += sensorData.pressure;
            logHeader += ",";
            logHeader += sensorData.humidity;
            logHeader += ",";
            logHeader += sensorData.gas_resistance;
            logHeader += ",";
            logHeader += sensorData.gas_index;
            logHeader += ",";
            logHeader += sensorData.meas_index;
            logHeader += ",";
            logHeader += sensorData.idac;
            logHeader += ",";
            logHeader += String(sensorData.status, HEX);
            logHeader += ",";
            logHeader += sensorData.status & BME68X_GASM_VALID_MSK;
            logHeader += ",";
            logHeader += sensorData.status & BME68X_HEAT_STAB_MSK;
            logHeader += "\r\n";
            Serial.print(logHeader);
            newLogdata = true;
          }
        } while (nFieldsLeft);
      }
    }

    if (bmv080_connected && bmv080.readSensor())
    {
        char buf[20];
        bmv_pm10 = bmv080.PM10();
        bmv_pm25 = bmv080.PM25();
        bmv_pm1 = bmv080.PM1();
        dtostrf(bmv_pm10, 4, 1, buf);
        Serial.print("PM10: ");
        Serial.print(buf);
        Serial.print("\t");
        Serial.print("PM2.5: ");
        Serial.print(bmv_pm25);
        Serial.print("\t");
        Serial.print("PM1: ");
        Serial.print(bmv_pm1);

        sensor_data_send(PKT_TYPE_SENSOR_BMV080_PM25, (float)bmv_pm25);  //todo

        if (bmv080.isObstructed() == true)
        {
            Serial.print("\tObstructed");
        }

        Serial.println();
    }
    if (mics_connected)
    {
      val = mics.measureCO();
      if (val > 999) val = 999;
      Serial.print("CO:");
      Serial.print(val);
      Serial.print("\t");
      val = mics.measureNO2();
      if (val > 999) val = 999;
      Serial.print("NO2:");
      Serial.print(val);
      Serial.print("\t");
      val = mics.measureC2H5OH();
      if (val > 999) val = 999;
      Serial.print("C2H5CH:");
      Serial.print(val);
      Serial.print("\t");
      val = mics.measureNH3();
      if (val > 999) val = 999;
      Serial.print("NH3:");
      Serial.print(val);
      Serial.print("\t");
      val = mics.measureC3H8();
      if (val > 999) val = 999;
      Serial.print("C3H8:");
      Serial.print(val);
      Serial.print("\t");
      val = mics.measureC4H10();
      if (val > 999) val = 999;
      Serial.print("C4H10:");
      Serial.print(val);
      Serial.print("\t");
      val = mics.measureCH4();
      if (val > 999) val = 999;
      Serial.print("C4:");
      Serial.print(val);
      Serial.print("\t");
      val = mics.measureH2();
      if (val > 999) val = 999;
      Serial.print("H2:");
      Serial.print(val);
      Serial.println("\t");
    }
    grove_adc_get();

    if (sd_init_flag) {
      File dataFile = SD.open("datalog.csv", FILE_WRITE);
      // if the file is available, write to it:
      if (dataFile) {
        dataFile.println(SDDataString);
        dataFile.close();
        // print to the serial port too:
        Serial.print("sd write: ");
        Serial.println(SDDataString);
      } else {
        Serial.println("error opening datalog.txt");
      }
    }
  }

  i++;

  myPacketSerial.update();
  if (myPacketSerial.overflow()) {
  }
  delay(10);

  // while( shutdown_flag) {
  //    delay(10);
  // }
}
