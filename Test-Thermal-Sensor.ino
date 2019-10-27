#include <DallasTemperature.h>

//-------------------------------------------------
//   DEFINE
//-------------------------------------------------
// VERSION NUMBER
#define SOFTWARE "TEST-THERMAL-SNESOR"
#define VERSION "1.0.0 - Compatible de AsyncTCP v1.0.0 et ESP32 v1.0.0"

//#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#define TEMPERATURE_PRECISION 11
// USB serial line bitrate
#define USBSERIAL_BITRATE 115200

// set GPIO pin numbers
//   Side 1
const int pTempDS18B20 = 13;  // the number of the TempDS18B20 pin

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(pTempDS18B20);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature DallasSensors(&oneWire);
uint8_t DallasDeviceCount;

// arrays to hold device addresses
DeviceAddress Device0_Thermometer, Device1_Thermometer, Device2_Thermometer;

float Temp0 = -256.0;
float Temp1 = -256.0;
float Temp2 = -256.0;

//**********************************************
//  function to print a 1-Wire device address
//**********************************************
void print1wireAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

//*******************************************************
//  function to convert a 1-Wire device address to String
//*******************************************************
String String1wireAddress(DeviceAddress deviceAddress)
{
  String address = "";
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) address += "0";
    address += String(deviceAddress[i], HEX);
  }
  return address;
}

//***************************************************
// function to print the temperature for a device
//***************************************************
void print1wireTemperature(DeviceAddress deviceAddress)
{
  float tempC = DallasSensors.getTempC(deviceAddress);
  Serial.print("Temp C : ");
  Serial.println(tempC);
}


void setup() {
  //-------------------------------
  // initialize serial
  //-------------------------------
  Serial.begin(USBSERIAL_BITRATE);
  delay(10);

  // put your setup code here, to run once:
  delay (2000); // retarder l'init des thermistance
  Serial.println(F("Initializing Dallas Temperature IC Control Library..."));
  // Start up the library
  DallasSensors.begin();

  delay (2000); // retarder l'init des thermistance
  // locate devices on the 1-Wire bus
  Serial.print(F("Locating 1-Wire devices..."));
  Serial.print("Found ");
  Serial.print(DallasSensors.getDeviceCount());
  Serial.print(" devices.");
  Serial.print(" Found ");
  DallasDeviceCount = DallasSensors.getDS18Count();
  Serial.print(DallasDeviceCount);
  Serial.println(" Dallas devices.");

  if (DallasSensors.getAddress(Device0_Thermometer, 0)) {
    Serial.print ("Device 0 address: ");
    print1wireAddress(Device0_Thermometer);
    Serial.println ();
    delay (2000);
  }
  else {
    Serial.println(F("  Unable to find address for Device 0"));
    delay (2000);
  }
  if (DallasSensors.getAddress(Device1_Thermometer, 1)) {
    Serial.print ("Device 1 address : ");
    print1wireAddress(Device1_Thermometer);
    Serial.println ();
    delay (2000);
  }
  else {
    Serial.println(F("  Unable to find address for Device 1"));
    delay (2000);
  }

  if (DallasSensors.getAddress(Device2_Thermometer, 2)) {
    Serial.print ("Device 2 address : ");
    print1wireAddress(Device2_Thermometer);
    Serial.println ();
    delay (2000);
  }
  else {
    Serial.println(F("  Unable to find address for Device 2"));
    delay (2000);
  }

  // report parasite power requirements
  Serial.print("Parasite power is : ");
  if (DallasSensors.isParasitePowerMode()) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }

  // set the resolution to N bit per device
  //  DallasSensors.setResolution(Device0_Thermometer, TEMPERATURE_PRECISION);
  //  DallasSensors.setResolution(Device1_Thermometer, TEMPERATURE_PRECISION);
  DallasSensors.setResolution(TEMPERATURE_PRECISION);

  delay (2000); // retarder l'init des thermistance
  Serial.print("Device 0 Resolution : ");
  Serial.println(DallasSensors.getResolution(Device0_Thermometer));
  Serial.print("Device 1 Resolution : ");
  Serial.println(DallasSensors.getResolution(Device1_Thermometer));
  Serial.print("Device 2 Resolution : ");
  Serial.println(DallasSensors.getResolution(Device2_Thermometer));

  // Get the initial temperatures
  DallasSensors.requestTemperatures(); // Send the command to get temperature readings
  Temp0 = DallasSensors.getTempCByIndex(0);
  Serial.print("Temperature 0 is : "); Serial.println(Temp0);
  Temp1 = DallasSensors.getTempCByIndex(1);
  Serial.print("Temperature 1 : "); Serial.println(Temp1);
  Temp2 = DallasSensors.getTempCByIndex(2);
  Serial.print("Temperature 2 : "); Serial.println(Temp2);
}

void loop() {
  // put your main code here, to run repeatedly:

}
