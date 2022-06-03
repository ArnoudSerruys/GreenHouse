// Includes
#include <DHT.h>
#include <SPI.h>
#include <WiFi.h>
#include <MFRC522.h>
#include <OneWire.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>

// definitions WiFi
const char* ssid = "MSI 6691";
const char* password = "123456789";

// definitions MQTT
const char* mqttServer = "192.168.137.64";
const int mqttPort = 1883;
const char* mqttUser = "Arnoud";
const char* mqttPassword = "raspberry";
const char* clientID = "client_greenhouse"; // MQTT client ID

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long publishPeriod = 10000;
unsigned long startPublish;

// Ground temperature sensor
#define SOILPIN  14
OneWire oneWire(SOILPIN);
DallasTemperature DS18B20(&oneWire);

float GroundTemp = 0;

// Temperature and humidity sensor (DHT22)
#define DHTPIN 4  // Pin connected to the DHT sensor
#define DHTTYPE DHT22  // DHT11 or DHT22
DHT dht(DHTPIN, DHTTYPE);

float Temp = 0;
float TempTarget = 0;
float TempRange = 0;
float Humidity = 0;

// Relays
#define RelayPump 26
bool pumpState;
bool pumpInCycle;
unsigned long timeOnPump;
unsigned long timeOffPump;
unsigned long pumpOnPeriod = 3000;
unsigned long pumpOffPeriod = 30000;

#define RelayFan 25
bool fanState;
#define RelayLights 33
bool lightsState;

// Heat Bed Controller
#define BedController 13
bool heatbedState;
#define PWM_Freq  1000

int DutyCycle = 0;

// Moisture sensor
#define MoisturePin 32

int Moisture = 0;
int MoistureMaxRange = 0;
int MoistureMinRange = 0;
int MoistureTarget = 0;

// RFID reader
#define RST_PIN    2
#define SS_PIN     5

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

byte readCard[4];
String tagID = "";
String Basilicum = "D3FCC119";
String Rozemarijn = "9CDF738";
String Plant = "Unknown";

LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lcdInfoPeriod = 4000;
unsigned long timeLCD;
unsigned long lcdInfoIdx = 0;

void setupWifi() {
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);
}

void clearLCD() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
}

void splashLCD() {
  clearLCD();

  lcd.setCursor(0, 0);
  lcd.print("Smart Greenhouse");
  lcd.setCursor(0, 1);
  lcd.print("Greenhouse Smart");
}

void infoPlantLCD() {
  String line1, line2;
  char buffer[40];
  
  clearLCD();
  
  sprintf(buffer,"Plant:%s",Plant.c_str());
  line1 = String(buffer);
  sprintf(buffer,"TagID:%s",tagID.c_str());
  line2 = String(buffer);
  
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void infoMeasuresLCD() {
  String line1, line2;
  char buffer[40];
  
  clearLCD();
  
  sprintf(buffer,"T:%+2.1f H:%+2.1f",Temp,Humidity);
  line1 = String(buffer);
  
  sprintf(buffer,"M:%+5d G:%+2.1f",Moisture,GroundTemp);
  line2 = String(buffer);
  
  // line1 = " T:" + String(Temp, 1) + "  H:" + String(Humidity, 1);
  //line2 = " M:" + String(Moisture) + "  G:" + String(GroundTemp, 1);

  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void infoActuatorsLCD() {
  String line1, line2;

  clearLCD();

  String strFan = "Off";
  if (fanState) strFan = "On ";
  String strLights = "Off";
  if (lightsState) strLights = "On ";
  String strPump = "Off";
  if (pumpState) strPump = "On ";
  String strHeatbed = "Off";
  if (heatbedState) strHeatbed = "On ";

  line1 = " Fa:" +  strFan + "  Ls:" + strLights;
  line2 = " Pu:" +  strPump + "  Hb:" + strHeatbed;

  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void startPump() {
  digitalWrite(RelayPump, LOW);
  pumpState = true;
  timeOnPump = millis();
}

void stopPump() {
  digitalWrite(RelayPump, HIGH);
  pumpState = false;
  timeOffPump = millis();
}

void cyclePump() {
  unsigned long timeNow = millis();

  if (pumpInCycle) {
    if (pumpState == true && (timeNow - timeOnPump >= pumpOnPeriod)) {
      stopPump();
    }

    if (pumpState == false && (timeNow - timeOffPump >= pumpOffPeriod)) {
      pumpInCycle = false;
    }
  }
  else {
    startPump();
    pumpInCycle = true;
  }
}

void startFan() {
  digitalWrite(RelayFan, LOW);
  fanState = true;
}

void stopFan() {
  digitalWrite(RelayFan, HIGH);
  fanState = false;
}

void startLights() {
  digitalWrite(RelayLights, LOW);
  lightsState = true;
}

void stopLights() {
  digitalWrite(RelayLights, HIGH);
  lightsState = false;
}

void startHeatbed(int dutycycle) {
  ledcWrite(0, map(dutycycle, 0, 100, 0, 256));
  heatbedState = true;
}

void stopHeatbed() {
  ledcWrite(0, 0);
  heatbedState = false;
}

void getMeasurements() {
  // Sensor Readings
  DS18B20.requestTemperatures();       // send the command to get temperatures
  float tmpTemp1 = DS18B20.getTempCByIndex(0);  // read temperature in °C
  if (tmpTemp1 > -126.5f) GroundTemp = tmpTemp1;

  float tmpTemp2 = dht.readTemperature();
  if (!isnan(tmpTemp2)) Temp = tmpTemp2;

  float tmpHumidity = dht.readHumidity();
  if (!isnan(tmpHumidity)) Humidity = tmpHumidity;

  Moisture = map(analogRead(MoisturePin), 1024, 4095, 100, 0);
}

boolean getID() {
  // Getting ready for Reading PICCs
  if ( ! mfrc522.PICC_IsNewCardPresent()) { //If a new PICC placed to RFID reader continue
    Serial.println("No new card");
    return false;
  }

  if ( ! mfrc522.PICC_ReadCardSerial()) { //Since a PICC placed get Serial and continue
    Serial.println("Couldn't read card");
    return false;
  }

  tagID = "";
  for ( uint8_t i = 0; i < 4; i++) { // The MIFARE PICCs that we use have 4 byte UID
    tagID.concat(String(mfrc522.uid.uidByte[i], HEX)); // Adds the 4 bytes in a single String variable
  }
  Serial.println("Card Read");
  tagID.toUpperCase();
  mfrc522.PICC_HaltA(); // Stop reading

  return true;
}

void MqttSendAllData() {
  // Check if WiFi still present
  if (WiFi.status() != WL_CONNECTED) {
    setupWifi();
  }

  if (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      return;
    }
  }

  if (Plant == "Basilicum") {
    client.publish("GreenHouse/Bak1/GroundTemp", String(GroundTemp).c_str());
    client.publish("GreenHouse/Bak1/Temperature", String(Temp).c_str());
    client.publish("GreenHouse/Bak1/Humidity", String(Humidity).c_str());
    client.publish("GreenHouse/Bak1/Moisture", String(Moisture).c_str());
    client.publish("GreenHouse/Bak1/Lights", String(lightsState).c_str());
    client.publish("GreenHouse/Bak1/Fan", String(fanState).c_str());
    client.publish("GreenHouse/Bak1/HeatBed", String(heatbedState).c_str());
    client.publish("GreenHouse/Bak1/Pump", String(pumpState).c_str());
  }

  if (Plant == "Rozemarijn") {
    client.publish("GreenHouse/Bak2/GroundTemp", String(GroundTemp).c_str());
    client.publish("GreenHouse/Bak2/Temperature", String(Temp).c_str());
    client.publish("GreenHouse/Bak2/Humidity", String(Humidity).c_str());
    client.publish("GreenHouse/Bak2/Moisture", String(Moisture).c_str());
    client.publish("GreenHouse/Bak2/Lights", String(lightsState).c_str());
    client.publish("GreenHouse/Bak2/Fan", String(fanState).c_str());
    client.publish("GreenHouse/Bak2/HeatBed", String(heatbedState).c_str());
    client.publish("GreenHouse/Bak2/Pump", String(pumpState).c_str());
  }

  Serial.println("Sent Data");
}

void dump() {

  Serial.print("now ");
  Serial.println(millis());
  Serial.print("pumpOn ");
  Serial.println(timeOnPump);
  Serial.print("PumpOff ");
  Serial.println(timeOffPump);
}

// Main setup function
void setup() {
  // init LCD display
  clearLCD();

  // Splash LCD
  splashLCD();

  // Setup WiFi
  setupWifi();

  // Setup Serial
  Serial.begin(115200);

  // Init ground temperature sensor (DS18B20)
  DS18B20.begin();    // initialize the DS18B20 sensor

  // Init Temperature and humidity sensor (DHT22)
  dht.begin();

  // Init RFID reader
  SPI.begin();      // Init SPI bus
  mfrc522.PCD_Init();   // Init MFRC522
  delay(4);       // Optional delay. Some board do need more time after init to be ready, see Readme
  mfrc522.PCD_DumpVersionToSerial();  // Show details of PCD - MFRC522 Card Reader details

  // pinmodes
  pinMode(MoisturePin, INPUT);
  pinMode(RelayLights, OUTPUT);
  pinMode(RelayFan, OUTPUT);
  pinMode(RelayPump, OUTPUT);

  // PWM init
  ledcAttachPin(BedController, 0);
  ledcSetup(0, PWM_Freq, 8);

  // Init actuators to default state
  stopLights();
  stopFan();
  stopHeatbed();
  stopPump();

  startPublish = millis();
  timeLCD = millis();
  lcdInfoIdx = 0;

  timeOnPump = millis();
  timeOffPump = timeOnPump;
  pumpInCycle = false;

  Plant = "Unknown";
  TempTarget = 0;
  TempRange = 100;
  MoistureTarget = 0;
  MoistureMaxRange = 100;
  MoistureMinRange = 100;

}

// Main loop function
void loop() {

  unsigned long timeNow = millis();

  // Check if we need to set a new plant
  if (getID())
  {
    // If a new plant is requested, set new parameters
    if (tagID == Basilicum && Plant != "Basilicum") {
      startLights();
      TempTarget = 20;
      TempRange = 2;
      MoistureTarget = 50;
      MoistureMaxRange = 10;
      MoistureMinRange = 15;
      Plant = "Basilicum";
    }
    else if (tagID == Rozemarijn && Plant != "Rozemarijn") {
      startLights();
      TempTarget = 25;
      TempRange = 2;
      MoistureTarget = 50;
      MoistureMaxRange = 10;
      MoistureMinRange = 5;
      Plant = "Rozemarijn";
    }
    else {
      if (Plant != "Unknown") {
        stopLights();
        TempTarget = 0;
        TempRange = 100;
        MoistureTarget = 0;
        MoistureMaxRange = 100;
        MoistureMinRange = 100;
        Plant = "Unknown";
      }
    }
  }

  getMeasurements();

  // Compare mean(groundtemperature, temperature) with desired value, if lower than desired, activate heatbed
  float meanTemp = (GroundTemp + Temp) / 2.0f;
  Serial.println(meanTemp);
  Serial.println(TempTarget);
  Serial.println(TempRange);
  if (meanTemp < TempTarget - TempRange) {
    Serial.println("cold");
    stopFan();
    startHeatbed(50);
  }
  else if (meanTemp > TempTarget + TempRange) {
    Serial.println("hot");
    stopHeatbed();
    startFan();
  }
  else if ( meanTemp < TempTarget + TempRange && meanTemp > TempTarget - TempRange) {
    Serial.println("perfect");
    stopHeatbed();
    stopFan();
  }

  // Compare moisture with desired value and activate pump if too low.
  if (Moisture < MoistureTarget - MoistureMinRange) {
    cyclePump();
  }

  if (timeNow - timeLCD >= lcdInfoPeriod) {
    switch (lcdInfoIdx) {
      case 0:
        infoPlantLCD();
        break;
      case 1:
        infoMeasuresLCD();
        break;
      case 2:
        infoActuatorsLCD();
        break;
      default:
        break;
    }
    lcdInfoIdx++;
    if (lcdInfoIdx > 2) lcdInfoIdx = 0;
    timeLCD = timeNow;
  }

  if (timeNow - startPublish >= publishPeriod) {
    MqttSendAllData();
    startPublish = timeNow;
  }
  delay(50);
  //dump();

  //Serial.println(dht.readTemperature());
}
