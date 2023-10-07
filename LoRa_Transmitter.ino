#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial gpsSerial(4, 3);
SoftwareSerial LoRaSerial(10, 11);
Adafruit_GPS GPS(&gpsSerial);

#define M0 7
#define M1 6

float latitude;
float longitude;

uint32_t latitudeIntPart = 0;  
uint32_t longitudeIntPart = 0;
uint32_t latitudeDecimalPart = 0;
uint32_t longitudeDecimalPart = 0;

static const uint32_t GPSBaud = 9600;

char c;

//--------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  
  LoRaSerial.begin(9600);
  gpsSerial.begin(GPSBaud);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);
}

// -------------------------------------------------------------------------------------
void loop() {
  gpsSerial.listen();
  clearGPS();
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    GPS.parse(GPS.lastNMEA());    
    latitude = GPS.latitudeDegrees;
    longitude = GPS.longitudeDegrees;
  
    // Enlem ve boylamı tam sayı ve ondalık olarak ayırın
    latitudeIntPart = static_cast<uint32_t>(latitude);
    latitudeDecimalPart = static_cast<uint32_t>((latitude - latitudeIntPart) * 1000000.0);
    
    longitudeIntPart = static_cast<uint32_t>(longitude);
    longitudeDecimalPart = static_cast<uint32_t>((longitude - longitudeIntPart) * 1000000.0);

    // LoRa ile veriyi gönderin
    sendGPSData();

    Serial.print("KONUM: ");
    Serial.print(GPS.latitudeDegrees, 6);
    Serial.print(", ");
    Serial.println(GPS.longitudeDegrees, 6);
    Serial.println("-------------------------------------");
  }
}
    
// -------------------------------------------------------------------------------------
void clearGPS() {
  while (!GPS.newNMEAreceived()) {
    c = GPS.read(); }
  GPS.parse(GPS.lastNMEA());
  while (!GPS.newNMEAreceived()) {
    c = GPS.read(); }
  GPS.parse(GPS.lastNMEA());
}

void sendGPSData() {
  LoRaSerial.listen();
  delay(100);
  LoRaSerial.write((byte)0x00);      // Alıcı Adresi HIGH
  LoRaSerial.write(2);               // Alıcı Adresi LOW
  LoRaSerial.write(18);              // Alıcı Kanalı 

  LoRaSerial.write((byte*)&latitudeIntPart, sizeof(uint32_t)); 
  LoRaSerial.write((byte*)&latitudeDecimalPart, sizeof(uint32_t));
  LoRaSerial.write((byte*)&longitudeIntPart, sizeof(uint32_t));
  LoRaSerial.write((byte*)&longitudeDecimalPart, sizeof(uint32_t));
}
