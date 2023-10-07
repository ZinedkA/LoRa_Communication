#include <Wire.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2); // Bağlantı pinlerini burada tanımlayın
SoftwareSerial LoRaSerial(10, 11); // LoRa modül bağlantılarına göre değiştirin
int M0 = 7;
int M1 = 6;

void setup() {
  Serial.begin(115200);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  LoRaSerial.begin(9600);
  lcd.begin(16, 2); // LCD ekranın boyutuna göre değiştirin
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("E:");
  lcd.setCursor(0, 1);
  lcd.print("B:");
}

void loop() {
  if (LoRaSerial.available() >= 25) {
    byte receiverAddressHigh = LoRaSerial.read();
    byte receiverAddressLow = LoRaSerial.read();
    byte receiverChannel = LoRaSerial.read();

    uint32_t latitudeIntPart = 0;
    uint32_t latitudeDecimalPart = 0;
    uint32_t longitudeIntPart = 0;
    uint32_t longitudeDecimalPart = 0;

    LoRaSerial.readBytes((byte*)&latitudeIntPart, sizeof(uint32_t));
    LoRaSerial.readBytes((byte*)&latitudeDecimalPart, sizeof(uint32_t));
    LoRaSerial.readBytes((byte*)&longitudeIntPart, sizeof(uint32_t));
    LoRaSerial.readBytes((byte*)&longitudeDecimalPart, sizeof(uint32_t));

    float latitude = latitudeIntPart + latitudeDecimalPart / 1000000.0;
    float longitude = longitudeIntPart + longitudeDecimalPart / 1000000.0;

    lcd.setCursor(3, 0);
    lcd.print("              "); // Eski veriyi temizle
    lcd.setCursor(3, 0);
    lcd.print(latitude, 6);

    lcd.setCursor(3, 1);
    lcd.print("              "); // Eski veriyi temizle
    lcd.setCursor(3, 1);
    lcd.print(longitude, 6);

    Serial.print("Alınan Konum: ");
    Serial.print(latitude, 6);
    Serial.print(", ");
    Serial.println(longitude, 6);

    while (LoRaSerial.available()) {
      LoRaSerial.read();    }    
  } 
}
