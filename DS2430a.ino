#include <OneWire.h>
#include <EEPROM.h>
#include <Bounce2.h>

#define BUS_PIN 6
#define SAVE_PIN 2
#define WRITE_PIN 3
#define WRITE2_PIN 4

OneWire bus(BUS_PIN); // OneWire bus on digital pin 6

Bounce saveBtn = Bounce(); // Read all button.
Bounce writeBtn = Bounce(); // Write data button.
Bounce write2Btn = Bounce(); // Write app reg data.

struct eeStruct { 
  byte data[32];
  byte appReg[8];
};
typedef struct eeStruct ee_t;

void setup() {
  pinMode(13, OUTPUT);  
  pinMode(SAVE_PIN, INPUT_PULLUP);
  saveBtn.attach(SAVE_PIN);
  saveBtn.interval(50);
  pinMode(WRITE_PIN, INPUT_PULLUP);
  writeBtn.attach(WRITE_PIN);
  writeBtn.interval(50);
  pinMode(WRITE2_PIN, INPUT_PULLUP);
  write2Btn.attach(WRITE2_PIN);
  write2Btn.interval(50);

  Serial.begin (9600);
  
  //byte data[32] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};
  //writeData(bus, data);
}

void loop() {
  boolean saveChanged = saveBtn.update();
  boolean writeChanged = writeBtn.update();
  boolean write2Changed = write2Btn.update();

  if (saveChanged && saveBtn.read() == LOW) {
    Serial.println("Save");
    saveAll();
  }

  if (writeChanged && writeBtn.read() == LOW) {
    Serial.println("Write EEPROM");
    writeData();
  }

  if (write2Changed && write2Btn.read() == LOW) {
    Serial.println("Write Application Register");
    writeAppReg();
  }
}

void saveAll() {
  if (!isChipConnected()) {
    blinkLed(5);
    return;
  }

  ee_t ee = ee_t();
  readData(bus, ee.data);
  readApplicationRegister(bus, ee.appReg);

  dump(ee.data, 32, "Read: EEPROM=");
  dump(ee.appReg, 8, "Read: APPREG=");

  EEPROM_writeAnything(0, &ee, sizeof(ee));

  ee_t ee2 = ee_t();
  EEPROM_readAnything(0, &ee2, sizeof(ee2));
  dump(ee2.data, 32, "Saved: EEPROM=");
  dump(ee2.appReg, 8, "Saved: APPREG=");

  if (!validate(&ee, &ee2, 0, sizeof(ee))) {
    blinkLed(10);
    return;
  }

  blinkLed(1);
}

void writeData() {
  if (!isChipConnected()) {
    blinkLed(5);
    return;
  }

  ee_t ee = ee_t();

  EEPROM_readAnything(0, &ee, sizeof(ee));
  dump(ee.data, 32, "Saved: EEPROM=");

  if (!writeData(bus, ee.data)) {
    blinkLed(10);
    return;
  }

  ee_t ee2 = ee_t();
  readData(bus, ee2.data);
  dump(ee2.data, 32, "Written: EEPROM=");

  if (!validate(&ee, &ee2, 0, 32)) { // Validate data only.
    blinkLed(10);
    return;
  }

  blinkLed(1);
}

void writeAppReg() {
  if (!isChipConnected()) {
    blinkLed(5);
    return;
  }

  ee_t ee = ee_t();

  EEPROM_readAnything(0, &ee, sizeof(ee));
  dump(ee.appReg, 8, "Saved: APPREG=");

  if (!writeApplicationRegister(bus, ee.appReg)) {
    blinkLed(10);
    return;
  }

  ee_t ee2 = ee_t();
  readApplicationRegister(bus, ee2.appReg);
  dump(ee2.appReg, 8, "Written: APPREG=");

  if (!validate(&ee, &ee2, 32, 32+8)) { // Validate data only.
    blinkLed(10);
    return;
  }

  blinkLed(1);
}

boolean isChipConnected() {
  if (!bus.reset()) {
    return false;
  }

  byte rom[8];
  readRom(bus, rom);
  if (rom[0] != 0x14) {
    return false;
  }

  return true;
}

void dump(byte *data, byte dataSize, char *str) {
  Serial.write(str);
  for (byte ii = 0; ii < dataSize; ii++) {
    Serial.write(' ');
    Serial.print(data[ii], HEX);
  }
  Serial.println();
}

void readRom(OneWire bus, byte data[8]) {
  bus.reset();
  //bus.skip(); // Asume we have single device on the bus.
  bus.write(0x33); // Read ROM command.
  for (byte i = 0; i < 8; i++) {
    data[i] = bus.read();
  }
}

void readData(OneWire bus, byte data[32]) {
  bus.reset();
  bus.skip(); // Asume we have single device on the bus.
  bus.write(0xF0); // Read memory command.
  bus.write(0x00); // Starting address.
  for (byte i = 0; i < 32; i++) {
    data[i] = bus.read();
  }
}

bool writeData(OneWire bus, byte data[32]) {
  /* 1. Write scratchpad (temp memory). */
  bus.reset();
  bus.skip(); // Asume we have single device on the bus.
  bus.write(0x0F); // Write scratchpad command.
  bus.write(0x00); // Starting address.
  for (byte i = 0; i < 32; i++) {
    bus.write(data[i]);
  }

  /* 2. Validate everything written correctly. */
  bus.reset();
  bus.skip();
  bus.write(0xAA); // Read scratchpad command.
  bus.write(0x00);
  for (byte ii = 0; ii < 32; ii++) {
    if (data[ii] != bus.read()) {
      bus.reset();
      return false;
    }
  }

  /* 3. Commit data (DS2430a will transfer data from internal memory to eeprom). */
  bus.reset();
  bus.skip();
  bus.write(0x55); // Issue "Copy Scratchpad" command.
  bus.write(0xA5); // Validation key.
  // Data line is held high for 10 ms by the bus master to provide
  // energy for copying data from the scratchpad to EEPROM.
  digitalWrite(BUS_PIN, HIGH);
  delay(10);

  bus.reset();
}

bool writeApplicationRegister(OneWire bus, byte data[8]) {
  /* 1. Write scratchpad (temp memory). */
  bus.reset();
  bus.skip(); // Asume we have single device on the bus.
  bus.write(0x99); // Write application register command.
  bus.write(0x00); // Starting address.
  for (byte i = 0; i < 8; i++) {
    bus.write(data[i]);
  }

  /* 2. Validate everything written correctly to scratchpad. */
  bus.reset();
  bus.skip();
  bus.write(0xC3); // Read application register command.
  bus.write(0x00);
  for (byte ii = 0; ii < 8; ii++) {
    if (data[ii] != bus.read()) {
      bus.reset();
      return false;
    }
  }

  /* 3. Commit application register from temp memory (DS2430a will transfer data from internal memory to eeprom). */
  bus.reset();
  bus.skip();
  bus.write(0x5A); // Issue "Copy and Lock Application Registry" command.
  bus.write(0xA5); // Validation key.
  // Data line is held high for 10 ms by the bus master to provide
  // energy for copying data from the scratchpad to EEPROM.
  digitalWrite(BUS_PIN, HIGH);
  delay(10);

  bus.reset();
}

void readApplicationRegister(OneWire ds, byte data[8]) {
  ds.reset();
  ds.skip(); // Asume we have single device on the bus.
  ds.write(0xC3); // Read application register command.
  ds.write(0x00); // Starting address.
  for (byte i = 0; i < 8; i++) {
    data[i] = ds.read();
  }
}

int EEPROM_writeAnything(int ee, const void* value, int sizeValue)
{
  const byte* p = (const byte*)value;
  int i;
  for (i = 0; i < sizeValue; i++)
    EEPROM.write(ee++, *p++);
  return i;
}

int EEPROM_readAnything(int ee, void* value, int sizeValue)
{
  byte* p = (byte*)value;
  int i;
  for (i = 0; i < sizeValue; i++)
    *p++ = EEPROM.read(ee++);
  return i;
}

boolean validate(const void* s1, const void* s2, int start, int s) {
  const byte* ee1 = (const byte*)s1;
  const byte* ee2 = (const byte*)s2;

  for (int i = start; i < s; i++) {
    if (ee1[i] != ee2[i]) {
      return false;
    }
  }

  return true;
}

void blinkLed(int cnt) {
  for (int i = 0; i < cnt; i++) {
    digitalWrite(13, HIGH);
    delay(150);
    digitalWrite(13, LOW);
    delay(150);
  }
}


