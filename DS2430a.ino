#include <Wire.h>
#include <OzOLED.h>
#include <OneWire.h>
#include <EEPROM.h>
#include <Bounce2.h>

#define BUS_PIN 6
#define SAVE_PIN 2 // Read and save connected DS2430a
#define WRITE_PIN 3 // Write EEPROM data to connected DS2430a from saved
#define WRITE2_PIN 4 // Write AppReg data to connected DS2430a from saved value
#define PRINT_PIN 5 // Print connected DS2430a (do not overwrite previously saved data)

OneWire bus(BUS_PIN); // OneWire bus on digital pin 6

Bounce saveBtn = Bounce(); // Read all button.
Bounce writeBtn = Bounce(); // Write data button.
Bounce write2Btn = Bounce(); // Write app reg data.
Bounce printBtn = Bounce(); // Print data.

struct eeStruct {
  byte data[32];
  byte appReg[8];
};
typedef struct eeStruct ee_t;

char hexmap[] = {
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

void setup() {
  pinMode(13, OUTPUT);
  pinMode(SAVE_PIN, INPUT_PULLUP);
  saveBtn.attach(SAVE_PIN);
  saveBtn.interval(3000);
  pinMode(WRITE_PIN, INPUT_PULLUP);
  writeBtn.attach(WRITE_PIN);
  writeBtn.interval(3000);
  pinMode(WRITE2_PIN, INPUT_PULLUP);
  write2Btn.attach(WRITE2_PIN);
  write2Btn.interval(3000);
  pinMode(PRINT_PIN, INPUT_PULLUP);
  printBtn.attach(PRINT_PIN);
  printBtn.interval(3000);

  OzOled.init();
  // Charge pump ON.
  OzOled.sendCommand(0x8d);
  OzOled.sendCommand(0x14);
  // Rotate 180.
  OzOled.sendCommand(0xc8);
  OzOled.sendCommand(0xa1);
  OzOled.sendCommand(0x0da);
  OzOled.sendCommand(0x012);
  
  printState("OK, Yurik");

  //byte data[32] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};
  //writeData(bus, data);
}

void printConnected() {
  if (!isChipConnected()) {
    printState("!!! Chip absent");
    blinkLed(5);
    return;
  }

  ee_t ee = ee_t();
  readData(bus, ee.data);
  readApplicationRegister(bus, ee.appReg);
  
  blinkLed(1);
  
  OzOled.clearDisplay();
  
  OzOled.printString("Print OK");
  OzOled.setCursorXY(0, 1);
  
  OzOled.setHorizontalMode();      //Set addressing mode to Horizontal Mode
  OzOled.printString("Connected chip: ");
  printData(ee.data, 32);
  printData(ee.appReg, 8);
  OzOled.setNormalDisplay();       //Set display to Normal mode
}

void printState(const char *lastState) {
  ee_t ee = ee_t();
  EEPROM_readAnything(0, &ee, sizeof(ee));

  OzOled.clearDisplay();
  
  OzOled.printString(lastState);
  OzOled.setCursorXY(0, 1);
  
  OzOled.setHorizontalMode();      //Set addressing mode to Horizontal Mode
  OzOled.printString("Saved EEPROM/AR:");
  printData(ee.data, 32);
  printData(ee.appReg, 8);
  OzOled.setNormalDisplay();       //Set display to Normal mode
}

void printData(byte* buf, byte bufSize) {
  char hex[2];
  for (int i = 0; i < bufSize; i++) {
    byte n = buf[i];
    byte2hex(n, hex);
    OzOled.printChar(hex[0]);
    OzOled.printChar(hex[1]);
  }
}

char byte2hex(byte n, char* buf) {
  buf[0] = hexmap[(n & 0xF0) >> 4];
  buf[1] = hexmap[n & 0x0F];
}

void loop() {
  boolean saveChanged = saveBtn.update();
  boolean writeChanged = writeBtn.update();
  boolean write2Changed = write2Btn.update();
  boolean printChanged = printBtn.update();  

  if (saveChanged && saveBtn.read() == LOW) {
    saveAll();
  }

  if (writeChanged && writeBtn.read() == LOW) {
    writeData();
  }

  if (write2Changed && write2Btn.read() == LOW) {
    writeAppReg();
  }
  
  if (printChanged && printBtn.read() == LOW) {
    printConnected();
  }
}

void saveAll() {
  if (!isChipConnected()) {
    printState("!!! Chip absent");
    blinkLed(5);
    return;
  }

  ee_t ee = ee_t();
  readData(bus, ee.data);
  readApplicationRegister(bus, ee.appReg);

  EEPROM_writeAnything(0, &ee, sizeof(ee));

  ee_t ee2 = ee_t();
  EEPROM_readAnything(0, &ee2, sizeof(ee2));

  if (!validate(&ee, &ee2, 0, sizeof(ee))) {
    blinkLed(10);
    printState("!!! Invalid");
    return;
  }

  blinkLed(1);
  printState("Read OK");
}

void writeData() {
  if (!isChipConnected()) {
    blinkLed(5);
    printState("!!! Chip absent");
    return;
  }

  ee_t ee = ee_t();

  EEPROM_readAnything(0, &ee, sizeof(ee));

  if (!writeData(bus, ee.data)) {
    blinkLed(10);
    printState("!!! Failed");
    return;
  }

  ee_t ee2 = ee_t();
  readData(bus, ee2.data);

  if (!validate(&ee, &ee2, 0, 32)) { // Validate data only.
    blinkLed(10);
    printState("!!! Invalid");
    return;
  }

  blinkLed(1);
  printState("Write EEPROM OK");
}

void writeAppReg() {
  if (!isChipConnected()) {
    blinkLed(5);
    printState("!!! Chip absent");
    return;
  }

  ee_t ee = ee_t();

  EEPROM_readAnything(0, &ee, sizeof(ee));

  if (!writeApplicationRegister(bus, ee.appReg)) {
    blinkLed(10);
    printState("!!! Failed");    
    return;
  }

  ee_t ee2 = ee_t();
  readApplicationRegister(bus, ee2.appReg);

  if (!validate(&ee, &ee2, 32, 32 + 8)) { // Validate data only.
    blinkLed(10);
    printState("!!! Invalid");
    return;
  }

  blinkLed(1);
  printState("!!! Write AR OK");
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
