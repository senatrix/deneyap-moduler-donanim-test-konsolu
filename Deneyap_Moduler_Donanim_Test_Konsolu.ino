#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ======================================================
// OLED VE I2C AYARLARI
// ======================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define I2C_SDA_PIN D10
#define I2C_SCL_PIN D11

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

uint8_t oledAddress = 0;


// ======================================================
// JOYSTICK AYARLARI
// ======================================================

// Ölçtüğümüz joystick yönleri:
// Yukarı  -> A0 yaklaşık 4095
// Aşağı  -> A0 yaklaşık 0
// Sol    -> A1 yaklaşık 4095
// Sağ    -> A1 yaklaşık 0

#define JOY_VERTICAL_PIN   A0
#define JOY_HORIZONTAL_PIN A1

const int JOY_LOW_THRESHOLD  = 900;
const int JOY_HIGH_THRESHOLD = 3000;

const int JOY_CENTER_LOW  = 1300;
const int JOY_CENTER_HIGH = 2600;

bool joystickReady = false;


// ======================================================
// GPIO TEST PINLERI
// ======================================================

// Otomatik GPIO loopback testi:
// D8 cikis olarak surulur.
// D9 giris olarak okunur.
// Test sirasinda D8 ile D9 arasina jumper kablo takilir.

#define GPIO_OUTPUT_PIN D8
#define GPIO_INPUT_PIN  D9

bool gpioLowOk = false;
bool gpioHighOk = false;
int gpioLowRead = LOW;
int gpioHighRead = LOW;


// ======================================================
// PWM TEST AYARLARI
// ======================================================

// Harici LED bu pinden sürülecek.
#define PWM_OUTPUT_PIN D13

const uint32_t PWM_FREQUENCY = 1000;
const uint8_t PWM_RESOLUTION = 8;
const uint8_t PWM_CHANNEL = 0;

int pwmPercent = 0;
bool pwmInitialized = false;


// ======================================================
// KESME TEST AYARLARI
// ======================================================

// Harici buton D14 ile GND arasina baglanir.
// INPUT_PULLUP kullanildigi icin harici direnc gerekmez.
#define INTERRUPT_BUTTON_PIN D14

// Mekanik buton titresimini engellemek icin 50 ms.
const uint32_t INTERRUPT_DEBOUNCE_US = 50000;

volatile bool interruptTestActive = false;
volatile uint32_t interruptPressCount = 0;
volatile uint32_t interruptBounceCount = 0;
volatile uint32_t interruptLastAcceptedUs = 0;
volatile uint32_t interruptLastIntervalUs = 0;

// Bir basış kabul edildikten sonra buton tamamen bırakılana kadar
// yeni düşen kenarları ikinci basış olarak sayma.
volatile bool interruptButtonLatched = false;

unsigned long lastInterruptScreenUpdate = 0;
const unsigned long INTERRUPT_SCREEN_INTERVAL_MS = 100;


// ======================================================
// LSM6DSM REGISTER VE ADRESLERI
// ======================================================

const uint8_t LSM_ADDRESS_1 = 0x6B;
const uint8_t LSM_ADDRESS_2 = 0x6A;

const uint8_t LSM_WHO_AM_I_REG = 0x0F;
const uint8_t LSM_EXPECTED_ID  = 0x6A;

const uint8_t LSM_CTRL1_XL = 0x10;
const uint8_t LSM_CTRL2_G  = 0x11;
const uint8_t LSM_CTRL3_C  = 0x12;

const uint8_t LSM_OUTX_L_G = 0x22;

uint8_t lsmAddress = 0;


// ======================================================
// MMC5603NJ REGISTER VE ADRESLERI
// ======================================================

const uint8_t MMC_ADDRESS = 0x30;

const uint8_t MMC_PRODUCT_ID_REG = 0x39;
const uint8_t MMC_EXPECTED_ID    = 0x10;

const uint8_t MMC_DATA_START = 0x00;
const uint8_t MMC_CONTROL_0  = 0x1B;


// ======================================================
// ANA MENU
// ======================================================

const char* mainMenuItems[] = {
  "1 I2C Tarama",
  "2 Sensor Testi",
  "3 GPIO Testi",
  "4 PWM Testi",
  "5 Kesme Testi",
  "6 Sistem Bilgisi"
};

const int MAIN_MENU_COUNT =
  sizeof(mainMenuItems) / sizeof(mainMenuItems[0]);

int selectedMainItem = 0;


// ======================================================
// SENSOR ALT MENUSU
// ======================================================

const char* sensorMenuItems[] = {
  "1 Kimlik Testi",
  "2 Canli Ivme",
  "3 Canli Gyro",
  "4 Canli Manyetik"
};

const int SENSOR_MENU_COUNT =
  sizeof(sensorMenuItems) / sizeof(sensorMenuItems[0]);

int selectedSensorItem = 0;
int placeholderMenuIndex = 0;


// ======================================================
// EKRAN DURUMLARI
// ======================================================

enum ScreenState {
  SCREEN_MAIN_MENU,
  SCREEN_I2C_RESULT,
  SCREEN_SENSOR_MENU,
  SCREEN_SENSOR_ID,
  SCREEN_LIVE_ACCEL,
  SCREEN_LIVE_GYRO,
  SCREEN_LIVE_MAG,
  SCREEN_GPIO_READY,
  SCREEN_GPIO_RESULT,
  SCREEN_PWM_TEST,
  SCREEN_INTERRUPT_TEST,
  SCREEN_SYSTEM_INFO,
  SCREEN_PLACEHOLDER
};

ScreenState currentScreen = SCREEN_MAIN_MENU;


// ======================================================
// JOYSTICK KOMUTLARI
// ======================================================

enum JoyAction {
  JOY_NONE,
  JOY_UP,
  JOY_DOWN,
  JOY_LEFT,
  JOY_RIGHT
};


// ======================================================
// I2C TARAMA VERILERI
// ======================================================

const int MAX_I2C_DEVICE_COUNT = 24;

uint8_t foundAddresses[MAX_I2C_DEVICE_COUNT];

int foundDeviceCount = 0;
int i2cPage = 0;


// ======================================================
// CANLI EKRAN ZAMANLAMASI
// ======================================================

unsigned long lastLiveUpdate = 0;

const unsigned long LIVE_UPDATE_INTERVAL_MS = 150;


// ======================================================
// I2C CIHAZ CEVAP KONTROLU
// ======================================================

bool i2cDeviceResponds(uint8_t address) {
  Wire.beginTransmission(address);

  return Wire.endTransmission() == 0;
}


// ======================================================
// I2C REGISTER YAZMA
// ======================================================

bool writeI2CRegister(
  uint8_t deviceAddress,
  uint8_t registerAddress,
  uint8_t value
) {
  Wire.beginTransmission(deviceAddress);

  Wire.write(registerAddress);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}


// ======================================================
// TEK REGISTER OKUMA
// ======================================================

bool readI2CRegister(
  uint8_t deviceAddress,
  uint8_t registerAddress,
  uint8_t &value
) {
  Wire.beginTransmission(deviceAddress);

  Wire.write(registerAddress);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(deviceAddress, (uint8_t)1) != 1) {
    return false;
  }

  if (!Wire.available()) {
    return false;
  }

  value = Wire.read();

  return true;
}


// ======================================================
// BIRDEN FAZLA REGISTER OKUMA
// ======================================================

bool readI2CRegisters(
  uint8_t deviceAddress,
  uint8_t startRegister,
  uint8_t* buffer,
  uint8_t length
) {
  Wire.beginTransmission(deviceAddress);

  Wire.write(startRegister);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received =
    Wire.requestFrom(deviceAddress, length);

  if (received != length) {
    while (Wire.available()) {
      Wire.read();
    }

    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    if (!Wire.available()) {
      return false;
    }

    buffer[i] = Wire.read();
  }

  return true;
}


// ======================================================
// OLED ADRESINI BUL
// ======================================================

bool findOLEDAddress() {
  if (i2cDeviceResponds(0x3C)) {
    oledAddress = 0x3C;

    return true;
  }

  if (i2cDeviceResponds(0x3D)) {
    oledAddress = 0x3D;

    return true;
  }

  return false;
}


// ======================================================
// LSM6DSM ADRESINI BUL
// ======================================================

uint8_t findLSMAddress() {
  if (i2cDeviceResponds(LSM_ADDRESS_1)) {
    return LSM_ADDRESS_1;
  }

  if (i2cDeviceResponds(LSM_ADDRESS_2)) {
    return LSM_ADDRESS_2;
  }

  return 0;
}


// ======================================================
// OLED'E HEX DEGER YAZ
// ======================================================

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    display.print("0");
  }

  display.print(value, HEX);
}


// ======================================================
// LSM6DSM'YI CALISTIR
// ======================================================

bool initializeLSM6DSM() {
  lsmAddress = findLSMAddress();

  if (lsmAddress == 0) {
    return false;
  }

  bool success = true;

  /*
    CTRL3_C = 0x44

    Bit 6 BDU = 1:
    Çok baytlı veri okunurken alt ve üst baytların
    farklı ölçümlere ait olmasını engeller.

    Bit 2 IF_INC = 1:
    Birden fazla register okunurken adresi
    otomatik artırır.
  */
  success &=
    writeI2CRegister(
      lsmAddress,
      LSM_CTRL3_C,
      0x44
    );

  /*
    CTRL1_XL = 0x40

    İvmeölçer:
    104 Hz örnekleme
    Artı-eksi 2 g ölçüm aralığı
  */
  success &=
    writeI2CRegister(
      lsmAddress,
      LSM_CTRL1_XL,
      0x40
    );

  /*
    CTRL2_G = 0x40

    Jiroskop:
    104 Hz örnekleme
    Artı-eksi 245 derece/saniye
  */
  success &=
    writeI2CRegister(
      lsmAddress,
      LSM_CTRL2_G,
      0x40
    );

  delay(30);

  return success;
}


// ======================================================
// IVME VE JIROSKOP VERILERINI OKU
// ======================================================

bool readLSMData(
  float &ax,
  float &ay,
  float &az,
  float &gx,
  float &gy,
  float &gz
) {
  /*
    Sensör çıkarılıp yeniden takılmışsa
    tekrar adres bul ve başlat.
  */
  if (
    lsmAddress == 0 ||
    !i2cDeviceResponds(lsmAddress)
  ) {
    if (!initializeLSM6DSM()) {
      return false;
    }
  }

  /*
    0x22'den başlayarak 12 bayt okunur:

    0x22-0x27: Gyro X, Y, Z
    0x28-0x2D: İvme X, Y, Z
  */
  uint8_t data[12];

  if (
    !readI2CRegisters(
      lsmAddress,
      LSM_OUTX_L_G,
      data,
      sizeof(data)
    )
  ) {
    return false;
  }

  /*
    Sensör her ekseni iki bayt olarak verir:
    önce düşük bayt, ardından yüksek bayt.
  */
  int16_t rawGx =
    (int16_t)(
      ((uint16_t)data[1] << 8) |
      data[0]
    );

  int16_t rawGy =
    (int16_t)(
      ((uint16_t)data[3] << 8) |
      data[2]
    );

  int16_t rawGz =
    (int16_t)(
      ((uint16_t)data[5] << 8) |
      data[4]
    );

  int16_t rawAx =
    (int16_t)(
      ((uint16_t)data[7] << 8) |
      data[6]
    );

  int16_t rawAy =
    (int16_t)(
      ((uint16_t)data[9] << 8) |
      data[8]
    );

  int16_t rawAz =
    (int16_t)(
      ((uint16_t)data[11] << 8) |
      data[10]
    );

  /*
    Artı-eksi 2 g ayarında:
    1 LSB = 0.061 mg = 0.000061 g
  */
  ax = rawAx * 0.000061f;
  ay = rawAy * 0.000061f;
  az = rawAz * 0.000061f;

  /*
    Artı-eksi 245 dps ayarında:
    1 LSB = 8.75 mdps = 0.00875 dps
  */
  gx = rawGx * 0.00875f;
  gy = rawGy * 0.00875f;
  gz = rawGz * 0.00875f;

  return true;
}


// ======================================================
// MANYETOMETRE VERILERINI OKU
// ======================================================

bool readMMCData(
  float &mx,
  float &my,
  float &mz
) {
  if (!i2cDeviceResponds(MMC_ADDRESS)) {
    return false;
  }

  /*
    0x21:
    Bit 5 Auto_SR_en = 1
    Bit 0 Take_meas_M = 1

    Otomatik set/reset açıkken bir manyetik
    alan ölçümü başlatır.
  */
  if (
    !writeI2CRegister(
      MMC_ADDRESS,
      MMC_CONTROL_0,
      0x21
    )
  ) {
    return false;
  }

  delay(10);

  /*
    0x00-0x08 arasında X, Y ve Z eksenlerinin
    20 bitlik ölçüm verileri bulunur.
  */
  uint8_t data[9];

  if (
    !readI2CRegisters(
      MMC_ADDRESS,
      MMC_DATA_START,
      data,
      sizeof(data)
    )
  ) {
    return false;
  }

  uint32_t rawX =
    ((uint32_t)data[0] << 12) |
    ((uint32_t)data[1] << 4) |
    ((uint32_t)data[6] >> 4);

  uint32_t rawY =
    ((uint32_t)data[2] << 12) |
    ((uint32_t)data[3] << 4) |
    ((uint32_t)data[7] >> 4);

  uint32_t rawZ =
    ((uint32_t)data[4] << 12) |
    ((uint32_t)data[5] << 4) |
    ((uint32_t)data[8] >> 4);

  /*
    20 bit modunda sıfır alan değeri 524288'dir.
    Her sayım yaklaşık 0.00625 mikrotesladır.
  */
  mx =
    ((int32_t)rawX - 524288) *
    0.00625f;

  my =
    ((int32_t)rawY - 524288) *
    0.00625f;

  mz =
    ((int32_t)rawZ - 524288) *
    0.00625f;

  return true;
}


// ======================================================
// JOYSTICK OKUMA
// ======================================================

JoyAction readJoystickAction() {
  int verticalValue =
    analogRead(JOY_VERTICAL_PIN);

  int horizontalValue =
    analogRead(JOY_HORIZONTAL_PIN);

  bool verticalCentered =
    verticalValue > JOY_CENTER_LOW &&
    verticalValue < JOY_CENTER_HIGH;

  bool horizontalCentered =
    horizontalValue > JOY_CENTER_LOW &&
    horizontalValue < JOY_CENTER_HIGH;

  /*
    Her hareketten sonra joystick merkeze
    dönmeden yeni komut kabul edilmez.
  */
  if (
    verticalCentered &&
    horizontalCentered
  ) {
    joystickReady = true;

    return JOY_NONE;
  }

  if (!joystickReady) {
    return JOY_NONE;
  }

  JoyAction action = JOY_NONE;

  if (
    verticalValue >
    JOY_HIGH_THRESHOLD
  ) {
    action = JOY_UP;
  }
  else if (
    verticalValue <
    JOY_LOW_THRESHOLD
  ) {
    action = JOY_DOWN;
  }
  else if (
    horizontalValue >
    JOY_HIGH_THRESHOLD
  ) {
    action = JOY_LEFT;
  }
  else if (
    horizontalValue <
    JOY_LOW_THRESHOLD
  ) {
    action = JOY_RIGHT;
  }

  if (action != JOY_NONE) {
    joystickReady = false;

    delay(35);
  }

  return action;
}


// ======================================================
// BUTON KESME SERVIS RUTINI
// ======================================================

void IRAM_ATTR handleInterruptButton() {
  if (!interruptTestActive) {
    return;
  }

  uint32_t currentUs = micros();

  /*
    Daha önce bir basış kabul edildiyse ve buton henüz
    kararlı biçimde bırakılmadıysa, bırakma sırasındaki
    mekanik sıçramaları yeni basış olarak sayma.
  */
  if (interruptButtonLatched) {
    interruptBounceCount++;
    return;
  }

  if (
    interruptLastAcceptedUs == 0 ||
    currentUs - interruptLastAcceptedUs >=
    INTERRUPT_DEBOUNCE_US
  ) {
    if (interruptLastAcceptedUs != 0) {
      interruptLastIntervalUs =
        currentUs -
        interruptLastAcceptedUs;
    }

    interruptLastAcceptedUs =
      currentUs;

    interruptPressCount++;
    interruptButtonLatched = true;
  }
  else {
    interruptBounceCount++;
  }
}


// ======================================================
// KESME TESTINI BASLAT / SIFIRLA
// ======================================================

void resetInterruptTest() {
  noInterrupts();

  interruptPressCount = 0;
  interruptBounceCount = 0;
  interruptLastAcceptedUs = 0;
  interruptLastIntervalUs = 0;
  interruptButtonLatched = false;
  interruptTestActive = true;

  interrupts();
}


// ======================================================
// KESME TESTINI DURDUR
// ======================================================

void stopInterruptTest() {
  noInterrupts();

  interruptTestActive = false;

  interrupts();
}


// ======================================================
// BUTONUN KARARLI BIRAKILMASINI KONTROL ET
// ======================================================

void updateInterruptButtonRelease() {
  static unsigned long releaseStartedMs = 0;

  if (!interruptTestActive) {
    releaseStartedMs = 0;
    return;
  }

  /*
    Buton bırakıldığında pin HIGH olur.
    HIGH seviyesi 30 ms boyunca kararlı kalırsa,
    bir sonraki gerçek basışa izin verilir.
  */
  if (
    digitalRead(INTERRUPT_BUTTON_PIN) ==
    HIGH
  ) {
    if (releaseStartedMs == 0) {
      releaseStartedMs = millis();
    }
    else if (
      millis() - releaseStartedMs >= 30
    ) {
      noInterrupts();

      interruptButtonLatched = false;

      interrupts();
    }
  }
  else {
    /*
      Bırakma sırasında tekrar LOW görülürse
      kararlı bırakma süresi yeniden başlatılır.
    */
    releaseStartedMs = 0;
  }
}


// ======================================================
// KESME SAYACLARINI GUVENLI KOPYALA
// ======================================================

void copyInterruptValues(
  uint32_t &pressCount,
  uint32_t &bounceCount,
  uint32_t &lastIntervalUs
) {
  noInterrupts();

  pressCount =
    interruptPressCount;

  bounceCount =
    interruptBounceCount;

  lastIntervalUs =
    interruptLastIntervalUs;

  interrupts();
}


// ======================================================
// ORTAK EKRAN BASLIGI
// ======================================================

void drawHeader(const char* title) {
  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(0, 0);

  display.println(title);

  display.drawFastHLine(
    0,
    10,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );
}


// ======================================================
// ACILIS EKRANI
// ======================================================

void showBootScreen() {
  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(2);
  display.setCursor(16, 8);
  display.println("DENEYAP");

  display.setTextSize(1);
  display.setCursor(22, 34);
  display.println("DONANIM TEST");

  display.setCursor(37, 46);
  display.println("KONSOLU");

  display.display();

  delay(1500);
}


// ======================================================
// ANA MENU EKRANI
// ======================================================

void drawMainMenu() {
  drawHeader(
    "  DONANIM KONSOLU"
  );

  for (
    int i = 0;
    i < MAIN_MENU_COUNT;
    i++
  ) {
    display.setCursor(
      0,
      14 + i * 8
    );

    if (i == selectedMainItem) {
      display.print(">");
    }
    else {
      display.print(" ");
    }

    display.println(
      mainMenuItems[i]
    );
  }

  display.display();
}


// ======================================================
// SENSOR ALT MENUSU
// ======================================================

void drawSensorMenu() {
  drawHeader(
    "     SENSOR TESTI"
  );

  for (
    int i = 0;
    i < SENSOR_MENU_COUNT;
    i++
  ) {
    display.setCursor(
      0,
      15 + i * 10
    );

    if (i == selectedSensorItem) {
      display.print(">");
    }
    else {
      display.print(" ");
    }

    display.println(
      sensorMenuItems[i]
    );
  }

  display.drawFastHLine(
    0,
    54,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );

  display.setCursor(0, 56);

  display.print(
    "SOL:Geri SAG:Sec"
  );

  display.display();
}


// ======================================================
// I2C TARAMA ILERLEME EKRANI
// ======================================================

void drawScanningScreen(
  uint8_t address
) {
  drawHeader(
    "     I2C TARANIYOR"
  );

  display.drawRect(
    10,
    27,
    108,
    12,
    SSD1306_WHITE
  );

  int progressWidth =
    map(
      address,
      1,
      126,
      0,
      104
    );

  display.fillRect(
    12,
    29,
    progressWidth,
    8,
    SSD1306_WHITE
  );

  display.setCursor(34, 48);

  display.print("Adres:0x");

  printHexByte(address);

  display.display();
}


// ======================================================
// I2C HATTINI TARA
// ======================================================

void scanI2CBus() {
  foundDeviceCount = 0;
  i2cPage = 0;

  for (
    uint8_t address = 1;
    address < 127;
    address++
  ) {
    if (address % 4 == 0) {
      drawScanningScreen(address);
    }

    if (
      i2cDeviceResponds(address) &&
      foundDeviceCount <
      MAX_I2C_DEVICE_COUNT
    ) {
      foundAddresses[
        foundDeviceCount
      ] = address;

      foundDeviceCount++;
    }

    delay(2);
  }
}


// ======================================================
// HARICI CIHAZ SAYISI
// ======================================================

int getExternalDeviceCount() {
  int count = 0;

  for (
    int i = 0;
    i < foundDeviceCount;
    i++
  ) {
    if (
      foundAddresses[i] !=
      oledAddress
    ) {
      count++;
    }
  }

  return count;
}


// ======================================================
// I2C TARAMA SONUCU
// ======================================================

void drawI2CResults() {
  drawHeader(
    "       I2C SONUCU"
  );

  display.setCursor(0, 12);

  display.print("Toplam:");
  display.print(foundDeviceCount);

  display.print(" Harici:");
  display.print(
    getExternalDeviceCount()
  );

  int totalPages =
    (foundDeviceCount + 2) / 3;

  if (totalPages < 1) {
    totalPages = 1;
  }

  if (i2cPage >= totalPages) {
    i2cPage =
      totalPages - 1;
  }

  int startIndex =
    i2cPage * 3;

  int y = 23;

  if (foundDeviceCount == 0) {
    display.setCursor(8, 30);

    display.println(
      "CIHAZ BULUNAMADI"
    );
  }
  else {
    for (
      int i = startIndex;
      i < foundDeviceCount &&
      i < startIndex + 3;
      i++
    ) {
      uint8_t address =
        foundAddresses[i];

      display.setCursor(0, y);

      display.print("0x");

      printHexByte(address);

      display.print(" ");

      if (
        address ==
        oledAddress
      ) {
        display.println(
          "OLED-sistem"
        );
      }
      else if (
        address ==
        MMC_ADDRESS
      ) {
        display.println(
          "MMC5603"
        );
      }
      else if (
        address ==
        LSM_ADDRESS_1 ||
        address ==
        LSM_ADDRESS_2
      ) {
        display.println(
          "LSM6DSM"
        );
      }
      else {
        display.println(
          "Harici cihaz"
        );
      }

      y += 10;
    }
  }

  display.drawFastHLine(
    0,
    53,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );

  display.setCursor(0, 56);

  display.print(
    "SOL:Geri SAG:Tara"
  );

  display.display();
}


// ======================================================
// SENSOR KIMLIK TESTI
// ======================================================

void drawSensorIdTest() {
  uint8_t detectedLSM =
    findLSMAddress();

  uint8_t lsmId = 0;
  uint8_t mmcId = 0;

  bool lsmFound =
    detectedLSM != 0;

  bool mmcFound =
    i2cDeviceResponds(
      MMC_ADDRESS
    );

  bool lsmRead =
    lsmFound &&
    readI2CRegister(
      detectedLSM,
      LSM_WHO_AM_I_REG,
      lsmId
    );

  bool mmcRead =
    mmcFound &&
    readI2CRegister(
      MMC_ADDRESS,
      MMC_PRODUCT_ID_REG,
      mmcId
    );

  bool lsmOk =
    lsmRead &&
    lsmId ==
    LSM_EXPECTED_ID;

  bool mmcOk =
    mmcRead &&
    mmcId ==
    MMC_EXPECTED_ID;

  drawHeader(
    "   9 EKSEN IMU TEST"
  );

  display.setCursor(0, 14);

  display.print(
    "IVME/GYRO: "
  );

  if (lsmOk) {
    display.println("OK");
  }
  else if (lsmFound) {
    display.println("HATA");
  }
  else {
    display.println("YOK");
  }

  display.setCursor(0, 24);

  display.print("ADR:");

  if (lsmFound) {
    display.print("0x");

    printHexByte(
      detectedLSM
    );
  }
  else {
    display.print("--");
  }

  display.print(" ID:");

  if (lsmRead) {
    display.print("0x");

    printHexByte(lsmId);
  }
  else {
    display.print("--");
  }

  display.setCursor(0, 35);

  display.print(
    "MAGNETOMETRE: "
  );

  if (mmcOk) {
    display.println("OK");
  }
  else if (mmcFound) {
    display.println("HATA");
  }
  else {
    display.println("YOK");
  }

  display.setCursor(0, 45);

  display.print("ADR:");

  if (mmcFound) {
    display.print("0x30");
  }
  else {
    display.print("--");
  }

  display.print(" ID:");

  if (mmcRead) {
    display.print("0x");

    printHexByte(mmcId);
  }
  else {
    display.print("--");
  }

  display.drawFastHLine(
    0,
    54,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );

  display.setCursor(0, 56);

  display.print(
    "SOL:Geri SAG:Yenile"
  );

  display.display();
}


// ======================================================
// SENSOR OKUMA HATASI EKRANI
// ======================================================

void drawReadError(
  const char* title
) {
  drawHeader(title);

  display.setCursor(11, 24);

  display.println(
    "SENSOR OKUNAMADI"
  );

  display.setCursor(5, 38);

  display.println(
    "BAGLANTIYI KONTROL ET"
  );

  display.setCursor(0, 56);

  display.print("SOL:Geri");

  display.display();
}


// ======================================================
// CANLI IVME EKRANI
// ======================================================

void drawLiveAcceleration() {
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;

  if (
    !readLSMData(
      ax,
      ay,
      az,
      gx,
      gy,
      gz
    )
  ) {
    drawReadError(
      "     CANLI IVME"
    );

    return;
  }

  float magnitude =
    sqrtf(
      ax * ax +
      ay * ay +
      az * az
    );

  drawHeader(
    "     CANLI IVME [g]"
  );

  display.setCursor(0, 14);

  display.print("X: ");
  display.print(ax, 3);

  display.setCursor(0, 24);

  display.print("Y: ");
  display.print(ay, 3);

  display.setCursor(0, 34);

  display.print("Z: ");
  display.print(az, 3);

  display.setCursor(0, 44);

  display.print("|a|: ");
  display.print(magnitude, 3);

  display.drawFastHLine(
    0,
    54,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );

  display.setCursor(0, 56);

  display.print("SOL:Geri");

  display.display();
}


// ======================================================
// CANLI JIROSKOP EKRANI
// ======================================================

void drawLiveGyroscope() {
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;

  if (
    !readLSMData(
      ax,
      ay,
      az,
      gx,
      gy,
      gz
    )
  ) {
    drawReadError(
      "     CANLI GYRO"
    );

    return;
  }

  float magnitude =
    sqrtf(
      gx * gx +
      gy * gy +
      gz * gz
    );

  drawHeader(
    "   CANLI GYRO [dps]"
  );

  display.setCursor(0, 14);

  display.print("X: ");
  display.print(gx, 2);

  display.setCursor(0, 24);

  display.print("Y: ");
  display.print(gy, 2);

  display.setCursor(0, 34);

  display.print("Z: ");
  display.print(gz, 2);

  display.setCursor(0, 44);

  display.print("|w|: ");
  display.print(magnitude, 2);

  display.drawFastHLine(
    0,
    54,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );

  display.setCursor(0, 56);

  display.print("SOL:Geri");

  display.display();
}


// ======================================================
// CANLI MANYETIK ALAN EKRANI
// ======================================================

void drawLiveMagnetometer() {
  float mx;
  float my;
  float mz;

  if (
    !readMMCData(
      mx,
      my,
      mz
    )
  ) {
    drawReadError(
      "  CANLI MANYETIK"
    );

    return;
  }

  float magnitude =
    sqrtf(
      mx * mx +
      my * my +
      mz * mz
    );

  drawHeader(
    "  MANYETIK ALAN [uT]"
  );

  display.setCursor(0, 14);

  display.print("X: ");
  display.print(mx, 1);

  display.setCursor(0, 24);

  display.print("Y: ");
  display.print(my, 1);

  display.setCursor(0, 34);

  display.print("Z: ");
  display.print(mz, 1);

  display.setCursor(0, 44);

  display.print("|B|: ");
  display.print(magnitude, 1);

  display.drawFastHLine(
    0,
    54,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );

  display.setCursor(0, 56);

  display.print("SOL:Geri");

  display.display();
}


// ======================================================
// GPIO TEST HAZIRLIK EKRANI
// ======================================================

void drawGPIOReadyScreen() {
  drawHeader(
    "      GPIO TESTI"
  );

  display.setCursor(0, 15);
  display.println(
    "Jumper kabloyu bagla:"
  );

  display.setTextSize(2);
  display.setCursor(18, 28);
  display.print("D8");
  display.print(" -> ");
  display.print("D9");

  display.setTextSize(1);

  display.drawFastHLine(
    0,
    53,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );

  display.setCursor(0, 56);
  display.print(
    "SOL:Geri SAG:Baslat"
  );

  display.display();
}


// ======================================================
// GPIO LOOPBACK TESTINI CALISTIR
// ======================================================

void runGPIOTest() {
  /*
    D9 bos kaldiginda rastgele deger okumamasi icin
    dahili pull-down direnci etkinlestirilir.
  */
  pinMode(
    GPIO_INPUT_PIN,
    INPUT_PULLDOWN
  );

  pinMode(
    GPIO_OUTPUT_PIN,
    OUTPUT
  );

  gpioLowOk = true;
  gpioHighOk = true;

  /*
    Temassizlik veya tek seferlik yanlis okumayi
    azaltmak icin LOW ve HIGH testi 5 kez tekrarlanir.
  */
  for (int i = 0; i < 5; i++) {
    digitalWrite(
      GPIO_OUTPUT_PIN,
      LOW
    );

    delay(8);

    gpioLowRead =
      digitalRead(
        GPIO_INPUT_PIN
      );

    if (gpioLowRead != LOW) {
      gpioLowOk = false;
    }

    digitalWrite(
      GPIO_OUTPUT_PIN,
      HIGH
    );

    delay(8);

    gpioHighRead =
      digitalRead(
        GPIO_INPUT_PIN
      );

    if (gpioHighRead != HIGH) {
      gpioHighOk = false;
    }
  }

  /*
    Test bittiginde cikisi guvenli olarak LOW'da birak.
  */
  digitalWrite(
    GPIO_OUTPUT_PIN,
    LOW
  );
}


// ======================================================
// GPIO TEST SONUC EKRANI
// ======================================================

void drawGPIOResultScreen() {
  bool overallOk =
    gpioLowOk &&
    gpioHighOk;

  drawHeader(
    "   GPIO LOOPBACK TEST"
  );

  display.setCursor(0, 14);
  display.print("Hat: D8 -> D9");

  display.setCursor(0, 25);
  display.print("LOW testi : ");

  if (gpioLowOk) {
    display.println("OK");
  }
  else {
    display.println("HATA");
  }

  display.setCursor(0, 35);
  display.print("HIGH testi: ");

  if (gpioHighOk) {
    display.println("OK");
  }
  else {
    display.println("HATA");
  }

  display.setCursor(0, 45);
  display.print("SONUC: ");

  if (overallOk) {
    display.println("BASARILI");
  }
  else {
    display.println("BASARISIZ");
  }

  display.drawFastHLine(
    0,
    54,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );

  display.setCursor(0, 56);
  display.print(
    "SOL:Geri SAG:Yenile"
  );

  display.display();
}


// ======================================================
// PWM DONANIMINI BASLAT
// ======================================================

void initializePWMOutput() {
  if (pwmInitialized) {
    return;
  }

  /*
    Arduino ESP32 3.x ve daha yeni sürümlerde
    LEDC fonksiyonları doğrudan pin ile çalışır.
  */
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3

  pwmInitialized = ledcAttach(
    PWM_OUTPUT_PIN,
    PWM_FREQUENCY,
    PWM_RESOLUTION
  );

#else

  /*
    Arduino ESP32 2.x ve eski Deneyap çekirdeklerinde
    kanal tabanlı LEDC kullanılır.
  */
  ledcSetup(
    PWM_CHANNEL,
    PWM_FREQUENCY,
    PWM_RESOLUTION
  );

  ledcAttachPin(
    PWM_OUTPUT_PIN,
    PWM_CHANNEL
  );

  pwmInitialized = true;

#endif
}


// ======================================================
// PWM YUZDESINI CIKISA UYGULA
// ======================================================

void applyPWMPercent() {
  initializePWMOutput();

  pwmPercent = constrain(
    pwmPercent,
    0,
    100
  );

  uint32_t dutyValue = map(
    pwmPercent,
    0,
    100,
    0,
    255
  );

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3

  ledcWrite(
    PWM_OUTPUT_PIN,
    dutyValue
  );

#else

  ledcWrite(
    PWM_CHANNEL,
    dutyValue
  );

#endif
}


// ======================================================
// PWM CIKISINI KAPAT
// ======================================================

void stopPWMOutput() {
  pwmPercent = 0;

  applyPWMPercent();
}


// ======================================================
// PWM TEST EKRANI
// ======================================================

void drawPWMTestScreen() {
  drawHeader(
    "       PWM TESTI"
  );

  display.setCursor(0, 14);
  display.print("Pin: D13");

  display.setCursor(62, 14);
  display.print("F:");
  display.print(PWM_FREQUENCY);
  display.print("Hz");

  display.setCursor(0, 26);
  display.print("Duty Cycle: %");
  display.print(pwmPercent);

  display.drawRect(
    5,
    37,
    118,
    12,
    SSD1306_WHITE
  );

  int barWidth = map(
    pwmPercent,
    0,
    100,
    0,
    114
  );

  if (barWidth > 0) {
    display.fillRect(
      7,
      39,
      barWidth,
      8,
      SSD1306_WHITE
    );
  }

  display.drawFastHLine(
    0,
    53,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );

  display.setCursor(0, 56);
  display.print("Y:+ A:- SAG:0/100");

  display.display();
}


// ======================================================
// KESME TEST EKRANI
// ======================================================

void drawInterruptTestScreen() {
  uint32_t pressCount;
  uint32_t bounceCount;
  uint32_t lastIntervalUs;

  copyInterruptValues(
    pressCount,
    bounceCount,
    lastIntervalUs
  );

  drawHeader(
    "      KESME TESTI"
  );

  display.setCursor(0, 14);
  display.println(
    "Buton: D14 -> GND"
  );

  display.setCursor(0, 25);
  display.print("Basma sayisi: ");
  display.print(pressCount);

  display.setCursor(0, 35);
  display.print("Son aralik: ");

  if (
    pressCount < 2 ||
    lastIntervalUs == 0
  ) {
    display.print("--");
  }
  else {
    display.print(
      lastIntervalUs / 1000
    );

    display.print(" ms");
  }

  display.setCursor(0, 45);
  display.print("Reddedilen kesme: ");
  display.print(bounceCount);

  display.drawFastHLine(
    0,
    54,
    SCREEN_WIDTH,
    SSD1306_WHITE
  );

  display.setCursor(0, 56);
  display.print(
    "SOL:Geri SAG:Sifirla"
  );

  display.display();
}


// ======================================================
// KESME EKRANINI PERIYODIK YENILE
// ======================================================

void updateInterruptTestScreen() {
  if (
    currentScreen !=
    SCREEN_INTERRUPT_TEST
  ) {
    return;
  }

  unsigned long currentTime =
    millis();

  if (
    currentTime -
    lastInterruptScreenUpdate <
    INTERRUPT_SCREEN_INTERVAL_MS
  ) {
    return;
  }

  lastInterruptScreenUpdate =
    currentTime;

  drawInterruptTestScreen();
}


// ======================================================
// SISTEM BILGISI EKRANI
// ======================================================

void drawSystemInfo() {
  drawHeader(
    "    SISTEM BILGISI"
  );

  display.setCursor(0, 14);
  display.println("I2C : D10 / D11");

  display.setCursor(0, 25);
  display.println("GPIO: D8 -> D9");

  display.setCursor(0, 36);
  display.println("PWM : D13 / 1 kHz");

  display.setCursor(0, 47);
  display.println("INT : D14 / FALLING");

  display.setCursor(0, 57);
  display.println("SOL:Geri");

  display.display();
}


// ======================================================
// HENUZ YAPILMAYAN TESTLER
// ======================================================

void drawPlaceholder() {
  drawHeader(
    mainMenuItems[
      placeholderMenuIndex
    ]
  );

  display.setCursor(14, 27);

  display.println(
    "BU TEST MODULU"
  );

  display.setCursor(16, 39);

  display.println(
    "SONRA EKLENECEK"
  );

  display.setCursor(0, 56);

  display.println(
    "SOL:Geri"
  );

  display.display();
}


// ======================================================
// ANA MENU SECIMINI AC
// ======================================================

void openMainMenuItem() {
  if (selectedMainItem == 0) {
    scanI2CBus();

    currentScreen =
      SCREEN_I2C_RESULT;

    drawI2CResults();
  }
  else if (
    selectedMainItem == 1
  ) {
    currentScreen =
      SCREEN_SENSOR_MENU;

    drawSensorMenu();
  }
  else if (
    selectedMainItem == 2
  ) {
    currentScreen =
      SCREEN_GPIO_READY;

    drawGPIOReadyScreen();
  }
  else if (
    selectedMainItem == 3
  ) {
    pwmPercent = 0;

    applyPWMPercent();

    currentScreen =
      SCREEN_PWM_TEST;

    drawPWMTestScreen();
  }
  else if (
    selectedMainItem == 4
  ) {
    resetInterruptTest();

    lastInterruptScreenUpdate =
      millis();

    currentScreen =
      SCREEN_INTERRUPT_TEST;

    drawInterruptTestScreen();
  }
  else if (
    selectedMainItem == 5
  ) {
    currentScreen =
      SCREEN_SYSTEM_INFO;

    drawSystemInfo();
  }
  else {
    placeholderMenuIndex =
      selectedMainItem;

    currentScreen =
      SCREEN_PLACEHOLDER;

    drawPlaceholder();
  }
}


// ======================================================
// SENSOR ALT MENU SECIMINI AC
// ======================================================

void openSensorMenuItem() {
  if (selectedSensorItem == 0) {
    currentScreen =
      SCREEN_SENSOR_ID;

    drawSensorIdTest();
  }
  else if (
    selectedSensorItem == 1
  ) {
    initializeLSM6DSM();

    currentScreen =
      SCREEN_LIVE_ACCEL;

    lastLiveUpdate =
      millis();

    drawLiveAcceleration();
  }
  else if (
    selectedSensorItem == 2
  ) {
    initializeLSM6DSM();

    currentScreen =
      SCREEN_LIVE_GYRO;

    lastLiveUpdate =
      millis();

    drawLiveGyroscope();
  }
  else {
    currentScreen =
      SCREEN_LIVE_MAG;

    lastLiveUpdate =
      millis();

    drawLiveMagnetometer();
  }
}


// ======================================================
// JOYSTICK KOMUTUNU ISLE
// ======================================================

void handleJoystickAction(
  JoyAction action
) {
  switch (currentScreen) {

    // --------------------------------------------------
    // ANA MENU
    // --------------------------------------------------

    case SCREEN_MAIN_MENU:

      if (action == JOY_UP) {
        selectedMainItem--;

        if (
          selectedMainItem < 0
        ) {
          selectedMainItem =
            MAIN_MENU_COUNT - 1;
        }

        drawMainMenu();
      }
      else if (
        action == JOY_DOWN
      ) {
        selectedMainItem++;

        if (
          selectedMainItem >=
          MAIN_MENU_COUNT
        ) {
          selectedMainItem = 0;
        }

        drawMainMenu();
      }
      else if (
        action == JOY_RIGHT
      ) {
        openMainMenuItem();
      }

      break;


    // --------------------------------------------------
    // I2C SONUC EKRANI
    // --------------------------------------------------

    case SCREEN_I2C_RESULT: {
      int totalPages =
        (foundDeviceCount + 2) / 3;

      if (totalPages < 1) {
        totalPages = 1;
      }

      if (action == JOY_LEFT) {
        currentScreen =
          SCREEN_MAIN_MENU;

        drawMainMenu();
      }
      else if (
        action == JOY_RIGHT
      ) {
        scanI2CBus();

        drawI2CResults();
      }
      else if (
        action == JOY_UP
      ) {
        i2cPage--;

        if (i2cPage < 0) {
          i2cPage =
            totalPages - 1;
        }

        drawI2CResults();
      }
      else if (
        action == JOY_DOWN
      ) {
        i2cPage++;

        if (
          i2cPage >=
          totalPages
        ) {
          i2cPage = 0;
        }

        drawI2CResults();
      }

      break;
    }


    // --------------------------------------------------
    // SENSOR ALT MENUSU
    // --------------------------------------------------

    case SCREEN_SENSOR_MENU:

      if (action == JOY_LEFT) {
        currentScreen =
          SCREEN_MAIN_MENU;

        drawMainMenu();
      }
      else if (
        action == JOY_UP
      ) {
        selectedSensorItem--;

        if (
          selectedSensorItem < 0
        ) {
          selectedSensorItem =
            SENSOR_MENU_COUNT - 1;
        }

        drawSensorMenu();
      }
      else if (
        action == JOY_DOWN
      ) {
        selectedSensorItem++;

        if (
          selectedSensorItem >=
          SENSOR_MENU_COUNT
        ) {
          selectedSensorItem = 0;
        }

        drawSensorMenu();
      }
      else if (
        action == JOY_RIGHT
      ) {
        openSensorMenuItem();
      }

      break;


    // --------------------------------------------------
    // SENSOR KIMLIK TESTI
    // --------------------------------------------------

    case SCREEN_SENSOR_ID:

      if (action == JOY_LEFT) {
        currentScreen =
          SCREEN_SENSOR_MENU;

        drawSensorMenu();
      }
      else if (
        action == JOY_RIGHT
      ) {
        drawSensorIdTest();
      }

      break;


    // --------------------------------------------------
    // CANLI SENSOR EKRANLARI
    // --------------------------------------------------

    case SCREEN_LIVE_ACCEL:
    case SCREEN_LIVE_GYRO:
    case SCREEN_LIVE_MAG:

      if (action == JOY_LEFT) {
        currentScreen =
          SCREEN_SENSOR_MENU;

        drawSensorMenu();
      }

      break;


    // --------------------------------------------------
    // GPIO TEST HAZIRLIK EKRANI
    // --------------------------------------------------

    case SCREEN_GPIO_READY:

      if (action == JOY_LEFT) {
        currentScreen =
          SCREEN_MAIN_MENU;

        drawMainMenu();
      }
      else if (
        action == JOY_RIGHT
      ) {
        runGPIOTest();

        currentScreen =
          SCREEN_GPIO_RESULT;

        drawGPIOResultScreen();
      }

      break;


    // --------------------------------------------------
    // GPIO TEST SONUC EKRANI
    // --------------------------------------------------

    case SCREEN_GPIO_RESULT:

      if (action == JOY_LEFT) {
        currentScreen =
          SCREEN_MAIN_MENU;

        drawMainMenu();
      }
      else if (
        action == JOY_RIGHT
      ) {
        runGPIOTest();

        drawGPIOResultScreen();
      }

      break;


    // --------------------------------------------------
    // PWM TEST EKRANI
    // --------------------------------------------------

    case SCREEN_PWM_TEST:

      if (action == JOY_LEFT) {
        stopPWMOutput();

        currentScreen =
          SCREEN_MAIN_MENU;

        drawMainMenu();
      }
      else if (
        action == JOY_UP
      ) {
        pwmPercent += 10;

        if (pwmPercent > 100) {
          pwmPercent = 100;
        }

        applyPWMPercent();

        drawPWMTestScreen();
      }
      else if (
        action == JOY_DOWN
      ) {
        pwmPercent -= 10;

        if (pwmPercent < 0) {
          pwmPercent = 0;
        }

        applyPWMPercent();

        drawPWMTestScreen();
      }
      else if (
        action == JOY_RIGHT
      ) {
        if (pwmPercent == 100) {
          pwmPercent = 0;
        }
        else {
          pwmPercent = 100;
        }

        applyPWMPercent();

        drawPWMTestScreen();
      }

      break;


    // --------------------------------------------------
    // KESME TEST EKRANI
    // --------------------------------------------------

    case SCREEN_INTERRUPT_TEST:

      if (action == JOY_LEFT) {
        stopInterruptTest();

        currentScreen =
          SCREEN_MAIN_MENU;

        drawMainMenu();
      }
      else if (
        action == JOY_RIGHT
      ) {
        resetInterruptTest();

        drawInterruptTestScreen();
      }

      break;


    // --------------------------------------------------
    // SISTEM BILGISI VE BOS TESTLER
    // --------------------------------------------------

    case SCREEN_SYSTEM_INFO:
    case SCREEN_PLACEHOLDER:

      if (action == JOY_LEFT) {
        currentScreen =
          SCREEN_MAIN_MENU;

        drawMainMenu();
      }

      break;
  }
}


// ======================================================
// CANLI EKRANI BELIRLI ARALIKLA YENILE
// ======================================================

void updateLiveScreen() {
  unsigned long currentTime =
    millis();

  if (
    currentTime -
    lastLiveUpdate <
    LIVE_UPDATE_INTERVAL_MS
  ) {
    return;
  }

  lastLiveUpdate =
    currentTime;

  if (
    currentScreen ==
    SCREEN_LIVE_ACCEL
  ) {
    drawLiveAcceleration();
  }
  else if (
    currentScreen ==
    SCREEN_LIVE_GYRO
  ) {
    drawLiveGyroscope();
  }
  else if (
    currentScreen ==
    SCREEN_LIVE_MAG
  ) {
    drawLiveMagnetometer();
  }
}


// ======================================================
// SETUP
// ======================================================

void setup() {
  analogReadResolution(12);

  // GPIO test pini acilista LOW durumda tutulur.
  pinMode(GPIO_OUTPUT_PIN, OUTPUT);
  digitalWrite(GPIO_OUTPUT_PIN, LOW);
  pinMode(GPIO_INPUT_PIN, INPUT_PULLDOWN);

  // PWM testi açılana kadar LED kapalı kalsın.
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  digitalWrite(PWM_OUTPUT_PIN, LOW);

  /*
    Kesme butonu D14 ile GND arasina baglanir.
    Dahili pull-up sayesinde buton birakilmisken HIGH,
    basiliyken LOW okunur.
  */
  pinMode(
    INTERRUPT_BUTTON_PIN,
    INPUT_PULLUP
  );

  attachInterrupt(
    digitalPinToInterrupt(
      INTERRUPT_BUTTON_PIN
    ),
    handleInterruptButton,
    FALLING
  );

  Wire.begin(
    I2C_SDA_PIN,
    I2C_SCL_PIN
  );

  Wire.setClock(100000);

  delay(500);

  if (!findOLEDAddress()) {
    while (true) {
      delay(1000);
    }
  }

  if (
    !display.begin(
      SSD1306_SWITCHCAPVCC,
      oledAddress
    )
  ) {
    while (true) {
      delay(1000);
    }
  }

  display.clearDisplay();

  display.display();

  initializeLSM6DSM();

  showBootScreen();

  drawMainMenu();
}


// ======================================================
// LOOP
// ======================================================

void loop() {
  JoyAction action =
    readJoystickAction();

  if (action != JOY_NONE) {
    handleJoystickAction(
      action
    );
  }

  updateLiveScreen();

  updateInterruptButtonRelease();

  updateInterruptTestScreen();

  delay(5);
}
