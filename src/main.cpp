#define LGFX_USE_V1
#include "Arduino.h"
#include <lvgl.h>
#include "demos/lv_demos.h"
#include <LovyanGFX.hpp>
#include "CST816D.h"
#include <Wire.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include "I2C_BM8563.h"
#include "esp_sleep.h"


bool ReminderState = false;                                              // to remind only once
#define uS_TO_S_FACTOR 1000000ULL                                        /* Conversion factor for micro seconds to seconds */
int TIME_TO_SLEEP = 2000; /* Time ESP32 will go to sleep (in seconds) */ // MAX is 2147483647us (35.79) minutes
int brightness = 255;
#define SENSOR_PIN A1
#define REFERENCE_RESISTANCE 100000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950
#define ESP32_ANALOG_RESOLUTION 4095
#define ESP32_ADC_VREF_MV 3300
int celsius1 = 0;
// I2C pins
#define I2C_SDA 4
#define I2C_SCL 5
I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire); // Create an RTC object using the default I2C address and Wire library
I2C_BM8563_DateTypeDef dateStruct;                // Define a structure to hold the date information
I2C_BM8563_TimeTypeDef timeStruct;                // Define a structure to hold the time information

// Touch panel interrupt and reset pins
#define TP_INT 0
#define TP_RST -1

// I2C address for the I/O extender
#define PI4IO_I2C_ADDR 0x43

void touchInterruptHandler()
{
  // Trigger a reset or deep sleep
  // If you want to reset the MCU, you can do something like:
  esp_restart(); // This restarts the ESP32-C3 MCU
}
// Function to initialize I/O extender
void init_IO_extender()
{
  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x01); // test register
  Wire.endTransmission();
  Wire.requestFrom(PI4IO_I2C_ADDR, 1);
  uint8_t rxdata = Wire.read();
  // //Serial.print("Device ID: ");
  // //Serial.println(rxdata, HEX);

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x03);                                                 // IO direction register
  Wire.write((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4)); // set pins 0, 1, 2 as outputs
  Wire.endTransmission();

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x07);                                                    // Output Hi-Z register
  Wire.write(~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4))); // set pins 0, 1, 2 low
  Wire.endTransmission();
}

// Function to set I/O pin state on the extender
void set_pin_io(uint8_t pin_number, bool value)
{

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x05); // test register
  Wire.endTransmission();
  Wire.requestFrom(PI4IO_I2C_ADDR, 1);
  uint8_t rxdata = Wire.read();
  // //Serial.print("Before the change: ");
  // //Serial.println(rxdata, HEX);

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x05); // Output register

  if (!value)
    Wire.write((~(1 << pin_number)) & rxdata); // set pin low
  else
    Wire.write((1 << pin_number) | rxdata); // set pin high
  Wire.endTransmission();

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x05); // test register
  Wire.endTransmission();
  Wire.requestFrom(PI4IO_I2C_ADDR, 1);
  rxdata = Wire.read();
  // //Serial.print("after the change: ");
  // //Serial.println(rxdata, HEX);
}

void alert()
{
  int i;
  for (i = 0; i < 4; i++)
  {
    set_pin_io(0, true); // Set pin 0 to high
    tone(3, 700);
    delay(50);            // Wait for 1000 milliseconds
    set_pin_io(0, false); // Set pin 0 to low
    tone(3, 0);           // Stop the buzzer from producing sound (frequency set to 0)
    delay(50);            // Wait for another 1000 milliseconds
  }
}

#include "NimBLEDevice.h"

static NimBLEServer *pServer;
String Sleep_time = "60";
String Alert = "no";
/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
  {
    // Serial.printf("Client address: %s\n", connInfo.getAddress().toString().c_str());

    /**
     *  We can use the connection handle here to ask for different connection parameters.
     *  Args: connection handle, min connection interval, max connection interval
     *  latency, supervision timeout.
     *  Units; Min/Max Intervals: 1.25 millisecond increments.
     *  Latency: number of intervals allowed to skip.
     *  Timeout: 10 millisecond increments.
     */
    pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
  }

  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
  {
    // Serial.printf("Client disconnected - start advertising\n");
    NimBLEDevice::startAdvertising();
  }

  void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override
  {
    // Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
  }

  /********************* Security handled here *********************/
  uint32_t onPassKeyDisplay() override
  {
    // Serial.printf("Server Passkey Display\n");
    /**
     * This should return a random 6 digit number for security
     *  or make your own static passkey as done here.
     */
    return 123456;
  }

  void onConfirmPassKey(NimBLEConnInfo &connInfo, uint32_t pass_key) override
  {
    // Serial.printf("The passkey YES/NO number: %" PRIu32 "\n", pass_key);
    /** Inject false if passkeys don't match. */
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override
  {
    /** Check that encryption was successful, if not we disconnect the client */
    if (!connInfo.isEncrypted())
    {
      NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
      // Serial.printf("Encrypt connection failed - disconnecting client\n");
      return;
    }

    // Serial.printf("Secured connection to: %s\n", connInfo.getAddress().toString().c_str());
  }
} serverCallbacks;

/** Handler class for characteristic actions */
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    // Serial.printf("%s : onRead(), value: %s\n",
    //  pCharacteristic->getUUID().toString().c_str(),
    //  pCharacteristic->getValue().c_str();
  }

  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    // Serial.printf("%s : onWrite(), value: %s\n",
    // pCharacteristic->getUUID().toString().c_str(),
    // pCharacteristic->getValue().c_str();
    String tempUUID = pCharacteristic->getUUID().toString().c_str();
    String tempValue = pCharacteristic->getValue().c_str();

    if (tempUUID == "0x2a16")
    {
      Sleep_time = tempValue;
    }
    if (tempUUID == "0x2a3f")
    {
      Alert = tempValue;
      alert();
    }
  }

  /**
   *  The value returned in code is the NimBLE host return code.
   */
  void onStatus(NimBLECharacteristic *pCharacteristic, int code) override
  {
    // Serial.printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
  }

  /** Peer subscribed to notifications/indications */
  void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override
  {
    std::string str = "Client ID: ";
    str += connInfo.getConnHandle();
    str += " Address: ";
    str += connInfo.getAddress().toString();
    if (subValue == 0)
    {
      str += " Unsubscribed to ";
    }
    else if (subValue == 1)
    {
      str += " Subscribed to notifications for ";
    }
    else if (subValue == 2)
    {
      str += " Subscribed to indications for ";
    }
    else if (subValue == 3)
    {
      str += " Subscribed to notifications and indications for ";
    }
    str += std::string(pCharacteristic->getUUID());

    // Serial.printf("%s\n", str.c_str());
  }
} chrCallbacks;

/** Handler class for descriptor actions */
class DescriptorCallbacks : public NimBLEDescriptorCallbacks
{
  void onWrite(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override
  {
    std::string dscVal = pDescriptor->getValue();
    // Serial.printf("Descriptor written value: %s\n", dscVal.c_str());
  }

  void onRead(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override
  {
    // Serial.printf("%s Descriptor read\n", pDescriptor->getUUID().toString().c_str());
  }
} dscCallbacks;

Thermistor *thermistor;

#include "ui/ui.h"

// Buffer size definition
#define buf_size 60

// LGFX class definition for display device interface
class LGFX : public lgfx::LGFX_Device
{
  // Internal components of the LGFX class
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void)
  {
    {
      // Configuration for the SPI bus and panel
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 80000000;
      cfg.freq_read = 20000000;
      cfg.spi_3wire = true;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 6;
      cfg.pin_mosi = 7;
      cfg.pin_miso = -1;
      cfg.pin_dc = 2;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = 10;
      cfg.pin_rst = -1;
      cfg.pin_busy = -1;
      cfg.memory_width = 240;
      cfg.memory_height = 240;
      cfg.panel_width = 240;
      cfg.panel_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = false;
      cfg.invert = true;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

// Global instances for the display and touch screen
LGFX tft;
CST816D touch(I2C_SDA, I2C_SCL, TP_RST, TP_INT);

/*Change to your screen resolution*/
static const uint32_t screenWidth = 240;
static const uint32_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[2][screenWidth * buf_size];

#if LV_USE_LOG != 0
// Custom print function for LVGL debugging
void my_print(lv_log_level_t level, const char *file, uint32_t line, const char *fn_name, const char *dsc)
{
  // //Serial.printf("%s(%s)@%d->%s\r\n", file, fn_name, line, dsc);
  Serial.flush();
}
#endif

// Function to handle flushing of display buffer to the screen
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  if (tft.getStartCount() == 0)
  {
    tft.endWrite();
  }

  tft.pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (lgfx::swap565_t *)&color_p->full);

  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

// Function to read touchpad input
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  bool touched;
  uint8_t gesture;
  uint16_t touchX, touchY;
  touched = touch.getTouch(&touchX, &touchY, &gesture);

  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;
    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

void screenBrightness(lv_event_t *e)
{

  set_pin_io(2, 1); // ELECROW C3 has no brightness control
  brightness = 255;
}

void setup()
{


  Wire.begin(4, 5);
  rtc.begin();
  I2C_BM8563_TimeTypeDef timeStruct; // Define a time structure
  timeStruct.hours = 10;     // Set the hour (0 - 23)
  timeStruct.minutes = 0; // Set the minute (0 - 59)
  timeStruct.seconds = 0; // Set the second (0 - 59)
  rtc.setTime(&timeStruct);          // Set the time in the RTC

 
  I2C_BM8563_DateTypeDef dateStruct;                   // Define a date structure
  dateStruct.weekDay = 0;                       // Set the weekday (0 - 6, where 0 is Sunday)
  dateStruct.month = 1; // Set the month (1 - 12)
  dateStruct.date = 1; // Set the day of the month (1 - 31)
  dateStruct.year = 2025;  // Set the year
  rtc.setDate(&dateStruct);                            // Set the date in the RTC



  thermistor = new NTC_Thermistor_ESP32(
      SENSOR_PIN,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE,
      ESP32_ADC_VREF_MV,
      ESP32_ANALOG_RESOLUTION);

  celsius1 = thermistor->readCelsius();

  char buf1[2]; // sprintf text buffer
  sprintf(buf1, "%2d", celsius1);

  /** Initialize NimBLE and set the device name */
  NimBLEDevice::init("Smart Flask");

  /**
   * Set the IO capabilities of the device, each option will trigger a different pairing method.
   *  BLE_HS_IO_DISPLAY_ONLY    - Passkey pairing
   *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
   *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
   */
  // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // use passkey
  // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

  /**
   *  2 different ways to set security - both calls achieve the same result.
   *  no bonding, no man in the middle protection, BLE secure connections.
   *
   *  These are the default values, only shown here for demonstration.
   */
  // NimBLEDevice::setSecurityAuth(false, false, true);

  NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  NimBLEService *pDeadService = pServer->createService("1809"); // Health Thermometer Service UUID
  NimBLECharacteristic *pBeefCharacteristic =
      pDeadService->createCharacteristic("2A1C", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY); // Temperature Measurement 0x2A1C

  pBeefCharacteristic->setValue(buf1);
  pBeefCharacteristic->setCallbacks(&chrCallbacks);

  /**
   *  2902 and 2904 descriptors are a special case, when createDescriptor is called with
   *  either of those uuid's it will create the associated class with the correct properties
   *  and sizes. However we must cast the returned reference to the correct type as the method
   *  only returns a pointer to the base NimBLEDescriptor class.
   */
  NimBLE2904 *pBeef2904 = pBeefCharacteristic->create2904();
  pBeef2904->setFormat(NimBLE2904::FORMAT_UTF8);
  pBeef2904->setCallbacks(&dscCallbacks);

  NimBLEService *pTimeService = pServer->createService("1811"); // Alert Notification Service UUID
  NimBLECharacteristic *pTimeUpdateCharacteristic =
      pTimeService->createCharacteristic("2A16", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY); // 0x2A16 Time Update Control Point

  pTimeUpdateCharacteristic->setValue(Sleep_time);
  pTimeUpdateCharacteristic->setCallbacks(&chrCallbacks);

  NimBLECharacteristic *pAlertCharacteristic =
      pTimeService->createCharacteristic("2A3F", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY); // 0x2A3F Alert Status

  pAlertCharacteristic->setValue("0");
  pAlertCharacteristic->setCallbacks(&chrCallbacks);

  /** Start the services when finished creating all Characteristics and Descriptors */
  pDeadService->start();
  pTimeService->start();

  /** Create an advertising instance and add the services to the advertised data */
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName("Maker's Fun Duck");
  pAdvertising->addServiceUUID(pDeadService->getUUID());
  pAdvertising->addServiceUUID(pTimeService->getUUID());
  /**
   *  If your device is battery powered you may consider setting scan response
   *  to false as it will extend battery life at the expense of less data sent.
   */
  pAdvertising->enableScanResponse(false);
  pAdvertising->start();

  // Initialization of serial communication, I2C, display, touch screen, and LVGL
  // Serial.begin(115200); /* prepare for possible serial debug */
  // //Serial.println("I am LVGL_Arduino");



  init_IO_extender();
  delay(100);

  set_pin_io(3, true);  // TP_RESET
  set_pin_io(2, true);  // LCD LED
  set_pin_io(4, true);  // LCD RESET
  set_pin_io(0, false); // Set pin 0 to low
  delay(100);
  tft.init();
  tft.initDMA();
  tft.startWrite();
  tft.setColor(0, 0, 0);
  tft.fillScreen(TFT_BLACK);
  delay(200);
  touch.begin();
  lv_init();

  delay(100);

#if LV_USE_LOG != 0
  // lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  lv_disp_draw_buf_init(&draw_buf, buf[0], buf[1], screenWidth * buf_size);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
  // lv_timer_handler(); /* let the GUI do its work */
  // uncomment one of these demos
  // lv_demo_widgets(); // OK

  // ////Serial.println("Setup done");

  if (0)
  {

    lv_deinit();
    set_pin_io(2, false); // LCD LED
    set_pin_io(3, false); // TP_RESET
    set_pin_io(4, false); // LCD RESET

    esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR); // sleep for set times
    esp_deep_sleep_start();
  }

  ui_init();
  lv_arc_set_value(ui_Arc1, celsius1);
  lv_label_set_text(ui_Label_Celcius, buf1);
  lv_timer_handler(); /* let the GUI do its work */

  delay(3000);

  TIME_TO_SLEEP = Sleep_time.toInt();

  // touch.interrupt_prepare(); //doesn't work. couldn't find the interrupt enable register

  // esp_sleep_enable_gpio_wakeup();
  // esp_deep_sleep_enable_gpio_wakeup(0, ESP_GPIO_WAKEUP_GPIO_LOW); // Wake up on a HIGH signal at GPIO0
  // esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR); // sleep for set times
  // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // sleep for set times
  // esp_deep_sleep_start();


}

void loop()
{
  celsius1 = thermistor->readCelsius();

  char buf1[1]; // sprintf text buffer
  sprintf(buf1, "%2d", celsius1);
  lv_arc_set_value(ui_Arc1, celsius1);
  lv_label_set_text(ui_Label_Celcius, buf1);
  lv_arc_set_value(ui_Arc2, brightness);
  brightness = brightness - 5;
  if (brightness == 0)
  {
    set_pin_io(2, 0); // ELECROW C3 has no brightness control
  }
  lv_timer_handler(); /* let the GUI do its work */
 // delay(1);
  rtc.getTime(&timeStruct); // Get the current time from the RTC
  if (timeStruct.hours > 10 && timeStruct.minutes % Sleep_time.toInt() == 0 && ReminderState)
  {
    brightness = 255;
    alert();
    ReminderState = false;
  }
  if (timeStruct.seconds < 0)
  {
    ReminderState = true;
  }
}
