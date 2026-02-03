#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_GFX.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_SSD1306.h>

#define bleServerName "VELOCITY_METER_ESP32"
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"
bool deviceConnected = false;
BLECharacteristic bmeTemperatureCelsiusCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeTemperatureCelsiusDescriptor(BLEUUID((uint16_t)0x2902));

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels

VL53L1X sensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  //while (!Serial) {}
  Serial.begin(115200);
  BLEDevice::init(bleServerName);
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* velocityService = pServer->createService(SERVICE_UUID);

  velocityService->addCharacteristic(&bmeTemperatureCelsiusCharacteristics);
  bmeTemperatureCelsiusDescriptor.setValue("Vel");
  bmeTemperatureCelsiusCharacteristics.addDescriptor(&bmeTemperatureCelsiusDescriptor);

  velocityService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();

  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C
  pinMode(2, INPUT);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.display();

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1)
      ;
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(10);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);
}

float measure_speed_mps() {
  int start_dist_mm = sensor.ranging_data.range_mm;
  // for(int i = 0; i<2;i++){
  //   sensor.read();
  //   start_dist_mm+= sensor.ranging_data.range_mm;
  // }
  //start_dist_mm = start_dist_mm/3;
  long start_time = micros();

  delayMicroseconds(1000);  // 1 ms delay

  sensor.read();
  long end_time = micros();
  int end_dist_mm = sensor.ranging_data.range_mm;
  // for(int i = 0; i<2;i++){
  //   sensor.read();
  //   end_dist_mm+= sensor.ranging_data.range_mm;
  // }
  //end_dist_mm = end_dist_mm/3;

  float delta_dist_m = (end_dist_mm - start_dist_mm) / 1000.0;  // mm to meters
  float delta_time_s = (end_time - start_time) / 1e6;           // μs to seconds

  if (delta_time_s == 0) return 0.0;   // Avoid division by zero
  return delta_dist_m / delta_time_s;  // Speed in meters per second
}


void loop() {
  sensor.read();

  Serial.print("range: ");
  Serial.print(sensor.ranging_data.range_mm);

  display.setTextSize(2);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);
  // display.println(F("Dist mm: "));
  // display.println(sensor.ranging_data.range_mm);
  // display.println(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  // display.display();

  display.println(F("Vel m/s: "));
  //display.println(sensor.ranging_data.range_mm);
  double a = measure_speed_mps();
  display.println(a);

  //if (deviceConnected) {
    static char temperatureCTemp[6];
    dtostrf(a, 6, 2, temperatureCTemp);
    bmeTemperatureCelsiusCharacteristics.setValue(temperatureCTemp);
    bmeTemperatureCelsiusCharacteristics.notify();
  //}
  //display.println(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));

  display.println(F("Dist mm:"));
  display.println(sensor.ranging_data.range_mm);
  display.display();

  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.print(sensor.ranging_data.ambient_count_rate_MCPS);

  Serial.println();
}