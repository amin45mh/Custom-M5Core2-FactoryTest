#include <M5Unified.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <SD.h>
#include "line3D.h"
#include "fft.h"

/***************************************************
 *           ACCELEROMETER CUBE START
 ***************************************************/


M5Canvas Disbuff            = M5Canvas(&M5.Display);
M5Canvas Dis3Dbuff          = M5Canvas(&M5.Display);
M5Canvas DisFFTbuff         = M5Canvas(&M5.Display);

line3D line3d;

float accX = 0;
float accY = 0;
float accZ = 0;

double theta = 0, last_theta = 0;
double phi = 0, last_phi = 0;
double alpha = 0.2;

line_3d_t x = {.start_point = {0, 0, 0}, .end_point = {0, 0, 0}};
line_3d_t y = {.start_point = {0, 0, 0}, .end_point = {0, 0, 0}};
line_3d_t z = {.start_point = {0, 0, 0}, .end_point = {0, 0, 30}};

line_3d_t rect_dis;

line_3d_t rect[12] = {
    {.start_point = {-1, -1, 1}, .end_point = {1, -1, 1}},
    {.start_point = {1, -1, 1}, .end_point = {1, 1, 1}},
    {.start_point = {1, 1, 1}, .end_point = {-1, 1, 1}},
    {.start_point = {-1, 1, 1}, .end_point = {-1, -1, 1}},
    {
        .start_point = {-1, -1, 1},
        .end_point   = {-1, -1, -1},
    },
    {
        .start_point = {1, -1, 1},
        .end_point   = {1, -1, -1},
    },
    {
        .start_point = {1, 1, 1},
        .end_point   = {1, 1, -1},
    },
    {
        .start_point = {-1, 1, 1},
        .end_point   = {-1, 1, -1},
    },
    {.start_point = {-1, -1, -1}, .end_point = {1, -1, -1}},
    {.start_point = {1, -1, -1}, .end_point = {1, 1, -1}},
    {.start_point = {1, 1, -1}, .end_point = {-1, 1, -1}},
    {.start_point = {-1, 1, -1}, .end_point = {-1, -1, -1}},
};
line_3d_t rect_source[12];

void setupRect() {
    for (int n = 0; n < 12; n++) {
        rect_source[n].start_point.x = rect[n].start_point.x * 30;
        rect_source[n].start_point.y = rect[n].start_point.y * 30;
        rect_source[n].start_point.z = rect[n].start_point.z * 30;
        rect_source[n].end_point.x   = rect[n].end_point.x * 30;
        rect_source[n].end_point.y   = rect[n].end_point.y * 30;
        rect_source[n].end_point.z   = rect[n].end_point.z * 30;
    }
}

void MPU6886Page() {

    M5.Imu.getAccelData(&accX, &accY, &accZ);
    if ((accX < 1) && (accX > -1)) {
        theta = asin(-accX) * 57.295;
    }
    if (accZ != 0) {
        phi = atan(accY / accZ) * 57.295;
    }

    theta = alpha * theta + (1 - alpha) * last_theta;
    phi   = alpha * phi + (1 - alpha) * last_phi;

    Disbuff.fillRect(0, 0, 320, 240, Dis3Dbuff.color565(0x0, 0x0, 0x0));


    z.end_point.x = 0;
    z.end_point.y = 0;
    z.end_point.z = 24;
    line3d.RotatePoint(&z.end_point, theta, phi, 0);
    line3d.RotatePoint(&z.end_point, &x.end_point, -90, 0, 0);
    line3d.RotatePoint(&z.end_point, &y.end_point, 0, 90, 0);

    line3d.printLine3D(&Disbuff, &x, TFT_GREEN);
    line3d.printLine3D(&Disbuff, &y, TFT_GREEN);
    line3d.printLine3D(&Disbuff, &z, TFT_GREEN);

    uint16_t linecolor = Disbuff.color565(0xff, 0x9c, 0x00);

    for (int n = 0; n < 12; n++) {
        line3d.RotatePoint(&rect_source[n].start_point,
                            &rect_dis.start_point, theta, phi, (double)0);
        line3d.RotatePoint(&rect_source[n].end_point, &rect_dis.end_point,
                            theta, phi, (double)0);
        line3d.printLine3D(&Disbuff, &rect_dis, linecolor);
    }

    last_theta = theta;
    last_phi   = phi;


    Disbuff.fillRect(0, 206, 320, 34, TFT_BLACK);
    Disbuff.pushSprite(0, 40);
    
}

/***************************************************
 *           ACCELEROMETER CUBE END
 ***************************************************/

/***************************************************
 *           MICROPHONE VISUALIZER START
 ***************************************************/
#define MIC_SAMPLE_RATE 44100
#define MIC_SAMPLE_COUNT 1024

static QueueHandle_t fftvalueQueue = nullptr;

static void i2sMicroFFTtask(void *arg) {
    uint8_t FFTDataBuff[128];
    uint8_t FFTValueBuff[24];
    int16_t *buffptr = (int16_t *)calloc(MIC_SAMPLE_COUNT, sizeof(int16_t));
    double data = 0;
    float adc_data;
    uint16_t ydata;
    uint32_t subData;

    while (1) {
        if (M5.Mic.isEnabled() &&
            M5.Mic.record(buffptr, MIC_SAMPLE_COUNT, MIC_SAMPLE_RATE)) {
            while (M5.Mic.isRecording()) {
                vTaskDelay(1);
            }
            fft_config_t *real_fft_plan =
                fft_init(1024, FFT_REAL, FFT_FORWARD, NULL, NULL);

            for (int count_n = 0; count_n < real_fft_plan->size; count_n++) {
                adc_data = (float)map(buffptr[count_n], INT16_MIN, INT16_MAX,
                                      -2000, 2000);
                real_fft_plan->input[count_n] = adc_data;
            }
            fft_execute(real_fft_plan);

            for (int count_n = 1; count_n < real_fft_plan->size / 4;
                 count_n++) {
                data = sqrt(real_fft_plan->output[2 * count_n] *
                                real_fft_plan->output[2 * count_n] +
                            real_fft_plan->output[2 * count_n + 1] *
                                real_fft_plan->output[2 * count_n + 1]);
                if ((count_n - 1) < 128) {
                    data                       = (data > 2000) ? 2000 : data;
                    ydata                      = map(data, 0, 2000, 0, 255);
                    FFTDataBuff[128 - count_n] = ydata;
                }
            }

            for (int count = 0; count < 24; count++) {
                subData = 0;
                for (int count_i = 0; count_i < 5; count_i++) {
                    subData += FFTDataBuff[count * 5 + count_i];
                }
                subData /= 5;
                FFTValueBuff[count] = map(subData, 0, 255, 0, 8);
            }
            xQueueSend(fftvalueQueue, (void *)&FFTValueBuff, 0);
            fft_destroy(real_fft_plan);
            // Serial.printf("mmp\r\n");
        } else {
            delay(10);
        }
    }
}

void microPhoneSetup() {
    fftvalueQueue = xQueueCreate(5, 24 * sizeof(uint8_t));
    if (fftvalueQueue == 0) {
        return;
    }

    // Mic and speaker share the I2S bus on Core2; run the mic until the
    // speaker test needs it.
    M5.Speaker.end();
    M5.Mic.begin();
    xTaskCreatePinnedToCore(i2sMicroFFTtask, "microPhoneTask", 4096, NULL, 3,
                            NULL, 0);

    DisFFTbuff.createSprite(260, 120);
}

void MicroPhoneFFT() {
    uint8_t FFTValueBuff[24];
    xQueueReceive(fftvalueQueue, (void *)&FFTValueBuff, portMAX_DELAY);
    DisFFTbuff.fillRect(0, 0, 260, 120, DisFFTbuff.color565(0x33, 0x20, 0x00));
    uint32_t colorY = DisFFTbuff.color565(0xff, 0x9c, 0x00);
    uint32_t colorG = DisFFTbuff.color565(0x66, 0xff, 0x00);
    uint32_t colorRect;
    for (int x = 0; x < 24; x++) {
        for (int y = 0; y < 9; y++) {
            if (y < FFTValueBuff[23 - x]) {
                colorRect = colorY;
            } else if (y == FFTValueBuff[23 - x]) {
                colorRect = colorG;
            } else {
                continue;
            }
            DisFFTbuff.fillRect(x * 12, 120 - y * 12 - 5, 10, 10, colorRect);
        }
    }
    DisFFTbuff.pushSprite(30, 50);
}

/***************************************************
 *           MICROPHONE VISUALIZER END
 ***************************************************/

/***************************************************
 *                PSRAM TEST START
 ***************************************************/
int checkPsram() {
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(20, 50);
  uint8_t *testbuff = (uint8_t *)ps_calloc(100 * 1024, sizeof(uint8_t));
  if (testbuff == nullptr) {
      M5.Lcd.setTextColor(TFT_RED);
      M5.Lcd.print("PSRAM malloc failed");
      M5.Lcd.setTextColor(TFT_WHITE);
      return -1;
  } else{
      M5.Lcd.setTextColor(TFT_GREEN);
      M5.Lcd.print("PSRAM malloc Successful");
      M5.Lcd.setTextColor(TFT_WHITE);
  }
  delay(100);

  M5.Lcd.setCursor(20, 80);
  for (size_t i = 0; i < 102400; i++) {
      testbuff[i] = 0xA5;
      if (testbuff[i] != 0xA5) {
          M5.Lcd.setTextColor(TFT_RED);
          M5.Lcd.print("PSRAM read failed");
          M5.Lcd.setTextColor(TFT_WHITE);
          return -1;
      }
  }
  M5.Lcd.setTextColor(TFT_GREEN);
  M5.Lcd.print("PSRAM W&R Successful");
  M5.Lcd.setTextColor(TFT_WHITE);
  return 0;
}
/***************************************************
 *                 PSRAM TEST END
 ***************************************************/

/***************************************************
 *                    TEST START
 ***************************************************/

enum TestStep {
  TEST_PSRAM,
  TEST_PORTS,
  TEST_LEDS,
  TEST_DISPLAY,
  TEST_TOUCH,
  TEST_BUTTONS,
  TEST_IMU,
  TEST_MICROPHONE,
  TEST_SPEAKER,
  TEST_VIBRATION,
  TEST_RTC,
  TEST_BATTERY,
  TEST_MICROSD,
  TEST_WIFI,
  TEST_COUNT
};


TestStep currentTest = TEST_PSRAM;

bool results[] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false};

bool touchDetected = false;
bool buttonDetected = false;
bool imuDetected = false;

void drawHeader(const char* title) {
  M5.Display.clear();
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(10, 10);
  M5.Display.println(title);

  if (currentTest == TEST_TOUCH){
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 210);
    M5.Display.println("Tap BtnB to PASS or BtnC to FAIL.");
  }
}

void drawNextButtons() {
  if (currentTest != TEST_TOUCH){
    M5.Display.fillRoundRect(140, 190, 80, 40, 8, GREEN);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(158, 202);
    M5.Display.print("PASS");

    M5.Display.fillRoundRect(230, 190, 80, 40, 8, RED);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(248, 202);
    M5.Display.print("FAIL");
  }
}

bool nextPressed() {
  auto touch = M5.Touch.getDetail();

  if (currentTest != TEST_TOUCH){
    if (touch.wasPressed()) {
      //Pass Button
      if (touch.x >= 140 && touch.x <= 220 && touch.y >= 190 && touch.y <= 230) {
        updateResults(true);
        return true;
      }

      //Fail Button
      if (touch.x >= 230 && touch.x <= 310 && touch.y >= 190 && touch.y <= 230) {
        updateResults(false);
        return true;
      }
    }
  }
  else{
    if (M5.BtnB.wasPressed()){
      updateResults(true);
      return true;
    }
    if (M5.BtnC.wasPressed()){
      updateResults(false);
      return true;
    }
  }

  return false;
}

void updateResults(bool pass) {
  //may add serial port messages in the future
  results[static_cast<int>(currentTest)] = pass;
}

void setupTestScreen() {
  switch (currentTest) {
    case TEST_PORTS:
      drawHeader("Port Test");
      M5.Display.setTextSize(2);
      M5.Display.setCursor(10, 60);
      M5.Display.printf("Angle Unit: %d", 0);
      M5.Display.setCursor(10, 120);
      M5.Display.printf("Blue Button Pressed: No");
      M5.Display.setCursor(10, 150);
      M5.Display.printf("Red Button Pressed: No");
      drawNextButtons();
      break;
    
    case TEST_LEDS:
      drawHeader("LED Test");
      M5.Display.setCursor(10, 90);
      M5.Display.setTextSize(2);
      drawNextButtons();
      break;
    
    case TEST_DISPLAY:
      drawHeader("Display Test");
      M5.Display.fillRect(10, 50, 60, 60, RED);
      M5.Display.fillRect(80, 50, 60, 60, GREEN);
      M5.Display.fillRect(150, 50, 60, 60, BLUE);
      M5.Display.fillRect(220, 50, 60, 60, WHITE);
      M5.Display.setCursor(10, 120);
      M5.Display.setTextSize(2);
      M5.Display.println("Press any of the colors.");
      drawNextButtons();
      break;

    case TEST_TOUCH:
      touchDetected = false;
      drawHeader("Touch Test");
      M5.Display.setCursor(10, 60);
      M5.Display.setTextSize(2);
      M5.Display.println("Touch the screen.");
      drawNextButtons();
      break;

    case TEST_BUTTONS:
      buttonDetected = false;
      drawHeader("Button Test");
      M5.Display.setCursor(10, 60);
      M5.Display.setTextSize(2);
      M5.Display.println("Press BtnA, BtnB, or BtnC");

      M5.Display.setTextColor(RED);

      M5.Display.setCursor(10, 100);
      M5.Display.println("Press BtnA");

      M5.Display.setCursor(10, 125);
      M5.Display.println("Press BtnB");

      M5.Display.setCursor(10, 150);
      M5.Display.println("Press BtnC");

      drawNextButtons();
      break;

    case TEST_IMU:
      imuDetected = false;
      drawHeader("IMU Test");
      drawNextButtons();
      break;

    case TEST_MICROPHONE:
      drawHeader("Microphone Test");
      drawNextButtons();
      break;

    case TEST_SPEAKER:
      drawHeader("Speaker Test");
      M5.Display.setCursor(10, 60);
      M5.Display.setTextSize(2);
      M5.Display.println("Playing tone...");
      M5.Mic.end();
      M5.Speaker.begin();
      M5.Speaker.tone(1000, 500);
      delay(600);
      M5.Display.setCursor(10, 120);
      M5.Display.setTextSize(2);
      M5.Display.println("Pass if you heard a beep.");
      drawNextButtons();
      break;

    case TEST_VIBRATION:
      drawHeader("Vibration Test");
      M5.Display.setCursor(10, 60);
      M5.Display.setTextSize(2);
      M5.Display.println("Vibrating...");
      M5.Power.setVibration(200);
      delay(700);
      M5.Power.setVibration(0);
      M5.Display.setCursor(10, 120);
      M5.Display.println("Pass if you felt a");
      M5.Display.setCursor(10, 150);
      M5.Display.println("vibration.");
      drawNextButtons();
      break;

    case TEST_RTC: {
      drawHeader("RTC Test");

      auto dt = M5.Rtc.getDateTime();

      M5.Display.setCursor(10, 60);
      M5.Display.setTextSize(2);
      M5.Display.printf("%04d-%02d-%02d",
                        dt.date.year,
                        dt.date.month,
                        dt.date.date);

      M5.Display.setCursor(10, 80);
      M5.Display.printf("%02d:%02d:%02d\n",
                        dt.time.hours,
                        dt.time.minutes,
                        dt.time.seconds);

      M5.Display.setCursor(10, 120);
      M5.Display.println("Pass if you see the");
      M5.Display.setCursor(10, 140);
      M5.Display.println("correct time.");
      drawNextButtons();
      break;
    }

    case TEST_BATTERY: {
      drawHeader("Battery / Power Test");

      int batteryLevel = -1;
      batteryLevel = M5.Power.getBatteryLevel();

      if (batteryLevel >= 0){

        updateResults(true);

        bool charging = M5.Power.isCharging();

        M5.Display.setCursor(10, 60);
        M5.Lcd.setTextColor(TFT_GREEN);
        M5.Display.setTextSize(2);
        M5.Display.printf("Battery: %d%%", batteryLevel);
        M5.Display.setCursor(10, 80);
        M5.Display.setTextSize(2);
        M5.Display.printf("Charging: %s\n", charging ? "Yes" : "No");
        M5.Lcd.setTextColor(TFT_WHITE);
      }
      
      else{
        updateResults(false);
        M5.Display.setCursor(10, 120);
        M5.Display.setTextSize(2);
        M5.Lcd.setTextColor(TFT_RED);
        M5.Display.println("Power chip did not respond.");
      }

      delay(3000);

      nextTest();

      break;
    }

    case TEST_MICROSD: {
      drawHeader("microSD Test");

      bool sdOK = SD.begin(GPIO_NUM_4);

      M5.Display.setCursor(10, 60);
      M5.Display.setTextSize(2);

      if (sdOK) {
        M5.Display.println("microSD detected");
      } else {
        M5.Lcd.setTextColor(TFT_RED);
        M5.Display.println("No microSD detected");
        M5.Display.setCursor(10, 80);
        M5.Display.setTextSize(2);
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Display.println("Pass if unused.");
      }

      drawNextButtons();

      break;
    }

    case TEST_WIFI: {
      drawHeader("Wi-Fi Test");

      WiFi.mode(WIFI_STA);
      int networks = WiFi.scanNetworks();

      M5.Display.setCursor(10, 60);
      M5.Display.setTextSize(2);

      if (networks >= 0) {
        updateResults(true);
        M5.Lcd.setTextColor(TFT_GREEN);
        M5.Display.printf("%d networks found:", networks);
        M5.Lcd.setTextColor(TFT_WHITE);
        for (int i = 0; i < (networks < 6 ? networks : 5); i++){
          M5.Display.setCursor(10, 80+20*i);
          M5.Display.println(" "+WiFi.SSID(i));
        }
        M5.Lcd.setTextColor(TFT_WHITE);
      } else {
        updateResults(false);
        M5.Lcd.setTextColor(TFT_RED);
        M5.Display.println("Wi-Fi scan failed");
        M5.Lcd.setTextColor(TFT_WHITE);
      }

      WiFi.scanDelete();
      WiFi.mode(WIFI_OFF);
      delay(3000);
      nextTest();
      break;
    }

    default:
      drawHeader("Tests Complete:");
      M5.Display.setCursor(10, 40);
      M5.Display.setTextSize(1);
      for (int i=0; i<static_cast<int>(TEST_COUNT); i++){
        M5.Display.setCursor(10, 40+i*10);

        auto color = RED;
        if (results[i]){
          color = GREEN;
        }
        M5.Display.setTextColor(color);

        String text;

        switch(i){
          case 0:{
            text = "PSRAM";
            break;
          }
          case 1:{
            text = "PORTS";
            break;
          }
          case 2:{
            text = "LED";
            break;
          }
          case 3:{
            text = "DISPLAY";
            break;
          }
          case 4:{
            text = "TOUCH";
            break;
          }
          case 5:{
            text = "BUTTONS";
            break;
          }
          case 6:{
            text = "IMU";
            break;
          }
          case 7:{
            text = "MICROPHONE";
            break;
          }
          case 8:{
            text = "SPEAKER";
            break;
          }
          case 9:{
            text = "VIBRATION";
            break;
          }
          case 10:{
            text = "RTC";
            break;
          }
          case 11:{
            text = "BATTERY";
            break;
          }
          case 12:{
            text = "MICROSD";
            break;
          }
          case 13:{
            text = "WIFI";
            break;
          }
        }
        M5.Display.println(text);
      }
      M5.Display.setTextSize(2);
      M5.Display.setCursor(10, 220);
      M5.Display.println("Reset device to run again");
      break;
  }
}

int startTime = 0;

// Port test variables
int potentiometerValue = 0;
int blueButtonState = 0;
int redButtonState = 0;

// LED test variables
#define LED_PIN 25
#define LED_COUNT 10
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
int ledCounter = 0;
int ledColorCounter = 0;

void nextTest() {
  if (currentTest < TEST_COUNT) {
    currentTest = (TestStep)(currentTest + 1);

    if (currentTest == TEST_DISPLAY){// Turn off LEDs
      strip.setBrightness(0);
      strip.show();
    }

    if (currentTest != TEST_PORTS){
      setupTestScreen();
    }
  }
}

void setup() {
  auto cfg = M5.config();

  cfg.output_power = true;
  cfg.internal_imu = true;
  cfg.internal_rtc = true;
  cfg.internal_spk = true;
  cfg.internal_mic = true;

  M5.begin(cfg);

  M5.Display.setRotation(1);
  M5.Display.setBrightness(180);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);

  //IMU Cube
  setupRect();
  line3d.setZeroOffset(160, 80);
  Dis3Dbuff.createSprite(320, 160);
  Disbuff.createSprite(320, 140);
  //********

  //Microphone
  microPhoneSetup();
  //**********

  //LEDs
  strip.begin();
  strip.setBrightness(50);

  //PSRAM
  drawHeader("PSRAM Check:");
  updateResults(!checkPsram());
  delay(2000);
  nextTest();

  //Port setup screen
  drawHeader("Connect the following:");
  M5.Display.setCursor(10, 60);
  M5.Display.setTextSize(2);
  M5.Display.println("Angle Unit: Port A");
  M5.Display.setCursor(10, 90);
  M5.Display.println("Dual Button Unit: Port C");
  M5.Display.setCursor(10, 120);
  M5.Display.println("Tap BtnA when ready.");
  //*****************

  while(1){
    M5.update();
    if (M5.BtnA.wasPressed()) {
      break;
    }
  }

  startTime = millis();

  pinMode(33, INPUT); // Port A Potentiometer
  pinMode(13, INPUT); // Port C Blue button
  pinMode(14, INPUT); // Port C Red button

  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Display.clear();
  setupTestScreen();
}

void loop() {
  M5.update();

  switch (currentTest) {
    case TEST_PORTS:{
      int newPotentiometerValue = analogRead(33);
      int blueButtonValue = analogRead(13);
      int redButtonValue = analogRead(14);

      if (abs(newPotentiometerValue - potentiometerValue) > 70){ //debaunce for potentiometer
        M5.Display.fillRect(10, 60, 240, 20, BLACK);
        potentiometerValue = newPotentiometerValue;
        M5.Display.setCursor(10, 60);
        M5.Display.printf("Angle Unit: %d", potentiometerValue);
      }
      if (!blueButtonValue && !blueButtonState){ //blue button pressed
        blueButtonState = 1;
        M5.Display.fillRect(10, 120, 310, 20, BLACK);
        M5.Display.setCursor(10, 120);
        M5.Lcd.println("Blue Button Pressed: Yes");
      }
      else if (blueButtonValue && blueButtonState){
        blueButtonState = 0;
        M5.Display.fillRect(10, 120, 310, 20, BLACK);
        M5.Display.setCursor(10, 120);
        M5.Lcd.print("Blue Button Pressed: No ");
      }

      if (!redButtonValue && !redButtonState){ //red button pressed
        redButtonState = 1;
        M5.Display.fillRect(10, 150, 310, 20, BLACK);
        M5.Display.setCursor(10, 150);
        M5.Lcd.print("Red Button Pressed: Yes");
      }
      else if (redButtonValue && redButtonState){
        redButtonState = 0;
        M5.Display.fillRect(10, 150, 310, 20, BLACK);
        M5.Display.setCursor(10, 150);
        M5.Lcd.print("Red Button Pressed: No ");
      }

      break;
    }

    case TEST_LEDS:{
      if (millis() - startTime >= 1000){
        startTime = millis();
        if (ledCounter == 5){
          ledCounter = 0;
          ledColorCounter++;
        }
        if (ledColorCounter == 3){
          ledColorCounter = 0;
        }
        if (ledCounter > 0){
          strip.setPixelColor(ledCounter-1, strip.Color(0, 0, 0));
          strip.setPixelColor(ledCounter+5-1, strip.Color(0, 0, 0));
        }
        else{
          strip.setPixelColor(4, strip.Color(0, 0, 0));
          strip.setPixelColor(9, strip.Color(0, 0, 0));
        }
        switch (ledColorCounter){
          case 0:
            strip.setPixelColor(ledCounter, strip.Color(255, 0, 0));
            strip.setPixelColor(ledCounter+5, strip.Color(255, 0, 0));
            break;
          case 1:
            strip.setPixelColor(ledCounter, strip.Color(0, 255, 0));
            strip.setPixelColor(ledCounter+5, strip.Color(0, 255, 0));
            break;
          case 2:
            strip.setPixelColor(ledCounter, strip.Color(0, 0, 255));
            strip.setPixelColor(ledCounter+5, strip.Color(0, 0, 255));
            break;
        }
        
        strip.show();
        ledCounter++;
      }
      break;
    }

    case TEST_DISPLAY:{
      auto touch = M5.Touch.getDetail();
      bool colorPressed = false;

      if (touch.isPressed()) {
        if (touch.x >= 10 && touch.x <= 50 && touch.y >= 70 && touch.y <= 130) {
          M5.Display.fillRect(0, 0, 320, 240, RED);
          colorPressed = true;
        }

        else if (touch.x >= 80 && touch.x <= 140 && touch.y >= 70 && touch.y <= 130) {
          M5.Display.fillRect(0, 0, 320, 240, GREEN);
          colorPressed = true;
        }

        else if (touch.x >= 150 && touch.x <= 210 && touch.y >= 70 && touch.y <= 130) {
          M5.Display.fillRect(0, 0, 320, 240, BLUE);
          colorPressed = true;
        }

        else if (touch.x >= 220 && touch.x <= 280 && touch.y >= 70 && touch.y <= 130) {
          M5.Display.fillRect(0, 0, 320, 240, WHITE);
          colorPressed = true;
        }

        if (colorPressed){
          delay(2000);
          setupTestScreen();
        }
      }
      break;
    }

    case TEST_TOUCH: {
      auto touch = M5.Touch.getDetail();

      if (touch.isPressed()) {
        touchDetected = true;

        M5.Display.fillRect(10, 100, 210, 60, BLACK);
        M5.Display.setTextSize(2);

        M5.Display.setCursor(10, 100);
        M5.Display.setTextColor(GREEN);
        M5.Display.printf("X: %d", touch.x);
        M5.Display.setCursor(10, 130);
        M5.Display.printf("Y: %d", touch.y);
        M5.Display.setTextColor(WHITE);
      }

      break;
    }

    case TEST_BUTTONS:
      if (M5.BtnA.wasPressed()) {
        buttonDetected = true;
        M5.Display.setCursor(10, 100);
        M5.Display.fillRect(10, 100, 210, 20, BLACK);
        M5.Display.setTextColor(GREEN);
        M5.Display.setTextSize(2);
        M5.Display.println("BtnA pressed");
        M5.Display.setTextColor(WHITE);
      }

      if (M5.BtnB.wasPressed()) {
        buttonDetected = true;
        M5.Display.setCursor(10, 125);
        M5.Display.fillRect(10, 125, 210, 20, BLACK);
        M5.Display.setTextColor(GREEN);
        M5.Display.setTextSize(2);
        M5.Display.println("BtnB pressed");
        M5.Display.setTextColor(WHITE);
      }

      if (M5.BtnC.wasPressed()) {
        buttonDetected = true;
        M5.Display.setCursor(10, 150);
        M5.Display.fillRect(10, 150, 210, 20, BLACK);
        M5.Display.setTextColor(GREEN);
        M5.Display.setTextSize(2);
        M5.Display.println("BtnC pressed");
        M5.Display.setTextColor(WHITE);
      }
      break;

    
    case TEST_IMU: {
      MPU6886Page();
      break;
    }

    case TEST_MICROPHONE: {
      MicroPhoneFFT();
      break;
    }

    default:
      break;
  }

  if (nextPressed()) {
    nextTest();
    delay(100);
  }
}