// Libraries
#include <LCD_I2C.h>

// Pins
const int PIN_LED = 14;

const int PIN_DIR = 18;
const int PIN_STEP = 19;
const int PIN_NFAULT = 34;
const int PIN_MOTOR_ON = 5;

const int PIN_UP = 25;
const int PIN_DOWN = 26;
const int PIN_LEFT = 27;
const int PIN_RIGHT = 32;
const int PIN_GO = 33;

const int PIN_LIMIT1 = 2;
const int PIN_LIMIT2 = 4;

const int PIN_VALVE1 = 12;
const int PIN_VALVE2 = 13;

// LCD
const int LCD_ADDR = 0x27;
const int LCD_WIDTH = 20;
const int LCD_HEIGHT = 4;

// TIMING
const int HOLD_THRESHOLD_MS = 500;
const int MOTOR_TICK_PERIOD_US = 500;

LCD_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT);

enum lcd_opt {
  LCD_JOG,
  LCD_DIR,
  LCD_COUNT,
  LCD_TPI,
  LCD_TEST,
  LCD_GO,
  LCD_NUMENTRIES,
};

enum Button {
  BUTTON_NONE,
  BUTTON_UP,
  BUTTON_UP_HOLD,
  BUTTON_DOWN,
  BUTTON_DOWN_HOLD,
  BUTTON_LEFT,
  BUTTON_LEFT_HOLD,
  BUTTON_RIGHT,
  BUTTON_RIGHT_HOLD,
  BUTTON_STARTSTOP,
  BUTTON_STARTSTOP_HOLD,
};

const char* LCD_TITLES[] = {
  "JOG: ",
  "DIR: ",
  "COUNT: ",
  "TPI: ",
  "TEST: ",
  "START: ",
};

lcd_opt selected_option = LCD_COUNT;
int lcd_offset = 1;

Button buttons() {
  static Button last_button = BUTTON_NONE;
  static bool button_processed = false;
  static unsigned long last_button_time = 0;

  Button button = BUTTON_NONE;
  
  if (!button_processed) {
    button = BUTTON_NONE;
    button_processed = true;
  }
  
  Button button_read;
  if (!digitalRead(PIN_UP)) {
    button_read = BUTTON_UP;
  }else if (!digitalRead(PIN_DOWN)) {
    button_read = BUTTON_DOWN;
  } else if (!digitalRead(PIN_LEFT)) {
    button_read = BUTTON_LEFT;
  } else if (!digitalRead(PIN_RIGHT)) {
    button_read = BUTTON_RIGHT;
  } else if (!digitalRead(PIN_GO)) {
    button_read = BUTTON_STARTSTOP;
  } else {
    button_read = BUTTON_NONE;
  }

  if (button_read != last_button) {
    last_button_time = millis();
    last_button = button_read;
    button = button_read;
    button_processed = false;    
  }
  
  if ((last_button != BUTTON_NONE) && ((millis() - last_button_time) > HOLD_THRESHOLD_MS)) {
    button = (Button)(button_read + 0); //TODO
  }

  return button;
}

unsigned int cfg_tpi = 1;

struct BufferedDisplay : public Print {
  char buffer[LCD_HEIGHT][LCD_WIDTH];
  int pos;
  int line;

  BufferedDisplay() {
    memset(buffer, ' ', sizeof(buffer));
    reset();
  }

  void reset() {
    pos = 0;
    line = 0;
  }

  virtual size_t write(uint8_t c) {
    if (c == 0 || c == '\r') {
      return 0;
    }
    if (c == '\n' && line < LCD_HEIGHT){
      while (pos < LCD_WIDTH) {
        this->buffer[line][pos++] = ' ';
      }
      line++;
      pos = 0;
    } else if (pos < LCD_WIDTH) {
      this->buffer[line][pos] = c;
      pos++;
    }
    return 1;
  }
};

volatile int position = 0;
volatile int targetPosition = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR onTimerISR() {
  static bool state = false;
  static bool direction = false;
  state = !state;
  bool step = false;
  if (state) {
    // Rising edge, adjust adjust position and step (if direction is correct)
    taskENTER_CRITICAL_ISR(&mux);
    if (direction && position < targetPosition) {
      step = true;
      position++;
    } else if (!direction && position > targetPosition) {
      step = true;
      position--;
    }
    taskEXIT_CRITICAL_ISR(&mux);
  } else {
    // Falling edge, set direction
    taskENTER_CRITICAL_ISR(&mux);
    if (position != targetPosition) {
      direction = position < targetPosition;
    }
    taskEXIT_CRITICAL_ISR(&mux);
  }
  digitalWrite(PIN_STEP, step);
  digitalWrite(PIN_DIR, direction);
  digitalWrite(PIN_MOTOR_ON, 1);
}


BufferedDisplay buffer;

void lcd_draw() {
  buffer.reset();
  for (int line = 0; line < LCD_HEIGHT; line++){
    int option = line + lcd_offset;
    buffer.print(option == selected_option ? '>' : ' ');
    buffer.print(LCD_TITLES[option]);
    
    switch (option) {
      case LCD_TEST:
        buffer.print("> to FIRE");
        break;
      case LCD_JOG:
        buffer.print(position);
        buffer.print('/');
        buffer.print(targetPosition);
        break;
      case LCD_TPI:
        buffer.print(cfg_tpi);
        break;
    }
    buffer.println();
  }
}

void ui_task(void*) {
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  while(true) {
    vTaskDelayUntil(&pxPreviousWakeTime, 100 / portTICK_RATE_MS);
    Button button = buttons();

    // Adjust selected menu option
    switch (button) {
      case BUTTON_UP:
        if (selected_option > 0) {
          selected_option = (lcd_opt)(selected_option - 1);
          if (lcd_offset > selected_option) {
            lcd_offset--;
          }
        }
      break;
      case BUTTON_DOWN:
        if (selected_option < LCD_NUMENTRIES - 1) {
          selected_option = (lcd_opt)(selected_option + 1);
          if (lcd_offset + LCD_HEIGHT - 1 < selected_option) {
            lcd_offset++;
          }
        }
      break;
    }
  
    switch(selected_option) {
      case LCD_TEST:
        switch (button){
          case BUTTON_RIGHT:
            digitalWrite(PIN_VALVE1, true);
            break;
          case BUTTON_NONE:
            digitalWrite(PIN_VALVE1, false);
            break;
        }
        break;
      case LCD_JOG:
        switch (button) {
          case BUTTON_RIGHT:
            targetPosition+=100;
          break;
          case BUTTON_LEFT:
            targetPosition-=100;
          break;
        }
        break;
      case LCD_TPI:
        switch (button) {
          case BUTTON_RIGHT:
            if (cfg_tpi < 999) cfg_tpi++;
          break;
          case BUTTON_LEFT:
            if (cfg_tpi > 1) cfg_tpi--;
          break;
        }
      break;
    }

    // Update the LCD buffer
    lcd_draw();
  }
}

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
const char* ssid = "Home";
const char* password = "domecile";
void otaSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);
  otaSetup();
  
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_NFAULT, INPUT);
  pinMode(PIN_MOTOR_ON, OUTPUT);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_VALVE1, OUTPUT);
  pinMode(PIN_VALVE2, OUTPUT);

  pinMode(PIN_LIMIT1, INPUT);
  pinMode(PIN_LIMIT2, INPUT);

  pinMode(PIN_UP, INPUT_PULLUP);
  pinMode(PIN_DOWN, INPUT_PULLUP);
  pinMode(PIN_LEFT, INPUT_PULLUP);
  pinMode(PIN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_GO, INPUT_PULLUP);

  digitalWrite(PIN_LED, true);
  hw_timer_t* timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimerISR, true);
  timerAlarmWrite(timer, MOTOR_TICK_PERIOD_US, true);
  timerAlarmEnable(timer);

  lcd.begin();
  lcd.backlight();
  lcd_draw();
  
  xTaskCreate(ui_task, "ui_task", 8192, NULL, 2, NULL);
}

void lcd_xmit(){
  for (int line = 0; line < LCD_HEIGHT; line++) {
    lcd.setCursor(0, line);
    for (int c = 0; c < LCD_WIDTH; c++) {
      lcd.write(buffer.buffer[line][c]);
    }
  }
}

void loop()
{
  lcd_xmit();
  ArduinoOTA.handle();
}
