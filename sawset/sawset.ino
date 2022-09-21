// Libraries
#include <LCD_I2C.h>
#include <EEPROM.h>

// PPI: counts teeth in an inch, including both ends
// tooth_spacing = 2 / (ppi - 1)
// 200 ticks = 8mm

const float INCHES_TO_TICKS_UNCAL = 25.4 * 200 / 8;

const int LIMIT_SWITCH_ZERO_OFFSET = 50;
const int LIMIT = 13200;
const int ZEROING_TARGET = -(LIMIT + 1000);

const float GRIND_START_IN = 7;
const float GRIND_LENGTH_IN = 5.5;

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
const int MOTOR_TICK_PERIOD_US = 600;
const int SLEEP_TIMER_MIN = 10;

LCD_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT);

enum lcd_opt {
  LCD_COUNT,
  LCD_TPI,
  LCD_LENGTH,
  LCD_ADVANCED,
  LCD_JOG,
  LCD_TICK_CALIB,
  LCD_LASER_OFFSET,
  LCD_TEST,
  LCD_GRIND,
  LCD_NUMENTRIES,
};

const char* LCD_TITLES[] = {
  "Strikes: ",
  "PPI: ",
  "Length: ",
  "-ADVANCED-",
  "Jog: ",
  "Tick Calib: ",
  "Cam Offset:",
  "Test: ",
  "Grind: Press >",
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

lcd_opt selected_option = LCD_COUNT;
int lcd_offset = 0;

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
    button = (Button)(button_read + 0);
  }

  return button;
}


struct BufferedDisplay : public Print {
  char buffer[LCD_HEIGHT][LCD_WIDTH];
  char buffer2[LCD_HEIGHT][LCD_WIDTH];
  int pos;
  int line;

  BufferedDisplay() {
    flush();
  }

  void flush() {
    memcpy(buffer2, buffer, sizeof(buffer));
    memset(buffer, ' ', sizeof(buffer));
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

struct Cfg {
  static portMUX_TYPE mux;
  unsigned int tpi = 8;
  unsigned int count = 1;
  unsigned int length = 12;
  unsigned int laser_offset = 1850;
  int tick_calib = 0;

  float inches_to_ticks() const {
    float cal = (float)this->tick_calib;
    return (1.0 + cal / 10000.0) * INCHES_TO_TICKS_UNCAL;
  }

  Cfg read() {
    taskENTER_CRITICAL(&this->mux);
    Cfg retval = *this;
    taskEXIT_CRITICAL(&this->mux);
    return retval;
  }

  void write(Cfg cfg) {
    taskENTER_CRITICAL(&this->mux);
    *this = cfg;
    taskEXIT_CRITICAL(&this->mux);
  }

  void save(){
    EEPROM.put(0, this->read());
    EEPROM.commit();
  }
  
  void restore() {
    Cfg cfg;
    EEPROM.get(0, cfg);
    this->write(cfg);
  }
};
portMUX_TYPE Cfg::mux = portMUX_INITIALIZER_UNLOCKED;

Cfg _cfg{};

volatile int _position = 0;
volatile int _targetPosition = ZEROING_TARGET;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t _atPosition = xSemaphoreCreateBinary();

void IRAM_ATTR onTimerISR() {
  static bool state = false;
  static int direction = 0;
  static int timeout = 0;
  static bool zero_completed = false;

  if (timeout > 0) {
    timeout--;
    return;
  }

  bool limit_pressed = digitalRead(PIN_LIMIT2);
  
  taskENTER_CRITICAL_ISR(&mux);
  if (limit_pressed && !zero_completed) {
    _position = -LIMIT_SWITCH_ZERO_OFFSET;
    _targetPosition = 0;
    zero_completed = true;
  }
  state = !state;
  bool step = false;  
  if (state) {
    // Rising edge, adjust adjust position and step (if direction is correct)
    if (direction == 1 && _position < _targetPosition) {
      step = true;
      _position++;
    } else if (direction == -1 && _position > _targetPosition) {
      step = true;
      _position--;
    }
  } else {
    // Falling edge, set direction
    int lastDirection = direction;
    if (_position < _targetPosition) {
      direction = 1;
    } else if (_position > _targetPosition) {
      direction = -1;
    } else {
      direction = 0;
    }
    if (direction != lastDirection) {
      timeout = 10;
    }
  }
  if (_position == _targetPosition) {
    xSemaphoreGiveFromISR(_atPosition, NULL);
  }
  taskEXIT_CRITICAL_ISR(&mux);
  digitalWrite(PIN_STEP, step);
  digitalWrite(PIN_DIR, direction == 1);
}


BufferedDisplay buffer;

void lcd_draw() {
  int position = get_position();
  int targetPosition = get_target_position();
  Cfg cfg = _cfg.read();
  
  for (int line = 0; line < LCD_HEIGHT; line++){
    int option = line + lcd_offset;
    buffer.print(option == selected_option ? '>' : ' ');
    buffer.print(LCD_TITLES[option]);
    
    switch (option) {
      case LCD_TEST:
        buffer.print("<STR FNC> ");
        buffer.print(digitalRead(PIN_LIMIT1));
        buffer.print(digitalRead(PIN_LIMIT2));
        break;
      case LCD_JOG:
        buffer.print(position);
        buffer.print('/');
        buffer.print(targetPosition);
        break;
      case LCD_TPI:
        buffer.print((float)cfg.tpi / 2);
        break;
      case LCD_COUNT:
        buffer.print(cfg.count);
        break;
      case LCD_LENGTH:
        buffer.print(cfg.length);
        buffer.print("\"");
        break;
      case LCD_LASER_OFFSET:
        buffer.print(cfg.laser_offset / 1000.0, 3);
        buffer.print("\"");
        break;
      case LCD_TICK_CALIB:
        buffer.print(cfg.tick_calib / 100.0, 2);
        buffer.print("%");
        break;
    }
    buffer.println();
  }
  buffer.flush();
}

enum MODE {
  MENU,
  RUNNING,
  ZEROING,
  GRINDING,
  ERROR,
};

MODE mode = ZEROING;

void move_nonblocking(int position) {
  // ensure semaphore is taken
  xSemaphoreTake(_atPosition, 0);

  portENTER_CRITICAL(&mux);
  xSemaphoreTake(_atPosition, 0);
  _targetPosition = position;
  portEXIT_CRITICAL(&mux);
}

void move_blocking(int position) {
  move_nonblocking(position);
  xSemaphoreTake(_atPosition, portMAX_DELAY);
}

int get_position() {
  portENTER_CRITICAL(&mux);
  int retval = _position;
  portEXIT_CRITICAL(&mux);
  return retval;
}

int get_target_position() {
  portENTER_CRITICAL(&mux);
  int retval = _targetPosition;
  portEXIT_CRITICAL(&mux);
  return retval;
}

int starting_point(const struct Cfg& cfg) {
  return (LIMIT / 2) - (cfg.inches_to_ticks() * cfg.length / 2);
}

int offset_starting_point(const struct Cfg& cfg) {
  return starting_point(cfg) + cfg.laser_offset * cfg.inches_to_ticks() / 1000.0;
}

bool _cancel;
void run_task(void*) {
  _cancel = false;
  Cfg cfg = _cfg.read();
  float spacing = (2.0 / ((cfg.tpi / 2.0) - 1.0)) * cfg.inches_to_ticks();
  int count = cfg.count;

  int start = starting_point(cfg);
  int available_ticks = LIMIT - start;
  if (count * spacing > available_ticks) {
    int max_count = available_ticks / spacing;
    buffer.println("Saw does not fit.");
    buffer.print("Max count is ");
    buffer.print(max_count);
    buffer.println(".");
    buffer.println();
    buffer.println("Press STOP to return");
    buffer.flush();
    while(!_cancel) {
      vTaskDelay(100 / portTICK_RATE_MS);
    }
    mode = MENU;
    vTaskDelete(NULL);
    return;
  }

  // Retract
  digitalWrite(PIN_VALVE2, true);
  vTaskDelay(200 / portTICK_RATE_MS);

  for (int i = 0; i < count && !_cancel; i++) {
    buffer.println("Setting...");
    buffer.print(i + 1);
    buffer.print(" / ");
    buffer.print(count);
    buffer.print(" (");
    buffer.print((i + 1)*100 / count);
    buffer.println("%)");
    buffer.flush();
    
    move_blocking(start + i * spacing);
    
    // Fire
    digitalWrite(PIN_VALVE1, true);
    vTaskDelay(100 / portTICK_RATE_MS);
    digitalWrite(PIN_VALVE1, false);
    vTaskDelay(75 / portTICK_RATE_MS);
  }

  move_blocking(offset_starting_point(cfg));

  // Extend
  digitalWrite(PIN_VALVE2, false);
  mode = MENU;
  vTaskDelete(NULL);
}

void grind_task(void*) {
  _cancel = false;
  Cfg cfg = _cfg.read();

  // Retract
  digitalWrite(PIN_VALVE2, true);
  vTaskDelay(200 / portTICK_RATE_MS);
  int pos1 = cfg.inches_to_ticks() * GRIND_START_IN;
  int pos2 = cfg.inches_to_ticks() * GRIND_LENGTH_IN;

  move_blocking(pos1);
  
  buffer.println("Press > to start");
  buffer.flush();
  while(digitalRead(PIN_RIGHT)) {
    vTaskDelay(100 / portTICK_RATE_MS);  
  }

  // Fire
  digitalWrite(PIN_VALVE1, true);
  vTaskDelay(100 / portTICK_RATE_MS);
  buffer.println("Press GO to stop");
  buffer.flush();
      
  while (!_cancel) {
    move_blocking(pos2);
    move_blocking(pos1);
  }
  // Unfire
  digitalWrite(PIN_VALVE1, false);
  mode = MENU;
  vTaskDelete(NULL);
}


void ui_task(void*) {
  int jog_in = 0;
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  TickType_t lastActivityTime = xTaskGetTickCount();
  
  Cfg cfg = _cfg.read();
  while(true) {
    vTaskDelayUntil(&pxPreviousWakeTime, 50 / portTICK_RATE_MS);
    Button button = buttons();
    int position = get_position();
    int targetPosition = get_target_position();

    if (button != BUTTON_NONE) {
      lastActivityTime = xTaskGetTickCount();
    }

    if (xTaskGetTickCount() - lastActivityTime > (SLEEP_TIMER_MIN*60*1000 / portTICK_RATE_MS)) {
      buffer.println("Machine sleeping.");
      buffer.println("Press UP to restart");
      buffer.println("(machine will zero)");
      buffer.flush();
      digitalWrite(PIN_MOTOR_ON, 0);
      mode = ERROR;
    }

    switch (mode) {
      case ERROR:
        if (button == BUTTON_UP) {
          ESP.restart();
        }
        break;
      
      case ZEROING:
        if (position == ZEROING_TARGET) {
          mode = ERROR;
          buffer.println("Zeroing failed.");
          buffer.println("Check limit switch,");
          buffer.println("then UP to retry.");
        } else {
          buffer.println(" Bad Axe Tool Works");
          buffer.println("Automatic Saw Setter");
          buffer.println();
          buffer.println("     zeroing...");
        }

        buffer.flush();
        if (position == 0 && targetPosition == 0) {
          move_nonblocking(offset_starting_point(cfg));
          mode = MENU;
        }
      break;
      case MENU:
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
              case BUTTON_LEFT:
                digitalWrite(PIN_VALVE1, true);
                vTaskDelay(100 / portTICK_RATE_MS);
                break;
              case BUTTON_RIGHT:
                digitalWrite(PIN_VALVE2, true);                
                vTaskDelay(100 / portTICK_RATE_MS);
                break;
              case BUTTON_NONE:
                digitalWrite(PIN_VALVE1, false);
                digitalWrite(PIN_VALVE2, false);
                move_nonblocking(starting_point(cfg));
                break;
            }
            break;
          case LCD_JOG:
            switch (button) {
              case BUTTON_LEFT:
                jog_in+=1;
                if (jog_in > 20) {
                  jog_in = 20 ;
                }
                move_nonblocking(jog_in * cfg.inches_to_ticks());
              break;
              case BUTTON_RIGHT:
                jog_in-=1;
                if (jog_in < 0) {
                  jog_in = 0;
                }
                move_nonblocking(jog_in * cfg.inches_to_ticks());
              break;
            }
            break;
          case LCD_TPI:
            switch (button) {
              case BUTTON_RIGHT:
                if (cfg.tpi < 64) cfg.tpi++;
              break;
              case BUTTON_LEFT:
                if (cfg.tpi > 8) cfg.tpi--;
              break;
            }
          break;
          case LCD_COUNT:
            switch (button) {
              case BUTTON_RIGHT:
                if (cfg.count < 640) cfg.count++;
              break;
              case BUTTON_LEFT:
                if (cfg.count > 1) cfg.count--;
              break;
            }
          break;
          case LCD_LENGTH:
            switch (button) {
              case BUTTON_RIGHT:
                if (cfg.length < 20) cfg.length += 2;
                move_nonblocking(offset_starting_point(cfg));
              break;
              case BUTTON_LEFT:
                if (cfg.length > 10) cfg.length -= 2;
                move_nonblocking(offset_starting_point(cfg));
              break;
            }
          break;
          case LCD_LASER_OFFSET:
            switch (button) {
              case BUTTON_RIGHT:
                if (cfg.laser_offset < 2250) cfg.laser_offset += 2;
              break;
              case BUTTON_LEFT:
                if (cfg.laser_offset > 1500) cfg.laser_offset -= 2;
              break;
            }
            move_nonblocking(offset_starting_point(cfg));
          break;
          case LCD_TICK_CALIB:
            switch (button) {
              case BUTTON_RIGHT:
                if (cfg.tick_calib < 100) cfg.tick_calib += 1;
                move_nonblocking(jog_in * cfg.inches_to_ticks());
              break;
              case BUTTON_LEFT:
                if (cfg.tick_calib > -100) cfg.tick_calib -= 1;
                move_nonblocking(jog_in * cfg.inches_to_ticks());
              break;
            }
          case LCD_GRIND:
            switch (button) {
              case BUTTON_RIGHT:
              mode = GRINDING;
              xTaskCreate(grind_task, "grind_task", 8192, NULL, 3, NULL);
              break;
            }
          break;
        }

        if (button == BUTTON_STARTSTOP) {
          _cfg.save();
          if (selected_option < LCD_ADVANCED) {
            mode = RUNNING;
            xTaskCreate(run_task, "run_task", 8192, NULL, 3, NULL);
          }
        }
        lcd_draw();
      break;
      case RUNNING:
      case GRINDING:
        if (button == BUTTON_STARTSTOP) {
          _cancel = true;
        }
      break;
    }
    _cfg.write(cfg);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Bad Axe Saw Setter");
  
  EEPROM.begin(sizeof(Cfg));
  
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
  digitalWrite(PIN_MOTOR_ON, 1);

  // Hold UP to skip reading config on boot.
  if (digitalRead(PIN_UP)) {
    _cfg.restore();
  }
  
  hw_timer_t* timer = timerBegin(0, 80, true); // 80MHz
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
      lcd.write(buffer.buffer2[line][c]);
    }
  }
}

void loop()
{
  lcd_xmit();
}
