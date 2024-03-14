#include <Bluepad32.h>

// Define the pin for the LED
int escPinValue = 0;
const int ESC_OUTPUT_PIN = 19; // D2 pin 
const int ESC_PWM_CHANNEL = 0;    // ESP32 has 16 channels which can generate 16 independent waveforms
const int ESC_PWM_FREQ = 500;     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int ESC_PWM_RESOLUTION = 12; // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits 
const int ESC_MAX_DUTY_CYCLE = (int)(pow(2, ESC_PWM_RESOLUTION) - 1); // The max duty cycle value based on PWM resolution (will be 255 if resolution is 8 bits)

const double ESC_MIN_DC_PERC_70 = 0.11845;
const double ESC_MAX_DC_PERC_70 = 0.17955;
const double ESC_MIN_DC_PERC_100 = 0.10495;
const double ESC_MAX_DC_PERC_100 = 0.19933;
double ESC_MIN_DC_PERC = ESC_MIN_DC_PERC_70;
double ESC_MAX_DC_PERC = ESC_MAX_DC_PERC_70;
 
const int STR_OUTPUT_PIN = 5; // D2 pin 
const int STR_PWM_CHANNEL = 1;    // ESP32 has 16 channels which can generate 16 independent waveforms
const int STR_PWM_FREQ = 100;     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int STR_PWM_RESOLUTION = 12; // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits 
const int STR_MAX_DUTY_CYCLE = (int)(pow(2, STR_PWM_RESOLUTION) - 1); // The max duty cycle value based on PWM resolution (will be 255 if resolution is 8 bits)

const double STR_MIN_DC_PERC = 0.10250;
const double STR_MIN_DC_PERC_CTR_TRIM = 0.12840;
const double STR_MAX_DC_PERC_TRIM = 0.17320;
const double STR_MIN_DC_PERC_TRIM = 0.11890;
const double STR_MAX_DC_PERC_CTR_TRM = 0.16802;
const double STR_MAX_DC_PERC = 0.19460;

double str_center_perc = 0.15000;
int str_center = str_center_perc * STR_MAX_DUTY_CYCLE;

double str_min_perc = 0.1;
int str_min = str_min_perc * STR_MAX_DUTY_CYCLE;
double str_max_perc = 0.2;
int str_max = str_max_perc * STR_MAX_DUTY_CYCLE;

int throttle_70 = 1;

// Read analog values from the joystick's X and Y axes
int yAxisValue;
int rxAxisValue;

// Map the analog values to the appropriate range (-512 to 512 in this example)
int mappedYValue;
// Map the analog values to the appropriate range (-512 to 512 in this example)
int mappedrXValue;
int trimValue;

int buttonX = 0, buttonXPressed = 0;
int buttonY = 0, buttonYPressed = 0;
int buttonA = 0, buttonAPressed = 0;
int buttonL2 = 0;
int dpadState = 0;

int upPressed;
int downPressed;
int leftPressed;
int rightPressed;

int steeringLocked = 0;
int driveLocked = 0;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    // Serial.printf(
    //     "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    //     "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    //     ctl->index(),        // Controller Index
    //     ctl->dpad(),         // D-pad
    //     ctl->buttons(),      // bitmask of pressed buttons
    //     ctl->axisX(),        // (-511 - 512) left X Axis
    //     ctl->axisY(),        // (-511 - 512) left Y axis
    //     ctl->axisRX(),       // (-511 - 512) right X axis
    //     ctl->axisRY(),       // (-511 - 512) right Y axis
    //     ctl->brake(),        // (0 - 1023): brake button
    //     ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    //     ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    //     ctl->gyroX(),        // Gyro X
    //     ctl->gyroY(),        // Gyro Y
    //     ctl->gyroZ(),        // Gyro Z
    //     ctl->accelX(),       // Accelerometer X
    //     ctl->accelY(),       // Accelerometer Y
    //     ctl->accelZ()        // Accelerometer Z
    // );
}

void dumpMouse(ControllerPtr ctl) {
    Serial.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(),        // Controller Index
                   ctl->buttons(),      // bitmask of pressed buttons
                   ctl->scrollWheel(),  // Scroll Wheel
                   ctl->deltaX(),       // (-511 - 512) left X Axis
                   ctl->deltaY()        // (-511 - 512) left Y axis
    );
}

void dumpKeyboard(ControllerPtr ctl) {
    // TODO: Print pressed keys
    Serial.printf("idx=%d\n", ctl->index());
}

void dumpBalanceBoard(ControllerPtr ctl) {
    Serial.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    
    // Read analog values from the joystick's X and Y axes
    rxAxisValue = ctl->axisRX();
    yAxisValue = ctl->axisY();
    buttonX = ctl->y();
    buttonY = ctl->x();
    buttonA = ctl->b();
    buttonL2 = ctl->l2();
    dpadState = ctl->dpad();

    upPressed = dpadState & DPAD_UP;
    downPressed = dpadState & DPAD_DOWN;
    leftPressed = dpadState & DPAD_LEFT;
    rightPressed = dpadState & DPAD_RIGHT;

    // DRIVE

    if (!driveLocked) {
      if (throttle_70) {
        ESC_MIN_DC_PERC = ESC_MIN_DC_PERC_70;
        ESC_MAX_DC_PERC = ESC_MAX_DC_PERC_70;
      } else {
        ESC_MIN_DC_PERC = ESC_MIN_DC_PERC_100;
        ESC_MAX_DC_PERC = ESC_MAX_DC_PERC_100;
      }
  
      // Map the analog values to the appropriate range (-512 to 512 in this example)
      mappedYValue = map(yAxisValue, -512, 512, ESC_MIN_DC_PERC * ESC_MAX_DUTY_CYCLE, ESC_MAX_DC_PERC * ESC_MAX_DUTY_CYCLE);
      mappedYValue = (-1) * mappedYValue + ESC_MIN_DC_PERC * ESC_MAX_DUTY_CYCLE + ESC_MAX_DC_PERC * ESC_MAX_DUTY_CYCLE;

      // change mode from 70 to 100%
      if (buttonA && !buttonAPressed) {
          throttle_70 = !throttle_70;
          buttonAPressed = 1;
      }
      if (!buttonA) {
        buttonAPressed = 0;
      }
    }
    
    if (buttonL2) {
      mappedYValue = 0.15 * ESC_MAX_DUTY_CYCLE;
    }

    // lock driving
    if (buttonY && !buttonYPressed) {
        driveLocked = !driveLocked;
        buttonYPressed = 1;
    }
    if (!buttonY) {
      buttonYPressed = 0;
    }


    // STEERING & TRIM
    if (leftPressed) {
        str_center_perc -= 0.005;
    }
    if (rightPressed) {
        str_center_perc += 0.005;
    }
    str_center = str_center_perc * STR_MAX_DUTY_CYCLE;
    
    if (str_center_perc < STR_MIN_DC_PERC_CTR_TRIM) {
      str_center_perc = STR_MIN_DC_PERC_CTR_TRIM;
    }

    if (str_center_perc > STR_MAX_DC_PERC_CTR_TRM) {
      str_center_perc = STR_MAX_DC_PERC_CTR_TRM;
    }
 
    str_min_perc = (STR_MIN_DC_PERC_TRIM - STR_MIN_DC_PERC) / (STR_MAX_DC_PERC_CTR_TRM - STR_MIN_DC_PERC_CTR_TRIM) * (str_center_perc - STR_MIN_DC_PERC_CTR_TRIM) + STR_MIN_DC_PERC;
    str_min = str_min_perc * STR_MAX_DUTY_CYCLE;
    
    str_max_perc = (STR_MAX_DC_PERC - STR_MAX_DC_PERC_TRIM) / (STR_MAX_DC_PERC_CTR_TRM - STR_MIN_DC_PERC_CTR_TRIM) * (str_center_perc - STR_MIN_DC_PERC_CTR_TRIM) + STR_MAX_DC_PERC_TRIM;
    str_max = str_max_perc * STR_MAX_DUTY_CYCLE;

  
    // Map the analog values to the appropriate range (-512 to 512 in this example)
    mappedrXValue = map(rxAxisValue, -512, 512, str_min, str_max);

    Serial.printf("Mapped Steer rX:: %d, Mapped Drive Y: %d, throttle_70: %d, Drive lock: %d, Steer lock: %d, Str cp %.5f, Str c %d, Str minp %.5f, Str min %d, Str max p %.5f, Str max %d\n",
                 mappedrXValue, 
                 mappedYValue, 
                 throttle_70, 
                 driveLocked,
                 steeringLocked, 
                 str_center_perc,
                 str_center, 
                 str_min_perc,
                 str_min, 
                 str_max_perc,
                 str_max);

    ledcWrite(STR_PWM_CHANNEL, mappedrXValue);
    ledcWrite(ESC_PWM_CHANNEL, mappedYValue);


    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    // if (ctl->b()) {
    //     // Turn on the 4 LED. Each bit represents one LED.
    //     static int led = 0;
    //     led++;
    //     // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
    //     // support changing the "Player LEDs": those 4 LEDs that usually indicate
    //     // the "gamepad seat".
    //     // It is possible to change them by calling:
    //     ctl->setPlayerLEDs(led & 0x0f);
    // }



    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);
}

void processMouse(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->scrollWheel() > 0) {
        // Do Something
    } else if (ctl->scrollWheel() < 0) {
        // Do something else
    }

    // See "dumpMouse" for possible things to query.
    dumpMouse(ctl);
}

void processKeyboard(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->isKeyPressed(Keyboard_A)) {
        // Do Something
        Serial.println("Key 'A' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftShift)) {
        // Do something else
        Serial.println("Key 'LEFT SHIFT' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
        // Do something else
        Serial.println("Key 'Left Arrow' pressed");
    }

    // See "dumpKeyboard" for possible things to query.
    dumpKeyboard(ctl);
}

void processBalanceBoard(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->topLeft() > 10000) {
        // Do Something
    }

    // See "dumpBalanceBoard" for possible things to query.
    dumpBalanceBoard(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else if (myController->isMouse()) {
                processMouse(myController);
            } else if (myController->isKeyboard()) {
                processKeyboard(myController);
            } else if (myController->isBalanceBoard()) {
                processBalanceBoard(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

 // Sets up a channel (0-15), a PWM duty cycle frequency, and a PWM resolution (1 - 16 bits) 
  // ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
  ledcSetup(ESC_PWM_CHANNEL, ESC_PWM_FREQ, ESC_PWM_RESOLUTION);

  // ledcAttachPin(uint8_t pin, uint8_t channel);
  ledcAttachPin(ESC_OUTPUT_PIN, ESC_PWM_CHANNEL);

   // Sets up a channel (0-15), a PWM duty cycle frequency, and a PWM resolution (1 - 16 bits) 
  // ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
  ledcSetup(STR_PWM_CHANNEL, STR_PWM_FREQ, STR_PWM_RESOLUTION);

  // ledcAttachPin(uint8_t pin, uint8_t channel);
  ledcAttachPin(STR_OUTPUT_PIN, STR_PWM_CHANNEL);


}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
}
