// Include PS3 Controller library
#include <Ps3Controller.h>
#include <Arduino.h>

// Define LED Pins led1 used to be 4
#define LED1_PIN 5
#define LED2_PIN 16
#define LED3_PIN 15
#define LED_BUILTIN 4

// Variables to hold LED states
bool led1State = false;
bool led2State = false;
bool led3State = false;

// Callback Function
void notify()
{

  // Cross button - LED1 momentary control
  if (Ps3.event.button_down.cross)
  {
    Serial.println("Cross pressed");
    led1State = true;
    digitalWrite(LED1_PIN, led1State);
  }
  if (Ps3.event.button_up.cross)
  {
    Serial.println("Cross released");
    led1State = false;
    digitalWrite(LED1_PIN, led1State);
  }

  // Triangle Button - LED2 toggle control
  if (Ps3.event.button_down.triangle)
  {
    Serial.println("Triangle presssed");
    led2State = !led2State;
    digitalWrite(LED2_PIN, led2State);
  }

  // Square Button - LED3 on
  if (Ps3.event.button_down.square)
  {
    Serial.println("Square pressed");
    led3State = true;
    digitalWrite(LED3_PIN, led3State);
  }

  // Circle Button - LED3 off
  if (Ps3.event.button_down.circle)
  {
    Serial.println("Circle pressed");
    led3State = false;
    digitalWrite(LED3_PIN, led3State);
  }
}

// On Connection function
void onConnect()
{
  // Print to Serial Monitor
  Serial.println("Connected.");
}
// const int LED_BUILTIN = 2;
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the onboard LED pin as an output
  Serial.begin(115200);

  // Define Callback Function
  Ps3.attach(notify);
  // Define On Connection Function
  Ps3.attachOnConnect(onConnect);
  // Emulate console as specific MAC address (change as required)
  Ps3.begin("00:00:00:00:00:00");
  // Ps3.begin("44:d8:32:3b:90:8f");

  // Set LED pins as outputs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  // Print to Serial Monitor
  Serial.println("Ready.");
}
int num = 0;
void loop()
{
  if (!Ps3.isConnected()) {
    Serial.println(num + "PS3 Controller is not Connected...\nPlease Connect :)");
    // num++;
  }
  delay(2000);

  digitalWrite(LED_BUILTIN, HIGH); // Turn on the onboard LED
  delay(500);                      // Delay for 500 milliseconds
  digitalWrite(LED_BUILTIN, LOW);  // Turn off the onboard LED
  delay(500);                      // Delay for another 500 milliseconds                // wait for 500 milliseconds
}