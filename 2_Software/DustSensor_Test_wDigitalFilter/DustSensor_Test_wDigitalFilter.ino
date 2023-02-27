 /*
  Standalone Sketch to use with an Arduino UNO and a
  Sharp Optical Dust Sensor GP2Y1010AU0F
*/

#include <U8g2lib.h>  // OLED display library

#define NO_DUST_VOLTAGE 700   // mV
#define MAX_VOLTAGE     5000  // mV

#define SENSOR_VOUT_PIN A0  // Dust sensor to Arduino A0 pin
#define SENSOR_LED_PIN  2   // Dust sensor LED to Arduino D2 pin
#define BUTTON_PIN 3        // Display mode button to Arduino D3 pin

// Sensor reading times (in uS)
#define SAMPLING_TIME 280   // from LED ON to sample
#define DELTA_TIME    40    // from sample to LED OFF
#define SLEEP_TIME    9680  // from LED OFF to LED ON

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2);

// Definition of display modes
typedef enum {
	Mode1_DisplayDust,    // Display dust density [ug/m3]
	Mode2_DisplayVoltage, // Display dust sensor voltage [V]
	Mode3_DisplaySpeed    // Display desired motor speed [%]
} DisplayMode;
// Estado inicial & Variável nextState (próximo estado)
DisplayMode nextMode = Mode1_DisplayDust;

//------------------------------------------------------------
// Function to read n samples from sensor
int filterADC(int adcValue) {
  const uint8_t numSamples = 10;
  static int flag_first = 1, samples[numSamples], sum = 0;
  int i;

  if(flag_first == 1) {
    flag_first = 0;
    for(i=0; i<numSamples; ++i) {
      samples[i] = adcValue;
      sum += samples[i];
    }
    return adcValue;
  }
  else {
    sum -= samples[0];
    for(i=0; i<(numSamples-1); ++i) {
      samples[i] = samples[i+1];
    }
    samples[numSamples-1] = adcValue;
    Serial.println(samples[numSamples-1]);
    sum += samples [numSamples-1];

    adcValue = sum/numSamples;
    return adcValue;
  }
}
//------------------------------------------------------------

int debounceButton(){
  static int buttonState;           // Current reading from the input pin
  static int lastButtonState = LOW; // Previous reading from the input pin
  int inputState = LOW;             // Input state to return

  static unsigned long lastDebounceTime = 0;  // Last time the output pin was toggled
  unsigned long debounceDelay = 10;           // Debounce time [ms]

  // Read the state of the switch
  int reading = digitalRead(BUTTON_PIN);

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  // If not, ignores the noise between the debounceDelay
  if ((millis() - lastDebounceTime) >= debounceDelay) {
    // If the button state has changed, take reading as the actual current state
    if (reading != buttonState) {
      buttonState = reading;
      //Serial.println("Button = HIGH");
      if (buttonState == HIGH) {
        inputState = !inputState; // Only outputs HIGH if buttonState == HIGH
      }
    }
  }
  // Save the reading, next time through the loop it'll be the lastButtonState
  lastButtonState = reading;
  return inputState;
}

//------------------------------------------------------------
// Function to sample dust density from sensor
void getDustDensity() {
  // Sensor data variables
  float sensorValue = 0,
        sensorVoltage = 0,
        dustDensity = 0;
  int motorSpeed = 0;

  // Read ADC value from dust sensor
  digitalWrite(SENSOR_LED_PIN, LOW);
  delayMicroseconds(SAMPLING_TIME);
  sensorValue = analogRead(SENSOR_VOUT_PIN);
  //delayMicroseconds(DELTA_TIME);
  digitalWrite(SENSOR_LED_PIN, HIGH);
  
  sensorValue = filterADC(sensorValue);

  // Convert ADC value to sensor voltage
  // 5V             -- 1024
  // sensorVoltage  -- sensorValue
  sensorVoltage = (MAX_VOLTAGE / 1024.0) * sensorValue;

  // Convert sensor voltage to dust density
  // linear function based on GP2Y1010AU0F datasheet
  if(sensorVoltage >= NO_DUST_VOLTAGE)
  {
    // 500  -- 0   ug/m3
    // 3400 -- 700 mV
    dustDensity = 0.185 * sensorVoltage - 129.5;

    // 100  -- 10  %
    // 3400 -- 700 mV
    motorSpeed = 0.033 * sensorVoltage - 13.1;
    if (motorSpeed > 100)
      motorSpeed = 100;
  }
  else {
    dustDensity = 0;
    motorSpeed = 10;
  }
  
  // Display readings
  Serial.println("\n");
  Serial.print(dustDensity); // unit: ug/m3
  Serial.print(", ");
  Serial.print(sensorVoltage/1000); // unit: V
  Serial.print(", ");
  Serial.print(motorSpeed); // unit: %

  // OLED Display
  u8g2.clearBuffer();                       // clear the internal memory
  u8g2.setFont(u8g2_font_pxplusibmvga9_tr); // choose a suitable font at https://github.com/olikraus/u8g2/wiki/fntlistall
  if(nextMode == Mode1_DisplayDust){
    u8g2.drawStr(0,10,"Dust Density: ");      // write something to the internal memory
    u8g2.setCursor(0, 28);
    u8g2.print(dustDensity);
    u8g2.drawStr(48,28," ug/m3");
  }
  if(nextMode == Mode2_DisplayVoltage){
    u8g2.drawStr(0,10,"Sensor Vout: ");    // write something to the internal memory
    u8g2.setCursor(0, 28);
    u8g2.print(sensorVoltage/1000);
    u8g2.drawStr(32,28," V");
  }
  if(nextMode == Mode3_DisplaySpeed){
    u8g2.drawStr(0,10,"Motor Speed: ");      // write something to the internal memory
    u8g2.setCursor(0, 28);
    u8g2.print(motorSpeed);
    u8g2.drawStr(24,28," %");
  }
  u8g2.sendBuffer();                        // Transfer internal memory to the display
}
//------------------------------------------------------------

//------------------------------------------------------------
// Function for display mode selection
void selectDisplayMode() {
  int input = 0;
  input = debounceButton();
  switch(nextMode){
    case Mode1_DisplayDust:
      if(input == HIGH)
        nextMode = Mode2_DisplayVoltage;
    break;

    case Mode2_DisplayVoltage:
      if(input == HIGH)
        nextMode = Mode3_DisplaySpeed;
    break;

    case Mode3_DisplaySpeed:
      if(input == HIGH)
        nextMode = Mode1_DisplayDust;
    break;
  }
}
//------------------------------------------------------------

//------------------------------------------------------------
// Program main functions
void setup(){
  // Pin mode configuration
  pinMode(SENSOR_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(9600);

  // OLED display initialization
  u8g2.begin();
  u8g2.clearBuffer();                     // Clear the internal memory
  u8g2.setFont(u8g2_font_profont15_tr);   // Choose a suitable font at https://github.com/olikraus/u8g2/wiki/fntlistall
  u8g2.drawStr(0,16,"Dust Sensor Test");  // Write something to the internal memory
  u8g2.sendBuffer();                      // Transfer internal memory to the display
  delay(2000);
}

void loop(){
  selectDisplayMode();
  static unsigned long previousMillis = 0;  // Last recorded timer0 value
  if (millis() - previousMillis >= 1000)	{
    previousMillis = millis();
		getDustDensity();
	}
}
//------------------------------------------------------------