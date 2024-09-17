#include <avr/interrupt.h>
#include <arduino.h>
#include <Versatile_RotaryEncoder.h>

#include "lcdgfx.h"
#include "lcdgfx_gui.h"
#include "font.h"

DisplaySSD1306_128x32_I2C display(-1);

#define CLK_PIN 2
#define DT_PIN 3
#define SW_PIN 4

#define LIGHT_PIN 6
#define FAN_PIN 7
#define PWM_PIN 9
#define SENS_PIN 8
#define LED_PIN 13

#define LCD_UPDATE 100
#define RPM_UPDATE 2000

Versatile_RotaryEncoder *versatile_encoder;

int pwmValue = 0;
bool ledState = true;
unsigned long current_rpm = 0;

unsigned long volatile rpm = 0;
unsigned long lastRpmUpdate = 0;
unsigned long lastLcdUpdate = 0;


const int canvasWidth = 128;
const int canvasHeight = 32;
uint8_t canvasData[canvasWidth*(canvasHeight/8)];
NanoCanvas1 canvas(canvasWidth, canvasHeight, canvasData);


ISR(PCINT0_vect) {
	rpm++;
}

char* toChar(unsigned long num) {
  int bufferSize = 11;
  char* buffer = (char*)malloc(bufferSize);
  if (buffer != NULL) {
    sprintf(buffer, "%lu", num);
  }
  return buffer;
}

void handleRotate(int8_t rotation) {
  pwmValue += rotation * 10; // Adjust sensitivity as needed
  pwmValue = constrain(pwmValue, 1, 255);

  OCR1A = map(pwmValue, 1, 255, 0, ICR1);

  Serial.print("ocr Value: ");
  Serial.println(map(pwmValue, 1, 255, 0, ICR1));

}

// Function to handle button press
void handlePress() {
  // Toggle the LED state
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState ? LOW : HIGH);
  digitalWrite(FAN_PIN, ledState ? LOW : HIGH);
  digitalWrite(LIGHT_PIN, ledState ? LOW : HIGH);


  Serial.print("LED State: ");
  Serial.println(ledState ? "ON" : "OFF");
}

void handleLongPress() {
	  Serial.println("Long");
}

void handleDoublePress() {
	  Serial.println("Double");
}


void setup() {

  Serial.begin(115200);
  Serial.print("Begin");

  pinMode(PWM_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(SENS_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);

  versatile_encoder = new Versatile_RotaryEncoder(CLK_PIN, DT_PIN, SW_PIN);

  // 5kHz frequency
//  TCCR1A = (1 << COM1A1) | (1 << WGM11);
//  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
//  ICR1 = 3199; // F_CPU / (Prescaler * Frequency) - 1 = 16000000 / (1 * 5000) - 1 = 3199

  // 25kHz
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << CS10) | (1 << WGM13);
  ICR1 = 320;
  OCR1A = 0;
  OCR1B = 0;

  versatile_encoder->setHandleRotate(handleRotate);
  versatile_encoder->setHandlePressRelease(handlePress);
  versatile_encoder->setHandleDoublePress(handleDoublePress);
  versatile_encoder->setHandleLongPress(handleLongPress);

  // Enable pin change interrupt for Port B
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  sei();

  display.begin();
  display.clear();
  canvas.setMode( CANVAS_MODE_TRANSPARENT );
  canvas.setFreeFont(font);
}

void loop() {
	versatile_encoder->ReadEncoder();
	unsigned long time = millis();

	if (abs(time - lastRpmUpdate) > RPM_UPDATE ) {
		current_rpm = (rpm / (((float) abs(time - lastRpmUpdate))/1000/60)) / 4;
		lastRpmUpdate = time;
		rpm = 0;
		Serial.print("RPM:");
		Serial.println(current_rpm);
	}

	if (abs(time - lastLcdUpdate) > LCD_UPDATE ) {
		lastLcdUpdate = time;

		canvas.clear();

		int barLength = map(pwmValue, 1, 255, 0, 60);
		canvas.fillRect(0, 4, barLength, 28);
		canvas.drawRect(0, 4, 60, 28);

		char* rpmStr = toChar(current_rpm);
		canvas.printFixed(68, 2, rpmStr, STYLE_NORMAL);
		free(rpmStr);

		display.drawCanvas(0, 0, canvas );

	}
}
