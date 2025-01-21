// Title: Implementing PID control
#include <util/atomic.h>
#include <LCD_I2C.h>

LCD_I2C lcd(0x27, 16, 2);

int sensorPin = 2;
int motorPin1 = 6;
int motorPin2 = 7;
int motorEnPin = 9;

unsigned long startTime = 0;
unsigned long lastLcdUpdateTimeMillis = 0;

// Variables for calculating speed of revolution
unsigned long prevTime = 0;
unsigned int counter = 0;

// Variables for the Low pass filter filter
float filtered_speed = 0;
float prev_filtered_speed = 0;

// Set target speed (RPM)
float target_speed = 600;

// Set the PID gains
float Kp = 0.72;
float Ki = 1.6;
float Kd = 0.01;

// Variables for calculating the errors
float error = 0;
float prevError = 0;
float integral_error = 0;
float derivative_error = 0;

// Increment the counter on every loop pass to calculate speed
void incrementCount() {
  counter++;
}

// actuate the motor with a pwm value and direction
void actuateMotor(int new_pwm_value, int new_direction) {
  if (new_direction > 0) {
    analogWrite(motorEnPin, new_pwm_value);
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    analogWrite(motorEnPin, new_pwm_value);
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
}

void stopMotor() {
  analogWrite(motorEnPin, 0);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

void setup() {
  Serial.begin(9600);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorEnPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(sensorPin), incrementCount, RISING);

  // Print CSV headers
  Serial.println("Time (milliseconds),Speed (RPM),Filtered Speed (RPM),Actuating Signal U(t)");

  prevTime = micros();
  startTime = micros();

  lcd.begin();
  lcd.backlight();
}

void loop() {
  // Condition to run the loop for only 10 seconds
  if (micros() - startTime <= 15 * 1.0e6) {
    // Read the counter in an atomic block to avoid potential misreads
    int counterCopy = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      counterCopy = counter;
    }

    // Compute the velocity using method 1: calculating from how much time
    // has elapsed between each iteration of the loop
    unsigned long currTime = micros();
    float elapsedTime = ((float)(currTime - prevTime)) / 1.0e6;
    float speed = (((float)counter / 20) / elapsedTime);  // rotations per second
    speed *= 60;                                          // rotations per minute

    // Applying a Low Pass Filter to the velocity
    filtered_speed = 0.854 * filtered_speed + 0.0728 * speed + 0.0728 * prev_filtered_speed;
    prev_filtered_speed = speed;

    // Calculate the errors
    error = target_speed - filtered_speed;
    integral_error = integral_error + error * elapsedTime;
    derivative_error = (error - prevError) / elapsedTime;

    // Compute the actuating signal
    float actuating_signal = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error);

    // Set the motor speed and direction
    int direction = 1;
    if (actuating_signal < 0) {
      direction = -1;
    }
    int pwm_value = (int)fabs(actuating_signal);  // PWM ranges from 0 (0% duty cycle) to 255 (100% duty cycle)
    if (pwm_value > 255) {                        // The interrupt cannot measure maximum speed
      pwm_value = 255;
    }

    // Actuate the motor
    actuateMotor(pwm_value, direction);

    // Resetting the values for the next loop
    counter = 0;
    prevTime = currTime;
    prevError = error;

    unsigned long currTimeMillis = currTime / 1.0e3;

    // update the lcd every 200ms
    if (currTimeMillis - lastLcdUpdateTimeMillis >= 200) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Speed: ");
      lcd.print(filtered_speed);
      lcd.setCursor(0, 1);
      lcd.print("U(t): ");
      lcd.print(actuating_signal);
      lastLcdUpdateTimeMillis = currTimeMillis;
    }

    // Log the values in a CSV format
    Serial.print(currTimeMillis);
    Serial.print(",");
    Serial.print(filtered_speed);
    // Serial.print(",");
    // Serial.print(error);
    // Serial.print(",");
    // Serial.print(integral_error);
    // Serial.print(",");
    // Serial.print(derivative_error);
    Serial.print(",");
    Serial.println(actuating_signal);

    // Add a delay of 1ms to maintain a continuous sampling freq
    delay(1);
  } else {
    stopMotor();
  }
}
