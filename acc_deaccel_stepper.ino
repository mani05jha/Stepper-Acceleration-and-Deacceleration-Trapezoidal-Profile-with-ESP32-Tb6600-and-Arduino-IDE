//{"ma":"10000","ms":"10000","gtp":"40000"}
#include <math.h>
#include <ArduinoJson.h>
#define PULSE_PIN 21
#define DIR_PIN 22
#define ENABLE_PIN 23

typedef struct {
  uint8_t step_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  int8_t dir;
  int64_t current_position;
  int64_t goto_position;
  uint64_t max_speed;
  uint64_t max_acceleration;
  JsonDocument doc;
} stepper_data;

stepper_data stepper1 = { 21, 22, 23, 1, 0, 0, 0, 0 };

void set_direction(int64_t current_position, int64_t goto_position, int8_t *dir) {
  ((current_position - goto_position) > 0) ? *dir = -1 : *dir = 1 ;
  digitalWrite(DIR_PIN, current_position - goto_position > 0 ? LOW : HIGH);
}

void acceleration(int64_t max_acceleration, int64_t current_position) {
  run_stepper(sqrt(2.0 / max_acceleration) * (sqrt(current_position + 1) - sqrt(current_position)));
}

void deacceleration(int64_t max_acceleration, int64_t current_position, int64_t goto_position) {
  run_stepper(sqrt(2.0 / max_acceleration) * (sqrt(goto_position - current_position) - sqrt(goto_position - current_position - 1)));
}

void cruise(int64_t max_speed) {
  run_stepper(1.0 / max_speed);
}

void run_stepper(double wait_time_in_usec) {
  // digitalWrite(PULSE_PIN, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(PULSE_PIN, LOW);
  // delayMicroseconds((wait_time_in_usec * 1000000) - 10);
  Serial.print((wait_time_in_usec * 1000000) - 10);
}

void stepper_task(stepper_data* stepper) {
  int64_t current_travel_position = 0;
  int64_t total_travel_position = abs(stepper->goto_position - stepper->current_position);
  uint64_t stop_accel_position = (stepper->max_speed * stepper->max_speed) / (2 * stepper->max_acceleration);
  uint64_t start_deaccel_position = total_travel_position - stop_accel_position;

  set_direction(stepper->current_position, stepper->goto_position, &stepper->dir);
  while (current_travel_position < total_travel_position) {
    if (current_travel_position < stop_accel_position) {
      acceleration(stepper->max_acceleration, current_travel_position);
    } else if ((current_travel_position > stop_accel_position) && (current_travel_position < start_deaccel_position)) {
      cruise(stepper->max_speed);
    } else {
      deacceleration(stepper->max_acceleration, current_travel_position, total_travel_position);
    }
    current_travel_position += 1;
    stepper->current_position += stepper->dir;
    Serial.print(" ");
    Serial.print(stepper-> current_position);
    Serial.println();
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(stepper1.dir_pin, OUTPUT);
  pinMode(stepper1.step_pin, OUTPUT);
  pinMode(stepper1.enable_pin, OUTPUT);

  digitalWrite(stepper1.dir_pin, LOW);
  digitalWrite(stepper1.step_pin, LOW);

  stepper1.max_speed = 10000;
  stepper1.max_acceleration = 10000;
  stepper1.goto_position = 50000;

  // stepper_task(&stepper1);
  Serial.printf("Stepper1 Current Position: %lld\n", stepper1.current_position);
}

void loop() {
  while (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data += '\0';
    DeserializationError error = deserializeJson(stepper1.doc, data);
    if (error) {
      Serial.println("Json Error");
    } else {
      stepper1.max_speed = atoi(stepper1.doc[String("ms")]);
      stepper1.max_acceleration = atoi(stepper1.doc[String("ma")]);
      stepper1.goto_position = atoi(stepper1.doc[String("gtp")]);
      Serial.printf("Stepper1 Max Speed: %lld, Stepper1 Max Acceleration: %lld, Stepper1 Goto Position: %lld\n", stepper1.max_speed, stepper1.max_acceleration, stepper1.goto_position);
      stepper_task(&stepper1);
      Serial.printf("Stepper1 Current Position: %lld\n", stepper1.current_position);
    }
  }
}