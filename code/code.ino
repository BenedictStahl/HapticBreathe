#include "HX711.h"
#include <math.h>

// -------------------------------------------------------------
// Pins
// -------------------------------------------------------------
const int LOADCELL_DOUT_PIN = 5;
const int LOADCELL_SCK_PIN = 7;
const int MOTOR_PIN = 9;

const int BUTTON_PIN = 2;
const int LED_PIN = 3;

// Minimum PWM to help the motor overcome stiction when enabled
const int MIN_MOTOR_PWM = 15;

// -------------------------------------------------------------
// Motor signal for calibration mode
// -------------------------------------------------------------
const int CALIBRATION_SIGNAL_PWM = 100;
const int CALIBRATION_SIGNAL_ON_TIME = 120;
const int CALIBRATION_SIGNAL_OFF_TIME = 100;

// -------------------------------------------------------------
// Loop timing
// -------------------------------------------------------------
const int LOOP_DELAY = 50;
const int CALIBRATION_LOOP_DELAY = 20;

// How long to keep the last PWM value when load cell is not ready
const unsigned long LOAD_CELL_TIMEOUT = 1000;

// -------------------------------------------------------------
// Button / LED settings
// -------------------------------------------------------------
const unsigned long DEBOUNCE_DELAY = 40;
const unsigned long LONG_PRESS_DURATION = 3000;

const int BLINK_ON_TIME = 250;
const int BLINK_OFF_TIME = 250;

int last_reading = HIGH;
int button_state = HIGH;
unsigned long last_debounce_time = 0;

unsigned long button_press_start_time = 0;
bool long_press_handled = false;

// -------------------------------------------------------------
// HX711 / calibration
// -------------------------------------------------------------
HX711 load_cell;

long zero_offset = 102000;
float scale_factor = 1000.0;

// -------------------------------------------------------------
// Auto-tare settings
// -------------------------------------------------------------
const int TARE_SAMPLES = 20;
const float TARE_BASELINE_OFFSET = 10.0;

// -------------------------------------------------------------
// Dynamic calibration of min / max readings
// -------------------------------------------------------------
float calibrated_min = -150.0;
float calibrated_max = 150.0;

float previous_calibrated_min = -150.0;
float previous_calibrated_max = 150.0;

bool has_valid_calibration = false;

bool is_calibrating = false;
unsigned long calibration_start_time = 0;
const unsigned long CALIBRATION_DURATION = 10000;

float calibration_min_signal = 0.0;
float calibration_max_signal = 0.0;
float calibration_signal_filtered = 0.0;
bool calibration_has_reading = false;

const float MIN_CALIBRATION_SPAN = 5.0;

unsigned long last_calibration_led_toggle = 0;
bool calibration_led_state = false;
const unsigned long CALIBRATION_LED_INTERVAL = 120;

// -------------------------------------------------------------
// Last valid values for plotting if HX711 is temporarily not ready
// -------------------------------------------------------------
float last_valid_signal = 0.0;
bool have_valid_signal = false;

float last_valid_calibration_signal = 0.0;
bool have_valid_calibration_signal = false;

int last_pwm = 0;
unsigned long last_valid_reading_time = 0;

// -------------------------------------------------------------
// Feedback modes
// -------------------------------------------------------------
enum FeedbackMode {
  MODE_ABSOLUTE = 0,
  MODE_RATE = 1,
  MODE_NO_BREATH_ALERT = 2
};

FeedbackMode current_mode = MODE_ABSOLUTE;

// -------------------------------------------------------------
// Signal filtering
// -------------------------------------------------------------
unsigned long last_time = 0;
float last_signal_filtered = 0.0;
float last_normalized_signal = 0.0;
float rate_filtered = 0.0;

float signal_smoothing = 0.8;
float rate_smoothing = 0.8;

bool filter_initialized = false;

// -------------------------------------------------------------
// Absolute mode tuning
// -------------------------------------------------------------
const int ABSOLUTE_MODE_MAX_PWM = 90;

// Percentage of total calibrated span from the lower end
const float ABSOLUTE_MODE_DEADZONE_PERCENT = 10.0;

// > 1.0 = softer ramp at the start
const float ABSOLUTE_MODE_CURVE_EXPONENT = 1.1;

// -------------------------------------------------------------
// Rate mode tuning
// -------------------------------------------------------------
const int RATE_MODE_MAX_PWM = 60;

float min_rate = 0.1;
float max_rate = 1.2;

// -------------------------------------------------------------
// No breath alert mode
// -------------------------------------------------------------
unsigned long last_breath_detected_time = 0;
unsigned long no_breath_timeout = 5000;

float breath_detect_threshold = 0.035;

unsigned long pulse_period = 2000;
unsigned long pulse_on_time = 400;
unsigned long pulse_ramp_time = 8000;

int alert_min_pwm = 20;
int alert_max_pwm = 80;

// -------------------------------------------------------------
// Function declarations — helpers (math / hardware utilities)
// -------------------------------------------------------------
float clamp_float(float x, float lo, float hi);
int map_float_to_pwm(float x, float in_min, float in_max);
int apply_motor_floor(int pwm);
float get_load_cell_signal();
float get_normalized_signal(float signal_value);
void print_plot_values(float signal_value, int pwm);

// -------------------------------------------------------------
// Function declarations — core logic
// -------------------------------------------------------------
void signal_calibration_start_or_end();
void blink_n_times(int n);
void indicate_current_mode();
void reset_signal_state();
void start_calibration();
void finish_calibration();
void update_calibration_led(unsigned long now);
void next_mode();

void setup() {
  Serial.begin(57600);

  load_cell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  long tare_sum = 0;

  for (int i = 0; i < TARE_SAMPLES; i++) {
    while (!load_cell.is_ready()) {
      delay(10);
    }
    tare_sum += load_cell.read();
  }

  zero_offset = tare_sum / TARE_SAMPLES;

  // Subtract small buffer so the resting value sits slightly above zero
  zero_offset -= (long)(TARE_BASELINE_OFFSET * scale_factor);

  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  reset_signal_state();
  indicate_current_mode();
}

void loop() {
  unsigned long now = millis();

  // -----------------------------------------------------------
  // Button handling
  // -----------------------------------------------------------
  int reading = digitalRead(BUTTON_PIN);

  if (reading != last_reading) {
    last_debounce_time = now;
  }

  if ((now - last_debounce_time) > DEBOUNCE_DELAY) {
    if (reading != button_state) {
      button_state = reading;

      if (button_state == LOW) {
        button_press_start_time = now;
        long_press_handled = false;
      } else {
        if (!is_calibrating && !long_press_handled) {
          next_mode();
        }
        long_press_handled = false;
      }
    }

    if (button_state == LOW && !long_press_handled && !is_calibrating) {
      if ((now - button_press_start_time) >= LONG_PRESS_DURATION) {
        long_press_handled = true;
        start_calibration();
      }
    }
  }

  last_reading = reading;

  // -----------------------------------------------------------
  // Calibration mode
  // -----------------------------------------------------------
  if (is_calibrating) {
    unsigned long calibration_now = millis();

    analogWrite(MOTOR_PIN, 0);
    update_calibration_led(calibration_now);

    if (load_cell.is_ready()) {
      float raw_signal = get_load_cell_signal();

      if (!calibration_has_reading) {
        calibration_signal_filtered = raw_signal;
        calibration_min_signal = raw_signal;
        calibration_max_signal = raw_signal;
        calibration_has_reading = true;
      } else {
        calibration_signal_filtered = signal_smoothing * raw_signal
                                    + (1.0 - signal_smoothing) * calibration_signal_filtered;

        if (calibration_signal_filtered < calibration_min_signal) {
          calibration_min_signal = calibration_signal_filtered;
        }

        if (calibration_signal_filtered > calibration_max_signal) {
          calibration_max_signal = calibration_signal_filtered;
        }
      }

      last_valid_calibration_signal = calibration_signal_filtered;
      have_valid_calibration_signal = true;

      print_plot_values(calibration_signal_filtered, 0);
    } else {
      float plot_signal;

      if (have_valid_calibration_signal) {
        plot_signal = last_valid_calibration_signal;
      } else {
        plot_signal = 0.0;
      }

      print_plot_values(plot_signal, 0);
    }

    if ((calibration_now - calibration_start_time) >= CALIBRATION_DURATION) {
      finish_calibration();
    }

    delay(CALIBRATION_LOOP_DELAY);
    return;
  }

  // -----------------------------------------------------------
  // Normal operation: wait for load cell
  // -----------------------------------------------------------
  if (!load_cell.is_ready()) {
    float plot_signal;

    if (have_valid_signal) {
      plot_signal = last_valid_signal;
    } else {
      plot_signal = 0.0;
    }

    if ((now - last_valid_reading_time) >= LOAD_CELL_TIMEOUT) {
      last_pwm = 0;
      analogWrite(MOTOR_PIN, 0);
    }

    print_plot_values(plot_signal, last_pwm);

    delay(LOOP_DELAY);
    return;
  }

  // -----------------------------------------------------------
  // Read and filter signal
  // -----------------------------------------------------------
  last_valid_reading_time = now;

  float raw_signal = get_load_cell_signal();
  last_valid_signal = raw_signal;
  have_valid_signal = true;

  if (!filter_initialized) {
    last_signal_filtered = raw_signal;
    last_normalized_signal = get_normalized_signal(raw_signal);
    last_time = now;
    rate_filtered = 0.0;
    filter_initialized = true;
    last_breath_detected_time = now;
  }

  float dt = (now - last_time) / 1000.0;
  if (dt <= 0.0) {
    dt = 0.001;
  }

  float signal_filtered = signal_smoothing * raw_signal
                        + (1.0 - signal_smoothing) * last_signal_filtered;

  float normalized_signal = get_normalized_signal(signal_filtered);

  float raw_rate = (normalized_signal - last_normalized_signal) / dt;

  rate_filtered = rate_smoothing * raw_rate
                + (1.0 - rate_smoothing) * rate_filtered;

  float activity_abs = fabs(rate_filtered);

  if (activity_abs > breath_detect_threshold) {
    last_breath_detected_time = now;
  }

  last_signal_filtered = signal_filtered;
  last_normalized_signal = normalized_signal;
  last_time = now;

  // -----------------------------------------------------------
  // Compute motor PWM based on current mode
  // -----------------------------------------------------------
  int pwm = 0;

  if (has_valid_calibration) {
    if (current_mode == MODE_ABSOLUTE) {
      float calibration_span = calibrated_max - calibrated_min;

      if (calibration_span < MIN_CALIBRATION_SPAN) {
        calibration_span = MIN_CALIBRATION_SPAN;
      }

      float signal_from_min = signal_filtered - calibrated_min;
      float deadzone = (ABSOLUTE_MODE_DEADZONE_PERCENT / 100.0) * calibration_span;

      if (signal_from_min <= deadzone) {
        pwm = 0;
      } else {
        float progress = (signal_from_min - deadzone) / (calibration_span - deadzone);
        progress = clamp_float(progress, 0.0, 1.0);

        float curved_progress = pow(progress, ABSOLUTE_MODE_CURVE_EXPONENT);

        pwm = (int)(curved_progress * ABSOLUTE_MODE_MAX_PWM);

        if (pwm > 0) {
          pwm = apply_motor_floor(pwm);
        }
      }
    }

    else if (current_mode == MODE_RATE) {
      float r = clamp_float(activity_abs, min_rate, max_rate);
      pwm = map_float_to_pwm(r, min_rate, max_rate);

      if (pwm > RATE_MODE_MAX_PWM) {
        pwm = RATE_MODE_MAX_PWM;
      }

      if (pwm > 0) {
        pwm = apply_motor_floor(pwm);
      }
    }

    else if (current_mode == MODE_NO_BREATH_ALERT) {
      unsigned long quiet_time = now - last_breath_detected_time;

      if (quiet_time >= no_breath_timeout) {
        unsigned long alert_elapsed = quiet_time - no_breath_timeout;

        float ramp_progress = (float)alert_elapsed / (float)pulse_ramp_time;
        if (ramp_progress > 1.0) {
          ramp_progress = 1.0;
        }

        int pulse_strength = alert_min_pwm + (int)((alert_max_pwm - alert_min_pwm) * ramp_progress);
        unsigned long pulse_phase = alert_elapsed % pulse_period;

        if (pulse_phase < pulse_on_time) {
          pwm = pulse_strength;
        }
      }

      if (pwm > 0) {
        pwm = apply_motor_floor(pwm);
      }
    }
  }

  last_pwm = pwm;
  analogWrite(MOTOR_PIN, pwm);

  print_plot_values(raw_signal, pwm);

  delay(LOOP_DELAY);
}

// -------------------------------------------------------------
// Helper functions
// -------------------------------------------------------------
float clamp_float(float x, float lo, float hi) {
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

int map_float_to_pwm(float x, float in_min, float in_max) {
  if (in_max == in_min) {
    return 0;
  }

  float result = (x - in_min) * 255.0 / (in_max - in_min);
  result = clamp_float(result, 0.0, 255.0);

  return (int)result;
}

int apply_motor_floor(int pwm) {
  if (pwm <= 0) {
    return 0;
  }

  if (pwm < MIN_MOTOR_PWM) {
    return MIN_MOTOR_PWM;
  }

  if (pwm > 255) {
    return 255;
  }

  return pwm;
}

float get_load_cell_signal() {
  long raw = load_cell.read();
  return (raw - zero_offset) / scale_factor;
}

float get_normalized_signal(float signal_value) {
  float span = calibrated_max - calibrated_min;

  if (span < MIN_CALIBRATION_SPAN) {
    span = MIN_CALIBRATION_SPAN;
  }

  float normalized = (signal_value - calibrated_min) / span;
  return clamp_float(normalized, 0.0, 1.0);
}

void print_plot_values(float signal_value, int pwm) {
  float plot_min;
  float plot_max;
  float display_offset;

  if (has_valid_calibration) {
    plot_min = calibrated_min;
    plot_max = calibrated_max;
    display_offset = -calibrated_min + 5.0;
  } else {
    plot_min = 0.0;
    plot_max = 0.0;
    display_offset = 0.0;
  }

  Serial.print(signal_value + display_offset, 2);
  Serial.print('\t');
  Serial.print(pwm);
  Serial.print('\t');
  Serial.print(plot_min + display_offset, 2);
  Serial.print('\t');
  Serial.println(plot_max + display_offset, 2);
}

// -------------------------------------------------------------
// Core functions — calibration feedback
// -------------------------------------------------------------
void signal_calibration_start_or_end() {
  int pwm = apply_motor_floor(CALIBRATION_SIGNAL_PWM);

  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    analogWrite(MOTOR_PIN, pwm);
    delay(CALIBRATION_SIGNAL_ON_TIME);

    digitalWrite(LED_PIN, LOW);
    analogWrite(MOTOR_PIN, 0);

    if (i < 1) {
      delay(CALIBRATION_SIGNAL_OFF_TIME);
    }
  }
}

// -------------------------------------------------------------
// Core functions — LED feedback
// -------------------------------------------------------------
void blink_n_times(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(BLINK_ON_TIME);

    digitalWrite(LED_PIN, LOW);
    delay(BLINK_OFF_TIME);
  }
}

void indicate_current_mode() {
  if (current_mode == MODE_ABSOLUTE) {
    blink_n_times(1);
  } else if (current_mode == MODE_RATE) {
    blink_n_times(2);
  } else if (current_mode == MODE_NO_BREATH_ALERT) {
    blink_n_times(3);
  }
}

// -------------------------------------------------------------
// Core functions — signal state management
// -------------------------------------------------------------
void reset_signal_state() {
  last_time = millis();

  if (load_cell.is_ready()) {
    float s = get_load_cell_signal();
    last_signal_filtered = s;
    last_normalized_signal = get_normalized_signal(s);
    last_valid_signal = s;
    have_valid_signal = true;
  } else {
    last_signal_filtered = 0.0;
    last_normalized_signal = 0.0;
  }

  rate_filtered = 0.0;
  filter_initialized = true;
  last_breath_detected_time = millis();
  last_pwm = 0;
  last_valid_reading_time = millis();
}

// -------------------------------------------------------------
// Core functions — calibration
// -------------------------------------------------------------
void start_calibration() {
  previous_calibrated_min = calibrated_min;
  previous_calibrated_max = calibrated_max;

  analogWrite(MOTOR_PIN, 0);
  digitalWrite(LED_PIN, LOW);

  signal_calibration_start_or_end();

  is_calibrating = true;
  calibration_start_time = millis();
  calibration_has_reading = false;

  calibration_min_signal = 0.0;
  calibration_max_signal = 0.0;
  calibration_signal_filtered = 0.0;

  have_valid_calibration_signal = false;

  calibration_led_state = true;
  last_calibration_led_toggle = millis();
  digitalWrite(LED_PIN, HIGH);

  filter_initialized = false;

  Serial.println("CALIBRATION_START");
}

void finish_calibration() {
  is_calibrating = false;
  digitalWrite(LED_PIN, LOW);
  analogWrite(MOTOR_PIN, 0);

  float calibration_span = 0.0;

  if (calibration_has_reading) {
    calibration_span = calibration_max_signal - calibration_min_signal;

    if (calibration_span >= MIN_CALIBRATION_SPAN) {
      calibrated_min = calibration_min_signal;
      calibrated_max = calibration_max_signal;
      has_valid_calibration = true;
    } else {
      calibrated_min = previous_calibrated_min;
      calibrated_max = previous_calibrated_max;
    }
  } else {
    calibrated_min = previous_calibrated_min;
    calibrated_max = previous_calibrated_max;
  }

  signal_calibration_start_or_end();

  reset_signal_state();

  Serial.print("CALIBRATION_DONE\tMIN=");
  Serial.print(calibrated_min, 2);
  Serial.print("\tMAX=");
  Serial.print(calibrated_max, 2);
  Serial.print("\tSPAN=");
  Serial.print(calibration_span, 2);
  Serial.print("\tVALID=");

  if (has_valid_calibration) {
    Serial.println(1);
  } else {
    Serial.println(0);
  }
}

void update_calibration_led(unsigned long now) {
  if (now - last_calibration_led_toggle >= CALIBRATION_LED_INTERVAL) {
    calibration_led_state = !calibration_led_state;

    if (calibration_led_state) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }

    last_calibration_led_toggle = now;
  }
}

// -------------------------------------------------------------
// Core functions — mode switching
// -------------------------------------------------------------
void next_mode() {
  if (current_mode == MODE_ABSOLUTE) {
    current_mode = MODE_RATE;
  } else if (current_mode == MODE_RATE) {
    current_mode = MODE_NO_BREATH_ALERT;
  } else {
    current_mode = MODE_ABSOLUTE;
  }

  reset_signal_state();
  indicate_current_mode();
}