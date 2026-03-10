// --- Pin Definitions (From your micro-ROS script) ---
#define EN_PIN  27
#define IN1_PIN 26
#define IN2_PIN 25
#define ENCODER_A_PIN 33
#define ENCODER_B_PIN 32

// --- Motor & Encoder Constants ---
const int freq = 980;
const int resolution = 8;
#define DEBOUNCE_US 50

// --- Variables ---
volatile long encoder_count = 0;
volatile unsigned long last_isr_time_A = 0;
volatile unsigned long last_isr_time_B = 0;

unsigned long previousMillis = 0;
const int dt_ms = 20; // 20ms = 50Hz sample rate for serial plotting

// Step response variables
bool stepApplied = false;
unsigned long startTime = 0;
const int stepPWM = 150; // The PWM value for the step (0-255)

// --- Interrupt Service Routines ---
void IRAM_ATTR encoder_isr_A() {
  unsigned long now = micros();
  if (now - last_isr_time_A < DEBOUNCE_US) return;
  last_isr_time_A = now;
  if (digitalRead(ENCODER_A_PIN) == digitalRead(ENCODER_B_PIN)) {
    encoder_count++;
  } else {
    encoder_count--;
  }
}

void IRAM_ATTR encoder_isr_B() {
  unsigned long now = micros();
  if (now - last_isr_time_B < DEBOUNCE_US) return;
  last_isr_time_B = now;
  if (digitalRead(ENCODER_A_PIN) == digitalRead(ENCODER_B_PIN)) {
    encoder_count--;
  } else {
    encoder_count++;
  }
}

void setup() {
  Serial.begin(115200);

  // Motor Setup
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  ledcAttach(EN_PIN, freq, resolution);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  ledcWrite(EN_PIN, 0);

  // Encoder Setup
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoder_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoder_isr_B, CHANGE);

  Serial.println("Time(ms),RPM");
  
  // Wait 2 seconds before applying the step
  delay(2000); 
  startTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // Apply the step input after the initial wait
  if (!stepApplied && (currentMillis - startTime >= 0)) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    ledcWrite(EN_PIN, stepPWM); 
    stepApplied = true;
  }

  // Calculate and print RPM at the given sample rate
  if (currentMillis - previousMillis >= dt_ms) {
    float dt = (currentMillis - previousMillis) / 1000.0; // convert to seconds
    
    // Safely read and reset the pulse count
    noInterrupts();
    long currentPulses = encoder_count;
    encoder_count = 0;
    interrupts();

    // RPM formula (using 115 pulses per revolution as provided previously)
    float currentRPM = (currentPulses * 60.0) / (115.0 * dt);

    // Print to serial
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.println(currentRPM);

    previousMillis = currentMillis;
  }

  // Stop the motor after 5 seconds to prevent runaway
  if (stepApplied && (currentMillis - startTime > 5000)) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(EN_PIN, 0);
    while(true); // Halt the program so it doesn't loop
  }
}
