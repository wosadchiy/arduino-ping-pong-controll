// ---------------------------------------------------------
// Automated Ball Tracking + Manual Overdrive + Watchdog
// ---------------------------------------------------------

const int stepPin = 9;         
const int dirPin = 5;          
const int enPin = 10;          
const int buttonPin1 = 12;    
const int buttonPin2 = 7;     

float angleX = 0, angleY = 0;
int normX = 0, normY = 0;

// PID & Speed
float Kp = 1.0;               
float omega = 0.0;            
float omega_preset = 0.0;     
const float max_omega = 40.0; 
const float manual_speed = 5.0;

// --- WATCHDOG SETTINGS ---
uint32_t last_packet_time = 0;   // Time of last valid packet from Python
const uint32_t timeout_ms = 200; // Stop if no data for 200ms
// --------------------------

String inputBuffer = ""; 

void setup() {
  Serial.begin(115200);
  inputBuffer.reserve(64);

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH);  

  TCCR1A = 0; TCCR1B = 0; 
  TCCR1B |= (1 << WGM12);      
  TCCR1B |= (1 << CS11);       
  OCR1A = 11172; 
}

void loop() {
  // 1. READ SERIAL
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      parseIncomingData(inputBuffer);
      inputBuffer = ""; 
      last_packet_time = millis(); // UPDATE WATCHDOG on valid packet
    } else {
      inputBuffer += inChar;
    }
  }

  // 2. CHECK WATCHDOG (If Python app is closed/frozen)
  bool python_active = (millis() - last_packet_time < timeout_ms);

  // 3. INPUT PRIORITY LOGIC
  bool btnLeft = (digitalRead(buttonPin1) == LOW);
  bool btnRight = (digitalRead(buttonPin2) == LOW);

  if (btnLeft) {
    omega_preset = -manual_speed;
  } 
  else if (btnRight) {
    omega_preset = manual_speed;
  } 
  else if (python_active) {
    // AUTOMATIC MODE (Only if Python is sending data)
    omega_preset = angleX * Kp;
    if (omega_preset > max_omega) omega_preset = max_omega;
    if (omega_preset < -max_omega) omega_preset = -max_omega;
  } 
  else {
    // CAMERA PROGRAM STOPPED OR BALL LOST
    omega_preset = 0;
    angleX = 0; // Reset last known error
  }

  // 4. ACCELERATION BLOCK
  static uint32_t tmr_accel;
  if (millis() - tmr_accel >= 1) {
    tmr_accel = millis();
    float accel_step = 0.15;
    if (abs(omega_preset - omega) > 0.05) {
      if (omega_preset > omega) omega += accel_step;
      else omega -= accel_step;
    } else {
      omega = omega_preset;
    }
  }

  // 5. TIMER & MOTOR CONTROL
  if (abs(omega) > 0.1) {
    digitalWrite(enPin, LOW); 
    digitalWrite(dirPin, (omega > 0) ? HIGH : LOW);
    unsigned int new_ocr = 11172 / abs(omega);
    if (new_ocr < 20) new_ocr = 20; 
    OCR1A = new_ocr;
    TCCR1A |= (1 << COM1A0); 
  } 
  else {
    TCCR1A &= ~(1 << COM1A0);
    digitalWrite(stepPin, LOW);
  }
}

void parseIncomingData(String line) {
  line.trim();
  if (line.length() == 0) return;
  int idx1 = line.indexOf(',');
  int idx2 = line.indexOf(',', idx1 + 1);
  int idx3 = line.indexOf(',', idx2 + 1);

  if (idx1 > 0 && idx2 > 0 && idx3 > 0) {
    angleX = line.substring(0, idx1).toFloat();
    angleY = line.substring(idx1 + 1, idx2).toFloat();
    normX  = line.substring(idx2 + 1, idx3).toInt();
    normY  = line.substring(idx3 + 1).toInt();
  }
}