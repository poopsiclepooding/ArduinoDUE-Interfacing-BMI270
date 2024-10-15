#include <math.h>

/* Define Pins */
const int redLedPin = 2;  // Pin for the slow blinking LED
const int blueLedPin = 3;  // Pin for the fast blinking LED
const int greenLedPin = 4;  // Pin for the fast blinking LED
const int buzzerPin = 5; // Connected to buzzer and used to make audio sounds

/* Define PWN Interval */
const long interval_normal = 500; // In milliseconds
const long interval_alert = 100; // In milliseconds

/* Define struct for fake sensor data (accelerometer) */
struct sensor_data_accel {
    float x;
    float y;
    float z;
};

/* Define struct for fake sensor data (gyroscope) */
struct sensor_data_gyro {
    float x;
    float y;
    float z;
};

// Define two arrays of the struct
const int accel_data_size = 20; // Size of the first array
const int gyro_data_size = 20; // Size of the second array
int i = 0;
float delta_time = 0.05;
float q[4]; // Quaternion
float angles[3] = {0.0f, 0.0f, 0.0f}; // Initial Angle
float velocity[3] = {0.0f, 0.0f, 0.0f}; // Initial velocity
float displacement[3] = {0.0f, 0.0f, 0.0f}; // Initial displacement
float total_displacement = 0.0f; // Total displacement
float total_velocity = 0.0f; // Total velocity
float total_accel = 0.0f; // Total acceleration
float prev_total_accel = 0.0f; // Previous total acceleration
float delta_accel = 0.0f; // Change in acceleration
float jerk = 0.0f; // Inital Jerk
float adjusted_accel[3] = {0.0f, 0.0f, 0.0f};  // Adjusted acceleration
float filtered_accel[3] = {0.0f, 0.0f, 0.0f};  // Filtered acceleration
float alpha = 0.8; // Filter Alpha
const float gravity[3] = {0.0f, 0.0f, 9.81f}; // Gravity
float displacement_threshold = 1; // Threshold for displacement (m)
float motion_threshold = 0.05; // Threshold for motion (m/s)
float jerk_threshold = 3; // Threshold for high jerk (m/s^3)
String current_state = "Stationary";
String prev_state = "Stationary";

bool ledState = LOW;

unsigned long current_millis = 0;
unsigned long prev_millis = 0;
unsigned long current_time;
unsigned long prev_time;

/* Function Prototype */
void euler_to_quaternion(float angles[3], float q[4]);
void rotate_vector(float q[4], sensor_data_accel accel_data, float adjusted_accel[3]);
void low_pass_filter(float input[3], float output[3], float alpha);
void State_Signal(String current_state, String prev_state, unsigned long &prev_millis, unsigned long current_millis, bool& ledState);

sensor_data_accel accel_data[accel_data_size] = { // Data in m/s^2
    {0.01, -0.01, 9.83},  // Should start stationary
    {0.01,  0.01, 9.80},  
    {-0.01,  0.01, 9.81},
    {0.01,  0.01, 9.80}, // Should remain stationary
    {0.1, 0.1, 9.80},  // Should start motion
    {0.2, 0.2, 9.79},
    {0.3, 0.3, 9.79},
    {0.4, 0.4, 9.79},
    {0.5, 0.5, 9.75},
    {-0.1, -0.04, 9.83},  // High jerk stop
    {-0.09, -0.04, 9.79},
    {-0.02, -0.08, 9.76}, // Should remain motion
    {0.00,  0.00, 9.81},  
    {0.6,  0.9, 9.80},  // Should be high jerk
    {0.5,  0.9, 9.77},
    {-0.04, 0.01, 9.82},
    {0.08, -0.05, 9.79},
    {-0.06, 0.04, 9.78},
    {0.01,  0.03, 9.85},
    {-0.09, -0.02, 9.81},
};

sensor_data_gyro gyro_data[gyro_data_size] = {  // Data in Radians/Sec 
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    // {0.00, 0.00, 0.00},
    {0.012, -0.005, 0.010},
    {0.015, -0.002, 0.013},
    {0.008, -0.010, 0.009},
    {0.010, 0.000, 0.011},
    {0.007, -0.003, 0.008},
    {0.014, -0.006, 0.012},
    {0.009, -0.004, 0.010},
    {0.016, 0.001, 0.013},
    {0.011, -0.007, 0.009},
    {0.013, -0.005, 0.010},
    {0.006, -0.003, 0.007},
    {0.018, 0.002, 0.014},
    {0.007, -0.006, 0.008},
    {0.014, -0.001, 0.012},
    {0.009, -0.008, 0.010},
    {0.013, -0.004, 0.011},
    {0.010, -0.002, 0.009},
    {0.015, 0.001, 0.012},
    {0.012, -0.005, 0.009},
    {0.008, -0.007, 0.011}
};

void setup() {
    // Start the Serial communication
    Serial.begin(9600);
    while (!Serial) {
        ; // Wait for Serial to initialize
    }

    /* Set up all pins */
    pinMode(redLedPin, OUTPUT);
    pinMode(blueLedPin, OUTPUT);
    pinMode(greenLedPin, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    Serial.println("Hello, this code interfaces BMI270 with Arduino DUE for calculating a system's states.");
}

void loop() {
  if (i<accel_data_size){

    // Get the current time for non-blocking timing
    unsigned long current_millis = millis();

    // Calculate delta time for jerk and displacement
    unsigned long current_time = millis();
    delta_time = (current_time - prev_time) / 1000.0;  // Time in seconds
    prev_time = current_time;

    /* Find true x,y,z coordinate acceleration by using both accelerometer and gyroscope reading */
    angles[0] += gyro_data[i].x * delta_time;  // Roll
    angles[1] += gyro_data[i].y * delta_time;  // Pitch
    angles[2] += gyro_data[i].z * delta_time;  // Yaw
    
    // For inverse roation of coordinate system 
    angles[0] = -angles[0];
    angles[1] = -angles[1];
    angles[2] = -angles[2];
    euler_to_quaternion(angles, q);
    rotate_vector(q, accel_data[i], adjusted_accel);

    /* Compensate for gravity */
    for (int j = 0; j < 3; j++) {
      adjusted_accel[j] = adjusted_accel[j] - gravity[j];
      Serial.print(adjusted_accel[j]);
      Serial.print(" ");
    }

    low_pass_filter(adjusted_accel, filtered_accel, alpha);

    /* Calculate velocity and distance */
    for (int j = 0; j < 3; j++) {
      velocity[j] += filtered_accel[j] * delta_time;
      displacement[j] += velocity[j] * delta_time;
      Serial.print("Final Acceleration: ");
      Serial.print(filtered_accel[j]);
      Serial.print(" | ");
      Serial.print("Final Velocity: ");
      Serial.print(velocity[j]);
      Serial.print(" | ");
      Serial.print("Final Displacement: ");
      Serial.print(displacement[j]);
      Serial.println(" | ");
    }
    Serial.println("Total Distance and velocity :");
    total_displacement = sqrt(displacement[0]*displacement[0] + displacement[1]*displacement[1] + displacement[2]*displacement[2]);
    total_velocity = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
    Serial.print("Total Displacement: ");
    Serial.print(total_displacement);
    Serial.print(" | ");
    Serial.print("Total Velocity: ");
    Serial.println(total_velocity);

    /* Check if stationary of motion */
    if (total_velocity < motion_threshold){
      current_state = "Stationary";
    } else {
      current_state = "Motion";
    }


    /* Check if high jerk */
    total_accel = sqrt(filtered_accel[0]*filtered_accel[0] + filtered_accel[1]*filtered_accel[1] + filtered_accel[2]*filtered_accel[2]);
    delta_accel = total_accel - prev_total_accel;
    prev_total_accel = total_accel;
    jerk = delta_accel / delta_time;
    Serial.print("Total Jerk: ");
    Serial.println(jerk);
    if (fabs(jerk) > jerk_threshold){   // ASSUMPTION : JERK BOTH POSITIVE OR NEGATIVE ALLOWED
      current_state = "High Jerk";
    }

    /* Check if displaced more than 1m */
    if (total_displacement > displacement_threshold){
      current_state = "Displaced";
    }

    if (current_state == "Stationary") {
      State_Signal(current_state, prev_state, prev_millis, current_millis, ledState);
    } else if (current_state == "Motion") {
      State_Signal(current_state, prev_state, prev_millis, current_millis, ledState);
    } else if (current_state == "High Jerk") {
      State_Signal(current_state, prev_state, prev_millis, current_millis, ledState);
    } else if (current_state == "Displaced") {
      State_Signal(current_state, prev_state, prev_millis, current_millis, ledState);
    }
    Serial.println(current_state);
    prev_state = current_state;

    i++;

  } else {
    // do nothing
  }
}

void euler_to_quaternion(float angles[3], float q[4]){
  float roll = angles[0];// * (M_PI / 180.0f);
  float pitch = angles[1];// * (M_PI / 180.0f);
  float yaw = angles[2];// * (M_PI / 180.0f);

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  q[0] = cr * cp * cy + sr * sp * sy;  // qx
  q[1] = sr * cp * cy - cr * sp * sy;  // qy
  q[2] = cr * sp * cy + sr * cp * sy;  // qz
  q[3] = cr * cp * sy - sr * sp * cy;  // qw
}

void rotate_vector(float q[4], sensor_data_accel accel_data, float adjusted_accel[3]){
  float q_conjugate[4] = {q[0], -q[1], -q[2], -q[3]};
  
  float w = 0;
  float x = accel_data.x;
  float y = accel_data.y;
  float z = accel_data.z;
  float vec_quat[4] = {w, x, y, z};

  /* Passive rotation using quaternion: q * v * q_conjugate 
    (t0, t1, t2, t3) = (r0, r1, r2, r3) ✕ (s0, s1, s2, s3) => 
    t0 = (r0s0 − r1s1 − r2s2 − r3s3)
    t1 = (r0s1 + r1s0 − r2s3 + r3s2)
    t2 = (r0s2 + r1s3 + r2s0 − r3s1)
    t3 = (r0s3 − r1s2 + r2s1 + r3s0)
  */
  float temp[4];
  temp[0] = q[0] * vec_quat[0] - q[1] * vec_quat[1] - q[2] * vec_quat[2] - q[3] * vec_quat[3];
  temp[1] = q[0] * vec_quat[1] + q[1] * vec_quat[0] - q[2] * vec_quat[3] + q[3] * vec_quat[2];
  temp[2] = q[0] * vec_quat[2] + q[1] * vec_quat[3] + q[2] * vec_quat[0] - q[3] * vec_quat[1];
  temp[3] = q[0] * vec_quat[3] - q[1] * vec_quat[2] + q[2] * vec_quat[1] + q[3] * vec_quat[0];

  // Multiply by the conjugate
  adjusted_accel[0] = temp[0] * q_conjugate[1] + temp[1] * q_conjugate[0] - temp[2] * q_conjugate[3] + temp[3] * q_conjugate[2];
  adjusted_accel[1] = temp[0] * q_conjugate[2] + temp[1] * q_conjugate[3] + temp[2] * q_conjugate[0] - temp[3] * q_conjugate[1];
  adjusted_accel[2] = temp[0] * q_conjugate[3] - temp[1] * q_conjugate[2] + temp[2] * q_conjugate[1] + temp[3] * q_conjugate[0];
}

void low_pass_filter(float input[3], float output[3], float alpha) {
    for (int i = 0; i < 3; i++) {
        output[i] = alpha * input[i] + (1 - alpha) * output[i];  // Apply filter
    }
}

void State_Signal(String current_state, String prev_state, unsigned long &prev_millis, unsigned long current_millis, bool& ledState){

  // If change in state the make all LEDs low before any change
  if (current_state != prev_state){
    digitalWrite(redLedPin, LOW);
    digitalWrite(blueLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
    ledState = LOW;
  }

  if (current_state == "Stationary"){
    if (current_millis - prev_millis >= interval_normal) {
      prev_millis = current_millis;
      
      // Toggle the LED state
      ledState = !ledState;
      digitalWrite(redLedPin, ledState ? HIGH : LOW);       // For stationary state, toggle the RED Led
    }
  } else if (current_state == "Motion") {
    if (current_millis - prev_millis >= interval_normal) {
      prev_millis = current_millis;
      
      // Toggle the LED state
      ledState = !ledState;
      digitalWrite(blueLedPin, ledState ? HIGH : LOW);       // For motion state, toggle the BLUE Led
    }
  } else if (current_state == "High Jerk") {
    if (current_millis - prev_millis >= interval_alert) {
      prev_millis = current_millis;
      
      // Toggle the LED state
      ledState = !ledState;
      digitalWrite(greenLedPin, HIGH);  // For high jerk state, set all Leds to HIGH
      digitalWrite(blueLedPin, HIGH);
      digitalWrite(redLedPin, HIGH);
      tone(buzzerPin, 1000, 200);   // Make a 1000Hz tone for 200ms
    }
  } else if (current_state == "Displaced") {
    if (current_millis - prev_millis >= interval_alert) {
      prev_millis = current_millis;

      digitalWrite(greenLedPin, HIGH);  // For displaced by more than 1m state, make the GREEN Led HIGH
      tone(buzzerPin, 10000, 200);   // Make a 10000Hz tone for 200ms
    }
  }
}
