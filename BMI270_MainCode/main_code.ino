#include <SPI.h>
#include <DueTimer.h>
#include "Wire.h"
#include "bmi2.h"
#include "bmi270.h"
#include <math.h>
#define PI 3.14159265358979323846

// #define DEBUG

/* Define Pins */
const int redLedPin = 2;  // Pin for the slow blinking LED
const int blueLedPin = 3;  // Pin for the fast blinking LED
const int greenLedPin = 4;  // Pin for the fast blinking LED
const int buzzerPin = 5; // Connected to buzzer and used to make audio sounds
const int startButtonPin = 6; // Button for starting the system
const int stopButtonPin = 7; // Button for stopping and resetting the system
#define BMI270_CS       10  // Chip Select (CS) Pin for BMI270

/* Define PWN Interval */
const long interval_normal = 500; // In milliseconds
const long interval_alert = 100; // In milliseconds
const long interval_buzz = 250; // In milliseconds

/* Define System Running */
bool isSystemRunning = false;

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

/* Define sensor data processing variables */
int i = 0;
float delta_time = 0.05;
sensor_data_accel accel_data = {0.00, 0.00, 0.00};
sensor_data_gyro gyro_data = {0.00, 0.00, 0.00};
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
float alpha = 0.95; // Filter Alpha
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


/*! I2C interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_I2C UINT8_C(0)

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_SPI UINT8_C(1)

static int8_t bmi2_set_config(struct bmi2_dev *bmi2_dev);
#if BMI270_INTERFACE_I2C == 1
/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t i2c_bus;
#elif BMI270_INTERFACE_SPI == 1
static uint8_t spi_bus;
#endif


/* Data Prcoessing Variables */
int data_acquire_frequency = 100; // Frequency of acquiring and processing new data

/* Create an instance of sensor data structure */
struct bmi2_sensor_data sensor_data = { 0 };

/* Structure to define BMI2 sensor configurations */
struct bmi2_dev bmi2;

int8_t BMI270_write_spi(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  uint32_t cnt;
  int8_t rev = 0;
  (void)(intf_ptr);
  digitalWrite(BMI270_CS, LOW);
  SPI.transfer(reg_addr);
  for (cnt = 0; cnt < length; cnt++)
  {
    SPI.transfer(*(reg_data + cnt));
  }
  digitalWrite(BMI270_CS, HIGH);
  return rev;
}

int8_t BMI270_read_spi(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  uint32_t cnt;
  int8_t rev = 0;
  (void)(intf_ptr);
  reg_addr = 0x80 | reg_addr;
  digitalWrite(BMI270_CS, LOW);
  SPI.transfer(reg_addr);
  for (cnt = 0; cnt < length; cnt++)
  {
    *(reg_data + cnt) = SPI.transfer(0x00);
  }
  digitalWrite(BMI270_CS, HIGH);
  return rev;
}


void bmi2xy_hal_delay_usec(uint32_t period_us, void *intf_ptr)
{
  delayMicroseconds(period_us);
}

/*! This API is used to perform I2C read operation with sensor */
int8_t bmi2xy_hal_i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  int8_t rslt = 0;
  //uint8_t dev_id = 0x68;
  uint8_t* dev_id = (uint8_t *)intf_ptr;

  rslt = BMI270_read_i2c(*dev_id, reg_addr, reg_data, length);

  return rslt;
}

/*! This API is used to perform I2C write operations with sensor */
int8_t bmi2xy_hal_i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  int8_t rslt = 0;
  //    uint8_t dev_id = 0x68;

  uint8_t* dev_id = (uint8_t *)intf_ptr;
  rslt = BMI270_write_i2c(*dev_id, reg_addr, (uint8_t *)reg_data, length);

  return rslt;
}

int8_t BMI270_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
  /* dev_addr: I2C device address.
    reg_addr: Starting address for writing the data.
    reg_data: Data to be written.
    count: Number of bytes to write */
  // Begin I2C communication with provided I2C address
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  // Done writting, end the transmission
  int8_t returned = Wire.endTransmission();

  if (returned)
  {
    /*
      case 1:Data too long to fit in transmit buffer
          break;
      case 2:received NACK on transmit of address.
          break;
      case 3:received NACK on transmit of data."
          break;
      case 4:Unspecified error.
          break;
      default:Unexpected Wire.endTransmission() return code:
    */
    return returned;
  }

  // Requests the required number of bytes from the sensor
  Wire.requestFrom((int)dev_addr, (int)count);

  uint16_t i;
  // Reads the requested number of bytes into the provided array
  for (i = 0; (i < count) && Wire.available(); i++)
  {
    reg_data[i] = Wire.read(); // This is for the modern Wire library
  }

  // This must return 0 on success, any other value will be interpreted as a communication failure.
  return 0;
}

int8_t BMI270_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
  /*  dev_addr: I2C device address.
    reg_addr: Starting address for reading the data.
    reg_data: Buffer to take up the read data.
    count: Number of bytes to read. */
  // Begin I2C communication with provided I2C address
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);

  uint16_t i;
  // Writes the requested number of bytes from the provided array
  for (i = 0; i < count; i++)
  {
    Wire.write(reg_data[i]); // This is for the modern Wire library
  }
  // Done writting, end the transmission
  int8_t returned = Wire.endTransmission();
  /*
      case 1:Data too long to fit in transmit buffer
      case 2:received NACK on transmit of address.
      case 3:received NACK on transmit of data.
      case 4:Unspecified error.
      default:Unexpected Wire.endTransmission() return code:
  */
  // This must return 0 on sucess, any other value will be interpretted as a communication failure.
  return returned;
}

int8_t bmi2_accel_set_config(struct bmi2_dev *bmi2_dev)
{
  /* Variable to define result */
  int8_t rslt;

  /* Initialize interrupts for accelerometer */
  uint8_t sens_int = BMI2_DRDY_INT; // This is the data ready interrupt in BMI270

  /* List the sensors which are required to enable */
  uint8_t sens_list = BMI2_ACCEL;

  /* Structure to define the type of the sensor and its configurations */
  struct bmi2_sens_config config;

  /* Configure type of feature */
  config.type = BMI2_ACCEL;

  /* Enable the selected sensors */
  rslt = bmi2_sensor_enable(&sens_list, 1, bmi2_dev);

  if (rslt == BMI2_OK)
  {
    /* Get default configurations for the type of feature selected */
    rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);

    if (rslt == BMI2_OK)
    {

      config.cfg.acc.odr = BMI2_ACC_ODR_100HZ;

      /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
      config.cfg.acc.range = BMI2_ACC_RANGE_2G; // Use range 2G
      config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;  // Depending on filter_perf (0)- > select normal mode , (1) -> select undersampling mode
      config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;  // Filter config, undersampling mode with avg 4 samples

      /* Set the accelerometer configurations */
      rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);

      if (rslt == BMI2_OK)
      {
        /* Map interrupt to pins */
        rslt = bmi2_map_data_int(sens_int, BMI2_INT2, bmi2_dev);  // Map BMI2_INT2 to generate data ready interrupt
      }
    }
  }

  return rslt;
}

int8_t bmi2_gyro_set_config(struct bmi2_dev *bmi2_dev)
{
   /* Variable to define result */
    int8_t rslt;

    /* Initialize interrupts for gyroscope */
    uint8_t sens_int = BMI2_DRDY_INT; // This is the data ready interrupt in BMI270

    /* List the sensors which are required to enable */
    uint8_t sens_list = BMI2_GYRO;

    /* Structure to define the type of the sensor and its configurations */
    struct bmi2_sens_config config;

    /* Configure type of feature */
    config.type = BMI2_GYRO;

    /* Enable the selected sensors */
    rslt = bmi2_sensor_enable(&sens_list, 1, bmi2_dev);

    if (rslt == BMI2_OK)
    {
        /* Get default configurations for the type of feature selected */
        rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);

        if (rslt == BMI2_OK)
        {
            /* The user can change the following configuration parameter according to their requirement */
            /* Output data Rate. It is set as 100Hz for gyro */
            config.cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
            /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps */
            config.cfg.gyr.range = BMI2_GYR_RANGE_2000; // Use 2000dps range
            config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;  // This defined the 3dB cutoff frequency for the low pass filer
            config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;  // Power optimized noise performance
            config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;  // Performance optimized filter performance

            /* Set the gyro configurations */
            rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);

            if (rslt == BMI2_OK)
            {
                /* Map interrupt to pins */
                rslt = bmi2_map_data_int(sens_int, BMI2_INT2, bmi2_dev); // Map BMI2_INT2 to generate data ready interrupt
            }
        }
    }

    return rslt;
}

void setup() {

  Serial.println("Hello, this code interfaces BMI270 with Arduino DUE for calculating a system's states.");
  Serial.println("States are defined as follows :");
  Serial.println("1. Stationary");
  Serial.println("2. Motion");
  Serial.println("3. High Jerk");
  Serial.println("4. Displaced By More Than 1m");
  Serial.println("In case of DEBUG defined, all values printed are in SI units");


  /* Setup up LED and Buzzer Pins */
  pinMode(redLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(stopButtonPin, INPUT_PULLUP);

  /* Variable to define result */
  int8_t rslt; 
  pinMode(21, OUTPUT);
  pinMode(BMI270_CS, OUTPUT);
  digitalWrite(BMI270_CS, HIGH);
  //  Wire.setModule(0);
  Serial.begin(115200); // Serial monitor baudrate 

  spi_bus = BMI270_CS;  // SPI Bus is BMI270_CS which is PIN 10
  

  /* To initialize the hal function */

  SPI.begin();  // Begin SPI connection

  /*! intf_ptr : The interface pointer is used to enable the user to link their interface descriptors for reference during the
  * implementation of the read and write interfaces to the hardware. */
  bmi2.intf_ptr = &spi_bus; 
  bmi2.intf = BMI2_SPI_INTF;  // Set communication protocol to SPI
  bmi2.read = BMI270_read_spi;  // Set read function ptr to defined read function
  bmi2.write = BMI270_write_spi;  // Set write function ptr to defined write function
  bmi2.read_write_len = 32; // Length of Read or Write using SPI
  bmi2.delay_us = bmi2xy_hal_delay_usec;  // Set function ptr to defined delay function

  /* Config file pointer should be assigned to NULL, so that default file address is assigned in bmi270_init */
  bmi2.config_file_ptr = NULL;

  /* Initialize bmi270 */
  rslt = bmi270_init(&bmi2);

  if (rslt == 0){
    Serial.println("bmi270_init done");
  } else {
    Serial.println("bmi270_init failed");
  }
  rslt = bmi2_accel_set_config(&bmi2);

  if (rslt == BMI2_OK)
  {
    Serial.println("Accel Config Succeded, Data Ready : X Y Z ");
  } else {
    Serial.print("Accel Config Failed");
  }

  rslt = bmi2_gyro_set_config(&bmi2);

  if (rslt == BMI2_OK)
  {
    Serial.println("Gyro Config Succeded, Data Ready : X Y Z ");
  } else {
    Serial.print("Gyro Config Failed");
  }
  
  Timer2.attachInterrupt(RawDataHandler).setFrequency(data_acquire_frequency).start();
}

void loop() {
  // do nothing since timeer interrupt handler is working
}

void RawDataHandler()
{
  // Check for start button press
  if (digitalRead(startButtonPin) == LOW) {
    isSystemRunning = true;
    Serial.println("System started.");
    delay(250);  // Debounce
  }

  // Check for stop button press
  if (digitalRead(stopButtonPin) == LOW) {
    isSystemRunning = false;
    reset_system();
    Serial.println("System stopped.");
    delay(250);  // Debounce
  }

  if (isSystemRunning) {

    // Timing calculation
    unsigned long current_millis = millis();

    // Calculate delta time for jerk and displacement
    unsigned long current_time = millis();
    delta_time = (current_time - prev_time) / 1000.0;  // Time in seconds
    prev_time = current_time;

    sensor_data.type = BMI2_ACCEL_GYRO; // Sets sesnor to acquire to both accelerometer and gyroscope data
    bmi2_get_sensor_data(&sensor_data, 1, &bmi2); // Defined in bmi2.c. Aquires the sensor data

    #ifdef DEBUG
    Serial.println("Raw Accel and Gyro Data : ")
    Serial.print("Accel x = ");
    Serial.print(sensor_data.sens_data.acc.x);
    Serial.print("\t");
    Serial.print("Accel y = ");
    Serial.print(sensor_data.sens_data.acc.y);
    Serial.print("\t");
    Serial.print("Accel z = ");
    Serial.print(sensor_data.sens_data.acc.z);
    Serial.print("\t");
    Serial.print("Gyro x = ");
    Serial.print(sensor_data.sens_data.gyr.x);
    Serial.print("\t");
    Serial.print("Gyro y = ");
    Serial.print(sensor_data.sens_data.gyr.y);
    Serial.print("\t");
    Serial.print("Gyro z = ");
    Serial.println(sensor_data.sens_data.gyr.z);
    #endif

    /* Gyro data is a int16_t with 32767 mapped to 2000dps and -32768 mapped to -2000dps. 
    *  We peform conversion of this data to rad/s by ((float)int16_value / 32768.0) * 2000.0 * (PI / 180.0) */
    gyro_data.x = ((float)sensor_data.sens_data.gyr.x / 32768) * 2000 * (PI / 180.0); 
    gyro_data.y = ((float)sensor_data.sens_data.gyr.y / 32768) * 2000 * (PI / 180.0);
    gyro_data.z = ((float)sensor_data.sens_data.gyr.z / 32768) * 2000 * (PI / 180.0);

    /* Accel data is a int16_t with 32767 mapped to +2G and -32768 mapped to -2G. (G = 9.81 m/s^2)
    *  We peform conversion of this data to rad/s by ((float)int16_value / 32768.0) * 19.62 */
    accel_data.x = ((float)sensor_data.sens_data.acc.x/ 32768) * 19.62;
    accel_data.y = ((float)sensor_data.sens_data.acc.y/ 32768) * 19.62; 
    accel_data.z = ((float)sensor_data.sens_data.acc.z/ 32768) * 19.62; 

    /* Find true x,y,z coordinate acceleration by using both accelerometer and gyroscope reading */
    angles[0] += gyro_data.x * delta_time;  // Roll
    angles[1] += gyro_data.y * delta_time;  // Pitch
    angles[2] += gyro_data.z * delta_time;  // Yaw

    // For inverse roation of coordinate system 
    angles[0] = -angles[0];
    angles[1] = -angles[1];
    angles[2] = -angles[2];
    euler_to_quaternion(angles, q);
    rotate_vector(q, accel_data[i], adjusted_accel);

    /* Compensate for gravity */
    for (int j = 0; j < 3; j++) {
      adjusted_accel[j] = adjusted_accel[j] - gravity[j];
    }

    /* Pass through a low pass filter */
    low_pass_filter(adjusted_accel, filtered_accel, alpha);

    /* Calculate velocity and distance */
    for (int j = 0; j < 3; j++) {
      velocity[j] += filtered_accel[j] * delta_time;
      displacement[j] += velocity[j] * delta_time;
      #ifdef DEBUG
      Serial.print("Final Acceleration Values = ");
      Serial.print(filtered_accel[j]);
      Serial.print(" | ");
      Serial.print("Final Velocity Values = ");
      Serial.print(velocity[j]);
      Serial.print(" | ");
      Serial.print("Final Displacement Values = ");
      Serial.print(displacement[j]);
      Serial.println(" | ");
      #endif
    }

    total_displacement = sqrt(displacement[0]*displacement[0] + displacement[1]*displacement[1] + displacement[2]*displacement[2]);
    total_velocity = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
    #ifdef DEBUG
    Serial.println("Total Distance and Velocity :");
    Serial.print(total_displacement);
    Serial.print(" and ");
    Serial.println(total_velocity);
    #endif

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
    #ifdef DEBUG
    Serial.println("Jerk :");
    Serial.println(jerk);
    #endif
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
  }

}



void euler_to_quaternion(float angles[3], float q[4]){
  float roll = angles[0];   // * (M_PI / 180.0f); Uncomment if calucation done in degrees per sec
  float pitch = angles[1];  // * (M_PI / 180.0f);
  float yaw = angles[2];    // * (M_PI / 180.0f);

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

  /* Passive rotation(rotation of coordinate system wrt to point) using quaternion: q * v * q_conjugate.
    Quaternion Multiplication :
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
      // Toggle the LED state
      ledState = !ledState;
      digitalWrite(greenLedPin, ledState ? HIGH : LOW);  // For high jerk state, toggle all Leds
      digitalWrite(blueLedPin, ledState ? HIGH : LOW);
      digitalWrite(redLedPin, ledState ? HIGH : LOW);
      if (current_millis - prev_millis >= interval_buzz){
        tone(buzzerPin, 1000, 200);   // Make a 1000Hz tone for 200ms
      }
      prev_millis = current_millis;
    }
  } else if (current_state == "Displaced") {
    if (current_millis - prev_millis >= interval_alert) {

      // Set LED High
      digitalWrite(greenLedPin, HIGH);  // For displaced by more than 1m state, make the GREEN Led HIGH
      if (current_millis - prev_millis >= interval_buzz){
        tone(buzzerPin, 10000, 200);   // Make a 10000Hz tone for 200ms
      }
      prev_millis = current_millis;
    }
  }
}

void reset_system() {
  current_state = "Stationary"; // Reset Current State
  prev_state = "Stationary"; // Reset Previous State
  displacement = {0.00, 0.00, 0.00};  // Reset position to 0
  velocity = {0.00, 0.00, 0.00};  // Reset velocity to 0
  angles = {0.00, 0.00, 0.00};  // Reset angles to 0
  ledState = LOW;
  digitalWrite(greenLedPin, LOW);
  digitalWrite(blueLedPin, LOW);
  digitalWrite(redLedPin, LOW);
  digitalWrite(buzzerPin, LOW);
}