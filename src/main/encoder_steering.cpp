// Steering with feedback from encoders
#include "encoder_steering.h"

// Declare servo motor objects
Servo servo_1;
Servo servo_2;

// Initialize PID
PID pid(50.0, 0, 0);
TaskHandle_t _servoControlTask = NULL;
QueueHandle_t steeringQueue = NULL;

#define SERVO_CONTROL_PERIOD 10 // ms
volatile int PID_Control_Period = SERVO_CONTROL_PERIOD;
bool servo_enable = false;

void steering_setup(double acquireRate, double kp, double ki, double kd) {

  // Initialize encoders
  Serial.println("Initializing encoders");
  pinMode(ENC_1, INPUT);
  attachInterrupt(ENC_1, enc_1_isr, RISING);
  pinMode(ENC_2, INPUT);
  attachInterrupt(ENC_2, enc_2_isr, RISING);

  // Initialize PID
  Serial.println("Initializing PID");
  Serial.printf("kp: %lf, ki: %lf, kd: %lf\n", kp, ki, kd);
  pid.updateCoeffs(kp, ki, kd);
  pid.setOutputBounds(-150, 150); // Limit to 150 mm extension/retraction
  pid.setSampleTime((1.0/acquireRate)*1000); // ms
  PID_Control_Period = (1.0/acquireRate)*1000;
  // pid.setDeadband(DIST_PER_TICK);

  // Create queue
  steeringQueue = xQueueCreate(1, sizeof(AngleData));  // this queue will take the yaw value
  if (steeringQueue == NULL) {
    Serial.println("Failed to create steering queue");
    while(1){
      Serial.println("Failed to create steering queue");
      delay(500);
    } // if queue creation fails
  }
  Serial.println("Steering queue created");
  
  // Start task
  BaseType_t ret = xTaskCreatePinnedToCore(
    servoControlTask,       // Task function
    "ServoControlTask",     // Task name
    4096,                   // Stack size (words)
    NULL,                   // Task parameters
    configMAX_PRIORITIES -1,// Priority (1 is default)
    &_servoControlTask,     // Task handle
    0                       // Core number (0 or 1)
  );
  if (ret != pdPASS) {
    Serial.println("Failed to create servo control task");
  } else {
    Serial.println("Servo control task created");
  }
}


void init_servos() {
  // Initialize servos
  Serial.println("Initializing servos");
  servo_1.attach(SERVO_1);
  servo_1.setPeriodHertz(SERVO_FREQ);
  servo_2.attach(SERVO_2);
  servo_2.setPeriodHertz(SERVO_FREQ);
  // No spin
  servo_1.write(90);
  servo_2.write(90);
}


// Get difference between two angles in radians [-pi, pi)
double angle_diff(double angle1, double angle2) {
  double diff = angle1 - angle2;
  while (diff >= PI) {
    diff -= 2 * PI;
  }
  while (diff < -PI) {
    diff += 2 * PI;
  }
  return diff;
}


/* 
 * Function to control servos 
 * @param data: AngleData struct containing desired and current yaw values
 */
void servo_control(AngleData data) {

  if (data.desiredForward == -1){
    return;
  }

  if (!servo_enable) {
    servo_enable = true;
    init_servos();
  }

  SteeringData des_lengths;
  SteeringData current_lengths = {(double)DIST_PER_TICK*((int)count_1/2), (double)DIST_PER_TICK*((int)count_2/2)};

  double input = angle_diff(data.desiredYaw*0.01745329, data.currentYaw*0.01745329); // Convert to radians
  // Serial.printf("PID Input: %lf\n", input);   // Comment When testing is done
  if (fabs(input) < FORWARD_THRESH) { // Go forward if we don't need to change our heading that much
    des_lengths.l1 = -100.0; // Servo 1
    des_lengths.l2 = -100.0; // Servo 2
  }
  else {
    double output = pid.compute(0.0, input);
    // Serial.printf("PID Output: %lf\n", output);
    des_lengths.l1 = -output;
    des_lengths.l2 = output;
  }

  Serial.printf("Lengths: %lf %lf\n", des_lengths.l1, des_lengths.l2);    //- -- Comment me when testing is done

  // Servo 1
  if (des_lengths.l1 - current_lengths.l1 > (DIST_PER_TICK)) {
    servo_1.write(90 + SERVO_SPEED_VALUE_ANGLE);
    forward_1 = true;
  } else if (des_lengths.l1 - current_lengths.l1 < -(DIST_PER_TICK)) {
    servo_1.write(90 - SERVO_SPEED_VALUE_ANGLE);
    forward_1 = false;
  } else {
    servo_1.write(90);
  }

  // Servo 2
  if (des_lengths.l2 - current_lengths.l2 > (DIST_PER_TICK)) {
    servo_2.write(90 - SERVO_SPEED_VALUE_ANGLE);
    forward_2 = true;
  } else if (des_lengths.l2 - current_lengths.l2 < -(DIST_PER_TICK)) {
    servo_2.write(90 + SERVO_SPEED_VALUE_ANGLE);
    forward_2 = false;
  } else {
    servo_2.write(90);
  }

}


void servoControlTask(void *pvParameters) {
    (void) pvParameters;
    
    AngleData incomingData;
    // initialize the incoming data
    incomingData.desiredYaw = 0;
    incomingData.currentYaw = 0;
    incomingData.desiredForward = 0;

    while (true) {
        // Check if we have incoming data
        if (xQueueReceive(steeringQueue, &incomingData, 0) == pdTRUE) {
            // Successfully received data
            Serial.printf("Desired yaw: %lf, Current yaw: %lf, Desired forward: %lf\n", incomingData.desiredYaw, incomingData.currentYaw, incomingData.desiredForward);
        }
        // Call servo_control function here
        servo_control(incomingData); // Modify parameters as needed
        
        // Delay for the desired period (in milliseconds)
        vTaskDelay(pdMS_TO_TICKS(PID_Control_Period)); // Adjust delay as needed
    }
}

/*
* Function to send the steering data to the steering task
* @param data: AngleData struct containing desired and current yaw values
*/
void sendSteeringData(AngleData data) {
    if (xQueueOverwrite(steeringQueue, &data) != pdPASS) {
        Serial.println("Failed to overwrite data in steering queue");
    }
}

/*
 * Don't move the servos
 */
void do_nothing() {
  servo_1.write(90);
  servo_2.write(90);
}

