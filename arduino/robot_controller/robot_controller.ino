#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// ==================================================
// L298N CONNECTIONS
// ==================================================

const uint8_t ENA = 6;
const uint8_t ENB = 5;

const uint8_t IN1 = 8;
const uint8_t IN2 = 9;
const uint8_t IN3 = 10;
const uint8_t IN4 = 11;

// ==================================================
// ULTRASONIC CONNECTIONS
// ==================================================

const uint8_t TRIG_PIN = 2;
const uint8_t ECHO_PIN = 3;

// ==================================================
// MOTOR SETTINGS
// ==================================================

const uint8_t DRIVE_SPEED = 190;
const uint8_t PICKUP_SPEED = 190;

const uint8_t TURN_START_SPEED = 230;
const uint8_t TURN_SPEED = 210;
const uint8_t SLOW_TURN_SPEED = 205;

const unsigned long TURN_START_BOOST_MS = 100;
const float SLOW_TURN_ZONE_DEGREES = 7.0;

const unsigned long MAX_DRIVE_TIME_MS = 5000;
const unsigned long TURN_TIMEOUT_MS = 6000;

const float MIN_TURN_ANGLE = 1.0;
const float MAX_TURN_ANGLE = 180.0;

const int RIGHT_GYRO_DIRECTION = 1;
const float TURN_ACCEPTANCE_DEGREES = 1.2;

const float MIN_TURN_PROGRESS_CHANGE = 0.08;
const unsigned long TURN_STALL_TIMEOUT_MS = 350;

// ==================================================
// PICKUP POSITIONING SETTINGS
// ==================================================

const float PICKUP_TARGET_CM = 15.0;
const float PICKUP_TOLERANCE_CM = 1.0;

const float MIN_VALID_DISTANCE_CM = 2.0;
const float MAX_VALID_DISTANCE_CM = 200.0;
const float EMERGENCY_MIN_DISTANCE_CM = 5.0;

const unsigned long FAR_FORWARD_PULSE_MS = 60;
const unsigned long NEAR_FORWARD_PULSE_MS = 25;
const unsigned long FAR_BACKWARD_PULSE_MS = 50;
const unsigned long NEAR_BACKWARD_PULSE_MS = 25;

const float LARGE_POSITION_ERROR_CM = 4.0;

const uint8_t REQUIRED_STABLE_READINGS = 5;
const uint8_t MAX_INVALID_READINGS = 5;

const unsigned long PICKUP_TIMEOUT_MS = 10000;
const unsigned long PULSE_SETTLE_MS = 100;

const float MAX_PICKUP_START_DISTANCE_CM = 40.0;
const uint8_t REQUIRED_INITIAL_READINGS = 3;
const float MAX_INITIAL_READING_SPREAD_CM = 4.0;

const float MAX_TRACKING_DISTANCE_CM = 35.0;
const float MAX_DISTANCE_JUMP_CM = 8.0;
const uint8_t MAX_TRACKING_FAILURES = 5;

// ==================================================
// MPU6050
// ==================================================

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float gyroZBias = 0.0;

// ==================================================
// MOTOR FUNCTIONS
// ==================================================

void setMotorSpeed(
  uint8_t leftSpeed,
  uint8_t rightSpeed
)
{
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void stopMotors()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setForwardDirection()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void setBackwardDirection()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveForward()
{
  setForwardDirection();
  setMotorSpeed(DRIVE_SPEED, DRIVE_SPEED);
}

void moveBackward()
{
  setBackwardDirection();
  setMotorSpeed(DRIVE_SPEED, DRIVE_SPEED);
}

void moveForwardAtSpeed(uint8_t speedValue)
{
  setForwardDirection();
  setMotorSpeed(speedValue, speedValue);
}

void moveBackwardAtSpeed(uint8_t speedValue)
{
  setBackwardDirection();
  setMotorSpeed(speedValue, speedValue);
}

void setRightTurnDirection()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void setLeftTurnDirection()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// ==================================================
// ULTRASONIC FUNCTIONS
// ==================================================

float readUltrasonicDistanceCM()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(
    ECHO_PIN,
    HIGH,
    25000UL
  );

  if (duration == 0)
  {
    return -1.0;
  }

  float distanceCM =
      duration * 0.0343f / 2.0f;

  if (
    distanceCM < MIN_VALID_DISTANCE_CM ||
    distanceCM > MAX_VALID_DISTANCE_CM
  )
  {
    return -1.0;
  }

  return distanceCM;
}

float readFilteredDistanceCM()
{
  const uint8_t SAMPLE_COUNT = 5;

  float samples[SAMPLE_COUNT];
  uint8_t validCount = 0;

  for (uint8_t i = 0; i < SAMPLE_COUNT; i++)
  {
    float distanceCM =
        readUltrasonicDistanceCM();

    if (distanceCM > 0.0)
    {
      samples[validCount] = distanceCM;
      validCount++;
    }

    delay(5);
  }

  if (validCount == 0)
  {
    return -1.0;
  }

  for (uint8_t i = 0; i < validCount; i++)
  {
    for (
      uint8_t j = i + 1;
      j < validCount;
      j++
    )
    {
      if (samples[j] < samples[i])
      {
        float temporary = samples[i];
        samples[i] = samples[j];
        samples[j] = temporary;
      }
    }
  }

  return samples[validCount / 2];
}

bool emergencyStopRequested()
{
  if (Serial.available() <= 0)
  {
    return false;
  }

  String incoming =
      Serial.readStringUntil('\n');

  incoming.trim();

  if (incoming == "STOP")
  {
    stopMotors();
    Serial.println(F("STOPPED"));
    return true;
  }

  Serial.print(F("ERROR,BUSY_POSITIONING,"));
  Serial.println(incoming);

  return false;
}

bool runPickupPulse(
  bool forward,
  unsigned long requestedDurationMs
)
{
  unsigned long startMillis = millis();

  if (forward)
  {
    moveForwardAtSpeed(PICKUP_SPEED);
  }
  else
  {
    moveBackwardAtSpeed(PICKUP_SPEED);
  }

  while (
    millis() - startMillis <
    requestedDurationMs
  )
  {
    if (emergencyStopRequested())
    {
      stopMotors();
      return false;
    }

    delay(2);
  }

  stopMotors();

  unsigned long actualDurationMs =
      millis() - startMillis;

  if (forward)
  {
    Serial.print(
      F("FORWARD_PULSE_DONE,DURATION_MS=")
    );
  }
  else
  {
    Serial.print(
      F("BACKWARD_PULSE_DONE,DURATION_MS=")
    );
  }

  Serial.println(actualDurationMs);

  delay(PULSE_SETTLE_MS);

  return true;
}

bool confirmPickupTarget()
{
  float initialDistances[
    REQUIRED_INITIAL_READINGS
  ];

  for (
    uint8_t i = 0;
    i < REQUIRED_INITIAL_READINGS;
    i++
  )
  {
    if (emergencyStopRequested())
    {
      return false;
    }

    initialDistances[i] =
        readFilteredDistanceCM();

    Serial.print(
      F("ULTRASONIC_INITIAL,DISTANCE_CM=")
    );
    Serial.println(initialDistances[i], 1);

    if (
      initialDistances[i] < 0.0 ||
      initialDistances[i] >
          MAX_PICKUP_START_DISTANCE_CM
    )
    {
      stopMotors();

      Serial.println(
        F(
          "COMMAND_ABORTED,POSITION_FOR_PICKUP,"
          "REASON=ULTRASONIC_TARGET_NOT_CONFIRMED"
        )
      );

      return false;
    }

    delay(40);
  }

  float minimumDistance =
      initialDistances[0];

  float maximumDistance =
      initialDistances[0];

  for (
    uint8_t i = 1;
    i < REQUIRED_INITIAL_READINGS;
    i++
  )
  {
    if (
      initialDistances[i] <
      minimumDistance
    )
    {
      minimumDistance =
          initialDistances[i];
    }

    if (
      initialDistances[i] >
      maximumDistance
    )
    {
      maximumDistance =
          initialDistances[i];
    }
  }

  if (
    maximumDistance - minimumDistance >
    MAX_INITIAL_READING_SPREAD_CM
  )
  {
    stopMotors();

    Serial.println(
      F(
        "COMMAND_ABORTED,POSITION_FOR_PICKUP,"
        "REASON=ULTRASONIC_READING_UNSTABLE"
      )
    );

    return false;
  }

  Serial.println(
    F("ULTRASONIC_TARGET_CONFIRMED")
  );

  return true;
}

void positionForPickup()
{
  Serial.println(
    F("COMMAND_STARTED,POSITION_FOR_PICKUP")
  );

  stopMotors();

  if (!confirmPickupTarget())
  {
    return;
  }

  unsigned long startMillis = millis();
  uint8_t stableReadingCount = 0;
  uint8_t invalidReadingCount = 0;
  uint8_t trackingFailureCount = 0;

  float lastTrustedDistanceCM =
      readFilteredDistanceCM();

  if (
    lastTrustedDistanceCM < 0.0 ||
    lastTrustedDistanceCM >
        MAX_TRACKING_DISTANCE_CM
  )
  {
    stopMotors();

    Serial.println(
      F(
        "COMMAND_ABORTED,POSITION_FOR_PICKUP,"
        "REASON=ULTRASONIC_TARGET_LOST"
      )
    );

    return;
  }

  while (
    millis() - startMillis <
    PICKUP_TIMEOUT_MS
  )
  {
    if (emergencyStopRequested())
    {
      Serial.println(
        F(
          "COMMAND_ABORTED,POSITION_FOR_PICKUP,"
          "REASON=STOP"
        )
      );
      return;
    }

    float distanceCM =
        readFilteredDistanceCM();

    if (distanceCM < 0.0)
    {
      stopMotors();
      stableReadingCount = 0;
      invalidReadingCount++;

      Serial.print(
        F("ULTRASONIC_INVALID,COUNT=")
      );
      Serial.println(invalidReadingCount);

      if (
        invalidReadingCount >=
        MAX_INVALID_READINGS
      )
      {
        Serial.println(
          F(
            "COMMAND_ABORTED,POSITION_FOR_PICKUP,"
            "REASON=ULTRASONIC_NO_READING"
          )
        );
        return;
      }

      delay(PULSE_SETTLE_MS);
      continue;
    }

    invalidReadingCount = 0;

    Serial.print(
      F("ULTRASONIC,DISTANCE_CM=")
    );
    Serial.println(distanceCM, 1);

    bool distanceTooFar =
        distanceCM >
        MAX_TRACKING_DISTANCE_CM;

    bool distanceJumped =
        abs(
          distanceCM -
          lastTrustedDistanceCM
        ) >
        MAX_DISTANCE_JUMP_CM;

    if (
      distanceTooFar ||
      distanceJumped
    )
    {
      stopMotors();
      stableReadingCount = 0;
      trackingFailureCount++;

      Serial.print(
        F(
          "ULTRASONIC_TRACKING_REJECTED,"
          "DISTANCE_CM="
        )
      );
      Serial.print(distanceCM, 1);

      Serial.print(
        F(",LAST_TRUSTED_CM=")
      );
      Serial.println(
        lastTrustedDistanceCM,
        1
      );

      if (
        trackingFailureCount >=
        MAX_TRACKING_FAILURES
      )
      {
        Serial.println(
          F(
            "COMMAND_ABORTED,POSITION_FOR_PICKUP,"
            "REASON=ULTRASONIC_TARGET_LOST"
          )
        );

        return;
      }

      delay(PULSE_SETTLE_MS);
      continue;
    }

    trackingFailureCount = 0;
    lastTrustedDistanceCM = distanceCM;

    if (
      distanceCM <=
      EMERGENCY_MIN_DISTANCE_CM
    )
    {
      stopMotors();

      Serial.print(
        F(
          "ERROR,OBJECT_TOO_CLOSE,"
          "DISTANCE_CM="
        )
      );
      Serial.println(distanceCM, 1);

      return;
    }

    float distanceErrorCM =
        distanceCM - PICKUP_TARGET_CM;

    if (
      abs(distanceErrorCM) <=
      PICKUP_TOLERANCE_CM
    )
    {
      stopMotors();
      stableReadingCount++;

      Serial.print(
        F("PICKUP_STABLE_READING,COUNT=")
      );
      Serial.print(stableReadingCount);

      Serial.print(F(",DISTANCE_CM="));
      Serial.println(distanceCM, 1);

      if (
        stableReadingCount >=
        REQUIRED_STABLE_READINGS
      )
      {
        Serial.print(
          F(
            "PICKUP_POSITION_REACHED,"
            "DISTANCE_CM="
          )
        );
        Serial.println(distanceCM, 1);

        Serial.println(
          F("COMMAND_DONE,POSITION_FOR_PICKUP")
        );

        return;
      }

      delay(PULSE_SETTLE_MS);
      continue;
    }

    stableReadingCount = 0;

    if (distanceErrorCM > 0.0)
    {
      unsigned long pulseDurationMs =
          (
            distanceErrorCM >
            LARGE_POSITION_ERROR_CM
          )
          ? FAR_FORWARD_PULSE_MS
          : NEAR_FORWARD_PULSE_MS;

      if (
        !runPickupPulse(
          true,
          pulseDurationMs
        )
      )
      {
        Serial.println(
          F(
            "COMMAND_ABORTED,POSITION_FOR_PICKUP,"
            "REASON=STOP"
          )
        );
        return;
      }
    }
    else
    {
      unsigned long pulseDurationMs =
          (
            -distanceErrorCM >
            LARGE_POSITION_ERROR_CM
          )
          ? FAR_BACKWARD_PULSE_MS
          : NEAR_BACKWARD_PULSE_MS;

      if (
        !runPickupPulse(
          false,
          pulseDurationMs
        )
      )
      {
        Serial.println(
          F(
            "COMMAND_ABORTED,POSITION_FOR_PICKUP,"
            "REASON=STOP"
          )
        );
        return;
      }
    }
  }

  stopMotors();
  Serial.println(
    F(
      "COMMAND_ABORTED,POSITION_FOR_PICKUP,"
      "REASON=PICKUP_POSITION_TIMEOUT"
    )
  );
}


void reportFilteredDistance()
{
  stopMotors();

  float distanceCM =
      readFilteredDistanceCM();

  if (distanceCM < 0.0)
  {
    Serial.println(F("DISTANCE_INVALID"));
    return;
  }

  Serial.print(F("DISTANCE_CM="));
  Serial.println(distanceCM, 1);
}

// ==================================================
// IMU FUNCTIONS
// ==================================================

void printImuReadings()
{
  mpu.getMotion6(
    &ax, &ay, &az,
    &gx, &gy, &gz
  );

  Serial.print(F("IMU,AX="));
  Serial.print(ax);

  Serial.print(F(",AY="));
  Serial.print(ay);

  Serial.print(F(",AZ="));
  Serial.print(az);

  Serial.print(F(",GX="));
  Serial.print(gx);

  Serial.print(F(",GY="));
  Serial.print(gy);

  Serial.print(F(",GZ="));
  Serial.println(gz);
}

void calibrateGyro()
{
  const int sampleCount = 500;
  int32_t totalGz = 0;

  stopMotors();

  Serial.println(F("CALIBRATING_IMU"));
  Serial.println(F("KEEP_ROBOT_STILL"));

  delay(1000);

  for (int i = 0; i < sampleCount; i++)
  {
    mpu.getRotation(
      &gx,
      &gy,
      &gz
    );

    totalGz += gz;

    delay(3);
  }

  gyroZBias =
      static_cast<float>(totalGz)
      / sampleCount;

  Serial.print(F("GYRO_Z_BIAS="));
  Serial.println(gyroZBias, 2);

  Serial.println(F("IMU_CALIBRATION_DONE"));
}

// ==================================================
// TIMED FORWARD/BACKWARD MOVEMENT
// ==================================================

void runTimedMovement(
  bool forward,
  unsigned long durationMs
)
{
  if (
    durationMs == 0 ||
    durationMs > MAX_DRIVE_TIME_MS
  )
  {
    Serial.println(F("ERROR,INVALID_DURATION"));
    return;
  }

  if (forward)
  {
    Serial.print(
      F("COMMAND_STARTED,FORWARD,DURATION_MS=")
    );
    Serial.println(durationMs);

    moveForward();
  }
  else
  {
    Serial.print(
      F("COMMAND_STARTED,BACKWARD,DURATION_MS=")
    );
    Serial.println(durationMs);

    moveBackward();
  }

  delay(durationMs);

  stopMotors();

  if (forward)
  {
    Serial.print(
      F("COMMAND_DONE,FORWARD,DURATION_MS=")
    );
  }
  else
  {
    Serial.print(
      F("COMMAND_DONE,BACKWARD,DURATION_MS=")
    );
  }

  Serial.println(durationMs);
}

// ==================================================
// IMU-BASED TURN
// ==================================================

float turnByAngle(
  bool turnRight,
  float targetAngle
)


{
  if (
    targetAngle < MIN_TURN_ANGLE ||
    targetAngle > MAX_TURN_ANGLE
  )
  {
    Serial.println(F("ERROR,INVALID_ANGLE"));
    return -1.0;
  }

  float signedAngle = 0.0;
  float progressAngle = 0.0;

  float lastProgressAngle = 0.0;
  unsigned long lastMovementMillis = millis();

  if (turnRight)
  {
    Serial.print(
      F(
        "COMMAND_STARTED,TURN_RIGHT,"
        "TARGET_ANGLE="
      )
    );
    Serial.println(targetAngle, 2);

    setRightTurnDirection();
  }
  else
  {
    Serial.print(
      F(
        "COMMAND_STARTED,TURN_LEFT,"
        "TARGET_ANGLE="
      )
    );
    Serial.println(targetAngle, 2);

    setLeftTurnDirection();
  }

  setMotorSpeed(
    TURN_START_SPEED,
    TURN_START_SPEED
  );

  unsigned long turnStartMillis = millis();
  unsigned long previousMicros = micros();
  unsigned long previousPrintMillis = 0;

  while (progressAngle < targetAngle)
  {
    unsigned long currentMicros = micros();
    unsigned long elapsedMillis =
        millis() - turnStartMillis;

    float deltaTimeSeconds =
        (currentMicros - previousMicros)
        / 1000000.0;

    previousMicros = currentMicros;

    mpu.getMotion6(
      &ax, &ay, &az,
      &gx, &gy, &gz
    );

    float gyroZDps =
        (
          static_cast<float>(gz)
          - gyroZBias
        ) / 131.0;

    gyroZDps *= RIGHT_GYRO_DIRECTION;

    signedAngle +=
        gyroZDps * deltaTimeSeconds;

    if (turnRight)
    {
      progressAngle = signedAngle;
    }
    else
    {
      progressAngle = -signedAngle;
    }

    if (progressAngle < 0.0)
    {
      progressAngle = 0.0;
    }

    if (
      progressAngle - lastProgressAngle
      >= MIN_TURN_PROGRESS_CHANGE
    )
    {
      lastProgressAngle = progressAngle;
      lastMovementMillis = millis();
    }

    if (elapsedMillis < TURN_START_BOOST_MS)
    {
      setMotorSpeed(
        TURN_START_SPEED,
        TURN_START_SPEED
      );
    }
    else if (
      targetAngle >= 15.0 &&
      progressAngle >=
          targetAngle - SLOW_TURN_ZONE_DEGREES
    )
    {
      setMotorSpeed(
        SLOW_TURN_SPEED,
        SLOW_TURN_SPEED
      );
    }
    else
    {
      setMotorSpeed(
        TURN_SPEED,
        TURN_SPEED
      );
    }

    if (
      millis() - previousPrintMillis
      >= 50
    )
    {
      Serial.print(F("TURN_PROGRESS="));
      Serial.print(progressAngle, 2);

      Serial.print(F(",SIGNED_ANGLE="));
      Serial.print(signedAngle, 2);

      Serial.print(F(",GYRO_Z_DPS="));
      Serial.println(gyroZDps, 2);

      previousPrintMillis = millis();
    }

    if (
      millis() - lastMovementMillis
      >= TURN_STALL_TIMEOUT_MS
    )
    {
      stopMotors();

      float remainingAngle =
          targetAngle - progressAngle;

      if (
        remainingAngle >= 0.0 &&
        remainingAngle <=
        TURN_ACCEPTANCE_DEGREES
      )
      {
        Serial.println(
          F(
            "WARNING,"
            "TURN_ACCEPTED_AFTER_STALL"
          )
        );

        return progressAngle;
      }

      Serial.println(F("ERROR,TURN_STALLED"));
      return -1.0;
    }

    if (
      millis() - turnStartMillis
      >= TURN_TIMEOUT_MS
    )
    {
      stopMotors();

      float remainingAngle =
          targetAngle - progressAngle;

      if (
        remainingAngle >= 0.0 &&
        remainingAngle <=
        TURN_ACCEPTANCE_DEGREES
      )
      {
        Serial.println(
          F("WARNING,TURN_ACCEPTED_NEAR_TARGET")
        );

        return progressAngle;
      }

      Serial.println(F("ERROR,TURN_TIMEOUT"));
      return -1.0;
    }

    delay(2);
  }

  stopMotors();
  return progressAngle;
}

// ==================================================
// COMMAND PROCESSING
// ==================================================

void processCommand(String command)
{
  command.trim();

  if (command == "PING")
  {
    Serial.println(F("PONG"));
    return;
  }

  if (command == "STOP")
  {
    stopMotors();
    Serial.println(F("STOPPED"));
    return;
  }

  if (command == "READ_IMU")
  {
    printImuReadings();
    return;
  }

  if (command == "CALIBRATE_IMU")
  {
    calibrateGyro();
    return;
  }

  if (command == "READ_DISTANCE")
  {
    reportFilteredDistance();
    return;
  }

  if (command == "POSITION_FOR_PICKUP")
  {
    positionForPickup();
    return;
  }

  if (command.startsWith("FORWARD "))
  {
    unsigned long durationMs =
        command.substring(8).toInt();

    runTimedMovement(
      true,
      durationMs
    );

    return;
  }

  if (command.startsWith("BACKWARD "))
  {
    unsigned long durationMs =
        command.substring(9).toInt();

    runTimedMovement(
      false,
      durationMs
    );

    return;
  }

  if (command.startsWith("TURN_RIGHT "))
  {
    float requestedAngle =
        command.substring(11).toFloat();

    float actualAngle =
        turnByAngle(
          true,
          requestedAngle
        );

    if (actualAngle >= 0.0)
    {
      Serial.print(
        F(
          "COMMAND_DONE,TURN_RIGHT,"
          "FINAL_ANGLE="
        )
      );
      Serial.println(actualAngle, 2);
    }

    return;
  }

  if (command.startsWith("TURN_LEFT "))
  {
    float requestedAngle =
        command.substring(10).toFloat();

    float actualAngle =
        turnByAngle(
          false,
          requestedAngle
        );

    if (actualAngle >= 0.0)
    {
      Serial.print(
        F(
          "COMMAND_DONE,TURN_LEFT,"
          "FINAL_ANGLE="
        )
      );
      Serial.println(actualAngle, 2);
    }

    return;
  }

  Serial.print(F("ERROR,UNKNOWN_COMMAND,"));
  Serial.println(command);
}

// ==================================================
// SETUP
// ==================================================

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  stopMotors();

  Serial.begin(115200);
  Serial.setTimeout(100);
  Wire.begin();

  mpu.initialize();

  delay(1000);

  Serial.println(
    F("MPU6050_READINGS_AVAILABLE")
  );

  printImuReadings();

  calibrateGyro();

  Serial.println(F("ARDUINO_READY"));
}

// ==================================================
// MAIN LOOP
// ==================================================

void loop()
{
  if (Serial.available() > 0)
  {
    String command =
        Serial.readStringUntil('\n');

    processCommand(command);
  }
}