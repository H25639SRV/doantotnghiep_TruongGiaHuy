  #define CUSTOM_SETTINGS
  #define INCLUDE_GAMEPAD_MODULE
  #include <Dabble.h>
  #include <SoftwareSerial.h>
  #include <NewPing.h>
  #include <Servo.h>

  #define ULTRASONIC_SENSOR_TRIG 11
  #define ULTRASONIC_SENSOR_ECHO 13
  #define SAFE_DISTANCE 50
  #define MAX_DISTANCE 400
  #define SERVO_PIN 7 

  SoftwareSerial mySerial(2, 3); // RX, TX
  NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);
  Servo myServo;

  const int relay = 12;
  const int motorA1 = 5;
  const int motorA2 = 6;
  const int motorB1 = 9;
  const int motorB2 = 10;
  const int BTState = 8;

  int speed = 200;
  String state = "";
  bool autoMode = false;

  void setup() {
    Serial.begin(9600);
    mySerial.begin(9600);
    Dabble.begin(9600);

    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);
    pinMode(relay, OUTPUT);
    pinMode(BTState, INPUT);

    myServo.attach(SERVO_PIN);
    myServo.write(90);

    Serial.println("Bluetooth & Gamepad Ready!");
    stopMotors();
  }

  void loop() {
    if (mySerial.available()) {
      state = mySerial.readStringUntil('\n');
      state.trim();
      Serial.println("BT: " + state);
      handleBluetooth(state);
    } 
    else {
      Dabble.processInput();
      
      if (GamePad.isTrianglePressed()) {
        autoMode = true;
        Serial.println("Autonomous Mode ON");
      }
      
      if (GamePad.isCrossPressed()) {
        autoMode = false;
        stopMotors();
        Serial.println("Autonomous Mode OFF");
      }

      if (autoMode) {
        runAutonomousMode();
      } 
      else {
        handleManualControl();
      }
    }
  }

  void runAutonomousMode() {
    static unsigned long lastCheckTime = 0;
    const unsigned long checkInterval = 200;  // Kiểm tra mỗi 200ms

    if (millis() - lastCheckTime < checkInterval) return;
    lastCheckTime = millis();

    int distanceFront = readDistance(90);
    Serial.print("Trước: ");
    Serial.println(distanceFront);

    if (distanceFront > SAFE_DISTANCE) {
    moveForward(255);
  } else {
    stopMotors();
    Serial.println(">>> Phát hiện vật cản. Đang tìm đường đi...");
    
    //Đọc khoảng cách 2 bên
    int distanceLeft = readDistance(0);
    int distanceRight = readDistance(180);

    if (distanceLeft > SAFE_DISTANCE && distanceRight > SAFE_DISTANCE) {     // Nếu cả hai bên đều có thể đi, chọn hướng có khoảng cách lớn hơn
      if (distanceLeft > distanceRight) {
        Serial.println("Cả hai bên đều trống, ưu tiên rẽ trái.");
        moveForwardLeft(180);
        delay(500);
      } else {
        Serial.println("Cả hai bên đều trống, ưu tiên rẽ phải.");
        moveForwardRight(255);
        delay(500);
      }
    } 
    else if (distanceLeft > SAFE_DISTANCE) {
      Serial.println("Chỉ bên trái trống, rẽ trái.");
      moveForwardLeft(255);
      delay(500);
    } 
    else if (distanceRight > SAFE_DISTANCE) {
      Serial.println("Chỉ bên phải trống, rẽ phải.");
      moveForwardRight(255);
      delay(500);
    } 
    else {
      moveBackward(230);
      delay(700);
      turnRight(180);
      }
    }
  }


  // Hàm đo khoảng cách với góc servo
  int readDistance(int angle) {
    myServo.write(angle);  
    delay(500);  // Chờ servo ổn định
    int distance = mySensor.ping_cm();  

    if (distance == 0) {
      Serial.print("Góc ");
      Serial.print(angle);
      Serial.println("° - Không đo được!");
      return MAX_DISTANCE;
    }

    Serial.print("Góc ");
    Serial.print(angle);
    Serial.print("° - Khoảng cách: ");
    Serial.print(distance);
    Serial.println(" cm");

    return distance;
  }


  void handleManualControl() {
    if (GamePad.isUpPressed() && GamePad.isLeftPressed()) {
      moveForwardLeft(255);
    } 
    else if (GamePad.isUpPressed() && GamePad.isRightPressed()) {
      moveForwardRight(255);
    } 
    else if (GamePad.isDownPressed() && GamePad.isLeftPressed()) {
      moveBackwardLeft(255);
    } 
    else if (GamePad.isDownPressed() && GamePad.isRightPressed()) {
      moveBackwardRight(255);
    } 
    else if (GamePad.isUpPressed()) {
      moveForward(255);
    } 
    else if (GamePad.isDownPressed()) {
      moveBackward(255);
    } 
    else if (GamePad.isLeftPressed()) {
      turnLeft(180);
    } 
    else if (GamePad.isRightPressed()) {
      turnRight(180);
    } 
    else {
      stopMotors();
    }
  }

  void handleBluetooth(String command) {
    if (command == "F") moveForward(speed);
    else if (command == "B") moveBackward(speed);
    else if (command == "L") turnLeft(speed);
    else if (command == "R") turnRight(speed);
    else if (command == "FL") moveForwardLeft(speed);
    else if (command == "FR") moveForwardRight(speed);
    else if (command == "BL") moveBackwardLeft(speed);
    else if (command == "BR") moveBackwardRight(speed);
    else if (command == "S") stopMotors();
    else if (command == "X") digitalWrite(relay, HIGH);
    else if (command == "x") digitalWrite(relay, LOW);
  }

  // Điều khiển động cơ
  void moveForward(int speed) {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, speed);
    analogWrite(motorB1, speed);
    analogWrite(motorB2, 0);
    Serial.println("Đi thẳng");
  }

  void moveBackward(int speed) {
    analogWrite(motorA1, speed);
    analogWrite(motorA2, 0);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, speed);
    Serial.println("Đi lùi");
  }

  void turnRight(int speed) {
    analogWrite(motorA1, speed);
    analogWrite(motorA2, 0);
    analogWrite(motorB1, speed);
    analogWrite(motorB2, 0);
    Serial.println("Phải");
  }

  void turnLeft(int speed) {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, speed);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, speed);
    Serial.println("Trái");
  }

  void moveForwardLeft(int speed) {
    analogWrite(motorA1, speed);
    analogWrite(motorA2, 0);
    analogWrite(motorB1, speed / 2); // Giảm tốc bên phải
    analogWrite(motorB2, 0);
  }

  void moveForwardRight(int speed) {
    analogWrite(motorA1, speed / 2); // Giảm tốc bên trái
    analogWrite(motorA2, 0);
    analogWrite(motorB1, speed);
    analogWrite(motorB2, 0);
  }

  void moveBackwardLeft(int speed) {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, speed);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, speed / 2);
  }

  void moveBackwardRight(int speed) {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, speed / 2);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, speed);
  }

  void stopMotors() {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, 0);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, 0);
  }
