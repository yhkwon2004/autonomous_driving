#include <Arduino.h>
#include "LaneDetection.h"

// PID 제어 변수 초기화
double Setpoint = 0, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 변수 선언 초기화
bool changingLane = false; 
int currentLane = 2; 
String autoMode = "N"; 
int potValue = 0; 

TrafficLightState trafficLight = RED; // 신호등 상태 변수 초기화

void setup() {
    Serial.begin(9600); 
    pinMode(leftMotorPin, OUTPUT);
    pinMode(rightMotorPin, OUTPUT);
    pinMode(steeringMotorPin, OUTPUT);
    
    for (int i = 0; i < 6; i++) {
        pinMode(triggerPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }
    
    pinMode(potPin, INPUT);
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC);
}

void loop() {
    handleSerialInput();
    
    if (trafficLight == RED) {
        stopMotors();
    } else if (trafficLight == GREEN) {
        potValue = analogRead(potPin);
        Input = map(potValue, 0, 1023, 0, 255);

        if (isObstacleDetected()) {
            handleObstacle();
        } else {
            myPID.Compute();  
            int steeringOutput = map(Output * 6, -20, 20, 0, 255);
            analogWrite(steeringMotorPin, constrain(steeringOutput, 0, 255));
            moveForward();
        }
    }
}

// 장애물 감지 함수
bool isObstacleDetected() {
    // 앞 범퍼의 센서 우선 순위
    for (int i = 0; i < 3; i++) {
        int distance = getDistance(triggerPins[i], echoPins[i]); // 거리 측정
        if (distance < OBSTACLE_DISTANCE_THRESHOLD) {
            return true; // 장애물 감지
        }
    }
    
    // 사이드 센서 체크
    for (int i = 3; i < 6; i++) {
        int distance = getDistance(triggerPins[i], echoPins[i]); // 거리 측정
        if (distance < OBSTACLE_DISTANCE_THRESHOLD) {
            return true; // 장애물 감지
        }
    }
    
    return false; // 장애물 미감지
}

// 장애물 처리 함수
void handleObstacle() {
    if (!changingLane) { // 차선 변경 중이 아닐 때
        changingLane = true; // 차선 변경 시작

        reverse(); // 장애물 감지 시 후진

        // 현재 차선에서 장애물 회피를 위한 차선 변경
        if (currentLane == 2) {
            changeLane("left"); // 2차선에서 장애물 감지 시 1차선으로 변경
        } else if (currentLane == 1) {
            changeLane("right"); // 1차선에서 장애물 감지 시 2차선으로 변경
        }

        // 장애물 회피 후 다시 원래 차선으로 돌아가기
        delay(1000); // 회피 후 잠시 대기
        if (currentLane == 1) {
            changeLane("right"); // 1차선에서 원래 차선으로 복귀
        } else if (currentLane == 2) {
            changeLane("left"); // 2차선에서 원래 차선으로 복귀
        }

        changingLane = false; // 차선 변경 상태 종료
    }
}

// 후진 함수
void reverse() {
    digitalWrite(leftMotorPin, LOW); // 왼쪽 모터 정지
    digitalWrite(rightMotorPin, LOW); // 오른쪽 모터 정지
    delay(500); // 후진 시간 조정 (0.5초 후진)
    moveForward(); // 후진 후 다시 전진
}

// 차선 변경 함수
void changeLane(String direction) {
    if (direction == "left" && currentLane == 2) {
        // 왼쪽으로 회전
        analogWrite(steeringMotorPin, map(-20 * 6, -20, 20, 0, 255)); // 조향 값 설정
        delay(500); // 회전 시간 조정
        currentLane = 1; // 차선 업데이트
    } else if (direction == "right" && currentLane == 1) {
        // 오른쪽으로 회전
        analogWrite(steeringMotorPin, map(20 * 6, -20, 20, 0, 255)); // 조향 값 설정
        delay(500); // 회전 시간 조정
        currentLane = 2; // 차선 업데이트
    }
}

// 거리 측정
int getDistance(int triggerPin, int echoPin) {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2); // 2 마이크로초 대기
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10); // 10 마이크로초 대기
    digitalWrite(triggerPin, LOW);

    long duration = pulseIn(echoPin, HIGH); // 에코 핀에서 HIGH 상태의 지속 시간 측정
    return duration * 0.034 / 2; // cm 단위로 변환하여 거리 반환
}

// 시리얼 입력 처리
void handleSerialInput() {
    if (Serial.available() > 0) { // 시리얼 데이터가 있을 경우
        String command = Serial.readStringUntil('\n'); // 시리얼로부터 명령어 수신
        int separatorIndex = command.indexOf(','); // ','의 인덱스 찾기

        if (separatorIndex != -1) {
            int lane = command.substring(0, separatorIndex).toInt(); // 차선 정보 추출
            int steeringAngle = command.substring(separatorIndex + 1).toInt(); // 조향 각도 추출

            // 신호등 색상 업데이트
            if (command.length() > separatorIndex + 1) {
                trafficLight = command.substring(separatorIndex + 1); // 신호등 색상 업데이트
            }

            // 차선 변경 및 조향 각도 설정
            if (lane == 1 && currentLane == 2) {
                changeLane("left"); // 2차선에서 1차선으로 변경
                Serial.println("left");
            } else if (lane == 2 && currentLane == 1) {
                changeLane("right"); // 1차선에서 2차선으로 변경
                Serial.println("right");
            } else {
                followLane(steeringAngle); // 차선 추적
                Serial.println("following lane");
            }
        }
    }
}

// 차선 추적
void followLane(int steeringAngle) {
    // 조향 각도를 조정하여 차선을 따라가도록 설정
    int steeringOutput = map(steeringAngle, -20, 20, 0, 255); // 조향 각도를 모터에 맞게 변환
    analogWrite(steeringMotorPin, constrain(steeringOutput, 0, 255)); // 조향 모터에 값 출력
    moveForward(); // 전진
}

// 전진
void moveForward() {
    digitalWrite(leftMotorPin, HIGH);
    digitalWrite(rightMotorPin, HIGH);
}

// 정지
void stopMotors() {
    digitalWrite(leftMotorPin, LOW);
    digitalWrite(rightMotorPin, LOW);
}

