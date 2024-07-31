#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#include <Arduino.h>

// 핀 설정
const int leftMotorPin = 3;      // 왼쪽 모터 핀
const int rightMotorPin = 5;     // 오른쪽 모터 핀
const int steeringMotorPin = 6;   // 조향 모터 핀
const int potPin = A0;            // 가변저항 핀

// 거리 센서 핀 배열
const int triggerPins[6] = {7, 8, 9, 10, 11, 12}; // 트리거 핀
const int echoPins[6] = {2, 3, 4, 5, 6, 7};       // 에코 핀

// 장애물 거리 임계값
const int OBSTACLE_DISTANCE_THRESHOLD = 20; // cm

// 신호등 상태 정의
enum TrafficLightState {
    RED,
    GREEN
};

// 함수 프로토타입
void handleSerialInput(); // 시리얼 입력 처리 함수
bool isObstacleDetected(); // 장애물 감지 함수
void handleObstacle();     // 장애물 처리 함수
void reverse();           // 후진 함수
void changeLane(String direction); // 차선 변경 함수
int getDistance(int triggerPin, int echoPin); // 거리 측정 함수
void followLane(int steeringAngle); // 차선 추적 함수
void moveForward();       // 전진 함수
void stopMotors();        // 정지 함수

#endif // LANEDETECTION_H
