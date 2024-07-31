/*
// 1. 비례 상수 (Kp) 조정
- Kp 값을 높이면 현재 오차에 대한 반응이 빨라져 차량의 반응성이 향상됨
- 그러나 Kp가 너무 높으면 오버슛이나 진동이 발생할 수 있으므로 주의해야 함.

// 2. 적분 상수 (Ki) 조정
- Ki 값을 조정하여 지속적인 오차를 보정
- Ki가 너무 높으면 시스템이 불안정해지고, 너무 낮으면 오차가 누적되어 목표에 도달하는 데 시간이 걸림.

// 3. 미분 상수 (Kd) 조정
- Kd 값을 통해 오차 변화율에 반응하도록 조정
- Kd가 적절하게 설정되면 반응을 더욱 부드럽게 만들 수 있지만, 너무 높으면 불안정성이 증가할 수 있음.

// 4. Ziegler-Nichols
- Kp를 점진적으로 증가시켜 시스템이 지속적으로 진동할 때의 값을 기록
- 이 값을 기준으로 Kp, Ki, Kd를 다음과 같이 설정:  Kp = 0.6 * Kp(진동 발생시), Ki = 2 * Kp / P, Kd = Kp * P / 8
- 여기서 P는 진동 주기

*/ //pid관련
//https://capitalists.tistory.com/239 자세한 내용 참고
#include <Arduino.h>
#include <PID_v1.h> // PID 라이브러리 추가

// PID 제어 변수 초기화
double Setpoint;    // 목표값 (가변 저항의 초기값)
double Input;       // 현재 입력값 (가변 저항에서 읽어온 값)
double Output;      // PID 제어의 출력값
double Kp = 2;      // 비례 상수
double Ki = 5;      // 적분 상수
double Kd = 1;      // 미분 상수

// PID 객체 생성: 입력, 출력, 목표값, 비례, 적분, 미분 상수, 제어 방향
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 핀 번호 설정
const int potPin = A0;             // 가변 저항이 연결된 아날로그 핀

// 모터 제어 핀 설정
const int leftMotorPin1 = 4;      // 왼쪽 모터 IN1
const int leftMotorPin2 = 5;      // 왼쪽 모터 IN2
const int rightMotorPin1 = 6;     // 오른쪽 모터 IN1
const int rightMotorPin2 = 7;     // 오른쪽 모터 IN2
const int steeringMotorPin1 = 8;  // 조향 모터 IN1
const int steeringMotorPin2 = 9;  // 조향 모터 IN2
const int rearMotorPin1 = 10;     // 후륜 모터 IN1
const int rearMotorPin2 = 11;     // 후륜 모터 IN2

void setup() {
    Serial.begin(9600);

    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);
    pinMode(steeringMotorPin1, OUTPUT);
    pinMode(steeringMotorPin2, OUTPUT);
    pinMode(rearMotorPin1, OUTPUT);
    pinMode(rearMotorPin2, OUTPUT);

    myPID.SetMode(AUTOMATIC); // PID 제어 모드를 자동으로 설정
}

void loop() {
    // 시리얼 통신을 통해 목표값 수신
    if (Serial.available() > 0) {
        Setpoint = Serial.parseFloat(); // 목표값을 시리얼로부터 읽어옴
        Setpoint = constrain(Setpoint, 0, 255); // 목표값을 0-255 범위로 제한
    }

    // 현재 입력값 읽기 (가변 저항)
    int potValue = analogRead(potPin); 
    Input = map(potValue, 0, 1023, 0, 255); // 아날로그 값을 0-255 범위로 변환
    myPID.Compute(); // PID 계산 수행

    // PID 출력값을 조향 모터에 전달
    analogWrite(steeringMotorPin1, Output); 
    analogWrite(steeringMotorPin2, Output); 

    // 후륜 모터 속도 조정 (PID 출력에 따라)
    int rearMotorSpeed = map(Output, 0, 255, 0, 255); // PID 출력값을 후륜 모터 속도로 변환
    analogWrite(rearMotorPin1, rearMotorSpeed); 
    analogWrite(rearMotorPin2, LOW);
    moveForward(); // 전진 함수 호출
}

// 전진
void moveForward() {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
}

// 정지
void stopMotors() {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
    digitalWrite(rearMotorPin1, LOW);
    digitalWrite(rearMotorPin2, LOW);
}
