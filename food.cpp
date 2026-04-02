// C++ code
//

#include <Servo.h>

#define PRESSVALUE 700 // 압력 기준 값
#define WARPVALUE 1000 // 휨 기준 값

#define IDLE 0
#define CLOSED_DOOR 1
#define OPEN_DOOR 2

#define F_NOT 3
#define F_IN 4

#define MEASURING 5
#define OVER 6

#define TURN 7
#define STOP 8




Servo lidServo;

const int buttonServoPin = 2;
const int buttonMotorPin = 3;

const int servoPin = 9;
const int motorPin = 10;

bool servoState = false;
bool motorState = false;

int lastServoButton = HIGH;
int lastMotorButton = HIGH;

unsigned long openTime = 0;







// A0: 휨
// A1: 압력
// A2: 가스


// D2: 뚜껑 버튼
// D3: 모터 버튼

// D9: 뚜껑 제어
// D10: 모터 제어


// --: 버튼1(문)
// --: 버튼2(모터)

int flag_press = 0; // 가스 센서 작동 시작 선택
int flag_warp = 0; // 메인 통 끄는 것이 맞는 지 체크
int pressure = 0; //음식물 있으면 1, 없으면 0
int flag_gas = 0; // 가스 센서 부저 여부
//int pressureValue = 0; //압력값
//int flag_main = 0; // 메인 통 도는 지 체크


int mo_button_press = 0; // 모터 버튼 눌림 여부 체크
int door_button_press = 0; // 문 버튼 눌림 여부 체크.

int gas_cnt_check = 0; // 가스 부터 작동 후 시간 지난 것 체크
int cnt_opened = 0; // 열린 시간 counter

int sum = 0, i = 0; //warp 계산 용 변수


int state_DOOR = CLOSED_DOOR;
int state_GAS = IDLE;
int state_PRESS = F_NOT;
int state_MOTOR = STOP;
int state_WARP = IDLE;


int state_DOOR1 = CLOSED_DOOR;
int state_GAS1 = IDLE;
int state_PRESS1 = F_NOT;
int state_MOTOR1 = STOP;
int state_WARP1 = IDLE;



// =====================================
// 가스 센서 기반 음식물 부패 판단 시스템
// =====================================


// ---------- 임계값 설정 ----------
const int PRESSURE_THRESHOLD = 120; // 음식물 투입을 판단하는 압력 기준값

const int GAS_WARNING_VALUE = 580; // 가스 농도가 이 값 이상이면 "주의" 상태
const int GAS_DANGER_VALUE = 700; // 가스 농도가 이 값 이상이면 "위험" 상태

const int GAS_DIFF_WARNING = 40;  // 5분 동안 가스 증가량이 이 값 이상이면 "부패 진행"
const int GAS_DIFF_DANGER = 80;  // 5분 동안 가스 증가량이 이 값 이상이면 "부패 심각"


// ---------- 가스 측정 변수 ----------
int GasStartValue = 0; // 가스 측정 시작 시점 값 (기준값)
int GasEndValue = 0; // 5분 후 가스 측정 값
int diff = 0; // 시작값과 종료값의 차이 (부패 판단 기준)


// ---------- 시간 변수 ----------
unsigned long waitStartTime = 0; // 압력 감지 후 안정화 대기 시작 시간
unsigned long measureStartTime = 0; // 가스 측정 시작 시간
unsigned long realTime = 0;



// ---------- 상태 변수 ----------
int waiting = 0; // 안정화 대기 중인지 여부
int measuring = 0; // 가스 측정 진행 중인지 여부


int ww = 1;
int www = 0;
int wait_autoturn;


int warp();
int press();

void setup()
{
    Serial.begin(9600);

    pinMode(buttonServoPin, INPUT_PULLUP);
    pinMode(buttonMotorPin, INPUT_PULLUP);

    pinMode(motorPin, OUTPUT);

    delay(500); // 서보 안정화
    lidServo.attach(servoPin);
    lidServo.write(0);
	
  	pinMode(7, OUTPUT);

}

void loop()
{

    realTime = millis();

    if (state_PRESS == F_IN)
        flag_press = 1;
    else
        flag_press = 0;


    if (state_WARP == MEASURING)
        flag_warp = warp();
    else
        flag_warp = 0;


    press();

  
  if (state_GAS == IDLE) {
    waiting = 1;
    measuring = 0;
    
    
  }

    if (state_GAS == MEASURING) {
        WaitStabilization();
        MeasureGas();
    }
    else
        flag_gas = 0;


    //waiting = pressure;
  




    // ===== 서보 버튼 =====
    int currentServoButton = digitalRead(buttonServoPin);
    door_button_press = currentServoButton;

    if (currentServoButton == HIGH && lastServoButton == LOW) {
        servoState = !servoState;

        if (servoState) {
            lidServo.write(90);
            state_DOOR = OPEN_DOOR;
            openTime = millis();
        }
        else {
            lidServo.write(0);
            state_DOOR = CLOSED_DOOR;
        }

        delay(200);
    }

    lastServoButton = currentServoButton;

    // ===== 서보 자동 닫힘 (5초) =====
    if (servoState && millis() - openTime >= 5000) {
        servoState = false;
        lidServo.write(0);
        state_DOOR = CLOSED_DOOR;
    }




    // ===== 모터 버튼 (토글) =====
    int currentMotorButton = digitalRead(buttonMotorPin);
    mo_button_press = currentMotorButton;
	
  
    if (state_GAS == OVER && ww == 1) {
   	
    wait_autoturn = millis();
    ww = 0;
    www = 1;
      
  }

  
  if (www == 1 && millis() - wait_autoturn >= 3000) {
    www = 0;
    ww = 1;
    currentMotorButton = 1;
    tone(7,62,250);
    state_GAS = IDLE;
    
  }


    if ((currentMotorButton == HIGH && lastMotorButton == LOW) || state_WARP == STOP) {
        motorState = !motorState;

        digitalWrite(motorPin, motorState ? HIGH : LOW);
        motorState ? state_MOTOR = TURN : state_MOTOR = STOP;
        delay(200);
    }

    lastMotorButton = currentMotorButton;







    ////////////////// FSM //////////////////



    if (flag_press == 1 && state_GAS == IDLE)
        state_GAS1 = MEASURING;
    else if (flag_press == 0 && state_GAS == IDLE)
        state_GAS1 = IDLE;
    else if (flag_gas == 1 && state_GAS == MEASURING)
        state_GAS1 = OVER;
    else if (state_DOOR == OPEN_DOOR || state_MOTOR == TURN)
        state_GAS1 = IDLE;
    else if (state_PRESS == F_NOT)
      	state_GAS1 = IDLE;


    if (pressure == 1)
        state_PRESS1 = F_IN;
    else if (pressure == 0)
        state_PRESS1 = F_NOT;
    else
        state_PRESS1 = state_PRESS;

    if (state_MOTOR == TURN && state_WARP == IDLE)
        state_WARP1 = MEASURING;
    else if (state_WARP == MEASURING && flag_warp == 1)
        state_WARP1 = STOP;
    else if (state_MOTOR == STOP)
        state_WARP1 = IDLE;



    if (state_PRESS != state_PRESS1) {
        waitStartTime = millis();
        waiting = 1;
    }


    state_GAS = state_GAS1;
    state_PRESS = state_PRESS1;
    state_WARP = state_WARP1;


   	Serial.println(measuring);


    //delay(500);

    ////////////////// FSM /////////////////////


}


int warp() {

    {
        sum += analogRead(A0);
        i++;
        if ((i > 30) && sum / i < WARPVALUE) {
            sum = 0;
            i = 0;
            return 1;
        }
        if (sum > 30000) {
            sum = 0;
            i = 0;
        }
    }

}

int press() {
    if (analogRead(A1) < PRESSVALUE)
        pressure = 1;
    else
        pressure = 0;
}



void WaitStabilization() { // 안정화 대기 함수 (1~2분)


    // 설정된 시간(여기선 1분)이 지나면 가스 측정 시작
    if (waiting && millis() - waitStartTime >= 2000) {

        StartGasMeasurement(); // 가스 측정 시작 함수 호출

        waiting = 0;       // 대기 종료
        measuring = 1;      // 측정 상태로 전환
    }
}


void StartGasMeasurement() { // 가스 측정 시작 함수

    GasStartValue = analogRead(A2); // 시작 가스값 저장
    measureStartTime = millis();    // 측정 시작 시간 기록
}


void MeasureGas() { // 가스 측정 및 처리 함수
	
    // 측정 중이고, 5분이 경과한 경우
    if (measuring && millis() - measureStartTime >= 3000) {


        GasEndValue = analogRead(A2); // 종료 시점 가스값 읽기

        diff = GasEndValue - GasStartValue; // 가스 변화량 계산

        CheckSpoilage();   // 부패 상태 판단

        measuring = 0; // 측정 종료 → 다시 대기 상태로 복귀
    }
}



void CheckSpoilage() { // 부패 상태 판단 함수

    // 위험 조건: 가스값 높고, 증가량도 큰 경우
    if (GasEndValue >= GAS_DANGER_VALUE && diff >= GAS_DIFF_DANGER) {
        flag_gas = 1;
        state_GAS = OVER;
    }


    // 정상 상태
    else {
        flag_gas = 0;
        state_GAS = MEASURING;
    }
}
