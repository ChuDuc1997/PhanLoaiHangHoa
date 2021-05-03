#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <AFMotor.h>
#include <Servo.h>

#define SERVO_PIN 10
#define MOTOR_SPEED 250
#define IR_SENSOR_1 A1
#define IR_SENSOR_3 A3

struct Node{
  int Key;
  Node* pNext;
};

Node* createNode(int Data){
  Node* result = new Node;
  if (result == NULL) {
    return NULL;
  }

  result->Key = Data;
  result->pNext = NULL;
  return result;
}

class Queue{
public:
  Node* pHead;
  Node* pTail;

public:
  Queue(){
    pHead = NULL;
    pTail = NULL;
  }

  bool isEmpty(){
    if(pHead == NULL){
      return true;
    }
    return false;
  }

  bool enQueue(int Data){
    Node* pNode = createNode(Data);
    if (pNode == NULL){
      return false;
    }

    if (isEmpty()){
      pHead = pNode;
      pTail = pHead;
      return true;
    }else{
      pTail->pNext = pNode;
      pTail = pNode;
      return true;
    }

    return false;
  }

  int deQueue(){
    if (isEmpty()){
      return 0;
    }else{
      Node* pNode = pHead;
      int result = pHead->Key;
      pHead = pHead->pNext;
      delete pNode;
      if(pHead == NULL){
        pTail = NULL;
      }
      return result;
    }
  }

  int top(){
    if (isEmpty()){
      return 0;
    }else{
      return pHead->Key;
    }
  }

  bool clear(){
    if(isEmpty()){
      return false;
    }else{
      while (pHead != NULL){
        Node* pNode = pHead;
        pHead = pHead->pNext;
        delete pNode;
      }
      pTail = NULL;
      return true;
    }
  }

  void output(){
    if (pHead == NULL){
      Serial.println("Queue is empty!");
    }
    else {
      for (Node* p = pHead; p != NULL; p = p->pNext){
        if (p->pNext == NULL){
          Serial.println(p->Key);
        }
        else {
          Serial.print(p->Key);
          Serial.print(" ");
        }
      }
    }
  }
};

enum Angle{
  TURN_LEFT = 45,
  MIDDLE = 90,
  TURN_RIGHT = 135
};

class SERVO{
protected:
  Servo servo;
  unsigned int pin;
  unsigned int angle;

public:
  SERVO(){
    pin = SERVO_PIN;
    angle = MIDDLE;
  }

  void attach(){
    servo.attach(pin);
  }

  void write(){
    servo.write(angle);
  }

  unsigned int setAngle(unsigned int a){
    angle = a;
  }

  unsigned int getAngle(){
    return angle;
  }
};

enum Direction{
  STOP,
  RUN
};

class MOTOR{
private:
  AF_DCMotor motor = AF_DCMotor(4, MOTOR34_1KHZ);
  unsigned int direction = STOP;

public:
  void setSpeed(uint8_t speed){
    motor.setSpeed(speed);
  }

  void stop(){
    motor.run(RELEASE);
  }

  void run(){
    motor.run(FORWARD);
  }

  void setDirection(unsigned int d){
    direction = d;
  }

  unsigned int getDirection(){
    return direction;
  }
};

SemaphoreHandle_t motorSemaphore;
SemaphoreHandle_t servoSemaphore;
SemaphoreHandle_t irSemaphore;

MOTOR motor;
SERVO servo = SERVO();
Queue queue = Queue();

void setup(){
  Serial.begin(9600);
  motor.setSpeed(MOTOR_SPEED);
  servo.attach();
  servo.write();
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_3, INPUT);

  motorSemaphore = xSemaphoreCreateBinary();
  servoSemaphore = xSemaphoreCreateBinary();
  irSemaphore = xSemaphoreCreateBinary();

  xTaskCreate(readIR, "Read IR", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(controlMotor, "Control Motor", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(controlServo, "Control Servo", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

void loop(){}

void readIR(void* p){
  int status_ir_sensor_1 = 0, status_ir_sensor_3 = 0;
  bool flag_ir_1 = false, flag_ir_3 = false;
  int key = 0;

  while(1){
    status_ir_sensor_1 = digitalRead (IR_SENSOR_1);
    status_ir_sensor_3 = digitalRead (IR_SENSOR_3);
    
    if (Serial.available() > 0) {
      String message = Serial.readStringUntil('\n');
      if(message == "STOP"){
        motor.setDirection(STOP);
      }else if(message == "RUN"){
        motor.setDirection(RUN);
      }
    }
    
    motor.setDirection(motor.getDirection());
    xSemaphoreGive(motorSemaphore);
    vTaskDelay(pdMS_TO_TICKS(50));

    if(status_ir_sensor_1 == 0 && flag_ir_1 == false){
      Serial.print("BOX:");
      flag_ir_1 = true;
      while(1){
        if (Serial.available() > 0) {
          int data = Serial.read() - '0';
          queue.enQueue(data);
          break;
        }
      }
    }else if(status_ir_sensor_1 == 1 && flag_ir_1 == true){
      flag_ir_1 = false;
    }

    if(status_ir_sensor_3 == 0 && flag_ir_3 == false){
      key = queue.deQueue();
      flag_ir_3 = true;

      if(key == 1){
        servo.setAngle(TURN_LEFT);
      }else if(key == 2){
        servo.setAngle(MIDDLE);
      }else if(key == 3){
        servo.setAngle(TURN_RIGHT);
      }

      xSemaphoreGive(servoSemaphore);
      vTaskDelay(pdMS_TO_TICKS(50));
    }else if(status_ir_sensor_3 == 1 && flag_ir_3 == true){
      flag_ir_3 = false;
    }
  }
}

void controlServo(void *p){
  while (1){
    xSemaphoreTake(servoSemaphore, portMAX_DELAY);
    servo.write();
  }
}

void controlMotor(void *p){
  while (1){
    xSemaphoreTake(motorSemaphore, portMAX_DELAY);
    switch (motor.getDirection()){
    case STOP:
      motor.stop();
      break;
    case RUN:
      motor.run();
      break;
    }
  }
}
