#ifndef nullptr

#define nullptr NULL

#endif



#include <ros.h>

#include <std_msgs/Float32MultiArray.h>

#include <AccelStepper.h>

#include <Servo.h>



// --- تعيين منافذ السيرفو لروبوت R2 ---

#define R2WristPin 9 

#define R2EndEffPin 8



// --- تعيين منافذ محركات الستيبر وحساسات النهاية ---

const int Step_pins[] = {28, 30, 32};

const int Dir_pins[]  = {29, 31, 33};

const int limitswitch[] = {0, 1, 2};



// --- ثوابت لعملية الـ Homing ---

#define R2theta1 185.0

#define R2theta2 60.0

#define R2theta3 -7.0

float Tr_m[] = {1.0, 4.5, 4.5};

const int stepsPerRevolution = 6400;



const long R2ZeroSteps[] = {

  (long)(R2theta1 * stepsPerRevolution / 360.0 * Tr_m[0]),

  (long)(R2theta2 * stepsPerRevolution / 360.0 * Tr_m[1]),

  (long)(R2theta3 * stepsPerRevolution / 360.0 * Tr_m[2])

};



Servo R2Wrist;

Servo R2EndEff;



AccelStepper steppers[] = {

  AccelStepper(AccelStepper::DRIVER, Step_pins[0], Dir_pins[0]),

  AccelStepper(AccelStepper::DRIVER, Step_pins[1], Dir_pins[1]),

  AccelStepper(AccelStepper::DRIVER, Step_pins[2], Dir_pins[2])

};



// دالة استقبال الأوامر من ROS

void messageCb(const std_msgs::Float32MultiArray& msg) {

  if (msg.data_length < 5) return;



  for (int i = 0; i < 3; i++) {

    steppers[i].moveTo(static_cast<long>(msg.data[i]));

  }



  R2Wrist.write(static_cast<int>(msg.data[3]));

  

  if (msg.data[4] >= 0.5f) {

    R2EndEff.write(160); // إغلاق

  } else {

    R2EndEff.write(38);  // فتح

  }

}



ros::NodeHandle nh;

ros::Subscriber<std_msgs::Float32MultiArray> sub("robot_commands", &messageCb);



void limitswitchsensor() {

  int speeds[] = {800, -800, -800}; 



  for (int count = 0; count < 3; count++) {    

    while (analogRead(limitswitch[count]) < 1000) {

      steppers[count].setSpeed(speeds[count]);

      steppers[count].runSpeed();

      nh.spinOnce();

          }

    

    steppers[count].setSpeed(0);

    steppers[count].setCurrentPosition(R2ZeroSteps[count]);

  }

}



void setup() {

  // 1. تثبيت منافذ الخطوة والاتجاه كـ LOW لمنع الحركة العشوائية

  for(int i = 0; i < 3; i++) {

    pinMode(Step_pins[i], OUTPUT);

    pinMode(Dir_pins[i], OUTPUT);

  }



  delay(1000);



  // 2. تهيئة الاتصال بـ ROS

  nh.getHardware()->setBaud(57600);

  nh.initNode();

  nh.subscribe(sub);



  // 3. ضبط السرعات والتسارع

  steppers[0].setMaxSpeed(800);

  steppers[0].setAcceleration(500);

  steppers[1].setMaxSpeed(800);

  steppers[1].setAcceleration(600);

  steppers[2].setMaxSpeed(800);

  steppers[2].setAcceleration(600);



  R2EndEff.attach(R2EndEffPin);

  R2EndEff.write(38); 

  R2Wrist.attach(R2WristPin);

  R2Wrist.write(0); 



  // 4. --- هنا الإضافة الذكية: الانتظار حتى يتم فتح الـ rosserial من الكمبيوتر فعلياً ---

  // سيبقى الأردوينو واقفاً هنا ولا يتحرك ولا يعاير حتى تشغيل الـ terminal لديك

  while (!nh.connected()) {

    nh.spinOnce();

  }



  // 5. بمجرد أن تتصل (وتحدث النفضة اللحظية وتستقر)، تنفذ المعايرة تلقائياً وبدقة

  limitswitchsensor();

}



void loop() {

  nh.spinOnce();

  steppers[0].run();

  steppers[1].run();

  steppers[2].run();

}
