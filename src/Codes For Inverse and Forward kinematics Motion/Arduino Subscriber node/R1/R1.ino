#ifndef nullptr
#define nullptr NULL
#endif

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <AccelStepper.h>
#include <Servo.h>

// --- تعيين منفذ السيرفو للمؤثر النهائي R1 ---
#define R1EndEffPin 10 

// --- تعيين منافذ محركات الستيبر وحساسات النهاية لروبوت R1 ---
// ترتيب المحركات: القاعدة (Base)، الكتف (Shoulder)، المرفق (Elbow)، المعصم (Wrist)
const int Step_pins[] = {34, 36, 38, 40};
const int Dir_pins[]  = {35, 37, 39, 41};

// حساسات النهاية التناظرية للكتف والمرفق
const int LimitPin[] = {3, 4 ,5};

// --- ثوابت لعملية الـ Homing والزوايا للذراع R1 ---
#define R1theta1    90.0
#define R1theta2   125.0
#define R1theta3   -90.0
#define R1theta4   -68.0

// نسب النقل للمحركات (Transmission Values)
float Tr_m[] = {3.3, 5.0, 4.0, 2.3};
const int stepsPerRevolution = 6400.0;

// حساب الخطوات المرجعية للوضع الصفري
const long R1ZeroSteps[] = {
  (long)(R1theta1 * stepsPerRevolution / 360.0 * Tr_m[0]),
  (long)(R1theta2 * stepsPerRevolution / 360.0 * Tr_m[1]),
  (long)(R1theta3 * stepsPerRevolution / 360.0 * Tr_m[2]),
  (long)(R1theta4 * stepsPerRevolution / 360.0 * Tr_m[3])
};

Servo R1EndEff;

// تهيئة المحركات الخطوية الأربعة للذراع R1
AccelStepper steppers[] = {
  AccelStepper(AccelStepper::DRIVER, Step_pins[0], Dir_pins[0]), // Base
  AccelStepper(AccelStepper::DRIVER, Step_pins[1], Dir_pins[1]), // Shoulder
  AccelStepper(AccelStepper::DRIVER, Step_pins[2], Dir_pins[2]), // Elbow
  AccelStepper(AccelStepper::DRIVER, Step_pins[3], Dir_pins[3])  // Wrist
};

// --- متغيرات لتأجيل أمر المؤثر النهائي إلى حين الوصول للهدف ---
int targetGripperState = 0;   // الأمر المطلوب للقابض (1 لإغلاق، 0 لفتح)
bool gripperPending = false;  // راية (Flag) تُفعل عند وجود أمر مؤجل للقابض

// تعريف دالة المعايرة مسبقاً
void limitswitchsensor();

// دالة استقبال الأوامر من ROS
void messageCb(const std_msgs::Float32MultiArray& msg) {
  // الحماية: التأكد من أن الحزمة تحتوي على 5 عناصر على الأقل
  if (msg.data_length < 5) return;

  // 1. تحديد أهداف الخطوات لكل محرك (بتحويل الـ Float إلى Long بدقة)
  for (int i = 0; i < 4; i++) {
    steppers[i].moveTo(static_cast<long>(msg.data[i]));
  }

  // 2. حفظ أمر القابض ورفع راية الانتظار
  targetGripperState = static_cast<int>(msg.data[4]);
  gripperPending = true;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32MultiArray> sub("robot_commands", &messageCb);

// دالة المعايرة للذراع R1
void limitswitchsensor() {
  // تعيين الموقع الصفري المباشر للقاعدة والمعصم لعدم وجود حساسات نهاية لهما
  steppers[0].setCurrentPosition(R1ZeroSteps[0]);
  
  int speeds[] = {1000,-1000 ,-1000};
  
  for (int count = 0; count < 3; count++) {
    unsigned long startTime = millis(); // حماية زمنية لكل محرك لمنع تجمد النظام
    
    // معايرة الكتف والمرفق باستخدام الحساسات A3 و A4
    while (analogRead(LimitPin[count]) < 1000) {
      steppers[count + 1].setSpeed(speeds[count]);
      steppers[count + 1].runSpeed();
      nh.spinOnce(); // إبقاء اتصالات ROS حية أثناء المعايرة
      
    }
    steppers[count + 1].setSpeed(0);
    steppers[count + 1].runSpeed();
    steppers[count + 1].setCurrentPosition(R1ZeroSteps[count + 1]);
  }
}

void setup() {
  // 1. تثبيت منافذ الخطوة والاتجاه كـ LOW لمنع الحركة العشوائية المبكرة
  for (int i = 0; i < 4; i++) {
    pinMode(Step_pins[i], OUTPUT);
    pinMode(Dir_pins[i], OUTPUT);
    digitalWrite(Step_pins[i], LOW);
    digitalWrite(Step_pins[i], LOW);
  }

  delay(1000);

  // 2. تهيئة الاتصال بـ ROS
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);

  // 3. تطبيق إعدادات السرعة والتسارع للمحركات
  for (int i = 0; i < 4; i++) {
    steppers[i].setMaxSpeed(800);
    steppers[i].setAcceleration(200);
  }

  // 4. إعداد السيرفو
  R1EndEff.attach(R1EndEffPin);
  R1EndEff.write(180); // الوضع الافتراضي: مفتوح

  // 5. --- الانتظار الذكي: الانتظار حتى يتم فتح الـ rosserial من الكمبيوتر فعلياً دون أي حركة مبكرة ---
  while (!nh.connected()) {
    nh.spinOnce();
  }
  
  // ملاحظة: تم إزالة limitswitchsensor() من هنا، بحيث لا يتحرك الروبوت تلقائياً عند الإقلاع،
  // بل ينتظر إرسال أمر التصفير (0) يدوياً من الـ Terminal تماماً مثل R2.
  limitswitchsensor();
}

void loop() {
  nh.spinOnce();
  
  // 1. تنفيذ الحركة بالتسارع لجميع المحركات الخطوية
  for (int i = 0; i < 4; i++) {
    steppers[i].run();
  }

  // 2. التحقق مما إذا كان هناك أمر مؤجل للمؤثر النهائي
  if (gripperPending) {
    bool allFinished = true;

    // فحص هل وصلت كافة المحركات للهدف أم لا
    for (int i = 0; i < 4; i++) {
      if (steppers[i].distanceToGo() != 0) {
        allFinished = false;
        break;
      }
    }

    // بعد وصول جميع المحركات إلى مواضعها تماماً، يتم تنفيذ حركة القابض
    if (allFinished) {
      if (targetGripperState == 1) {
        R1EndEff.write(0);   // إغلاق
      } else {
        R1EndEff.write(180); // فتح
      }
      
      gripperPending = false; // خفض الراية بعد التنفيذ
    }
  }
}
