#include "digitalServoLibrary.h"
#define ID2   2


//------------------------monitor--------------------------//
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // pixel ความกว้าง
#define SCREEN_HEIGHT 64 // pixel ความสูง 

// กำหนดขาต่อ I2C กับจอ OLED
#define OLED_RESET     -1 //ขา reset เป็น -1 ถ้าใช้ร่วมกับขา Arduino reset
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//------------------------monitor--------------------------//


int button_reduce = 2; // the number of the pushbutton pin for reduce
int button_add = 3; // the number of the pushbutton pin ฟกก

int button_set = 4; // the number of the pushbutton pin set
int button_start = 5; //the number of the pushbutton pin for start
int state_button_start = 0; //กำหนด state ปุ่ม start
int set_state = 0;
int motorspeed;

int add_state = 0;
int add_reduce_count = 1; // variable to store the number of button presses for button_add and reduce
int default_speed = 100; //set speed default for button_start
int speedmotor_start = 100;//speed motor default start
int count = 1;
int set_round = 0; //set round
int state_round = 1;
int set_speed = 1;
int show_rounds = set_round;

int commandValue = 0;


///--------------------------debouncing a push switch---------------------------//
//reduce
unsigned long  debounceDelayCOM = 15;
int buttonState1 = LOW;     // the current state of the button
int lastButtonState1 ; // the previous state of the button
unsigned long lastDebounceTime1 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay1 = debounceDelayCOM;    // the debounce time; increase if the output flickers
//add
int buttonState2 = LOW;     // the current state of the button
int lastButtonState2 ; // the previous state of the button
unsigned long lastDebounceTime2 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay2 = debounceDelayCOM;    // the debounce time; increase if the output flickers

//start
int buttonState3 = LOW;     // the current state of the button
int lastButtonState3 ; // the previous state of the button
unsigned long lastDebounceTime3 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay3 = debounceDelayCOM;    // the debounce time; increase if the output flickers

//set
int buttonState4 = LOW;     // the current state of the button
int lastButtonState4 ; // the previous state of the button
unsigned long lastDebounceTime4 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay4 = debounceDelayCOM;    // the debounce time; increase if the output flickers






///--------------------------debouncing a push switch---------------------------//

//----------- variable of Bundle servo --------------------------------------------------------//
int BUNDLE_SERVO_INITIAL_DEGREE = 230;  // don't less than 100 and more than 900.
int BUNDLE_ROUND = 1;  // จำนวนรอบที่ต้องการม้วนลวด
int BUNDLE_SERVO_ROUND_LIMIT_CCW = 3; // 2x - 1 -> จำนวนรอบของ servo ตอนม้วน
//const int BUNDLE_SERVO_ROUND_LIMIT_CCW = (2 * BUNDLE_ROUND) - 1; // 2x - 1 -> จำนวนรอบของ servo ตอนม้วน
int BUNDLE_SERVO_DEGREE_TO_BREAK = BUNDLE_SERVO_INITIAL_DEGREE + (1 * BUNDLE_ROUND);  // (130 is constant value) องศาในการหยุดม้วนลวด
int BUNDLE_SERVO_ROUND_LIMIT_CW = BUNDLE_SERVO_ROUND_LIMIT_CCW + 1; // จำนวนรอบของ servo ตอนคายกลไกม้วน
int BUNDLE_SERVO_CCW_SPEED = -500;  // ความเร็วและทิศทางในการม้วน
int BUNDLE_SERVO_CW_SPEED = 500;    // ความเร็วและทิศทางในการคายม้วน
int RANGE_SETPOINT = 200;    // ช่วงตำแหน่งในการเช็ครอบของ servo
int bundleServo_positionUpdate = -2048;   // ตัวแปรเก็บค่าองศา
int bundleServo_roundUpdate = 0;    // ตัวแปรเก็บค่ารอบ
bool bundleServo_debouncingRoundChecker = false;
bool bundleServo_debouncingPositionChecker = false;
bool checkBundleServoRound_LastState = false;
unsigned long checkBundleServoRound_LastDebounceTime = 0;
unsigned long checkBundleServoRound_DebounceDelay = 500;    // ตัวแปรเก็บค่าช่วงเวลาป้องกันช่วงของค่าองศาที่เพี้ยนตอนเริ่มต้นหมุน
//-----------------

int feederServo_positionUpdate = -2048;   // ตัวแปรเก็บค่าองศา
int feederServo_roundUpdate = 0;    // ตัวแปรเก็บค่ารอบ
bool feederServo_debouncingRoundChecker = false;
//----------- variable of Bundle Arm servo ----------------------------------------------------//

const int BUNDLE_ARM_SERVO_SPEED = 1000;    // ตัวแปรเก็บค่าความเร็วของแขนกลไกม้วนลวดปากถุง
//---------------------------------------------------------------------------------------------//

int ommandValue = 0;
bool commandState = true;   // ตัวแปรเช็คว่าทำงานในฟังก์ชันเสร็จหรือยัง
void BundleArmServoInitialDegree() {  // ฟังก์ชันหมุน digital servo ของแขนตัวม้วนลวดไปตำแหน่งเริ่มต้น
  LobotSerialServoMove(Serial, ID2, 1, BUNDLE_ARM_SERVO_SPEED);

  delay(2000);    // delay เพื่อรอ digital servo หมุนไปยังตำแหน่งให้เสร็จ ป้องกันกลไกม้วนลวดทำงานควบจังหวะกัน
  commandState = true;
}

void BundleArmServoEndDegree() {  // ฟังก์ชันหมุน digital servo ของแขนตัวม้วนลวดไปตำแหน่งของการม้วนลวดปากถุง
  LobotSerialServoMove(Serial, ID2, 600, BUNDLE_ARM_SERVO_SPEED);

  delay(2000);    // delay เพื่อรอ digital servo หมุนไปยังตำแหน่งให้เสร็จ ป้องกันกลไกม้วนลวดทำงานควบจังหวะกัน
  commandState = true;
}
//--------------- End Bundle Arm Servo Function -----------------------------------------------//
//###############################################################################################
//---------------- Bundle Servo Function ------------------------------------------------------//
// ฟังก์ชันหมุน digital servo เพื่อม้วน/คายลวดปากถุง
void BundleServoRotate(int _roundLimit, int _bundleServoDirection, int _setPoint) {


  checkBundleServoRound(_bundleServoDirection);   // เช็ครอบ digital servo ของตัวม้วนลวด โดยจะ return ค่ามาในตัวแปร bundleServo_roundUpdate
  Serial.print("bundle servo round: ");
  Serial.print(bundleServo_roundUpdate);


  
  Serial.print("\t");
  //  Serial.println(set_round - bundleServo_roundUpdate);

  //--------------------------make by ton-------------------------//
  OLED.clearDisplay(); // ลบภาพในหน้าจอทั้งหมด
  OLED.setTextColor(WHITE, BLACK);  //กำหนดข้อความสีขาว ฉากหลังสีดำ
  OLED.setCursor(20, 0); // กำหนดตำแหน่ง x,y ที่จะแสดงผล
  OLED.setTextSize(2); // กำหนดขนาดตัวอักษร
  OLED.print("MC LAB"); // แสดงผลข้อความ ALL

  OLED.setCursor(0, 20);
  OLED.setTextSize(2);
  OLED.print("Speed: ");
  OLED.print(add_reduce_count);

  OLED.setCursor(0, 40);
  OLED.setTextSize(2);
  OLED.print("Rnd: ");
  OLED.print(set_round - bundleServo_roundUpdate);

  OLED.display(); // สั่งให้จอแสดงผล


  //------------------------make by ton ----------------------//


  if (bundleServo_roundUpdate > _roundLimit) {  // ให้หยุดหมุนเมื่อจำนวนรอบของ digital servo หมุนเกินรอบที่จำกัดไว้
    BundleServoBreak();
    return;
  }

  if (bundleServo_roundUpdate < _roundLimit) {  // ให้หมุนต่อไปเมื่อจำนวนรอบของ digital servo ยังหมุนไม่เกินรอบที่จำกัดไว้
    //    Serial.println();
    LobotSerialServoSetMode(Serial, ID2, 1, _bundleServoDirection);


    commandState = false;
    return;
  }

  if ( bundleServo_roundUpdate == _roundLimit ) {   // เมื่อจำนวนรอบของ digital servo เท่ากับรอบที่จำกัดไว้ ให้เช็คตำแหน่งที่จะหยุด
    //    Serial.println("Last round");
    if ( checkBundleServoPositionToBreak(_setPoint) == true ) {   // ให้หยุดหมุน digital servo เมื่อหมุนมายังช่วงตำแหน่งที่ต้องการหยุดแล้ว
      BundleServoBreak();
      LobotSerialServoMove(Serial, ID2, _setPoint, 500);
      return;
    }
    commandState = false;
  }
}

void BundleServoBreak() {   // ฟังก์ชันหยุดหมุน digital servo ของตัวม้วน/คายลวดปากถุง
  //  Serial.println("bundle servo Breaking!");
  LobotSerialServoSetMode(Serial, ID2, 0, 0);    // คำสั่งหยุดหมุน digital servo
  bundleServo_roundUpdate = 0;    // รีเซตจำนวนรอบที่หมุน
  checkBundleServoRound_LastState = false;
  commandState = true;
}

void BundleServoInitialDegree() {   // ฟังก์ชันหมุน digital servo ไปองศาเริ่มต้นการม้วนลวดปากถุง
  LobotSerialServoMove(Serial, ID2, BUNDLE_SERVO_INITIAL_DEGREE, 1000);
  commandState = true;
}

void checkBundleServoRound(int _bundleServoDirection) {   // ฟังก์ชันเช็ครอบการหมุนของ digital servo ตัวม้วน/คายลวดปากถุง

  bundleServo_positionUpdate = LobotSerialServoReadPosition(Serial, ID2);  // (-194 to 1194) อ่านค่าตำแหน่ง digital servo ของตัวม้วนลวดและเก็บไว้ในตัวแปร
  //  Serial.print("bundle servo position: ");
  //  Serial.print(bundleServo_positionUpdate);
  //  Serial.print("\t");

  if (_bundleServoDirection > 0) { // CW -> ถ้าค่า _bundleServoDirection มีค่ามากกว่า 0 แสดงว่า digital servo หมุนตามเข็ม เป็นการคายกลไกม้วนลวด
    if ( bundleServo_positionUpdate >= BUNDLE_SERVO_INITIAL_DEGREE - 80 &&
         bundleServo_positionUpdate <= BUNDLE_SERVO_INITIAL_DEGREE - 30 ) {  // ถ้าตำแหน่ง digital servo อยู่ในช่วงนี้ แสดงว่าอาจหมุนมาครบ 1 รอบแล้ว ทำการตรวจเช็คในเงื่อนไขต่อไป

      // ค่าตัวแปร bundleServo_debouncingRoundChecker เป็นการเช็คป้องกันการนับซ้ำรอบเดิม
      // และมีการเช็คช่วงเวลา Debouncing ป้องกันการอ่านค่าที่เพี้ยนตอนเริ่มต้นหมุน
      if (bundleServo_debouncingRoundChecker == true &&
          (millis() - checkBundleServoRound_LastDebounceTime) > checkBundleServoRound_DebounceDelay) {
        //bundleServo_roundUpdate++;



        bundleServo_debouncingRoundChecker = false;
        checkBundleServoRound_LastState = false;
      }
      return;
    }
    else if ( bundleServo_positionUpdate <= BUNDLE_SERVO_INITIAL_DEGREE - 100 ||
              bundleServo_positionUpdate >= BUNDLE_SERVO_INITIAL_DEGREE - 10 ) {

      if (checkBundleServoRound_LastState == false) {
        checkBundleServoRound_LastState = true;
        checkBundleServoRound_LastDebounceTime = millis();
      }

      bundleServo_debouncingRoundChecker = true;
    }
  }
  
}

bool checkBundleServoPositionToBreak(int _setPoint) {   // ฟังก์ชันเช็คองศาการหมุนของ digital servo เพื่อหยุดการม้วน/คายลวดปากถุง
  bundleServo_positionUpdate = LobotSerialServoReadPosition(Serial, ID2);        // -194 to 1194

  if (abs(bundleServo_positionUpdate - _setPoint) < RANGE_SETPOINT ) {
    return true;
  }
  return false;
}
//--------------- End Bundle Servo Function ---------------------------------------------------//





void setup() {
  Serial.begin(115200);
  //Serial.begin(19200);
  // put your setup code here, to run once:
  pinMode(button_add, INPUT);
  pinMode(button_reduce, INPUT);
  pinMode (button_set, INPUT);
  pinMode (button_start, INPUT);
  BundleArmServoInitialDegree();
  
  //-----------------monitor_setup---------------//
  if (!OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // สั่งให้จอ OLED เริ่มทำงานที่ Address 0x3C
    Serial.println("SSD1306 allocation failed");
  } else {
    Serial.println("ArdinoAll OLED Start Work !!!");
  }

}



void show_monitor() {
  OLED.clearDisplay(); // ลบภาพในหน้าจอทั้งหมด
  OLED.setTextColor(WHITE, BLACK);  //กำหนดข้อความสีขาว ฉากหลังสีดำ
  OLED.setCursor(20, 0); // กำหนดตำแหน่ง x,y ที่จะแสดงผล
  OLED.setTextSize(2); // กำหนดขนาดตัวอักษร
  OLED.println("MC LAB"); // แสดงผลข้อความ ALL

  OLED.setCursor(0, 20);
  OLED.setTextSize(2);
  OLED.print("Set Speed ");


  OLED.setCursor(20, 40);
  OLED.setTextSize(2);
  OLED.print("-  ");
  OLED.print(add_reduce_count); // แสดงผลข้อความ ALL
  OLED.print("  +");


  OLED.display(); // สั่งให้จอแสดงผล
  delay(50);
}

int check_set_round_digit(int set_round) {
  return int(log10(set_round) + 1);
}

void show_monitor_round(int set_round, int rnd_increment) {
  OLED.clearDisplay(); // ลบภาพในหน้าจอทั้งหมด
  OLED.setTextColor(WHITE, BLACK);  //กำหนดข้อความสีขาว ฉากหลังสีดำ
  OLED.setCursor(20, 0); // กำหนดตำแหน่ง x,y ที่จะแสดงผล
  OLED.setTextSize(2); // กำหนดขนาดตัวอักษร
  OLED.println("Set Round ");

  OLED.setCursor(0, 20);
  OLED.setTextSize(2);
  OLED.print("Inc: ");
  OLED.println(rnd_increment);

  OLED.setCursor(18, 40);
  OLED.setTextSize(2);
  // OLED.setTextColor(BLACK, WHITE); //กลับสีข้อความกับพื้นหลัง
  OLED.print("- ");
//  Serial.print("digit =  ");
//  Serial.print(check_set_round_digit(set_round));
//
//  for (int i = check_set_round_digit(set_round) + 1; i <= 4; i++) {
//    if (i > 1) {
//      OLED.print(" ");
//    }
//  }
  OLED.print(set_round); // แสดงผลข้อความ ALL
  OLED.print(" +");


  OLED.display(); // สั่งให้จอแสดงผล
  delay(20);
}
void show_monitor_result() {
  OLED.clearDisplay(); // ลบภาพในหน้าจอทั้งหมด
  OLED.setTextColor(WHITE, BLACK);  //กำหนดข้อความสีขาว ฉากหลังสีดำ
  OLED.setCursor(20, 0); // กำหนดตำแหน่ง x,y ที่จะแสดงผล
  OLED.setTextSize(2); // กำหนดขนาดตัวอักษร
  OLED.print("MC LAB"); // แสดงผลข้อความ ALL

  OLED.setCursor(0, 20);
  OLED.setTextSize(2);
  OLED.print("Speed: ");
  OLED.print(add_reduce_count);

  OLED.setCursor(0, 40);
  OLED.setTextSize(2);
  OLED.print("Rnd: ");
  OLED.print(set_round); // แสดงผลข้อความ ALL




  OLED.display(); // สั่งให้จอแสดงผล
  delay(20);
}
//-------------------------------------//
void show_monitor_run() {

  OLED.clearDisplay(); // ลบภาพในหน้าจอทั้งหมด
  OLED.setTextColor(WHITE, BLACK);  //กำหนดข้อความสีขาว ฉากหลังสีดำ
  OLED.setCursor(20, 0); // กำหนดตำแหน่ง x,y ที่จะแสดงผล
  OLED.setTextSize(2); // กำหนดขนาดตัวอักษร
  OLED.print("MC LAB"); // แสดงผลข้อความ ALL

  OLED.setCursor(0, 20);
  OLED.setTextSize(2);
  OLED.print("Round :");
  OLED.print(show_rounds);


  OLED.display(); // สั่งให้จอแสดงผล
  delay(20);

}
//-------------------------------------//

void show_round() {
//  Serial.print("Set ROUND =  ");
//  Serial.println( set_round);
}
void show_count() {
//  Serial.print("speed  = ");
//  Serial.println( add_reduce_count);
}


int setup_step = 1;
float round_scale[5] = {1, 10, 50, 100, 500};
int round_scale_idx = 2;

bool check_newround = true;
bool found_new_round=false;

bool get_ready = true;

bool check_ready_update_round(int x){
  if (x >200 && x<500){
    return true;
  }
  else return false;
}



void loop() {
  int add = digitalRead(button_add);
  int reduce = digitalRead(button_reduce);
  int set = digitalRead(button_set);
  int start = digitalRead(button_start);

  switch (setup_step) {
    case 1:
      show_monitor();
      if (add != lastButtonState1) {    // check if the state has changed
        lastDebounceTime1 = millis();       // record the time the state changed
      }
      if ((millis() - lastDebounceTime1) > debounceDelay1) { // check if the debounce delay has elapsed

        if (add != buttonState1) {     // check if the button state is stable
          buttonState1 = add;
          if (buttonState1 == HIGH) {
            delay(30);
            if (add_reduce_count >= 7) {
              delay(30);
              add_reduce_count = 1;
              show_count();
            } else {
              delay(30);
              add_reduce_count++;
              show_count();

            }
          }


        }
      }
      lastButtonState1 = add;


      if (reduce != lastButtonState2) {    // check if the state has changed
        lastDebounceTime2 = millis();       // record the time the state changed
      }
      if ((millis() - lastDebounceTime2) > debounceDelay2) { // check if the debounce delay has elapsed

        if (reduce != buttonState2) {     // check if the button state is stable
          buttonState2 = reduce;
          if ( reduce == HIGH) {//button for reduce
            if ( add_reduce_count <= 1 ) {
              delay(30);
              add_reduce_count = 1;
              show_count();
            } else {
              delay(30);
              add_reduce_count--;
              show_count();
            }
          }
        }
      }
      lastButtonState2 = reduce;




      if (start == HIGH) {
        setup_step = 2;
        delay(100);
        Serial.println("Set Round Mode");
      }



      break;
    case 2:
      show_monitor_round(set_round, round_scale[round_scale_idx]);
      delay(50);

      if (add != lastButtonState1) {    // check if the state has changed
        lastDebounceTime1 = millis();       // record the time the state changed
      }
      if ((millis() - lastDebounceTime1) > debounceDelay1) { // check if the debounce delay has elapsed

        if (add != buttonState1) {     // check if the button state is stable
          buttonState1 = add;
          if (add == HIGH) {
            delay(30);
            set_round = set_round + round_scale[round_scale_idx];
            if (set_round > 2000) {
              set_round = 2000;

              Serial.print("Set round = ");
              Serial.println( set_round);
            }
            show_monitor_round(set_round, round_scale[round_scale_idx]);
          }
        }
      } lastButtonState1 = add;


      if (reduce != lastButtonState2) {    // check if the state has changed
        lastDebounceTime2 = millis();       // record the time the state changed
      }
      if ((millis() - lastDebounceTime2) > debounceDelay2) { // check if the debounce delay has elapsed

        if (reduce != buttonState2) {     // check if the button state is stable
          buttonState2 = reduce;
          if (reduce == HIGH) {
            delay(30);
            set_round = set_round - round_scale[round_scale_idx];
            if (set_round < 0) {
              set_round = 0;

              Serial.print("Set round = ");
              Serial.println( set_round);
            }
          }
        }
      } lastButtonState2 = reduce;

      if (set != lastButtonState4) {    // check if the state has changed
        lastDebounceTime4 = millis();       // record the time the state changed
      }
      if ((millis() - lastDebounceTime4) > debounceDelay4) { // check if the debounce delay has elapsed

        if (set != buttonState4) {     // check if the button state is stable
          buttonState4 = set;

          if (set == HIGH) {
            round_scale_idx++;
            round_scale_idx = round_scale_idx % 5;
          }
        }
      } lastButtonState4 = set;


      if (start != lastButtonState3) {    // check if the state has changed
        lastDebounceTime3 = millis();       // record the time the state changed
      }
      if ((millis() - lastDebounceTime3) > debounceDelay3) { // check if the debounce delay has elapsed

        if (start != buttonState3) {     // check if the button state is stable
          buttonState3 = start;
          if (start == HIGH) {
            setup_step = 3;
            Serial.println("out case 2");
            delay(30);
            count = set_round;
            Serial.print("Submit ROUND =  ");
            Serial.println(count);
          }
        }
      } lastButtonState3 = start;
      break;

    case 3:
      show_monitor_result();

      if (start != lastButtonState3) {    // check if the state has changed
        lastDebounceTime3 = millis();       // record the time the state changed
      }
      if ((millis() - lastDebounceTime3) > debounceDelay3) { // check if the debounce delay has elapsed

        if (start != buttonState3) {     // check if the button state is stable
          buttonState3 = start;
          if (start == HIGH) {
            delay(50);
            BUNDLE_ROUND = set_round;
            BUNDLE_SERVO_ROUND_LIMIT_CCW = set_round;
            setup_step = 4;
          }
        }
      } lastButtonState3 = start;
      break;

    case 4:

      set_speed = (add_reduce_count * 100) + default_speed;
      Serial.println(set_speed);
      delay(100);

      BundleServoRotate(BUNDLE_SERVO_ROUND_LIMIT_CCW, set_speed, BUNDLE_SERVO_DEGREE_TO_BREAK);   
      
      bool get_ready_old;
      get_ready_old = get_ready;
      get_ready = check_ready_update_round(bundleServo_positionUpdate);
//      Serial.print("get_ready: ");
//      if (get_ready) {
//        Serial.println("Yes");
//      }
//      else{
//        Serial.println("No");
//      }

      if (get_ready_old && !get_ready){
        bundleServo_roundUpdate++;
//        Serial.print("Round ");        
//        Serial.println(bundleServo_roundUpdate);        
      }

      if (start != lastButtonState3) {    // check if the state has changed
        lastDebounceTime3 = millis();       // record the time the state changed
      }
      if ((millis() - lastDebounceTime3) > debounceDelay3) { // check if the debounce delay has elapsed

        if (start != buttonState3) {     // check if the button state is stable
          buttonState3 = start;
          if (start == HIGH) {
            delay(50);
            BundleServoBreak();
            setup_step = 1;
            delay(200);

          }
        }
      } lastButtonState3 = start;

      break;
  }
  if (commandState == true) {   // เช็คว่าฟังก์ชันที่ทำงานอยู่ทำงานจบแล้วหรือยัง ถ้า commandState มีค่าเป็น true แสดงว่าทำงานจบแล้ว ให้รีเซตค่า commandValue
//    Serial.println("commandvalue");
    commandState = false;
    setup_step = 1;
    set_round = 0;
    add_reduce_count = 1;
  }
}
