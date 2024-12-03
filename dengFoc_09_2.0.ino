//DengFOC V2.0
//古风科技


#include "DengFOC.h"
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>

int Sensor_DIR=1;    //传感器方向
int Motor_PP=7;    //电机极对数
float target_angle = 0; // 用于存储目标角度
bool stop_requested = false; // 停止标志

// 阻力检测相关变量
float last_angle = 0;     // 上一次的角度值
unsigned long last_change_time = 0;  // 上次角度变化的时间
bool is_moving = false;   // 电机是否在运动
const float angle_threshold = 0.1;   // 角度变化阈值
const unsigned long stall_timeout = 50;  // 停滞超时时间(0.05秒)

// 最大角度设置相关变量
unsigned long stop_press_time = 0;  // stop按键按下时间
const unsigned long set_max_angle_timeout = 10000;  // 设置最大角度需要按住的时间(10秒)
const unsigned long reset_default_timeout = 12000;  // 恢复默认值需要按住的时间(13秒)
const float DEFAULT_MAX_ANGLE = 300;  // 默认最大角度值
float max_angle = DEFAULT_MAX_ANGLE;    // 最大角度值

// 接收数据结构
struct ControlMessage {
  bool up;
  bool down;
  bool stop;
} controlData;

// EEPROM地址定义
const int EEPROM_ADDR = 0;

void saveMaxAngle() {
  EEPROM.writeFloat(EEPROM_ADDR, max_angle);
  EEPROM.commit();
  Serial.printf("保存最大角度: %f\n", max_angle);
}

void loadMaxAngle() {
  max_angle = EEPROM.readFloat(EEPROM_ADDR);
  if (isnan(max_angle) || max_angle < 0 || max_angle > 1000) {
    max_angle = DEFAULT_MAX_ANGLE;  // 如果读取的值无效，使用默认值
  }
  Serial.printf("加载最大角度: %f\n", max_angle);
}

void resetToDefault() {
  max_angle = DEFAULT_MAX_ANGLE;
  saveMaxAngle();
  Serial.println("已恢复默认最大角度: 300");
}

// 数据接收回调函数
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  memcpy(&controlData, data, sizeof(controlData));
  
  if (controlData.stop) {
    // 记录stop按下的开始时间
    if (stop_press_time == 0) {
      stop_press_time = millis();
    }
    
    // 检查是否达到恢复默认值的时间阈值
    unsigned long press_duration = millis() - stop_press_time;
    if (press_duration >= reset_default_timeout) {
      resetToDefault();
      stop_press_time = 0;  // 重置计时器
    }
    // 检查是否达到设置最大角度的时间阈值
    else if (press_duration >= set_max_angle_timeout) {
      max_angle = DFOC_M0_Angle();
      saveMaxAngle();
      Serial.printf("设置新的最大角度: %f\n", max_angle);
    }
    
    // 正常的stop功能
    stop_requested = true;
    target_angle = DFOC_M0_Angle();
    is_moving = false;
  } else {
    stop_press_time = 0;  // 释放stop按键，重置计时器
    
    if (controlData.up) {
      stop_requested = false;
      target_angle = max_angle;
      is_moving = true;
      last_angle = DFOC_M0_Angle();
      last_change_time = millis();
    } else if (controlData.down) {
      stop_requested = false;
      target_angle = 0;
      is_moving = true;
      last_angle = DFOC_M0_Angle();
      last_change_time = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(16);  // 初始化EEPROM
  loadMaxAngle();    // 加载保存的最大角度值
  
  // 设置WiFi模式并打印MAC地址
  WiFi.mode(WIFI_STA);
  Serial.print("接收端MAC地址: ");
  Serial.println(WiFi.macAddress());
  
  // 初始化ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW初始化错误");
    return;
  }
  
  // 注册接收回调函数
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW初始化成功");
  
  DFOC_Vbus(12.6);   //设定驱动器供电电压
  DFOC_alignSensor(Motor_PP,Sensor_DIR);
}

//检测阻力函数
void check_stall() {
  if (!is_moving) return;
  
  float current_angle = DFOC_M0_Angle();
  unsigned long current_time = millis();
  
  // 检查角度是否发生显著变化
  if (abs(current_angle - last_angle) > angle_threshold) {
    last_angle = current_angle;
    last_change_time = current_time;
  }
  // 如果超过停滞时间没有显著变化
  else if (current_time - last_change_time > stall_timeout) {
    // 停止到当前位置
    target_angle = current_angle;
    is_moving = false;
    stop_requested = true;
    Serial.println("检测到阻力，电机停止！");
  }
}

int count=0;
void loop() 
{
  runFOC();
  
  //位置-速度-力（加入电流环后）
  DFOC_M0_SET_ANGLE_PID(1,0,0,100000,10);//p, i, d, 谐波平滑渡 ，  最大速度
  DFOC_M0_SET_VEL_PID(0.02,1,0,100000,2);//p, i, d, 谐波平滑渡（最大了），最大电流
  DFOC_M0_SET_CURRENT_PID(5,200,0,100000);
  DFOC_M0_set_Velocity_Angle(target_angle); // 直接使用目标角度
 

    // 检查是否遇到阻力
  check_stall();
  
  count++;
  if(count>30)
  {
      count=0;
      //Serial.printf("%f\n", DFOC_M0_Current());
      Serial.printf("%f,%f,%f,\n", DFOC_M0_Current(), DFOC_M0_Velocity(),DFOC_M0_Angle());
  }
  //接收串口
  serialReceiveUserCommand();

}
