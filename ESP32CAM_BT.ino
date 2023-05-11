#include "esp_camera.h"
#include "ESP32_pins.h"
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include "BluetoothSerial.h"
#include "OneButton.h"
#include <Wire.h>

BluetoothSerial SerialBT;
int default_distance = 900;
OneButton button(NAV_BUTTON, true); 
byte button_mode = 0;
byte button_start = 0;
byte default_button;


void setup() {
  Wire.setPins(I2C_SDA,I2C_SCL);
  Wire.begin();
  Serial.begin(115200);
  pinMode(MOTOR_RIGHT,OUTPUT);
  pinMode(MOTOR_LEFT,OUTPUT);
  pinMode(NAV_BUTTON,INPUT);
  initBT();
  initCamera();
  Vibration(START);
  default_button = digitalRead(NAV_BUTTON);
  xTaskCreate(
    additionalTask1,
    "Button",
    10000,
    NULL,
    3,
    NULL);
    
  xTaskCreate(
    additionalTask2,
    "tof",
    10000,
    NULL,
    2,
    NULL);

}

void initBT(){
  if(!SerialBT.begin("ESP32CAM-CLASSIC-BT")){
    Serial.println("An error occurred initializing Bluetooth");
    ESP.restart();
  }else{
    Serial.println("Bluetooth initialized");
  }

  SerialBT.register_callback(btCallback);
  Serial.println("The device started, now you can pair it with bluetooth");
}

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected!");
  }else if(event == ESP_SPP_DATA_IND_EVT){
    String stringRead = String(*param->data_ind.data);
    int paramInt = stringRead.toInt() - 48;
    switch(paramInt){
      case 0:
        setCameraParam(2);
      break;
      case 1:
        Vibration(MOVE_LEFT);
      break;
      case 2:
        Vibration(MOVE_RIGHT);
      break;
      case 3:
        Vibration(STAIR_UPHILL);
      break;
      case 4:
        Vibration(STAIR_DOWNHILL);
      break;
     default:
     break;
    }
    Serial.printf("paramInt: %d\n", paramInt);
  }
}

void writeSerialBT(camera_fb_t *fb){
  SerialBT.write(fb->buf, fb->len);
  SerialBT.flush();
}

void initCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }

}

void setCameraParam(int paramInt){
  sensor_t *s = esp_camera_sensor_get();
  switch(paramInt){
    case 4:
      s->set_framesize(s, FRAMESIZE_UXGA);
    break;

    case 3:
      s->set_framesize(s, FRAMESIZE_SXGA);
    break;

    case 2:
      s->set_framesize(s, FRAMESIZE_XGA);
      s->set_brightness(s,0);
      s->set_contrast(s,2);
      s->set_saturation(s,-1);
    break;

    case 1:
      s->set_framesize(s, FRAMESIZE_SVGA);
    break;

    case 0:
    default:
      s->set_framesize(s, FRAMESIZE_VGA);
    break;
  }

  capture();
}

void capture(){
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  fb = esp_camera_fb_get();
  if(!fb){
    esp_camera_fb_return(fb);
    return;
  }

  if(fb->format != PIXFORMAT_JPEG){
    return;
  }

  writeSerialBT(fb);
  esp_camera_fb_return(fb);
}

void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt) 
{
  unsigned short result=0; 
  Wire.beginTransmission(82);
  Wire.write(byte(addr));
  Wire.endTransmission();
  Wire.requestFrom(82, (int)cnt);
  if (cnt <= Wire.available()) {
    *datbuf++ = Wire.read();
    *datbuf++ = Wire.read();
  }
}

int ReadDistance()
{      
    unsigned short distance;
    unsigned char i2c_rx_buf[2];
    //모듈의 0x00번지부터 2바이트 읽어오기
    SensorRead(0x00,i2c_rx_buf,2);
    
    //읽어온 두 바이트를 변수 하나로 합침
    // i2c_rx_buf[0] : 상위 바이트
    // i2c_rx_buf[1] : 하위 바이트
    distance=i2c_rx_buf[0];
    distance=distance<<8;
    distance|=i2c_rx_buf[1];
 
    //300ms 대기
    delay(300); 
    return distance;
}

void Vibration(int pattern){
  switch(pattern){
    case 0:
      Vibration_dubble(200,300);
    break;
    
    case 1:
      Vibration_run(MOTOR_LEFT,200,300);
    break;
    
    case 2:
      Vibration_run(MOTOR_RIGHT,200,300);
    break;
    
    case 3:
      Vibration_dubble(150,600);
    break;

    case 4:
      Vibration_dubble(255,200);
      delay(200);
      Vibration_dubble(255,200);
    break;

    case 5:
      Vibration_run(MOTOR_RIGHT,200,300);
      Vibration_run(MOTOR_LEFT,200,300);
    break;

    case 6:
      Vibration_run(MOTOR_LEFT,200,300);
      Vibration_run(MOTOR_RIGHT,200,300);
    break;

    case 7:
      Vibration_dubble(150,200);
      delay(100);
      Vibration_dubble(150,200);
    break;

    case 8:
      Vibration_run(MOTOR_RIGHT,150,200);
      delay(200);
      Vibration_run(MOTOR_RIGHT,150,200);
    break;

    case 9:
      Vibration_run(MOTOR_LEFT,150,200);
      delay(200);
      Vibration_run(MOTOR_LEFT,150,200);
    break;
    
    default:
    break;
  }
}

void Vibration_run(int pattern,int v_size,int v_time){
  analogWrite(pattern,v_size);
  delay(v_time);
  analogWrite(pattern,0);
}

void Vibration_dubble(int v_size,int v_time){
  analogWrite(MOTOR_RIGHT,v_size);
  analogWrite(MOTOR_LEFT,v_size);
  delay(v_time);
  analogWrite(MOTOR_RIGHT,0);
  analogWrite(MOTOR_LEFT,0);
}

void singleClick(){
  switch(button_mode){
    case 0://네이게이션 모드
      if(button_start){
        button_start = 0;//모드 종료
        //어플에서 네비게이션 모드 종료
        SerialBT.print("M0OFF");
        Vibration(MODE_END);
      }else{
        button_start = 1;//모드 실행
        //어플에서 네비게이션 모드 실행
        SerialBT.print("M0ON");
        Vibration(MODE_START);
      }
      break;
    case 1://사물 인식 모드
      if(button_start){
        button_start = 0;//모드 종료
        //사물 인식 모드 종료
        SerialBT.print("M1OFF");
        Vibration(MODE_END);
      }else{
        button_start = 1;//모드 실행
        //사물 인식 모드 실행
        SerialBT.print("M1ON");
        Vibration(MODE_START);
      }
      break;
    default:
      break;
  }
}



void doubleClick(){
  switch(button_mode){
    case 0://네이게이션 모드
      button_mode = 1; //모드 변환
      button_start = 0;//모드 실행 종료
      Vibration(MODE_CHANGE);
      Serial.println("Mode change 0");
      break;
    case 1://사물 인식 모드
      button_mode = 0;
      button_start = 0;
      Vibration(MODE_CHANGE);
      Serial.println("Mode change 1");
      break;
    default:
      break;
  }
}

int chkButton (void){
  const  unsigned long ButTimeout  = 500;
  static unsigned long msecLst;
  unsigned long msec = millis ();
  //GERRY MOD
  const int debDuration = 100;
  static unsigned long  debStartTime = 0;

  if (msecLst && (msec - msecLst) > ButTimeout)  {
    msecLst = 0;
  //GERRY MOD
    //return SingleClick; 
    return YesSingle;
  }

  byte but = digitalRead(NAV_BUTTON);
  if (default_button != but)  {
    //GERRY MOD
    if (millis() - debStartTime < debDuration) {
      return None;
    }
    debStartTime = millis();
  
    default_button = but;
    if (LOW == but)  {   // press
      if (msecLst)  { // 2nd press
        msecLst = 0;
        return DoubleClick;
      }
      else {
        msecLst = 0 == msec ? 1 : msec;
    //GERRY MOD
        return SingleClick; //SINGLE?
      }
    }
  }
  return None;
}

void loop() {

  
}

void additionalTask1( void * parameter )
{
  for(;;){
    switch (chkButton ())  {
    case SingleClick:
      Serial.println ("single?");
      break;

    case DoubleClick:
      Serial.println ("Its double");
      doubleClick();
      break;

    //GERRY MOD
    case YesSingle:
      Serial.println ("YesSingle");
      singleClick();
      break;
    }
    vTaskDelay(50);
  }
  // 종료되면 Task 삭제
  // 하지만 위의 무한루프 때문에 호출될 수 없음
  vTaskDelete( NULL );
}

void additionalTask2( void * parameter )
{
  for(;;){
    int distance = ReadDistance();//mm단위
    Serial.println(distance);
    if(distance <= default_distance - 200){//앞에 장애물 감지
      Vibration(FRONT_OBSTACLE);
    }
  }
  // 종료되면 Task 삭제
  // 하지만 위의 무한루프 때문에 호출될 수 없음
  vTaskDelete( NULL );
}
