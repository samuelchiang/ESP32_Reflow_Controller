
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager for WiFi and Configurations
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include <SPIFFS.h>
#include <HTTPClient.h> //for line notification
#include <PubSubClient.h> //for mqtt
#include <max6675.h>
#include <Adafruit_GFX.h> //for oled
#include <Adafruit_SSD1306.h> //for oled
#include <ezButton.h>
//------------------------------------------ pin --------------------------------------------------------------
//buzzer
int BUZZER_PIN = 25;     //蜂鳴器控制腳位

//Max6675 pin
int thermoSO1 = 18; //SO1
int thermoCS1 = 19; //CS1
int thermoSCK1 = 23; //SCK1

//For Reset WiFi Button
const int BTN_WIFI_PIN = 17;  //TX2
//For Start Button
const int BTN_START_PIN = 16;  //RX2

//For Heater
const int Heater_PIN = 32;  //RX2

const int ONBOARD_LED = 2;
//------------------------------------------ global variables --------------------------------------------------

char DeviceId[32] = "ReflowIron";  // Device ID, SSID
char APPassword[32] = "1qaz2wsx";  // Wifi AP Password

//MQTT
char mqttServer[40] = "";  // MQTT伺服器位址
char mqttPort[6]  = "1883";
char mqttUserName[32] = "";  // 使用者名稱
char mqttPwd[32] = "";  // MQTT密碼

static int taskCore = 0;
//------------------------------------------ Second Configurations ----------------------------------
class Config {
    public:
        Config(){} 
        void load();
        void save();
        bool wifi_enable;
};
void Config::load() {
  if (SPIFFS.begin(true)) {
    if (SPIFFS.exists("/config2.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config2.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(1024);
     
        deserializeJson(doc, buf.get(), DeserializationOption::NestingLimit(20));
        serializeJsonPretty(doc, Serial);

        if (!doc.isNull() && doc.containsKey("wifi_enable")) {
          Serial.println("\nparsed json");
          wifi_enable = doc["wifi_enable"];
        } else {
          Serial.println("failed to load json config");
          wifi_enable = false;
          save();
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
}

void Config::save() {
    DynamicJsonDocument doc(1024);
    doc["wifi_enable"]   = wifi_enable;
    File configFile = SPIFFS.open("/config2.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    serializeJsonPretty(doc, Serial);
    serializeJson(doc, configFile);
    configFile.close();
}
Config config;

//------------------------------------------ OLED --------------------------------------------------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

class SCREEN {
    public:
        SCREEN() { 
          display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
        } 
        void init();
        void drawAPmode();
        void drawError(String text);
        void drawText(String text);
    private:
        Adafruit_SSD1306 display;
};

void SCREEN::init() {
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, OLED_RESET, true)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
}

void SCREEN::drawAPmode(){
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("AP Mode"));
  display.setTextSize(1);
  char text[64];
  sprintf(&text[0], "\nSSID:%s\nPassword:%s", DeviceId, APPassword);
  display.println(text);
  display.display();
}

void SCREEN::drawError(String text){
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(text);
  
  display.display();
}


void SCREEN::drawText(String text){
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(text);
  display.display();
}

SCREEN screen;

//------------------------------------------ Buzzer --------------------------------------------------------------
#define BUZZER_IDLE       0
#define BUZZER_BEEP_DELAY 1
#define BUZZER_BEEPING    2

class BUZZER {
    public:
        BUZZER(int pin);
        void loop();
        void beep(unsigned long beepTime);
        void beep(unsigned long beepTime, unsigned long delay);
    private:
        int _pin;
        unsigned long _delayTime;
        unsigned long _beepTime;
        unsigned long _startTime;
        int _buzzerState;
};

BUZZER::BUZZER(int pin) { 
  _pin = pin;
  _buzzerState = BUZZER_IDLE;
  _delayTime = 0;
  _beepTime  = 0;
  _startTime = 0;
  pinMode(_pin, OUTPUT);
} 
void BUZZER::beep(unsigned long beepTime) {
  beep(beepTime, 0);
}

void BUZZER::beep(unsigned long beepTime, unsigned long delay) {
  _delayTime = delay;
  _beepTime  = beepTime;
  _buzzerState = BUZZER_BEEP_DELAY;
  _startTime = millis();
}

void BUZZER::loop() {
  switch(_buzzerState) {
    case BUZZER_IDLE:
      break;

    case BUZZER_BEEP_DELAY:
      if ((unsigned long)(millis() - _startTime) >= _delayTime) {
        _buzzerState = BUZZER_BEEPING;
        _startTime = millis();

        digitalWrite(_pin, HIGH);
      }

      break;

    case BUZZER_BEEPING:
      if ((unsigned long)(millis() - _startTime) >= _beepTime) {
        _buzzerState = BUZZER_IDLE;
        digitalWrite(_pin, LOW);
      }

      break;
    default:
      break;
  }
}
BUZZER buzzer(BUZZER_PIN);

//------------------------------------------ WiFi and Configurations ----------------------------------

bool shouldSaveConfig;
void saveConfigCallback () {
  shouldSaveConfig = true;
}

class WIFI {
    public:
        WIFI(char *AP_SSID, char* AP_PASS){ 
            ap_ssid = AP_SSID;
            ap_pass = AP_PASS;
        } 
        void init();
        void reset();
        void check();
    private:
        void readConfig();
        char *ap_ssid;
        char* ap_pass;
};


void WIFI::init() {
  readConfig();
  
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFiManager wm;
  shouldSaveConfig = false;
  wm.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter custom_mqttServer("mqttServer", "mqtt server", mqttServer, 40);
  WiFiManagerParameter custom_mqttPort("mqttPort", "mqtt port", mqttPort, 6);
  WiFiManagerParameter custom_mqttUserName("mqttUserName", "mqtt user name", mqttUserName, 32);
  WiFiManagerParameter custom_mqttPwd("mqttPwd", "mqtt password", mqttPwd, 32);
  WiFiManagerParameter custom_DeviceId("DeviceId", "Device ID", DeviceId, 32);
  
  //add all your parameters here
  wm.addParameter(&custom_mqttServer);
  wm.addParameter(&custom_mqttPort);
  wm.addParameter(&custom_mqttUserName);
  wm.addParameter(&custom_mqttPwd);
  wm.addParameter(&custom_DeviceId);
  

  bool res;
  screen.drawAPmode();
  wm.setConfigPortalTimeout(180);//seconds
  res = wm.autoConnect(ap_ssid,ap_pass); // password protected ap
  if(!res) {
      Serial.println("Failed to connect, restart");
      ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("connected...yeey :)");
      screen.drawText("WiFi connected");
  }

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    DynamicJsonDocument doc(1024);
    
    doc["mqttServer"]   = custom_mqttServer.getValue();
    doc["mqttPort"]     = custom_mqttPort.getValue();
    doc["mqttUserName"] = custom_mqttUserName.getValue();
    doc["mqttPwd"]      = custom_mqttPwd.getValue();
    doc["DeviceId"]     = custom_DeviceId.getValue();

    
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    
    serializeJsonPretty(doc, Serial);
    serializeJson(doc, configFile);
    configFile.close();
    //end save
    shouldSaveConfig = false;
    readConfig();
  }  
}

void WIFI::readConfig(){
  //clean FS, for testing
  // SPIFFS.format();
  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin(true)) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(1024);
     
        deserializeJson(doc, buf.get(), DeserializationOption::NestingLimit(20));
        serializeJsonPretty(doc, Serial);

        if (!doc.isNull()) {
          Serial.println("\nparsed json");

          if (doc.containsKey("mqttServer")){
            strcpy(mqttServer, doc["mqttServer"]);  
          }
          if (doc.containsKey("mqttPort")){
            strcpy(mqttPort, doc["mqttPort"]);
          }
          if (doc.containsKey("mqttUserName")){
            strcpy(mqttUserName, doc["mqttUserName"]);
          }
          if (doc.containsKey("mqttPwd")){
            strcpy(mqttPwd, doc["mqttPwd"]);
          }
          if (doc.containsKey("DeviceId")){
            strcpy(DeviceId, doc["DeviceId"]);
          }
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
}


void WIFI::check()
{
  static unsigned long samplingTime = millis();
  static int countdown=5;
  
  if(millis()-samplingTime > 5000)
  {
      samplingTime=millis();
      
      if(WiFi.status()!= WL_CONNECTED) {
        countdown--;
        if(countdown==0){
            Serial.println("Failed to reconnect");
            ESP.restart();
        }
      }else{
        countdown=5;
      }
  }
}

//------------------------------------------ MQTT --------------------------------------------------------------
WiFiClient espClient;
PubSubClient mqttClient(espClient);
char CommandTopic[64];
char DataTopic[64];
bool mqttConnected=false;
bool mqttFirstTry=true;

void mqtt_init() {
    mqttClient.setServer(mqttServer, atoi(mqttPort)); 
    sprintf(&CommandTopic[0],"cmd/%s/", DeviceId);  //"cmd/DeviceId/#";
    sprintf(&DataTopic[0],"data/%s/", DeviceId);  //"data/DeviceId/";
}

void mqtt_loop(){
  if(WiFi.status()!= WL_CONNECTED) {
    return;
  }
  if (mqttFirstTry && !mqttClient.connected()) {
      if (mqttClient.connect(DeviceId, mqttUserName, mqttPwd)) {
        Serial.println("MQTT connected");
        screen.drawText("MQTT connected");
        mqttClient.subscribe(CommandTopic);
        mqttClient.setCallback(mqttCallback);
        mqttConnected=true;
      } else {
        mqttConnected=false;
      }
  }else{
    mqttClient.loop();  
  }
  mqttFirstTry=false;
}

void mqtt_publish(const char* topic, String str){
    if(!mqttConnected){
      return;
    }
    // 宣告字元陣列
    byte arrSize = str.length() + 1;
    char msg[arrSize];
    Serial.print("Publish topic: ");
    Serial.print(topic);
    Serial.print(" message: ");
    Serial.print(str);
    Serial.print(" arrSize: ");
    Serial.println(arrSize);
    str.toCharArray(msg, arrSize); // 把String字串轉換成字元陣列格式
    if (!mqttClient.publish(topic, msg)){
      Serial.println("Faliure to publish, maybe you should check the message size: MQTT_MAX_PACKET_SIZE 128");       // 發布MQTT主題與訊息
    }
}


//------------------------------------------ Push Buttons ------------------------------------------------------
#include <functional>
#define LONG_PRESS_CALLBACK_SIGNATURE std::function<void(void)> long_pressed_callback
#define SHORT_PRESS_CALLBACK_SIGNATURE std::function<void(void)> short_pressed_callback
const int SHORT_PRESS_TIME = 1000; // 1000 milliseconds
const int LONG_PRESS_TIME  = 1000; // 1000 milliseconds
class BUTTON {
    public:
        BUTTON(int PIN)  {pin=PIN;} 
        void init();
        void loop();
        void setLongPressedCallback(LONG_PRESS_CALLBACK_SIGNATURE);
        void setShortPressedCallback(SHORT_PRESS_CALLBACK_SIGNATURE);
    private:
        ezButton *button;
        int pin;
        unsigned long pressedTime  = 0;
        unsigned long releasedTime = 0;
        bool isPressing = false;
        bool isLongDetected = false;
        LONG_PRESS_CALLBACK_SIGNATURE;
        SHORT_PRESS_CALLBACK_SIGNATURE;
};

void BUTTON::init() {
    button = new ezButton(pin);
    button->setDebounceTime(50);
}

void BUTTON::setLongPressedCallback(LONG_PRESS_CALLBACK_SIGNATURE) {
    this->long_pressed_callback = long_pressed_callback;
}

void BUTTON::setShortPressedCallback(SHORT_PRESS_CALLBACK_SIGNATURE) {
    this->short_pressed_callback = short_pressed_callback;
}

void BUTTON::loop(){
   button->loop(); // MUST call the loop() function first

  if(button->isPressed()){
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
  }

  if(button->isReleased()) {
    isPressing = false;
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if( pressDuration < SHORT_PRESS_TIME ){
      Serial.println("A short press is detected");
      if(short_pressed_callback){
        short_pressed_callback();
      }
    }
      
  }

  if(isPressing == true && isLongDetected == false) {
    long pressDuration = millis() - pressedTime;

    if( pressDuration > LONG_PRESS_TIME ) {
      Serial.println("A long press is detected");
      isLongDetected = true;
      if(long_pressed_callback){
        long_pressed_callback();
      }
    }
  }
}

//------------------------------------------ Max6675 K-type themocouple ----------------------------------------

MAX6675 thermocouple(thermoSCK1, thermoCS1, thermoSO1);

//------------------------------------------ Heater ------------------------------------------------------

// Setting PWM properties
const int HeaterPWMChannel = 2;
const int HeaterPWMfreq = 1;
const int resolution = 8;
float refresh_rate = 200;                   //PID loop time in ms
float setpointDiff = 100;   //In degrees C

int SOAK_POINT = 150;
float SOAK_KP = 1.5;
float SOAK_KI = 0.06;
float SOAK_KD = 0.8;
int SOAK_DURATION=100000; //100s

int PEAK_POINT = 217;
float PEAK_KP = 2;
float PEAK_KI = 0.6;
float PEAK_KD = 8;
int PEAK_DURATION=75000; //75s


class Heater {
    public:
        Heater() { 
          state = 0;
          kp=2.5;
          ki=0.06;
          kd=0.8;
          state_reflow=0;
        } 
        void init();
        void loop();
        bool start(int percent);
        void stop();
        void set_pid_control(int set_point, float set_kp, float set_ki, float set_kd);
        void update_state();
        void upload_data();
        void start_reflow();
        void loop_reflow();
    private:
        int   state;
        int   percent;
        void  loop_pid();
        float get_temp();
        void  ramp_up();
        void  pid_control();
        float kp;
        float ki;
        float kd;
        float setpoint;       //In degrees C 
        float pid_p, pid_i, pid_d;
        int   soak_point;       //In degrees C 
        int   peak_point;       //In degrees C 
        int   state_reflow;
};

void Heater::init()
{
  //setup_heater
  pinMode(Heater_PIN, OUTPUT);
  // configure LED PWM functionalitites
  ledcSetup(HeaterPWMChannel, HeaterPWMfreq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Heater_PIN, HeaterPWMChannel);
  //Stop all
  stop();
}  

void Heater::loop(){
  loop_pid();
  update_state();
  upload_data();
  loop_reflow();
}

void Heater::update_state(){
  static float elapsedTime, prev_time;        //Variables for time control
  elapsedTime = millis() - prev_time; 
  if(elapsedTime > refresh_rate){  
    char text[64];
    float temp =  thermocouple.readCelsius();
    
    sprintf(&text[0], "Heater:%d\nTemp:%3.0f\nWiFi:%s\nMQTT:%s", percent, temp, config.wifi_enable? "yes" : "no",  mqttConnected ? "yes" : "no");
    screen.drawText(String(text));
    prev_time = millis();
  }
}
    
void Heater::upload_data(){
  static float elapsedTime, prev_time;        //Variables for time control
  elapsedTime = millis() - prev_time; 
  if(elapsedTime > 1000){  
    char text[64];
    float temp =  thermocouple.readCelsius();  
    sprintf(&text[0], "{\"Heater\":%d,\"Temp\":%3.0f, \"pid_p\":%3.0f,\"pid_i\":%3.0f, \"pid_d\":%3.0f}", percent, temp, pid_p, pid_i, pid_d);
    //screen.drawText(String(text));
    mqtt_publish(DataTopic, String(text));
    prev_time = millis();
  }
}


bool Heater::start(int percent) {
  if (percent > 100 || percent < 0) {
    Serial.printf("start Error percent %d\n", percent);
    return false;
  }
  percent = percent;
  state=0;
  int dutyCycle = map(percent, 0, 100, 0, 255);
  ledcWrite(HeaterPWMChannel, dutyCycle);
  Serial.printf("start with dutyCycle:%d percent:%d\r\n", dutyCycle, percent);
  return true;
}

void Heater::stop() {
  ledcWrite(HeaterPWMChannel, 0);
  state=0;
  percent = 0;
}

float Heater::get_temp(){
  return thermocouple.readCelsius();
}

void Heater::set_pid_control(int set_point, float set_kp, float set_ki, float set_kd){
    Serial.printf("Heater::set_pid_control set_point= %d, kp=%f, ki=%f kd=%f\n", set_point, set_kp, set_ki, set_kd);
    setpoint = set_point;       //In degrees C
    kp=set_kp;
    ki=set_ki;
    kd=set_kd;
    state=1;
}

void Heater::loop_pid() {  
  //state
  // 0, not active
  // 1, ramp_up
  // 2, PID_control
  switch (state){
    case 1:
      ramp_up();
      break;
    case 2:
      pid_control(); 
      break;
  }
}

//Main PID compute and execute function
void Heater::pid_control(){
  static float elapsedTime, prev_time;        //Variables for time control
  
  float PID_total;
  float now_pid_error, prev_pid_error;
  float real_temp;           //We will store here the real temp 
  
  elapsedTime = millis() - prev_time;   
  if(elapsedTime > refresh_rate){    
    //1. We get the temperature and calculate the error
    real_temp = get_temp();
    now_pid_error = setpoint - real_temp;
  
    //2. We calculate PID values
    pid_p = kp * now_pid_error;
    pid_d = kd*((now_pid_error - prev_pid_error)/refresh_rate);
    //2.2 Decide if we apply I or not. Only when error is very small
    if(-3 < now_pid_error && now_pid_error < 3){
      pid_i = pid_i + (ki * now_pid_error);
    }else{
      pid_i = 0;
    }

    //3. Calculate and map total PID value
    PID_total = pid_p + pid_i + pid_d;  
    PID_total = map(PID_total, 0, 150, 0, 255);

    //4. Set limits for PID values
    if(PID_total < 0){
        PID_total = 0;
    }else if(PID_total > 255) {
        PID_total = 255; 
    } 

    percent = map(PID_total, 0, 255, 0, 100);

    //5. Write PWM signal to the Heater
    ledcWrite(HeaterPWMChannel, PID_total);
    
    //6. Save values for next loop
    prev_time = millis();                       //Store time for next loop
    prev_pid_error = now_pid_error;             //Store error for next loop
  }  
}//End PID_control loop


//Fucntion for ramping up the temperature
void Heater::ramp_up(void){  
  static float elapsedTime, prev_time;        //Variables for time control
  float real_temp;           //We will store here the real temp 
  
  //Rising temperature to (setpoint - setpointDiff)
  elapsedTime = millis() - prev_time; 
  if(elapsedTime > refresh_rate){  
    real_temp = get_temp();
    
    if(real_temp < (setpoint - setpointDiff)){
      ledcWrite(HeaterPWMChannel, 255);//Turn on heater
      percent = 100;
    }else{
      ledcWrite(HeaterPWMChannel, 0);        //Turn Off heater
      percent = 0;
      state = 2;                       //Already hot so we go to PID control
    }
    
    prev_time = millis();
  }
}//End of ramp_up loop

void Heater::start_reflow(){
    Serial.println("state_reflow=1");
    state_reflow=1;
    soak_point=SOAK_POINT;
    peak_point=PEAK_POINT;
    set_pid_control(SOAK_POINT, SOAK_KP, SOAK_KI, SOAK_KD);
}

void Heater::loop_reflow(){
  static float elapsedTime, prev_time, soak_start_time, peak_start_time;        //Variables for time control
  float real_temp;           //We will store here the real temp 

  if(state_reflow==0){
    return;
  }
  
  elapsedTime = millis() - prev_time; 
  if(elapsedTime > refresh_rate){  
    real_temp = get_temp();
    
    if(state_reflow==1 && real_temp>soak_point){
        Serial.printf("state_reflow=2 temp %f > soak_point %f\n", real_temp, soak_point);
        state_reflow=2;
        //delay 100s then set to peak point
        soak_start_time = millis();
    }

    if(state_reflow==3 && real_temp>peak_point){
        Serial.printf("state_reflow=4 temp %f > peak_point %f\n", real_temp, peak_point);
        state_reflow=4;
        //delay 100s then set to peak point
        peak_start_time = millis();
    }
    
    prev_time = millis();
  }

  if(state_reflow==2 && millis() - soak_start_time > SOAK_DURATION){  
    set_pid_control(PEAK_POINT, PEAK_KP, PEAK_KI, PEAK_KD);
    Serial.println("state_reflow=3");
    state_reflow=3;
    soak_start_time=0;
  }

  if(state_reflow==4 && millis() - peak_start_time > PEAK_DURATION){  
    Serial.println("state_reflow=0");
    stop();
    state_reflow=0;
    peak_start_time=0;
    buzzer.beep(100);
  }
}

Heater heater;

//====================================static callback =====================================================================

BUTTON wifi_btn(BTN_WIFI_PIN);
BUTTON start_btn(BTN_START_PIN);

void wifi_btn_short_pressed(){
   Serial.println("WiFi short press is detected");
   config.wifi_enable=!config.wifi_enable;
   config.save();
   ESP.restart();
}

void wifi_btn_long_pressed(){
   screen.drawText("Reset Wifi");
   delay(2000); // wait for a second
   WiFiManager wm;
   wm.resetSettings();
   ESP.restart();
}

void start_btn_short_pressed(){
   buzzer.beep(100);
   heater.start_reflow();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]\n");

  if(strcmp(topic, CommandTopic) != 0 ){
      return;
   }
  //{ "command": "setPIDControl", "params": { "temp": 120, "kp":2.5, "ki":0.06, "kd":0.8 }}
  //{ "command": "stopPIDControl"}

  //Parse Data
    DynamicJsonDocument doc(200);
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.println(F("deserializeJson() failed "));
      //Serial.println(error.f_str());=
      return;
    }
    const char* command = doc["command"];
    // Print values.
    if(strcmp(command, "setPIDControl") == 0){
      JsonVariant params = doc["params"];
      if (params.isNull()) {
        Serial.println("error command arguments");
        return;
      }
      buzzer.beep(100);
      JsonVariant temp_jv = doc["params"]["temp"];
      JsonVariant kp_jv = doc["params"]["kp"];
      JsonVariant ki_jv = doc["params"]["ki"];      
      JsonVariant kd_jv = doc["params"]["kd"];
      float setpoint = temp_jv.as<float>();
      float kp = kp_jv.as<float>();
      float ki = ki_jv.as<float>();
      float kd = kd_jv.as<float>();
      heater.set_pid_control(setpoint, kp, ki, kd);
    }else if(strcmp(command, "stopPIDControl") == 0){
      heater.stop();
    } 
  
}

//=======================================Task ========================================================

void Task1( void *pvParameters ){
  while(1){
      wifi_btn.loop();
      delay(1); 
  }     
}

void setup_task() {
    xTaskCreatePinnedToCore(
       Task1,
       "Task1",   // A name just for humans
       10000,  // This stack size can be checked & adjusted by reading the Stack Highwater
       NULL,
       0,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
       NULL, 
       taskCore);
}

//=========================================================================================================

WIFI wifi(DeviceId, APPassword);

void setup()
{
  Serial.begin(115200);
  screen.init();
  heater.init();  
  wifi_btn.init();
  wifi_btn.setShortPressedCallback(wifi_btn_short_pressed);
  wifi_btn.setLongPressedCallback(wifi_btn_long_pressed);
  start_btn.init();
  start_btn.setShortPressedCallback(start_btn_short_pressed);
  config.load();
  setup_task();
  pinMode(ONBOARD_LED,OUTPUT);
  if(config.wifi_enable){
    digitalWrite(ONBOARD_LED,HIGH);
    wifi.init();
    mqtt_init();
  }else{
    digitalWrite(ONBOARD_LED,LOW);
  }
}
void loop()
{
  if(config.wifi_enable){
    wifi.check();
    mqtt_loop();
  }
  heater.loop();
  start_btn.loop();
  buzzer.loop();
}
