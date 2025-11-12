#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <VectorXf.h>

#include "MPU6500_Raw.h"
MPU6500 mpu;

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET, 3400000, 400000);


#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

uint32_t interval, last_cycle;
uint32_t loop_micros;
uint32_t cycle_count;

#define SOK_BUT 2
#define SNEXT_BUT 3
#define SESC_BUT 4

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

enum {
  fsm_Not_pressed = 0,
  fsm_pressing = 1,
  fsm_menu = 2,
  fsm_calibrating = 3,
  fsm_calibrated = 4,
  fsm_show_values = 5,
  fsm_notCalibrated = 6,
  fsm_play = 7,
  fsm_parameters = 8
};

fsm_t fsm1;
bool SOKButton, SESCButton, SNEXTButton;
int count_menu = 0;
int count_calibrate = 0;
bool is_Calibrated = 0;
int dice_numbers;
int param_dice_numbers;
int number_of_dices = 1;

float wx_calibrated, wy_calibrated, wz_calibrated, ax_calibrated, ay_calibrated, az_calibrated;
float wx_total, wy_total, wz_total, ax_total, ay_total, az_total;

void set_interval(float new_interval)
{
  interval = new_interval * 1000000L;   // In microseconds
}  

void set_state(fsm_t &fsm, int new_state) {
  if (fsm.state != new_state) {
    fsm.state = new_state;
    fsm.tis = 0;
    fsm.tes = millis();
  }
}

void setup() 
{
  // Builtin LED
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SOK_BUT, INPUT_PULLUP);
  pinMode(SNEXT_BUT, INPUT_PULLUP);
  pinMode(SESC_BUT,INPUT_PULLUP);
  
  Serial.begin(115200);
  // Our cycle time
  set_interval(20e-3); // 20 ms -> 50 Hz

  // IMU initalization
  const int I2C0_SDA = 20;
  const int I2C0_SCL = 21;
  pinMode(I2C0_SDA, INPUT_PULLUP);
  pinMode(I2C0_SCL, INPUT_PULLUP);

  set_state(fsm1, fsm_Not_pressed); //inicializa maquina estados
  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  
  delay(20);
  
  MPU6500Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
  
  while (!mpu.setup(0x68, setting)) { 
    Serial.println("MPU connection failed.");
    delay(500); // Wait to try again     
  }

  // OLED initalization
  const int I2C1_SDA = 18;
  const int I2C1_SCL = 19;
  pinMode(I2C1_SDA, INPUT_PULLUP);
  pinMode(I2C1_SCL, INPUT_PULLUP);
  
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  while(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    delay(500);  // Wait to try again
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
}

// Struct to store IMU readings
typedef struct {
  Vec3f w;
  Vec3f a;
  uint32_t cycle_time, last_cycle_time; // IMU cycle tracking
} imu_values_t;

imu_values_t imu;

void loop() 
{
  uint8_t b;
  if (Serial.available()) {  // Only do this if there is serial data to be read
  
    b = Serial.read();
    Serial.print((int)b);
    
  } 

  // Do this only every "interval" microseconds 
  uint32_t now = micros();
  uint32_t delta = now - last_cycle; 
  if (delta >= interval) {
    loop_micros = micros();
    last_cycle = now;
    cycle_count++;

    // Read, if ready, the MPU
    if (mpu.update()) {
      imu.last_cycle_time = imu.cycle_time;
      imu.cycle_time = micros();

      imu.w.x = mpu.getGyroX();
      imu.w.y = mpu.getGyroY();
      imu.w.z = mpu.getGyroZ();

      imu.a.x = mpu.getAccX();
      imu.a.y = mpu.getAccY();
      imu.a.z = mpu.getAccZ();
    }

    fsm1.tis = millis() - fsm1.tes;
    // Read the buttons
    // ...
    SOKButton = !digitalRead(SOK_BUT);
    SESCButton = !digitalRead(SESC_BUT);
    SNEXTButton = !digitalRead(SNEXT_BUT);
    
    //Debugging
    
    /*
    if (SOKButton){
      Serial.printf("Butão Apertado");
    } 
    */
    
    if (fsm1.state == fsm_menu){Serial.printf("State = Menu\n");}
    if (fsm1.state == fsm_Not_pressed){Serial.printf("State = Not pressed\n");}
    if (fsm1.state == fsm_pressing){Serial.printf("State = Pressing\n");}
    if (fsm1.state == fsm_calibrating){Serial.printf("State = Calibrating\n");}
    if (fsm1.state == fsm_calibrated){Serial.printf("State = Calibrated\n");}
    if (fsm1.state == fsm_show_values){Serial.printf("State = Show values\n");}
    if (fsm1.state == fsm_notCalibrated){Serial.printf("State = Not Calibrated!\n");}
    if (fsm1.state == fsm_play){Serial.printf("State = Playing\n");}
    // Place here the state machines
    // Transições Not Pressed
  if (fsm1.state == fsm_Not_pressed && SOKButton) {
    set_state(fsm1, fsm_pressing);
  }

    // Transições Pressing
  if (fsm1.state == fsm_pressing && fsm1.tis >= 2000 && SOKButton) {  // PRESSING -> MENU
    set_state(fsm1, fsm_menu);
    count_menu = 0;
  }

  if (fsm1.state == fsm_pressing && !SOKButton){  //PRESSING -> NOT PRESSED
    set_state(fsm1,fsm_Not_pressed);
  }


  // Transições Menu
  
  /*if (fsm1.state == fsm_menu && SESCButton) { //Menu -> NOT PRESSED
    set_state(fsm1, fsm_Not_pressed);
  }
  */
  if (fsm1.state == fsm_menu && count_menu == 1 && SOKButton){  //Menu -> Calibrating 
    wx_calibrated = 0;    //resetamos os valores calibrados
    wy_calibrated = 0;
    wz_calibrated = 0;
    ax_calibrated = 0;
    ay_calibrated = 0;
    az_calibrated = 0;

    set_state(fsm1,fsm_calibrating);
  }
  if (fsm1.state == fsm_menu && count_menu == 2 && SOKButton){  //Menu -> Show values
    set_state(fsm1,fsm_show_values);
    count_menu = 0;
  }

  if(fsm1.state == fsm_menu && count_menu == 3 && is_Calibrated && SOKButton){ //Menu -> Play
    set_state(fsm1,fsm_play);
    count_menu = 0;
    //Display diz "roll the dice"
  }
  if (fsm1.state == fsm_menu && count_menu == 3 && !is_Calibrated && SOKButton){ //Menu -> Not calibrated
    set_state(fsm1,fsm_notCalibrated);
    count_menu = 0;
    //Display diz "You need to calibrate before rolling the dice"
  }
  if(fsm1.state == fsm_menu && count_menu == 4 && SOKButton){ //Menu -> Play
    set_state(fsm1,fsm_parameters);
  }

  //Transições notCalibrated
  if (fsm1.state == fsm_notCalibrated && fsm1.tis >= 4000){
    set_state(fsm1,fsm_menu);
  }

  //Transições Playing
  if (fsm1.state == fsm_play && SESCButton){
    set_state(fsm1,fsm_menu);
  }

  //Transições Calibrating
  if (fsm1.state == fsm_calibrating && fsm1.tis >= 2000){
    set_state(fsm1,fsm_calibrated);
    wx_calibrated = wx_total / count_calibrate;
    wy_calibrated = wy_total / count_calibrate;
    wz_calibrated = wz_total / count_calibrate;
    ax_calibrated = ax_total / count_calibrate;
    ay_calibrated = ay_total / count_calibrate;
    az_calibrated = (az_total / count_calibrate) - 1;
  } 

  //Transições Calibrated
  if (fsm1.state == fsm_calibrated && fsm1.tis >=3000){
    wx_total = 0;
    wy_total = 0;
    wz_total = 0;
    ax_total = 0;
    ay_total = 0;
    az_total = 0;
    count_calibrate = 0;    //reset de todos os valores para próximo calculo
    is_Calibrated = true;
    set_state(fsm1,fsm_menu);
  }

  //Transições Show Values
  if (fsm1.state == fsm_show_values && SESCButton){
    set_state(fsm1, fsm_menu);
  }

  //Transições Parameters
  if (fsm1.state == fsm_parameters && SESCButton){
    set_state(fsm1, fsm_menu);
  }

  //OUTPUTS

  //OUTPUT MENU
  if (fsm1.state == fsm_menu){
    //AQUI ESCREVER O DISPLAY DO MENU !!!
    //
    //
    display.clearDisplay();
    display.setCursor(64,0);
    display.printf("Menu");
    display.setCursor(0,16);
    display.printf("Calibrate");
    display.setCursor(0,32);
    display.printf("Show accelerometer values");
    display.printf("Count = %d\n", count_menu);
    display.display();
  }
  //OUTPUT NOT_PRESSED OU PRESSING
  if(fsm1.state == fsm_Not_pressed || fsm1.state == fsm_pressing){
    // OLED output
    display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    
    display.printf("Wx %.2f\n", imu.w.x);
    display.printf("Wy %.2f\n", imu.w.y);
    display.printf("Wz %.2f\n", imu.w.z);

    display.setCursor(64, 0);     // Start at top-left corner
    display.printf("Ax %.2f", imu.a.x);
    display.setCursor(64, 8);
    display.printf("Ay %.2f", imu.a.y);
    display.setCursor(64, 16);
    display.printf("Az %.2f", imu.a.z);
    display.setCursor(0, 24);
    display.display();

    // Serial output
    Serial.printf("IMU_dt %d; ", imu.cycle_time - imu.last_cycle_time);

    Serial.printf("Wx %.2f; ", imu.w.x);
    Serial.printf("Wy %.2f; ", imu.w.y);
    Serial.printf("Wz %.2f; ", imu.w.z);

    Serial.printf("Ax %.2f; ", imu.a.x);
    Serial.printf("Ay %.2f; ", imu.a.y);
    Serial.printf("Az %.2f; ", imu.a.z);

    //Serial.print("T ");
    //Serial.print(mpu.getTemperature(), 2);

    Serial.print("loop ");
    Serial.print(micros() - now);

    Serial.println();
    }

    if (fsm1.state == fsm_show_values){
          display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    
    display.printf("Wx %.2f\n", imu.w.x - wx_calibrated);
    display.printf("Wy %.2f\n", imu.w.y - wy_calibrated);
    display.printf("Wz %.2f\n", imu.w.z - wz_calibrated);

    display.setCursor(64, 0);     // Start at top-left corner
    display.printf("Ax %.2f", imu.a.x - ax_calibrated);
    display.setCursor(64, 8);
    display.printf("Ay %.2f", imu.a.y - ay_calibrated);
    display.setCursor(64, 16);
    display.printf("Az %.2f", imu.a.z + az_calibrated);

    display.display();
    }
    //Ação estado calibrating
    if (fsm1.state == fsm_calibrating){ //vai somando os valores das amostras, e o numero de amostras
      wx_total = wx_total + imu.w.x;
      wy_total = wy_total + imu.w.y;
      wz_total = wz_total + imu.w.z;
      ax_total = ax_total + imu.a.x;
      ay_total = ay_total + imu.a.y;
      az_total = az_total + imu.a.z;
      count_calibrate= count_calibrate + 1;
      Serial.printf("Wx total %.2f; ", wx_total);
      Serial.printf("Wy total %.2f; ", wy_total);
      Serial.printf("Ax total %.2f; ", ax_total);
      Serial.printf("Az total %.2f; ", az_total);
      Serial.printf("Count_calibrate %d",count_calibrate);
    }
    //Ação estado Calibrated
    if (fsm1.state == fsm_calibrated){
      Serial.printf("Wx_calibrated: %.2f",wx_calibrated);
      Serial.printf("Wy_calibrated: %.2f",wy_calibrated);
      Serial.printf("Wz_calibrated: %.2f",wz_calibrated);
      Serial.printf("Ax_calibrated: %.2f",ax_calibrated);
      Serial.printf("Ay_calibrated: %.2f",ay_calibrated);
      Serial.printf("Az_calibrated: %.2f",az_calibrated);
    }

    //Ação Parameters
    if (fsm1.state == fsm_parameters && SOKButton){
      param_dice_numbers = (param_dice_numbers + 1) % 4;
      switch(param_dice_numbers)
      {
        case(0):
        dice_numbers = 4;
        break;
        
        case(1):
        dice_numbers = 6;
        break;
        
        case(2):
        dice_numbers = 10;
        break;

        case(3):
        dice_numbers = 20;
        break;
      }
    }

    if (fsm1.state == fsm_parameters && SNEXTButton){
      number_of_dices = ((number_of_dices)% 4) + 1;
    }
    if (fsm1.state == fsm_notCalibrated){
    // escrever Not calibrated
    }

    if (fsm1.state == fsm_parameters){
      Serial.printf("Number of dices :%d \n",number_of_dices);
      Serial.printf("Possible numbers : D%d \n",dice_numbers);
      //FAZER DISPLAY INTERFACE
    }
    // muda valor contador se tiver no menu
    if (fsm1.state == fsm_menu && SNEXTButton){
      count_menu = (count_menu + 1) % 5;
    }
  }
}
