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

// --- BOTÕES COM RISING EDGE ---
typedef struct {
  bool value;       // leitura atual (já invertida: pressionado=1)
  bool prev_value;  // leitura anterior
  bool re;          // rising edge: 0 -> 1
} button_t;

button_t SOK, SESC, SNEXT;

static inline void update_buttons(void) {
  // SOK
  SOK.prev_value = SOK.value;
  SOK.value = !digitalRead(SOK_BUT);
  SOK.re = (!SOK.prev_value && SOK.value);

  // SESC
  SESC.prev_value = SESC.value;
  SESC.value = !digitalRead(SESC_BUT);
  SESC.re = (!SESC.prev_value && SESC.value);

  // SNEXT
  SNEXT.prev_value = SNEXT.value;
  SNEXT.value = !digitalRead(SNEXT_BUT);
  SNEXT.re = (!SNEXT.prev_value && SNEXT.value);
}

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

enum {
  fsm_begin = 0,
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
int count_menu = 1;
int count_calibrate = 0;
bool is_Calibrated = 0;
int dice_numbers = 6;
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

void dot(int x, int y, int r) {
  if (r < 1) r = 1;
  display.fillCircle(x, y, r, SSD1306_WHITE);
}
void draw(uint8_t n, uint8_t faces) {
  display.clearDisplay();
  int values=0;
  int diceSize = 20;
  int spacing = diceSize + 4;
  int cols = min(n, 2);
  int rows = (n + 1) / 2;

  int startX = (SCREEN_WIDTH - cols * spacing + 4) / 2;
  int startY = (SCREEN_HEIGHT - rows * spacing + 4) / 2;

  for (int i = 0; i < n; i++) {
    int col = i % 2;
    int row = i / 2;
    int x0 = startX + col * spacing;
    int y0 = startY + row * spacing;

    int cx = x0 + diceSize / 2;
    int cy = y0 + diceSize / 2;
    int offset = diceSize / 4;
    int dotRadius = diceSize / 10;

    int value = random(1, faces + 1);
    Serial.print("Die "); Serial.print(i + 1);
    Serial.print(": value="); Serial.println(value);

    // --- CASE 1: D4 (triangular) ---
    if (faces == 4) {
      // Draw triangle (like a d4)
      int half = diceSize / 2;
      int topX = cx;
      int topY = y0;
      int leftX = x0;
      int leftY = y0 + diceSize;
      int rightX = x0 + diceSize;
      int rightY = y0 + diceSize;

      display.drawLine(leftX, leftY, topX, topY, SSD1306_WHITE);
      display.drawLine(topX, topY, rightX, rightY, SSD1306_WHITE);
      display.drawLine(rightX, rightY, leftX, leftY, SSD1306_WHITE);

      // Display the rolled number inside
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(cx - 2, cy - 1);
      display.print(value);
      values=values+value;

    }
    // --- CASE 2: Standard 6-sided or higher ---
    else {
      // Draw square die
      display.drawRect(x0, y0, diceSize, diceSize, SSD1306_WHITE);

      if (faces > 6) 
      {
        // Display number if above 6
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);

        // Measure approximate text width (each character ≈ 6 pixels wide at size 1)
        int textWidth = String(value).length() * 6;
        int textHeight = 8; // standard font height

        // Compute centered cursor position
        int textX = cx - textWidth / 2;
        int textY = cy - textHeight / 2;

        display.setCursor(textX, textY);
        display.print(value);
        values=values+value;

      }
      else {
        // Draw dots (standard die)
        switch (value) {
          case 1:
            dot(cx, cy, dotRadius);
            values=values+value;
            break;
          case 2:
            dot(x0 + offset, y0 + offset, dotRadius);
            dot(x0 + diceSize - offset, y0 + diceSize - offset, dotRadius);
            values=values+value;
            break;
          case 3:
            dot(cx, cy, dotRadius);
            dot(x0 + offset, y0 + offset, dotRadius);
            dot(x0 + diceSize - offset, y0 + diceSize - offset, dotRadius);
            values=values+value;
            break;
          case 4:
            dot(x0 + offset, y0 + offset, dotRadius);
            dot(x0 + diceSize - offset, y0 + offset, dotRadius);
            dot(x0 + offset, y0 + diceSize - offset, dotRadius);
            dot(x0 + diceSize - offset, y0 + diceSize - offset, dotRadius);
            values=values+value;
            break;
          case 5:
            dot(cx, cy, dotRadius);
            dot(x0 + offset, y0 + offset, dotRadius);
            dot(x0 + diceSize - offset, y0 + offset, dotRadius);
            dot(x0 + offset, y0 + diceSize - offset, dotRadius);
            dot(x0 + diceSize - offset, y0 + diceSize - offset, dotRadius);
            values=values+value;
            break;
          case 6:
            dot(x0 + offset, y0 + offset, dotRadius);
            dot(x0 + diceSize - offset, y0 + offset, dotRadius);
            dot(x0 + offset, cy, dotRadius);
            dot(x0 + diceSize - offset, cy, dotRadius);
            dot(x0 + offset, y0 + diceSize - offset, dotRadius);
            dot(x0 + diceSize - offset, y0 + diceSize - offset, dotRadius);
            values=values+value;
            break;
        }
      }
    }
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,55);
  display.printf("Sum of the values:");
  display.println(values);
  display.display();
}

void drawex(uint8_t n, uint8_t faces) {
  display.clearDisplay();
  int values=0;
  int diceSize = 20;
  int spacing = diceSize + 4;
  int cols = min(n, 2);
  int rows = (n + 1) / 2;

  int startX = (SCREEN_WIDTH - cols * spacing + 4) / 2;
  int startY = (SCREEN_HEIGHT - rows * spacing + 4) / 2;

  for (int i = 0; i < n; i++) {
    int col = i % 2;
    int row = i / 2;
    int x0 = startX + col * spacing;
    int y0 = startY + row * spacing;

    int cx = x0 + diceSize / 2;
    int cy = y0 + diceSize / 2;
    int offset = diceSize / 4;
    int dotRadius = diceSize / 10;

    int value = faces;
    Serial.print("Die "); Serial.print(i + 1);
    Serial.print(": value="); Serial.println(value);

    // --- CASE 1: D4 (triangular) ---
    if (faces == 4) {
      // Draw triangle (like a d4)
      int half = diceSize / 2;
      int topX = cx;
      int topY = y0;
      int leftX = x0;
      int leftY = y0 + diceSize;
      int rightX = x0 + diceSize;
      int rightY = y0 + diceSize;

      display.drawLine(leftX, leftY, topX, topY, SSD1306_WHITE);
      display.drawLine(topX, topY, rightX, rightY, SSD1306_WHITE);
      display.drawLine(rightX, rightY, leftX, leftY, SSD1306_WHITE);

      // Display the rolled number inside
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(cx - 2, cy - 1);
      display.print(value);
      values=values+value;

    }
    // --- CASE 2: Standard 6-sided or higher ---
    else {
      // Draw square die
      display.drawRect(x0, y0, diceSize, diceSize, SSD1306_WHITE);

      if (faces >= 6) 
      {
        // Display number if above 6
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);

        // Measure approximate text width (each character ≈ 6 pixels wide at size 1)
        int textWidth = String(value).length() * 6;
        int textHeight = 8; // standard font height

        // Compute centered cursor position
        int textX = cx - textWidth / 2;
        int textY = cy - textHeight / 2;

        display.setCursor(textX, textY);
        display.print(value);
        values=values+value;

      }
    }
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.printf("Dices:%d,  d%d",n ,faces);
  display.display();
}

typedef struct 
{
  Vec3f w;
  Vec3f a;
  uint32_t cycle_time, last_cycle_time; // IMU cycle tracking
} imu_values_t;
imu_values_t imu;

enum {
  fsm_start = 0,
  fsm_detect = 1,
  fsm_roll = 2,
};

typedef struct {
  int state;
  unsigned long tes;  // Time entering state
  int movement_count; // Count consecutive movement cycles
} fsm_d;

fsm_d fsmd;

void set_state(fsm_d &fsm, int new_state) {
  if (fsm.state != new_state) {
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.movement_count = 0; // reset counter on state change
  }
}

void print_display(fsm_t& fsm, int countmenu){
    //MENU DISPLAY  
    
  if (fsm.state == fsm_menu && fsm.tis % 1500 <= 750){
    display.clearDisplay();
    display.setCursor(52,0);
    display.printf("Menu");
    display.setCursor(0,16);
    display.printf("Calibrate ");
    display.setCursor(0,24);
    display.printf("Play ");
    display.setCursor(0,32);
    display.printf("Set parameters");
    display.setCursor(0,40);
    display.printf("Calibrated values");
    display.display();
  }
  
    if (countmenu == 1 && fsm.tis % 1500 >= 750){
    display.clearDisplay();
    display.setCursor(52,0);
    display.printf("Menu");
    display.setCursor(0,16);
    display.printf("Calibrate <-");
    display.setCursor(0,24);
    display.printf("Play ");
    display.setCursor(0,32);
    display.printf("Set parameters");
    display.setCursor(0,40);
    display.printf("Calibrated values");
    display.display();
    }
    if (countmenu == 2 && fsm.tis % 1500 >= 750){
    display.clearDisplay();
    display.setCursor(52,0);
    display.printf("Menu");
    display.setCursor(0,16);
    display.printf("Calibrate ");
    display.setCursor(0,24);
    display.printf("Play <-");
    display.setCursor(0,32);
    display.printf("Set parameters");
    display.setCursor(0,40);
    display.printf("Calibrated values");
    display.display();
    }
  if (countmenu == 3 && fsm.tis % 1500 >= 750){
    display.clearDisplay();
    display.setCursor(52,0);
    display.printf("Menu");
    display.setCursor(0,16);
    display.printf("Calibrate ");
    display.setCursor(0,24);
    display.printf("Play ");
    display.setCursor(0,32);
    display.printf("Set parameters <-");
    display.setCursor(0,40);
    display.printf("Calibrated values");
    display.display();
    }
    if (countmenu == 4 && fsm.tis % 1500 >= 750){
    display.clearDisplay();
    display.setCursor(52,0);
    display.printf("Menu");
    display.setCursor(0,16);
    display.printf("Calibrate ");
    display.setCursor(0,24);
    display.printf("Play ");
    display.setCursor(0,32);
    display.printf("Set parameters");
    display.setCursor(0,40);
    display.printf("Calibrated values <-");
    display.display();
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

  set_state(fsm1, fsm_begin); //inicializa maquina estados
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

    // ----------------- Read the buttons (RISING EDGE) -----------------
    update_buttons();

    // Mantém estas flags para o hold de 2s já implementado
    SOKButton   = SOK.value;
    SESCButton  = SESC.value;
    SNEXTButton = SNEXT.value;

    // ------------------ TRANSIÇÕES ------------------

    // Not Pressed -> Pressing (pressionar OK)
    if (fsm1.state == fsm_begin && SOKButton) {
      set_state(fsm1, fsm_pressing);
    }

    // Pressing -> Menu (OK pressionado por 2s)
    if (fsm1.state == fsm_pressing && fsm1.tis >= 2000 && SOKButton) {
      set_state(fsm1, fsm_menu);
      count_menu=1;
    }

    // Pressing -> Not Pressed (soltar OK)
    if (fsm1.state == fsm_pressing && !SOKButton) {
      set_state(fsm1, fsm_begin);
    }

    // Menu -> Not Pressed (ESC rising edge)
    if (fsm1.state == fsm_menu && SESC.re) {
      set_state(fsm1, fsm_begin);
    }

    // Menu -> Calibrating (count_menu==1, OK rising edge)
    if (fsm1.state == fsm_menu && count_menu == 1 && SOK.re) {
      wx_calibrated = wy_calibrated = wz_calibrated = 0;
      ax_calibrated = ay_calibrated = az_calibrated = 0;
      set_state(fsm1, fsm_calibrating);
    }

    // Menu -> Show Values (count_menu==4, OK rising edge)
    if (fsm1.state == fsm_menu && count_menu == 4 && SOK.re) {
      set_state(fsm1, fsm_show_values);
      count_menu = 1;
    }

    // Menu -> Not Calibrated (count_menu==2, OK rising edge, !is_Calibrated)
    if (fsm1.state == fsm_menu && count_menu == 2 && SOK.re && !is_Calibrated) {
      set_state(fsm1, fsm_notCalibrated);
      count_menu = 1;
    }

    // Menu -> Parameters (count_menu==3, OK rising edge)
    if (fsm1.state == fsm_menu && count_menu == 3 && SOK.re) {
      set_state(fsm1, fsm_parameters);
      count_menu = 1;
    }

    // Menu -> Playing (count_menu==2, OK rising edge, is_Calibrated)
    if (fsm1.state == fsm_menu && count_menu == 2 && SOK.re && is_Calibrated) {
      set_state(fsm1, fsm_play);
      count_menu = 1;
    }

    // Avançar opção no menu (NEXT rising edge)
    if (fsm1.state == fsm_menu && SNEXT.re) {
      count_menu = (count_menu % 4) + 1;
    }

    // -------- OUTROS ESTADOS COM RISING EDGES --------

    // Show Values -> Menu (ESC rising edge)
    if (fsm1.state == fsm_show_values && SESC.re) {
      set_state(fsm1, fsm_menu);
    }

    // Not Calibrated -> Menu (timeout de 3s)
    if (fsm1.state == fsm_notCalibrated && fsm1.tis >= 3000) {
      set_state(fsm1, fsm_menu);
    }

    // Calibrating -> Calibrated (condição de tempo + médias)
    if (fsm1.state == fsm_calibrating && fsm1.tis >= 2000) {
      set_state(fsm1, fsm_calibrated);
      wx_calibrated = wx_total / count_calibrate;
      wy_calibrated = wy_total / count_calibrate;
      wz_calibrated = wz_total / count_calibrate;
      ax_calibrated = ax_total / count_calibrate;
      ay_calibrated = ay_total / count_calibrate;
      az_calibrated = (az_total / count_calibrate);
    }

    // Calibrated -> Menu (condição de tempo + reset)
    if (fsm1.state == fsm_calibrated && fsm1.tis >= 3000) {
      wx_total = 0;
      wy_total = 0;
      wz_total = 0;
      ax_total = 0;
      ay_total = 0;
      az_total = 0;
      count_calibrate = 0;
      is_Calibrated = true;
      set_state(fsm1, fsm_menu);
    }

    // Playing -> Menu (ESC rising edge)
    if (fsm1.state == fsm_play && SESC.re) {
      set_state(fsm1, fsm_menu);
    }

    // Parameters -> Menu (ESC rising edge)
    if (fsm1.state == fsm_parameters && SESC.re) {
      set_state(fsm1, fsm_menu);
    }

    // Parameters: incrementa "possible numbers of dices" (OK rising edge)
    if (fsm1.state == fsm_parameters && SOK.re) {
      param_dice_numbers = (param_dice_numbers + 1) % 4;
      switch (param_dice_numbers) {
        case 0: dice_numbers = 4;  break;
        case 1: dice_numbers = 6;  break;
        case 2: dice_numbers = 10; break;
        case 3: dice_numbers = 20; break;
      }
    }

    // Parameters: incrementa "number_of_dices" (NEXT rising edge)
    if (fsm1.state == fsm_parameters && SNEXT.re) {
      number_of_dices = (number_of_dices % 4) + 1;
    }

    // ---------------- FIM DAS TRANSIÇÕES ----------------


    // ----------------------- OUTPUTS -----------------------

    // OUTPUT PLAY
    if(fsm1.state==fsm_play)
    {
      drawex(number_of_dices,dice_numbers);
      float ax = imu.a.x;
      float ay = imu.a.y;
      bool moving = (fabs(ax) > 1 || fabs(ay) > 1);

      switch (fsmd.state) {

        case fsm_start:
          if (moving) {
            Serial.println("Movement detected -> fsm_detect");
            set_state(fsmd, fsm_detect);
          }
          break;

        case fsm_detect:
          if (moving) {
            fsmd.movement_count++;
            Serial.printf("Movement cycle %d\n", fsmd.movement_count);
            if (fsmd.movement_count >= 3) {
              Serial.println("3 cycles of movement -> fsm_roll");
              set_state(fsmd, fsm_roll);
            }
          } else {
            Serial.println("Movement stopped -> fsm_start");
            set_state(fsmd, fsm_start);
          }
          break;

        case fsm_roll:
          {
            uint8_t diceCount = random(1, 5);
            Serial.printf("Rolling %d dice...\n", diceCount);
            draw(number_of_dices,dice_numbers);
            delay(5000); // <-- mantém do teu primeiro código
            Serial.println("Returning to start state");
            set_state(fsmd, fsm_start);
          }
          break;
      }
    }

    // OUTPUT MENU
    if (fsm1.state == fsm_menu){
      //AQUI ESCREVER O DISPLAY DO MENU !!!
      /*
    
      display.clearDisplay();
      display.setCursor(64,0);
      display.printf("Menu");
      display.setCursor(0,16);
      display.printf("Calibrate");
      display.setCursor(0,32);
      display.printf("Show accelerometer values");
      display.printf("Count = %d\n", count_menu);
      display.display();
      
      */
     print_display(fsm1, count_menu);
    }

    // OUTPUT NOT_PRESSED OU PRESSING
    if(fsm1.state == fsm_begin || fsm1.state == fsm_pressing){
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

      Serial.print("loop ");
      Serial.print(micros() - now);

      Serial.println();
    }

    // OUTPUT SHOW VALUES
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
      display.printf("Az %.2f", imu.a.z - az_calibrated + 1);

      display.display();
    }

    // Ação estado calibrating (acumular amostras)
    if (fsm1.state == fsm_calibrating){
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

    // Ação estado Calibrated (prints)
    if (fsm1.state == fsm_calibrated){
      Serial.printf("Wx_calibrated: %.2f",wx_calibrated);
      Serial.printf("Wy_calibrated: %.2f",wy_calibrated);
      Serial.printf("Wz_calibrated: %.2f",wz_calibrated);
      Serial.printf("Ax_calibrated: %.2f",ax_calibrated);
      Serial.printf("Ay_calibrated: %.2f",ay_calibrated);
      Serial.printf("Az_calibrated: %.2f",az_calibrated);
    }

    // OUTPUT PARAMETERS
    if (fsm1.state == fsm_parameters){
      Serial.printf("Number of dices :%d \n",number_of_dices);
      Serial.printf("Possible numbers : D%d \n",dice_numbers);
      drawex(number_of_dices,dice_numbers);
      //FAZER DISPLAY INTERFACE
    }
  }
}
