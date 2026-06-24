#define THERMISTOR_PIN      A2      // Термистор
#define IR_SENSOR_PIN       A1      // Термистор
#define RINSE_AID_PIN       A0      // Термистор

#define DISPLAY_DIO_PIN     36      
#define DISPLAY_CLK_PIN     38

#define SALT_SENSOR_PIN     48     
#define FULL_SENSOR_PIN     50
#define FLOOD_SENSOR_PIN    52
#define DOOR_SENSOR_PIN     23

//#define IR_LIGHT_PIN        6
//#define RINSE_AID_LIGHT_PIN 6
#define LIGHT_1_PIN         6
#define LIGHT_2_PIN         7
#define BUSY_LIGHT_PIN      8
#define BUZZER_PIN          12

#define BUTTON_1_PIN        4
#define BUTTON_2_PIN        5

#define DRAIN_REL_PIN       22
#define INPUT_REL_PIN       24
#define STEEPER_REL_PIN     26
#define SOLENOID_1_REL_PIN  28
#define SOLENOID_2_REL_PIN  30
#define CAPSULE_REL_PIN     32
#define MAIN_PUMP_REL_PIN   34

#define SERIES_RESISTOR 50000.0   // Номинал постоянного резистора
#define THERM_NOMINAL 48900.0     // Сопротивление при номинальной температуре
#define TEMPERATURE_NOMINAL 25.0  // Номинальная температура (обычно 25°C)
#define B_COEFFICIENT 3950.0      // B-коэффициент


#define DRAIN_1_TIME 20000

#define EB_NO_COUNTER
#define EB_NO_CALLBACK
#define EB_NO_BUFFER
#include "GyverSegment.h"
#include "GyverTimer.h"
#include "EncButton.h"


Disp1637Colon disp(DISPLAY_DIO_PIN, DISPLAY_CLK_PIN, true);
SegRunner runDisp(&disp); 

Button btn1(BUTTON_1_PIN);
Button btn2(BUTTON_2_PIN);
MultiButton btn3;

GTimer delTmr(MS);

bool paused = false;
bool timerPauseFlag = false;
bool isFull = false; //true - full, false - empty
bool isSalt = false; //true - 
bool isDoor = false; //false - close, true` - open
bool isFlood = false; //true - close, false - open
int wash_stage = 0;


void dispPrint(const char str[4] = "null") {
  disp.setCursor(0);
  disp.print(str);
  disp.update();
}

void colonTick() {
  static uint32_t strobe_colon_timer = 0; 
  static bool colon_state = true;

  if (millis() - strobe_colon_timer > 500) {
    strobe_colon_timer = millis();
    colon_state = !colon_state;
    disp.colon(colon_state);
  }
}

void runningStringPrint(const char str[16] = "no info") {
  runDisp.setText(str);
  runDisp.start();
  runDisp.waitEnd();
}

float getTemperature() {
  float steinhart;

  steinhart = SERIES_RESISTOR / (1023.0 / (float)analogRead(THERMISTOR_PIN) - 1.0) / THERM_NOMINAL;
  steinhart = log(steinhart);                      // ln(R/Ro)
  steinhart /= B_COEFFICIENT;                      // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                     // Переворачиваем
  steinhart -= 273.15;                             // Переводим Кельвины в Цельсии

  return steinhart;
}


void relayAllOff() { //выключение всех реле
  for (byte i = DRAIN_REL_PIN; i < MAIN_PUMP_REL_PIN +1; i+=2) {
    digitalWrite(i, HIGH);
  }
}

void debug() {
  static uint32_t tm = millis();
  if (millis() - tm > 1000) {
    tm = millis();
    Serial.print("Wash stage: ");
    Serial.print(wash_stage);
    Serial.print("\n\rTemperature: ");
    Serial.print(getTemperature());
    Serial.print("\n\rSalt: ");
    Serial.print(digitalRead(SALT_SENSOR_PIN));
    Serial.print("\n\rIs Full: ");
    Serial.print(digitalRead(FULL_SENSOR_PIN));
    Serial.print("\n\rDoor: ");
    Serial.print(digitalRead(DOOR_SENSOR_PIN));
    Serial.print("\n\rB-1: ");
    Serial.print(digitalRead(BUTTON_1_PIN));
    Serial.print("\n\rB-2: ");
    Serial.print(digitalRead(BUTTON_2_PIN));
    Serial.print("\n\rFlood: ");
    Serial.print(digitalRead(FLOOD_SENSOR_PIN));
    Serial.println("\n\r");
  }
}

void sensorsHandler() {
  isFull = !digitalRead(FULL_SENSOR_PIN);
  isSalt = !digitalRead(SALT_SENSOR_PIN);
  isDoor = !digitalRead(DOOR_SENSOR_PIN);
  isFlood = !digitalRead(FLOOD_SENSOR_PIN);
}

void defPins() {
  digitalWrite(LIGHT_1_PIN, LOW);
  digitalWrite(LIGHT_2_PIN, LOW);
  digitalWrite(BUSY_LIGHT_PIN, LOW);
}

bool waitAsync(uint32_t waitTime) {
  if (timerPauseFlag) {
    timerPauseFlag = false;
    delTmr.resume();
  }

  if (!delTmr.isEnabled()) {
    delTmr.setTimeout(waitTime);
    delTmr.start();
  }
  return delTmr.isReady();
}

void wash() {
  if (paused) {
    relayAllOff();
    if (delTmr.isEnabled()) {
      timerPauseFlag = true;
      delTmr.stop();
    }
    return;
  }

  switch (wash_stage) {
  case 0:
    //relayAllOff();
    break;

  case 1:
    digitalWrite(DRAIN_REL_PIN, LOW);

    if (!waitAsync(25000)) return;

    digitalWrite(DRAIN_REL_PIN, HIGH);

    wash_stage++;
    break;

  case 2:
    digitalWrite(INPUT_REL_PIN, LOW); //вкл налив

    if (!isFull) //ожидание налива
      return;

    digitalWrite(INPUT_REL_PIN, HIGH); //выкл налив

    if (!waitAsync(5000)) return;
    wash_stage++; 
    break;

  case 3:
    digitalWrite(MAIN_PUMP_REL_PIN, LOW); //вкл главный насос
    //digitalWrite(STEEPER_REL_PIN, LOW); //вкл поворотный механизм
    
    if (!waitAsync(25000)) return;
    wash_stage++;
    break;

  case 4:
    // digitalWrite(CAPSULE_REL_PIN, LOW);

    if (!waitAsync(500)) return;

    digitalWrite(CAPSULE_REL_PIN, HIGH);
    
    if (!waitAsync(60000)) return;
    wash_stage++;
    break;

  case 5:
    // digitalWrite(CAPSULE_REL_PIN, LOW);

    if (!waitAsync(500)) return;

    digitalWrite(CAPSULE_REL_PIN, HIGH);

    if (!waitAsync(60000)) return;
    wash_stage++;
    break;

  case 6:
    digitalWrite(MAIN_PUMP_REL_PIN, HIGH); //вкл главный насос

    if (!waitAsync(10000)) return;
    wash_stage++;
    break;
  case 7:
    digitalWrite(DRAIN_REL_PIN, LOW);

    if (!waitAsync(35000)) return;

    digitalWrite(DRAIN_REL_PIN, HIGH);

    wash_stage = 0;
    break;
  default:
    break;
  }
}


void buttons() {
  btn3.tick(btn1, btn2);

  if (btn1.click()) {
    wash_stage = 1;
  }

  if (btn3.click()) {
    paused = !paused;
  }

  if (btn2.click()) {
    static bool _St = false;

    digitalWrite(STEEPER_REL_PIN, _St);
    _St = !_St;
  }

}


void protector() {
  if (isFlood) {
    wash_stage = 0;

    relayAllOff();
    dispPrint("flod");
    digitalWrite(DRAIN_REL_PIN, LOW);
    delay(60000);
    digitalWrite(DRAIN_REL_PIN, HIGH);
  }

  paused = isDoor;
  if (isDoor) {
    dispPrint("door");
  }
}

void setup() {
  Serial.begin(115200);
  disp.brightness(2);
  btn1.setBtnLevel(HIGH);
  btn2.setBtnLevel(HIGH);
  
  pinMode(THERMISTOR_PIN,       INPUT);
  pinMode(IR_SENSOR_PIN,        INPUT);
  pinMode(RINSE_AID_PIN,        INPUT);
  pinMode(SALT_SENSOR_PIN,      INPUT_PULLUP);
  pinMode(FULL_SENSOR_PIN,      INPUT_PULLUP);
  pinMode(FLOOD_SENSOR_PIN,     INPUT_PULLUP);
  pinMode(DOOR_SENSOR_PIN,      INPUT_PULLUP);
  //pinMode(IR_LIGHT_PIN,         OUTPUT);
  //pinMode(RINSE_AID_LIGHT_PIN,  OUTPUT);
  pinMode(LIGHT_1_PIN,          OUTPUT);
  pinMode(LIGHT_2_PIN,          OUTPUT);
  pinMode(BUSY_LIGHT_PIN,       OUTPUT);
  pinMode(BUZZER_PIN,           OUTPUT);
  pinMode(DRAIN_REL_PIN,        OUTPUT);
  pinMode(INPUT_REL_PIN,        OUTPUT);
  pinMode(STEEPER_REL_PIN,      OUTPUT);
  pinMode(SOLENOID_1_REL_PIN,   OUTPUT);
  pinMode(SOLENOID_2_REL_PIN,   OUTPUT);
  pinMode(CAPSULE_REL_PIN,      OUTPUT);
  pinMode(MAIN_PUMP_REL_PIN,    OUTPUT);

  relayAllOff();
  defPins();  
  
  //runningStringPrint("kx-home");
  delay(500);
  tone(BUZZER_PIN, 1000, 50);
}

void loop() {
  disp.setCursor(0);
  disp.print(wash_stage);
  disp.update();
  
  debug();
  buttons();
  sensorsHandler();
  protector();
  colonTick();
  wash();
}
