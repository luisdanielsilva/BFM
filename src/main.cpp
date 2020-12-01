#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // YWROBOT based on the work on DFRobot
#include <PushButton.h>        // https://github.com/kristianklein/PushButton
#include <EEPROM.h>            // write and read EEPROM (to save and load settings)

// Function Declaratio
void leave();
void clear_led(int led);
int tray_calibration();
void BlinkLED(int led, int time02, int times);
void rotate_tray(long int _speed, long int _position_space_mm, int step_pin1, int enable_pin1);
void rotate_pump(unsigned long int volume_to_fill, int step_pin1, int enable_pin1, int step_pin2, int enable_pin2, int step_pin3, int enable_pin3);
void rotate_motor(unsigned long int _speed, long int temp_motor_steps, int step_pin1, int enable_pin1);
long int calculate_speed(long int _speed, long int _stepsPerRevolution);
long int calculate_travel_mm(long int ftravel_mm);
void eepromWriteInt(int address, int value);
int eepromReadInt(int address);

//************************************************************************************************
// INFORMATION
// 23 - ADD SUPPORT TO AND ADDITIONAL PUMP TO TEST 1 PUMP + TRAY
// 24 - ADD SUPPORT FOR PUMP DIRECTION
// 25 - ADD SUPPORT FOR 2 PUMPS AT THE SAME TIME
// 26 - ADD SUPPORT FOR MOVING THE TRAY / SENSOR INTERRUPT FOR SENSORS
// 27 - SENSOR INTERRUPT FOR SENSORS
// 28 - CREATE NEW ROTATE FUNCTION FOR TRAY (TO AVOID THE BIT MANIPULATION NEEDED FOR PUMPS/TRAY)
// 29 - SETUP A WATCHDOG TO AVOID BLOCKING THE MACHINE
// 30 - IMPLEMENTED CALIBRATION SEQUENCE USING SENSORS - NOT FINISHED
// 31 - IMPLEMENTED CALIBRATION SEQUENCE USING SENSORS - DIFFERENT STRATEGY
// 32 - CORRECTED BUGS. IMPLEMENTED LED'S + BUTTONS ON MEGA2560
// 33 - FIXED BUG THAT PREVENTED SAME TIME FILLING BY DIFFERENT PUMPS. CORRECTED BUGS
// 34 - IMPLEMENT SMALL FILLING SEQUENCE. IMPLEMENTED INTERRUPT TO BLINK LED
// 35 - MENU BUGS - CHANGED BUTTON PIN DEFINITION + PIN CONFIGURATION FROM INPUT_PULLUP TO INPUT
// 36 - CORRECTED BUTTON BUGS THAT PRESSES MULTIPLE BUTTONS AT THE SAME TIME. CORRECTED THE SELECTION OF THE PUMP DIRECTION
// 37 - CORRECTED MENU SELECTION FOR OPTIONS (TRAY,DIR,PUMP), FROM selected_option -> cursor_position. CORRECTED REMEMBERS PREVIOUS SELECTED OPTION.
// 38 - REMEMBERS MENU SELECTION FOR MODE. START CORRESPONDS TO SELECTED OPTION
// 39 - RESTORED MAIN ACTION FUNCTIONS. CORRECTED SOME MENU BUGS, CLEARED NOT USEFUL COMMENTS
// 40 - CORRECTED BACK BUTTON KEEPING LAST SELECTED OPTION WHEN GOING BACK AND OK AGAIN
// 41 - DOSE MODE SUPPORTS ANY NUMBER OF PUMPS
// 42 - SIMPLIFIED DOSE MODE SELECTION OF PUMPS FROM A SWITCH CASE CYCLE TO A SINGLE STATEMENT. MOVED PUMP LED TO INSIDE ROTATION FUNCTION
// 43 - FEATURE ABOVE TESTED.
// 44 - CHANGED CALIBRATION PROCESS. IF CALIBRATION VALUE THEN IT RECALCULATES THE NUMBER OF NEEDED ROTATIONS
// 45 - ONLY CALIBRATE ONCE THE BACK VALUE IS CLICKED. CLEANED UP CALIBRATION VARIABLES
// 46 - CHANGED CONVERT_ML2ROT -> VOLUME_CALIBRATION(). MOVED CALIBRATION TO THE BACK BUTTON ON THE CALIBRATION ROUTINE
// 47 - REMOVED 2 VARIABLES FROM CALIBRATION PROCESSES; CHANGED DEBOUNCE TIME FROM 250->200; INITIAL TRAY CALIBRATION WORKS BUT A LIMIT TO THE AMOUNT OF ROTATIONS/TIME IS NEEDED IN CASE OF SENSOR FAILURE
// 48 - COUNTING LAPS, RE-ARRANGED INTERRUPTS
// 49 - TRAY CALIBRATION PROCESS IMPLEMENTED. COUNTS LAPS. DISTINGUISHES FROM CALIBRATION AND OPERATION. IMPLEMENTED PROTECTION ON 60 LAPS AS MAXIMUM THEN IT SWITCHED THE MOTOR OFF
// 50 - COMMENTED EEPROM RELATED FUNCTIONS, COMMENTED PUMP SELECTION RELATED FUNCTIONS. REMOVED ENABLE HIGH/LOW FOR DIRECTION SETTING FUNCTION. REPLACED BITWISE FUNCTION FOR A HIGH/LOW FUNCTION FOR MOTOR MOVEMENT SINCE FOR SOME REASON ROTATING THE PUMP WITH BITWISE FUNCTION MADE IT NOT CHANGE DIRECTION PROPERLY
// 51 - REMOVED set_pump_direction(). MADE NO SENSE SINCE WE ARE ALREADY SETTING UP DIRECTION ON THE MENU FUNCTION
// 52 - changed tray_position_offset from 6 to 10.
// 53 - added function to use distance in mm instead of laps; changed stepsPerRevolution to long int;
// 54 - added correct sequence. adjusted interrupt routines. adejusted calibration routine. CALIBRATION ROUTINE WORKING.
// 55 - Simplifying code. replacing some variables and ENUMS in menu variables to make code readabl
// 56 - removed tray_flag_operation, sensor_left, sensor_right, value_dbl variables. Renamed menu_number_1 and 2 to menu_line_1. Moved rotation to initial position to if(both interrupts are triggered) so that it only happens if calibration is successful
// 57 - update_lcd returns boolean result. Renamed variables in eepromWriteInt(), eepromReadInt(). show tray positions on leave(). Generic function to set_direction for a given pin (write 1 or 0 to a pin). Removed set_tray_direction().
// 58 - simplified DIR menu. Solved TRAY movement topic.
// 59 - changed true to HIGH and false to LOW. changed interrupt flags from INT to BOOL. removed LED modifiers to clear_sensor_info so that only one function synchronizes the LED and SENSOR states; Changed Menu name of START DIR to PUMP DIR to better reflect what it is. grouped ENUM declarations
// 60 - performed entire sequence. separate stepsPerRevolution for pumps and tray. adjusted volumes to be dispensed in CAL and DOSE modes. Minor delays added to the main sequence
// 61 - changed LED for each sensor directly on the interrupts. REMOVED LINE: "int menu_items_limit = NUM_OF_MENU_ITEMS - 1;" Changed Sensor LED's. Switched driver, hence also the settings. Enable eeprom load_settings()
// 62 - IT WORKS!!!!
// 63 - moved to VSCODE
// 64 - 
// 65 - 
// 66 - Used GIT to commit the first version

// BUTTONS
#define BUTTON_AUTO_MAN 17
#define BUTTON_AUX_TRAY 16
#define BUTTON_START 19
#define BUTTON_NOLABEL1 14
#define BUTTON_NOLABEL2 15
#define BUTTON_STOP 18

// LED'S
#define LED_STOP 42
#define LED_START 46
#define LED_SENSOR1 44
#define LED_PUMP 45
#define LED_SENSOR2 43

#define LED_AUTOMAN 49
#define LED_AUXTRAY 48
#define LED_NOLABEL 47

#define LED_PIN 13

// MOTOR CONTROL
// TRAY
#define TRAY_DIR 8
#define TRAY_STEP 9
#define TRAY_ENABLE 10

// PUMP 1
#define PUMP1_DIR 40
#define PUMP1_STEP 36
#define PUMP1_ENABLE 38

// PUMP 2 - ASSESS IF THIS IS REALLY NEEDED SINCE CONTROL IS DONE ON A BIT WISE BASE
#define PUMP2_DIR 32
#define PUMP2_STEP 34
#define PUMP2_ENABLE 30

// PUMP 3 - ASSESS IF THIS IS REALLY NEEDED SINCE CONTROL IS DONE ON A BIT WISE BASE
#define PUMP3_DIR 33
#define PUMP3_STEP 35
#define PUMP3_ENABLE 31

// SENSORS
#define INT_SENSOR_1 2
#define INT_SENSOR_2 3

// DIRECTIONS
#define CW 1
#define CC 0

//CALIBRATION -----------------------------------------------------------------------------
float system_calibration_constant = 0.6;
long int system_calibration_offset = 0.0;
long int system_calibration_calculated = 0.0;

// MOTORS
const long int stepsPerRevolution_tray = 200;         // number of steps needed for one revolution
const long int stepsPerRevolution_pumps = 200;        // number of steps needed for one revolution
const int Tray_Speed = 1200;                          // Speed in RPM
const int Pump_Speed = 1000;                          // Speed in RPM

// TRAY
int tray_next_position = 0;                           // next position
int tray_current_position = 0;                        // current position
int tray_last_position = 0;                           // last position
bool tray_flag_calibration = LOW;                     // flag to say if a calibration has been done or not
bool tray_flag_calibration_ongoing = LOW;
int temp_tray_position = 0;                           // global variable to control fwd/bwd movement of tray
int tray_home_position = 0;                           // tray home position
const int tray_max_positions = 10;                    // amount of positions
bool flag_sequence = 0;                               // flag to say when the tray is coming back as part of a sequence and not from manual movement
bool flag_calibration = 0;                            // flag to say when the tray is coming back as part of a CALIBRATION SEQUENCE
volatile bool interrupt_flag_HOME = LOW;              // flag to signal that has gone to the interrupt and back
volatile bool interrupt_flag_END = LOW;               // flag to signal that has gone to the interrupt and back

// OUTPUT
long int steps_mm = 0;
long int travel_steps = 0;
// TRAY SETTINGS
const int microstepping = 16;             // DEPENDS ON DRIVER -> TMC2130 = 16, TB6600 = 4
const int leadscrew_pitch = 8;            // FIXED: axle characteristic
// POSITIONS SETTINGS
const long int tray_position_interval = 20;         // distance between each position
const long int tray_position_start_stop_delta = 12; // used in calibration. distance between sensors and first/last position
const long int tray_position_calibration_interval = 500; // used in calibration. distance between sensors and first/last position
long int tray_position_movement = 0;
#define MINUTES 60

// BUTTONS
PushButton upButton(BUTTON_AUX_TRAY);
PushButton downButton(BUTTON_AUTO_MAN);
PushButton OKButton(BUTTON_START);
PushButton backButton(BUTTON_STOP);
PushButton auxButton(BUTTON_NOLABEL2);

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

// MENU CONTROL FLAGS ------------------------------------------------------------------------------------
boolean entered_valueoption = 0;
volatile boolean selected_action = 0;
boolean entered_menu = 0;
boolean selected_option = 0;
boolean flag_option = LOW;
int iterator = 0;
#define DEBOUNCE_TIME 200
#define MENU_GENERAL_TIME 50
#define SECONDS5 5000
#define SECONDS2 2000
#define SECONDS1 1000
#define SECONDS05 500
#define SECONDS025 250

enum menu_list
{
  START,
  MODE,
  VOLUME,
  CALIBRATION,
  PUMPS,
  PUMP_DIR,
  TRAY,
  SAVE_SETTINGS,
  RESET_SETTINGS
};

enum menu_mode
{
  MENU_MODE_SEQ,
  MENU_MODE_CAL,
  MENU_MODE_DOSE
};

enum menu_type
{
  VALUE,
  OPTION,
  ACTION
};

// Action Control Flags  ---------------------------------------------------------------------------------
boolean flag_menu_cursor = LOW;

// MENU --------------------------------------------------------------------------------------------------
#define MAX_NUM_OF_OPTIONS      4
#define NUM_OF_MENU_ITEMS       9
#define VALUE_MAX_DIGITS        4
#define PUMP1 0
#define PUMP2 1
#define PUMP3 2
int menu_line_1 = 0;
int menu_line_2 = 1;
boolean flag_lcd_update_change_value = HIGH;
char value_str[VALUE_MAX_DIGITS + 1];
int cursor_position = 0;


typedef struct
{
  const char *name_;
  menu_type type; //0: value type, 1:option type, 2:action type
  int value;
  int decimals;
  int lim;
  const char *options[4];
  const char *suffix;
  int option_selected;
} menu_item;
menu_item menu[NUM_OF_MENU_ITEMS];

byte menuCursor[8] = {
    B01000, //  *
    B00100, //   *
    B00010, //    *
    B00001, //     *
    B00010, //    *
    B00100, //   *
    B01000, //  *
    B00000  //
};

// ***********************************************************          FUNCTIONS          **************************************************************

void leave()
{
  Serial.println("-------  LEAVING   -------");

  Serial.print(F("LAST Cursor Position: ")); // show last cursor position
  Serial.println(cursor_position);
  Serial.print(F("LAST Option(s) Selected: ")); // show last selected option to see if it coincides with cursor position
  Serial.println(menu[menu_line_1].option_selected);
  Serial.print(F("Positions: "));
  Serial.print(tray_last_position);
  Serial.print(F(" - "));
  Serial.print(tray_current_position);
  Serial.print(F(" - "));
  Serial.println(tray_next_position);
  Serial.print(F("MODE:        "));
  Serial.println(menu[1].option_selected);
  Serial.print(F("PUMPS:       "));
  Serial.println(menu[4].option_selected);
  Serial.print(F("PUMP DIR:    "));
  Serial.println(menu[5].option_selected);
  Serial.print(F("TRAY:        "));
  Serial.println(menu[6].option_selected);
  Serial.print(F("VOLUME:     "));
  Serial.print(menu[2].value);
  Serial.println(" ml");
  Serial.print(F("CALIBRATION: "));
  Serial.print(menu[3].value);
  Serial.println(" ml");

  selected_option = LOW;
  entered_valueoption = LOW;
  selected_action = LOW;
  entered_menu = LOW;
  cursor_position = 0;

  clear_led(LED_STOP);

  flag_lcd_update_change_value = HIGH; // update lcd as soon as we leave any menu
}

void eeprom_savesettings()
{
  // Save all settings currently selected to EEPROM
  // Values, options, etc
  Serial.println(F("SAVING EEPROM SETTINGS"));

  eepromWriteInt(0, menu[1].option_selected); // MODE
  delay(SECONDS5);
  eepromWriteInt(2, menu[2].value); // FILLING VOLUME
  delay(SECONDS05);
  eepromWriteInt(4, menu[3].value); // CALIBRATION VOLUME
  delay(SECONDS05);
  eepromWriteInt(6, menu[4].option_selected); // PUMP SELECTION
  delay(SECONDS05);
  eepromWriteInt(8, menu[5].option_selected); // PUMP DIRECTION
  delay(SECONDS05);

  Serial.println(F("EEPROM CONTENT: "));
  Serial.print(F("Mode: "));
  Serial.println(eepromReadInt(0));
  Serial.print(F("Volume: "));
  Serial.println(eepromReadInt(2));
  Serial.print(F("Calibration: "));
  Serial.println(eepromReadInt(4));
  Serial.print(F("Pumps: "));
  Serial.println(eepromReadInt(6));
  Serial.print(F("Start Dir: "));
  Serial.println(eepromReadInt(8));

  Serial.println(F("EEPROM SETTINGS SAVED !!"));
}

void eeprom_loadsettings()
{
  // Load all settings previously save to EEPROM
  // Values, options, etc
  menu[1].option_selected = eepromReadInt(0); // MODE
  delay(SECONDS5);
  menu[2].value = eepromReadInt(2); // FILLING VOLUME
  delay(SECONDS05);
  menu[3].value = eepromReadInt(4); // CALIBRATION VOLUME
  delay(SECONDS05);
  menu[4].option_selected = eepromReadInt(6); // PUMP SELECTION
  delay(SECONDS05);
  menu[5].option_selected = eepromReadInt(8); // PUMP DIRECTION
  delay(SECONDS05);
  Serial.println(F("EEPROM SETTINGS LOADED !!"));
}

void eeprom_resetsettings()
{
  menu[1].option_selected = 0;             // MODE
  
  menu[2].value = 0;                       // FILLING VOLUME
  
  menu[3].value = 0;                       // CALIBRATION VOLUME
  
  menu[4].option_selected = 0;             // PUMP SELECTION
  
  menu[5].option_selected = 0;             // PUMP DIRECTION

  Serial.println("Settings reset!");
}

boolean update_lcd()
{

  if (flag_lcd_update_change_value == HIGH)
  {
    lcd.clear(); // clear display and set cursor in origin
    lcd.setCursor(0, 0);

    lcd.print(menu_line_1); // print 1st line
    lcd.print("|");
    lcd.print(menu[menu_line_1].name_);

    lcd.setCursor(0, 1); // set cursor to 2nd line

    lcd.print(menu_line_2); // print 2nd line
    lcd.print("|");
    lcd.print(menu[menu_line_2].name_);

    lcd.setCursor(0, 0); // set cursor back to origin

    lcd.blink(); // blink cursor
  }
  flag_lcd_update_change_value = LOW;

  return flag_lcd_update_change_value;
}

bool initialization()
{
  if (!tray_flag_calibration)
  {
    // Title Screen
    lcd.backlight();
    lcd.setCursor(3, 0);
    lcd.print(F("BOOTING ..."));
    lcd.setCursor(0, 2);
    lcd.print(F("Pentagram, Lda"));
    delay(SECONDS05);
    lcd.clear();

    Serial.print(F("Booting ..."));

    // Blink LED's
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_AUXTRAY, HIGH);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_AUTOMAN, HIGH);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_START, HIGH);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_STOP, HIGH);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_SENSOR2, HIGH);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_PUMP, HIGH);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_SENSOR1, HIGH);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_NOLABEL, HIGH);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_AUXTRAY, LOW);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_AUTOMAN, LOW);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_START, LOW);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_STOP, LOW);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_SENSOR2, LOW);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_PUMP, LOW);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_SENSOR1, LOW);
      delay(MENU_GENERAL_TIME);
      digitalWrite(LED_NOLABEL, LOW);
      delay(MENU_GENERAL_TIME);

      Serial.print(" ");
      Serial.print(i + 1);
      Serial.print(" ...");
    }

    delay(SECONDS1);

    Serial.println(" READY!");
    tray_flag_calibration = tray_calibration(); // perform tray calibration and set the flag.
    //  tray_flag_calibration = 1;                      // added to skip calibration commented function above this one

    if (tray_flag_calibration != 0)
    {
      Serial.println(F("Calibration: DONE!"));
      Serial.print(F("Calibration Flag: "));
      Serial.println(tray_flag_calibration);
      BlinkLED(LED_START, 125, 50);
    }
    if (tray_flag_calibration == 0)
    {
      Serial.println(F("Calibration ERROR!"));
      Serial.print(F("Calibration Flag: "));
      Serial.println(tray_flag_calibration);
      BlinkLED(LED_STOP, 125, 50); // maybe create an interrupt so the RED led is always blinking if calibration was not performed and not allow the machine to work without calibration
    }
  }
  update_lcd();

  return tray_flag_calibration;
}

void eepromWriteInt(int address, int value)
{
  //http://shelvin.de/eine-integer-zahl-in-das-arduiono-eeprom-schreiben/

  byte low, high;
  low = value & 0xFF;
  high = (value >> 8) & 0xFF;
  EEPROM.write(address, low); // dauert 3,3ms
  EEPROM.write(address + 1, high);
  return;
}

int eepromReadInt(int address)
{
  //http://shelvin.de/eine-integer-zahl-in-das-arduiono-eeprom-schreiben/

  byte low, high;
  low = EEPROM.read(address);
  high = EEPROM.read(address + 1);
  return low + ((high << 8) & 0xFF00);
}

void BlinkLED(int led, int time02, int times)
{
  for (int i = 0; i <= times; i++)
  {
    digitalWrite(led, !digitalRead(led));
    delay(time02);
  }
}

bool set_direction(int pin, bool pin_direction)
{
  // tray has no configurable direction, only starting DIRECTION always goes from HOME to END
  digitalWrite(pin, pin_direction);
  return digitalRead(pin);
}

int set_tray_position(int tray_target_position)
{

  int tray_position_difference = 0; // variable that holds the amount of positions between past and next

  if (tray_flag_calibration == HIGH && (interrupt_flag_HOME == LOW && interrupt_flag_END == LOW))
  {
    if (tray_current_position <= tray_max_positions)
    {
      if (tray_target_position > tray_current_position)
      {
        tray_position_difference = tray_target_position - tray_current_position;
        set_direction(TRAY_DIR, CW);
        Serial.print("Tray Position Difference - FWD: "); // I should define the direction here.
        Serial.println(tray_position_difference);
      }
      if (tray_target_position < tray_current_position)
      {
        tray_position_difference = tray_current_position - tray_target_position;
        set_direction(TRAY_DIR, CC);
        Serial.print("Tray Position Difference - BWD: ");
        Serial.println(tray_position_difference);
      }
      tray_position_movement = tray_position_difference * tray_position_interval;

      Serial.print("Position to move: ");
      Serial.println(tray_position_difference);
      Serial.print("Interval between positions: ");
      Serial.println(tray_position_interval);
      Serial.print("Total Movement (mm): ");
      Serial.println(tray_position_movement);

      rotate_tray(Tray_Speed, tray_position_movement, TRAY_STEP, TRAY_ENABLE);

      tray_last_position = tray_current_position;
      tray_current_position = tray_target_position;
      tray_next_position = tray_current_position + 1;
    }
    else if (tray_current_position > tray_max_positions)
    {
      //MOVE TO POSITION 1
      Serial.println("NOT MOVING!");
      Serial.println("tray_current_position > tray_max_positions");

      tray_last_position = 0;
      tray_next_position = 0;
      tray_current_position = 0;
    }
  }
  else if (tray_flag_calibration == LOW)
  {
    Serial.println(F("Please perform calibration by restarting the machine !!"));
    tray_current_position = 255;
    tray_next_position = 255;
    tray_last_position = 255;
  }

  Serial.print("LAST POSITION:    ");
  Serial.println(tray_last_position);
  Serial.print("CURRENT POSITION: ");
  Serial.println(tray_current_position);
  Serial.print("NEXT POSITION:    ");
  Serial.println(tray_next_position);

  return tray_current_position;
}

void clear_sensor_info()
{
  // clear sensor flags and turn OFF LED's
  // END sensor
  interrupt_flag_END = LOW;

  // HOME SENSOR
  interrupt_flag_HOME = LOW;

  // flag was cleaned but if by any reason the sensor is triggered, tray is stopped and does not run. SAFETY!
  if ((interrupt_flag_HOME == HIGH && interrupt_flag_END == HIGH))
  {
    digitalWrite(TRAY_ENABLE, HIGH); // TURN-OFF DRIVER
    Serial.println("Sensor Error - TRAY STOPPED!");
    digitalWrite(LED_STOP, HIGH); // Signal STOP for error
  }
}

void clear_position_info()
{
  tray_last_position = 0;
  tray_next_position = 0;
  tray_current_position = 0;
}

// PUMPS
long int volume_calibration_steps(int volumetofill)
{
  // this function converts volume to motor steps to be used in the dispense() function
  // if there is a calibration value it uses it to calculate the corresponding motor steps
  long int pump_steps = 0;
  int temp_volume = 0;

  // system_calibration_constant is used because we only have a rotation <-> ml relationship and not a steps <-> ml relationship
  // pump_steps = volumetofill * (stepsPerRevolution_pumps / system_calibration_constant);

  if (menu[3].value != 0)
  {
    if (menu[3].value >= menu[2].value)
    {
      temp_volume = menu[3].value - menu[2].value;
      temp_volume = volumetofill - temp_volume;
      //Serial.println("CAL > VOL ");
    }
    else if (menu[3].value < menu[2].value)
    {
      temp_volume = menu[2].value - menu[3].value;
      temp_volume = volumetofill + temp_volume;
      //Serial.println("CAL < VOL ");
    }
  }

  //Serial.print("TEMP Volume: ");
  //Serial.println(temp_volume);  

  pump_steps = temp_volume * (stepsPerRevolution_pumps / system_calibration_constant);

  Serial.print("Volume to Fill: ");
  Serial.print(volumetofill);
  Serial.println(" ml");
  Serial.print("PUMP STEPS: ");
  Serial.println(pump_steps);

  return pump_steps;
}

void dispense(long int volume_to_fill, int pumps)
{
  switch (pumps)
  {
  case 1:
    Serial.println("USING PUMPS: 1");
    rotate_pump(volume_to_fill, PUMP1_STEP, PUMP1_ENABLE, 0, 0, 0, 0);
    break;
  case 2:
    Serial.println("USING PUMPS: 1, 2");
    rotate_pump(volume_to_fill, PUMP1_STEP, PUMP1_ENABLE, PUMP2_STEP, PUMP2_ENABLE, 0, 0);
    break;
  case 3:
    Serial.println("USING PUMPS: 1, 2, 3");
    rotate_pump(volume_to_fill, PUMP1_STEP, PUMP1_ENABLE, PUMP2_STEP, PUMP2_ENABLE, PUMP3_STEP, PUMP3_ENABLE);
    break;
  }
}

void rotate_pump(unsigned long int volume_to_fill, int step_pin1, int enable_pin1, int step_pin2, int enable_pin2, int step_pin3, int enable_pin3)
{
  long int temp_speed_us = 0;

  // convert global Pump_Speed from RPM to microseconds
  temp_speed_us = calculate_speed(Pump_Speed, stepsPerRevolution_pumps);
  
  Serial.print(F("PUMP SPEED: "));
  Serial.print(temp_speed_us);
  Serial.print(F(" uS"));

  digitalWrite(LED_PUMP, HIGH); // turn ON pump LED

  Serial.print(F("DIRECTION SETTING: "));
  Serial.println(menu[menu_line_1].option_selected);
  Serial.print(digitalRead(PUMP1_DIR));
  Serial.print(F(" - "));
  Serial.print(digitalRead(PUMP2_DIR));
  Serial.print(F(" - "));
  Serial.println(digitalRead(PUMP3_DIR));

  if (step_pin2 == 0)
  {
    delay(SECONDS025);

    digitalWrite(enable_pin1, LOW);

    Serial.println("Rotating PUMP: 1");

    for (unsigned long int i = 0; i <= volume_to_fill; i++)
    {
      digitalWrite(PUMP1_STEP, HIGH);
      delayMicroseconds(temp_speed_us);
      digitalWrite(PUMP1_STEP, LOW);
      delayMicroseconds(temp_speed_us);
    }

    digitalWrite(enable_pin1, HIGH);

    delay(SECONDS025);
  }
  else if (step_pin3 == 0)
  {
    delay(SECONDS025);

    digitalWrite(enable_pin1, LOW);
    digitalWrite(enable_pin2, LOW);

    Serial.println("Rotating PUMP: 2");

    for (unsigned long int i = 0; i <= volume_to_fill; i++)
    {
      digitalWrite(PUMP1_STEP, HIGH);
      digitalWrite(PUMP2_STEP, HIGH);
      delayMicroseconds(temp_speed_us);
      digitalWrite(PUMP1_STEP, LOW);
      digitalWrite(PUMP2_STEP, LOW);
      delayMicroseconds(temp_speed_us);
    } // if Serial.print here then motor will sound like it is halting between rotations. Alternative is to change this cycle from rotation base to step based.

    digitalWrite(enable_pin1, HIGH);
    digitalWrite(enable_pin2, HIGH);

    delay(SECONDS025);
  }
  else
  {
    delay(SECONDS025);

    digitalWrite(enable_pin1, LOW);
    digitalWrite(enable_pin2, LOW);
    digitalWrite(enable_pin3, LOW);

    Serial.println("Rotating PUMP: 3");

    for (unsigned long int i = 0; i <= volume_to_fill; i++)
    {
      digitalWrite(PUMP1_STEP, HIGH);
      digitalWrite(PUMP2_STEP, HIGH);
      digitalWrite(PUMP3_STEP, HIGH);
      delayMicroseconds(temp_speed_us);
      digitalWrite(PUMP1_STEP, LOW);
      digitalWrite(PUMP2_STEP, LOW);
      digitalWrite(PUMP3_STEP, LOW);
      delayMicroseconds(temp_speed_us);
    }

    digitalWrite(enable_pin1, HIGH);
    digitalWrite(enable_pin2, HIGH);
    digitalWrite(enable_pin3, HIGH);

    delay(SECONDS025);
  }
  digitalWrite(LED_PUMP, LOW); // turn OFF pump LED
}

// TRAY
int tray_calibration()
{

  Serial.println(F("Calibration Routine!"));

  tray_flag_calibration_ongoing = HIGH; // Signals begin of calibration

  // Set initial direction
  set_direction(TRAY_DIR, CW);

  // if no sensor is active it starts calibrating, means it has to start moving in CC direction
  if ((interrupt_flag_HOME != HIGH && interrupt_flag_END != HIGH))
  {
    // use (max positions * space between positions) + (start_stop_delta) as max movement possible in any direction
    Serial.println(F("Rotating ..."));
    rotate_tray(Tray_Speed, ((tray_max_positions * tray_position_interval) + (tray_position_start_stop_delta)), TRAY_STEP, TRAY_ENABLE);
    // calibration disabled due to inclusion of new function for motor rotation
  }

  if ((interrupt_flag_END == HIGH) && (interrupt_flag_HOME == HIGH))
  {
    tray_flag_calibration = HIGH;
    Serial.println(F("BOTH SENSORS ACTIVATED!"));
    interrupt_flag_END = LOW;
    interrupt_flag_HOME = LOW;

    // clear the LED info before starting a sequence
    digitalWrite(LED_SENSOR1, LOW);
    digitalWrite(LED_SENSOR2, LOW);

    delay(SECONDS5); // change once calibration is finished
    // Set initial direction
    set_direction(TRAY_DIR, CW);
    Serial.print(F("MOVE TRAY POSITION TO INITIAL POSITION: "));

    // Move to the first position after the sensor to start movement
    rotate_tray(Tray_Speed, tray_position_start_stop_delta, TRAY_STEP, TRAY_ENABLE);
  }
  else
  {
    tray_flag_calibration = LOW;
    interrupt_flag_END = LOW;
    interrupt_flag_HOME = LOW;
    Serial.println("CALIBRATION NOT POSSIBLE!");
  }

  tray_flag_calibration_ongoing = LOW; // Signals END of calibration

  return tray_flag_calibration;
}

void rotate_tray(long int _speed, long int _position_space_mm, int step_pin1, int enable_pin1)
{

  // turn both SENSOR LED's off
  digitalWrite(LED_SENSOR1, LOW);
  digitalWrite(LED_SENSOR2, LOW);

  long int temp_motor_steps = 0;
  long int temp_speed_us = 0;

  temp_speed_us = calculate_speed(_speed, stepsPerRevolution_tray);
  temp_motor_steps = calculate_travel_mm(_position_space_mm);

  rotate_motor(temp_speed_us, temp_motor_steps, step_pin1, enable_pin1);

  // if you're rotating but you've already had signal from both stop sensors then OFF the motor and break out
  // OR if you've reached the maximum nr of laps (60), the motor is stopped to prevent damage.
  // WE NEED TO COUNT THE mm SO WE KNOW WE HAVE EXCEDDED THE MAXIMUM DISTANCE EVEN IF SOME SENSOR HAS NOT BEEN TRIGGERED
  if ((interrupt_flag_HOME == HIGH && interrupt_flag_END == HIGH))
  {
    Serial.println("Both Sensors triggered OR maximum travel reached!");
    digitalWrite(enable_pin1, HIGH); // TURN-OFF DRIVER
  }
}

void rotate_motor(unsigned long int _speed, long int temp_motor_steps, int step_pin1, int enable_pin1)
{

  digitalWrite(enable_pin1, LOW); // TURN-ON DRIVER

  for (int i = 0; i < temp_motor_steps; i++) // rotate motor a given number of temp_motor_steps at a _speed using step_pin1 and enable_pin1
  {
    digitalWrite(step_pin1, HIGH);
    delayMicroseconds(_speed);
    digitalWrite(step_pin1, LOW);
    delayMicroseconds(_speed);
    if ((interrupt_flag_HOME == HIGH && interrupt_flag_END == HIGH))
      //return -1;        // STOPS immediatelly from rotating. // function is returning a value in non-returning function
      break;
  }
  Serial.println("MOTOR STOPS!");
  delayMicroseconds((unsigned int)1000000);
  digitalWrite(enable_pin1, HIGH); // TURN-OFF DRIVER

  Serial.print("Motor Steps: ");
  Serial.println(temp_motor_steps);
  Serial.print("Speed: ");
  Serial.print(_speed);
  Serial.println(" uS");
}

long int calculate_travel_mm(long int ftravel_mm)
{

  travel_steps = ftravel_mm * ((stepsPerRevolution_tray * microstepping) / leadscrew_pitch);

  //  Serial.print("FUNCTION CALCULATE_TRAVEL: ");
  //  Serial.print(travel_steps);
  //  Serial.println(" steps");

  return travel_steps;
}

long int calculate_speed(long int _speed, long int _stepsPerRevolution)
{

  float steps_per_second = 0; // truncating a float to int -> error chance here
  float temp_speed = 0;

    Serial.println("");
    Serial.print("FUNCTION CALCULATE_SPEED: Calculate_speed: ");
    Serial.print(_speed);
    Serial.println(" RPM");

  steps_per_second = (_speed * _stepsPerRevolution) / MINUTES;

    Serial.print("FUNCTION CALCULATE_SPEED: steps_per_second: ");
    Serial.println(steps_per_second);

  temp_speed = (1 / steps_per_second);

  temp_speed = temp_speed / 2;        // to find Ton and Toff
  temp_speed = temp_speed / 0.000001; // to convert to microseconds (input to delayMicroseconds() function)

  temp_speed = (int)temp_speed;

  Serial.print("CALCULATE_SPEED: ");
  Serial.print(temp_speed);
  Serial.println(" uS");

  return temp_speed;
}

void update_sensor_leds()
{

  digitalWrite(LED_SENSOR1, interrupt_flag_END);
  digitalWrite(LED_SENSOR2, interrupt_flag_HOME);
}

void clear_led(int led)
{

  digitalWrite(led, LOW);
}

// INTERRUPTS
void sensor_END_ON()
{

  // global variables used in interrupts must be declared as VOLATILE
  // Turn LED's ON/OFF
  digitalWrite(LED_SENSOR2, HIGH);

  // Reverse the motor direction CC -> CW
  set_direction(TRAY_DIR, CW);

  interrupt_flag_END = HIGH;

  // This sensor does not stop the motor because the intention is to circle back to the original/load position
  Serial.println(F("******************* END *******************"));
  Serial.println(F("DIRECTION SET TO: CW"));
  Serial.print(F("FLAG SET TO: "));
  Serial.println(interrupt_flag_END);
  Serial.println(F("******************* END *******************"));
}

void sensor_HOME_ON()
{

  // global variables used in interrupts must be declared as VOLATILE

  // Turn LED's ON/OFF
  digitalWrite(LED_SENSOR1, HIGH);

  // Reverse the motor direction CW -> CC
  set_direction(TRAY_DIR, CC);

  interrupt_flag_HOME = HIGH;

  Serial.println(F("******************* HOME *******************"));
  Serial.println(F("DIRECTION SET TO: CC"));
  Serial.print(F("FLAG SET TO: "));
  Serial.println(interrupt_flag_HOME);
  Serial.println(F("TRAY OFF!"));
  Serial.println(F("******************* HOME *******************"));
}

// ********************************************************* SETUP BEGINS HERE ********************************************************************************************
void setup()
{
  // INTERRUPT USED TO BLINK LED'S EVERY 0,5s - set timer1 interrupt at 1Hz
  TCCR1A = 0;                          // set entire TCCR1A register to 0
  TCCR1B = 0;                          // same for TCCR1B
  TCNT1 = 0;                           // initialize counter value to 0
  OCR1A = 15624;                       // set compare match register for 1hz increments: (16*10^6) / (1*1024) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);              // turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set CS12 and CS10 bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);             // enable timer compare interrupt

  Serial.begin(115200); // Initializing serial port

  // BUTTONS
  pinMode(BUTTON_AUTO_MAN, INPUT_PULLUP);
  pinMode(BUTTON_AUX_TRAY, INPUT_PULLUP);
  pinMode(BUTTON_START, INPUT_PULLUP);
  pinMode(BUTTON_NOLABEL1, INPUT_PULLUP);
  pinMode(BUTTON_NOLABEL2, INPUT_PULLUP);
  pinMode(BUTTON_STOP, INPUT_PULLUP);

  // LED'S
  pinMode(LED_AUTOMAN, OUTPUT);
  pinMode(LED_AUXTRAY, OUTPUT);
  pinMode(LED_NOLABEL, OUTPUT);
  pinMode(LED_STOP, OUTPUT);
  pinMode(LED_SENSOR1, OUTPUT);
  pinMode(LED_PUMP, OUTPUT);
  pinMode(LED_SENSOR2, OUTPUT);
  pinMode(LED_START, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Setup TRAY
  pinMode(TRAY_DIR, OUTPUT);
  pinMode(TRAY_STEP, OUTPUT);
  pinMode(TRAY_ENABLE, OUTPUT);
  digitalWrite(TRAY_DIR, LOW);
  digitalWrite(TRAY_ENABLE, HIGH);

  // Setup PUMP - 1
  pinMode(PUMP1_DIR, OUTPUT);
  pinMode(PUMP1_STEP, OUTPUT);
  pinMode(PUMP1_ENABLE, OUTPUT);
  digitalWrite(PUMP1_DIR, LOW);
  digitalWrite(PUMP1_ENABLE, HIGH);

  // Setup PUMP - 2
  pinMode(PUMP2_DIR, OUTPUT);
  pinMode(PUMP2_STEP, OUTPUT);
  pinMode(PUMP2_ENABLE, OUTPUT);
  digitalWrite(PUMP2_DIR, LOW);
  digitalWrite(PUMP2_ENABLE, HIGH);

  // Setup PUMP - 3
  pinMode(PUMP3_DIR, OUTPUT);
  pinMode(PUMP3_STEP, OUTPUT);
  pinMode(PUMP3_ENABLE, OUTPUT);
  digitalWrite(PUMP3_DIR, LOW);
  digitalWrite(PUMP3_ENABLE, HIGH);

  // SENSORS
  pinMode(INT_SENSOR_1, INPUT_PULLUP);
  pinMode(INT_SENSOR_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_SENSOR_1), sensor_END_ON, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT_SENSOR_2), sensor_HOME_ON, FALLING);

  //LCD SETUP
  lcd.init();
  lcd.begin(20, 4);
  lcd.clear();

  // Creates the byte for the 3 custom characters
  lcd.createChar(0, menuCursor);

  // Initialize Data Structure
  menu[0].name_ = "START"; // START MENU
  menu[0].type = ACTION;
  menu[0].value = 0;
  menu[0].decimals = 0;
  menu[0].lim = 0;
  menu[0].options[0] = "EMPTY";
  menu[0].options[1] = "EMPTY";
  menu[0].options[2] = "EMPTY";
  menu[0].options[3] = "EMPTY";
  menu[0].suffix = "EMPTY";
  menu[0].option_selected = 0;

  menu[1].name_ = "MODE"; // MODE MENU
  menu[1].type = OPTION;
  menu[1].value = 0;
  menu[1].decimals = 0;
  menu[1].lim = 3;
  menu[1].options[0] = "SEQ.";
  menu[1].options[1] = "CAL.";
  menu[1].options[2] = "DOSE";
  menu[1].options[3] = "EMPTY";
  menu[1].suffix = "EMPTY";
  menu[1].option_selected = 0;

  menu[2].name_ = "VOLUME"; // VOLUME MENU
  menu[2].type = VALUE;
  menu[2].value = 30;
  menu[2].decimals = 0;
  menu[2].lim = 500;
  menu[2].options[0] = "EMPTY";
  menu[2].options[1] = "EMPTY";
  menu[2].options[2] = "EMPTY";
  menu[2].options[3] = "EMPTY";
  menu[2].suffix = "mL";
  menu[2].option_selected = 0;

  menu[3].name_ = "CALIBRATION"; // CALIBRATION MENU
  menu[3].type = VALUE;
  menu[3].value = 0;
  menu[3].decimals = 0;
  menu[3].lim = 500;
  menu[3].options[0] = "EMPTY";
  menu[3].options[1] = "EMPTY";
  menu[3].options[2] = "EMPTY";
  menu[3].options[3] = "EMPTY";
  menu[3].suffix = "mL";
  menu[3].option_selected = 0;

  menu[4].name_ = "PUMPS"; // PUMP MENU
  menu[4].type = OPTION;
  menu[4].value = 0;
  menu[4].decimals = 0;
  menu[4].lim = 3;
  menu[4].options[0] = "1";
  menu[4].options[1] = "2";
  menu[4].options[2] = "3";
  menu[4].options[3] = "EMPTY";
  menu[4].suffix = "EMPTY";
  menu[4].option_selected = 0;

  menu[5].name_ = "PUMP DIR."; // PUMP DIRECTION MENU
  menu[5].type = OPTION;
  menu[5].value = 0;
  menu[5].decimals = 0;
  menu[5].lim = 2;
  menu[5].options[0] = "CW";
  menu[5].options[1] = "CC";
  menu[5].options[2] = "EMPTY";
  menu[5].options[3] = "EMPTY";
  menu[5].suffix = "EMPTY";
  menu[5].option_selected = 0;

  menu[6].name_ = "TRAY";                               // TRAY MENU
  menu[6].type = OPTION;
  menu[6].value = 0;
  menu[6].decimals = 0;
  menu[6].lim = 4;
  menu[6].options[0] = "HOME";
  menu[6].options[1] = "END";
  menu[6].options[2] = "FWD";
  menu[6].options[3] = "BWD";
  menu[6].suffix = "EMPTY";
  menu[6].option_selected = 0;

  menu[7].name_ = "SAVE SETTINGS";                      // SAVE SETTINGS MENU
  menu[7].type = ACTION;
  menu[7].value = 0;
  menu[7].decimals = 0;
  menu[7].lim = 0;
  menu[7].options[0] = "EMPTY";
  menu[7].options[1] = "EMPTY";
  menu[7].options[2] = "EMPTY";
  menu[7].options[3] = "EMPTY";
  menu[7].suffix = "EMPTY";
  menu[7].option_selected = 0;

  menu[8].name_ = "RESET SETTINGS";                     // RESET SETTINGS MENU
  menu[8].type = ACTION;
  menu[8].value = 0;
  menu[8].decimals = 0;
  menu[8].lim = 0;
  menu[8].options[0] = "EMPTY";
  menu[8].options[1] = "EMPTY";
  menu[8].options[2] = "EMPTY";
  menu[8].options[3] = "EMPTY";
  menu[8].suffix = "EMPTY";
  menu[8].option_selected = 0;

  eeprom_loadsettings(); // load settings previously saved - TESTED

  // Start the machine. Light every LED and run tray calibration routine
  initialization();

  upButton.setDebounceTime(DEBOUNCE_TIME);
  downButton.setDebounceTime(DEBOUNCE_TIME);
  OKButton.setDebounceTime(DEBOUNCE_TIME);
  backButton.setDebounceTime(DEBOUNCE_TIME);
  auxButton.setDebounceTime(DEBOUNCE_TIME);

  // disable double click to avoid having two clicks in a fast way
  upButton.disableDoubleClick();
  downButton.disableDoubleClick();
  OKButton.disableDoubleClick();
  backButton.disableDoubleClick();
  auxButton.disableDoubleClick();

  // needed due to change of input to INPUT_PULLUP. Was not working when changing to Mega2560
  upButton.setActiveLogic(LOW);
  downButton.setActiveLogic(LOW);
  OKButton.setActiveLogic(LOW);
  backButton.setActiveLogic(LOW);
  auxButton.setActiveLogic(LOW);

  // activate interrupts
  sei();
}

ISR(TIMER1_COMPA_vect)
{ //timer1 interrupt 1Hz toggles pin 13 (LED)
  //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  digitalWrite(LED_START, !digitalRead(LED_START));
}

void loop()
{
  // change the order of the button scan or add time between each scan
  upButton.update();
  downButton.update();
  OKButton.update();
  backButton.update();
  auxButton.update();

  update_sensor_leds();
  // Delay here in case it's too fast switching options e.g. delay(50);
  delay(MENU_GENERAL_TIME);

  if (backButton.isClicked())
  {
    Serial.println(F("-------  BUTTON PRESSED: -> BACK <-  --------"));

    // When BACK is pressed while it is on one of these two volume type menus: VOLUME and CALIBRATION
    if (entered_valueoption)
    {

      if (menu[menu_line_1].type == VALUE) // If type VOLUME (VALUE)
      {
        switch (menu_line_1)
        {
        case 2: // VOLUME
          Serial.println(F("YOU LEFT -> VOLUME <- MENU"));
          break;

        case 3: // CALIBRATION
          Serial.println(F("YOU LEFT -> CALIBRATION <- MENU"));
          //volume_calibration(menu[2].value);
          volume_calibration_steps(menu[2].value); // Under test - it's not doing anything here since it's not saving the steps value which is calculated
          break;
        }
      }
      if (menu[menu_line_1].type == OPTION) // If type OPTION
      {
        Serial.println(F("YOU LEFT -> OPTION <- MENU"));
      }
    }

    if (entered_menu == HIGH || entered_valueoption == HIGH || selected_option == HIGH)
    {
      selected_option = LOW;
      Serial.println(F("LEFT SUB-MENU"));
    }

    flag_option = HIGH; // used to keep the last saved menu option as cursor position

    leave();
  }

  if (upButton.isClicked())
  {
    Serial.println(F("-------  BUTTON PRESSED: -> UP <-  ----------"));

    if (entered_menu == HIGH || entered_valueoption == HIGH)
    {                                      // If entered a VALUE or OPTION menu
      if (menu[menu_line_1].type == VALUE) // If entered a VALUE type
      {
        menu[menu_line_1].value = menu[menu_line_1].value + 1; // ADD one because this is the UP button

        Serial.print(F("VOLUME: ")); // print value for Debug()
        Serial.print(menu[menu_line_1].value);
        Serial.println(" ml");

        lcd.setCursor(15, 3); // Set right cursor position

        if (menu[menu_line_1].value >= 100) // offset print of value when it's 3 digits AKA +100
        {
          lcd.setCursor(14, 3);
        }
        else if (menu[menu_line_1].value < 100) // offset print of value when it's 2 digits AKA -99
        {
          lcd.setCursor(15, 3);
        }
        if (menu[menu_line_1].value < 10)
        {
          lcd.print("   ");
          lcd.setCursor(16, 3);
        }
        lcd.print(menu[menu_line_1].value); // print value to LCD
        lcd.setCursor(12, 3);               // set cursor to the right place
        lcd.write(byte(0));
        lcd.setCursor(12, 3);
        lcd.blink(); // blink cursor
      }
    }

    if (selected_option == HIGH)
    {
      if (!flag_menu_cursor)
      {
        cursor_position = 0;
        flag_menu_cursor = HIGH;
      }

      if (cursor_position == 0)
      {
        lcd.print(" ");
        cursor_position = (menu[menu_line_1].lim - 1);
      }
      else
      {
        lcd.print(" ");
        cursor_position--;
      }

      lcd.setCursor(14, cursor_position);
      lcd.write(byte(0));
      lcd.setCursor(14, cursor_position);
    }

    if (entered_valueoption == LOW)
    {
      // move menu position UP by 1 ->> UP BUTTON CLICK
      if (menu_line_1 == START)
      {
        menu_line_1 = RESET_SETTINGS;
        menu_line_2 = START;
      }
      else
      {
        if (menu_line_2 == START)
        {
          menu_line_2 = RESET_SETTINGS;
          menu_line_1 = SAVE_SETTINGS;
        }
        else
        {
          menu_line_1--;
          menu_line_2--;
        }
      }
      flag_lcd_update_change_value = HIGH;
    }
  }

  if (downButton.isClicked())
  {
    Serial.println(F("-------  BUTTON PRESSED: -> DOWN <-  --------"));

    if (selected_action == HIGH || entered_valueoption == HIGH)
    { // menu entered
      if (menu[menu_line_1].type == VALUE)
      {
        if (menu[menu_line_1].value == 0)
        {
          menu[menu_line_1].value = 0;
        }
        else
        {
          menu[menu_line_1].value = menu[menu_line_1].value - 1;
        }

        Serial.print(F("VOLUME: "));
        Serial.print(menu[menu_line_1].value);
        Serial.println(" ml");

        lcd.setCursor(13, 3);

        if (menu[menu_line_1].value >= 100)
        {
          lcd.print(" ");
          lcd.setCursor(14, 3);
        }
        else if (menu[menu_line_1].value < 100)
        {
          lcd.print("  ");
          lcd.setCursor(15, 3);
        }
        if (menu[menu_line_1].value < 10)
        {
          lcd.print("   ");
          lcd.setCursor(16, 3);
        }

        lcd.print(menu[menu_line_1].value);
        lcd.setCursor(12, 3);
        lcd.write(byte(0));
        lcd.setCursor(12, 3);
        lcd.blink();
      }
    }

    if (selected_option == HIGH)
    {
      if (!flag_menu_cursor)
      {
        cursor_position = 0;
        flag_menu_cursor = HIGH;
      }

      if (cursor_position == menu[menu_line_1].lim - 1)
      {
        lcd.print(" ");
        cursor_position = 0;
      }
      else
      {
        lcd.print(" ");
        cursor_position++;
      }

      lcd.setCursor(14, cursor_position);
      lcd.write(byte(0));
      lcd.setCursor(14, cursor_position);
    }

    if (entered_valueoption == LOW)
    {
      // move menu position DOWN by 1 ->> DOWN BUTTON CLICK
      if (menu_line_2 == RESET_SETTINGS)
      {
        menu_line_1 = RESET_SETTINGS;
        menu_line_2 = START;
      }
      else
      {
        if (menu_line_1 == RESET_SETTINGS)
        {
          menu_line_1 = START;
          menu_line_2 = MODE;
        }
        else
        {
          menu_line_1++;
          menu_line_2++;
        }
      }
      flag_lcd_update_change_value = HIGH;
    }
  }

  if (OKButton.isClicked())
  {
    Serial.println(F("-------  BUTTON PRESSED: -> ENTER <-  -------"));

    if (menu[menu_line_1].type == VALUE || menu[menu_line_1].type == OPTION) // if entered on a VALUE or OPTION type menu
    {
      entered_valueoption = HIGH;       // FLAG if it is a VALUE or OPTION menu
    }
    if (menu[menu_line_1].type == ACTION)                       // if entered on a ACTION type menu
    {
      selected_action = HIGH;           // FLAG if it is an ACTION menu
    }
    if (selected_action == HIGH || entered_valueoption == HIGH) // if entered ANY type of menu
    {
      entered_menu = HIGH;              // FLAG if it entered a MENU
    }

    if (entered_menu)
    {
      if (menu[menu_line_1].type == ACTION)
      { // When entering this option, always prints the value and suffix of that is stored for that selected option
        lcd.setCursor(0, 3);

        if (tray_flag_calibration != 0) // only perform tray related operations if calibration is done.
        {
          lcd.print("Working ...");
        }
        else
        {
          lcd.print("Calibration NOT DONE");
          BlinkLED(LED_STOP, 125, 50); // If calibration not done then blink LED_STOP to signal error
        }
      }
      if (menu[menu_line_1].type == VALUE)
      { // When entering VALUE type menu, always prints the value and suffix of that is stored for that selected option

        lcd.setCursor(5, 3);
        lcd.print(F("Value:"));

        if (menu[menu_line_1].value >= 100) // adjust the cursor when the value is ABOVE 100
        {
          lcd.setCursor(14, 3);
        }
        else if (menu[menu_line_1].value < 100) // adjust the cursor when the value is BELOW 100
        {                                       // remove the else if any problem in this menu
          lcd.setCursor(15, 3);
        }
        if (menu[menu_line_1].value < 10) // added because the initial value presented was
        {                                 // not on the right position
          lcd.print("   ");
          lcd.setCursor(16, 3);
        }

        lcd.print(menu[menu_line_1].value); // print existing value
        lcd.setCursor(18, 3);
        lcd.print(menu[menu_line_1].suffix); // print existing units e.g. ml
        lcd.setCursor(13, 3);                // reset the cursor to when changing UP or DOWN
        lcd.noBlink();                       // remove blinking because it entered a VALUE menu
      }
      if (menu[menu_line_1].type == OPTION)
      { // When entering OPTION type menu, always print all available OPTIONS

        for (int i = 0; i < menu[menu_line_1].lim; i++)
        { // Display all possible menu options up to the pre-defined limit
          lcd.setCursor(15, i);
          lcd.print(menu[menu_line_1].options[i]);
        }

        if (flag_option == HIGH)
        {
          cursor_position = menu[menu_line_1].option_selected;
          lcd.setCursor(14, cursor_position); // set the cursor to the previously defined position.
          lcd.write(byte(0));
          lcd.setCursor(14, cursor_position);
          flag_option = LOW;
        }
        else
        {
          lcd.setCursor(14, cursor_position); // set the cursor to the previously defined position.
          lcd.write(byte(0));
          lcd.setCursor(14, cursor_position);
        }
        selected_option = HIGH;
        iterator = iterator + 1;
      }
    }

    if (selected_option)
    {                    // These are the ACTIONS for the OPTIONS menus
      if (iterator == 2) // actions to take place before leaving after the second OK button press
      {
        iterator = 0;

        if (menu_line_1 == TRAY)
        { // Actions for the TRAY menu related options
          Serial.println(F("SELECTION MENU: TRAY"));
          switch (cursor_position)
          {
          case 0:
            menu[menu_line_1].option_selected = cursor_position;

            Serial.println(F("MOVING .... home"));
            temp_tray_position = tray_home_position;
            set_tray_position(temp_tray_position);

            flag_option = HIGH;

            break;
          case 1:
            menu[menu_line_1].option_selected = cursor_position;

            Serial.println(F("MOVING .... end"));

            temp_tray_position = tray_max_positions;
            set_tray_position(temp_tray_position);

            flag_option = HIGH;

            break;
          case 2:
            menu[menu_line_1].option_selected = cursor_position;

            Serial.println(F("MOVING .... FWD"));
            if (temp_tray_position < tray_max_positions)
            {
              temp_tray_position = temp_tray_position + 1; // add one position to the current tray position
            }
            else
            {
              temp_tray_position = temp_tray_position;
              digitalWrite(LED_STOP, HIGH);
              Serial.println(F("Illegal position: bigger than MAX"));
            }
            set_tray_position(temp_tray_position);

            flag_option = HIGH;

            break;
          case 3:
            menu[menu_line_1].option_selected = cursor_position;

            Serial.println(F("MOVING .... BWD"));
            if (temp_tray_position > tray_home_position)
            {
              temp_tray_position = temp_tray_position - 1; // add one position to the current tray position
            }
            else
            {
              temp_tray_position = temp_tray_position;
              digitalWrite(LED_STOP, HIGH);
              Serial.println(F("Illegal position: smaller than HOME"));
            }
            set_tray_position(temp_tray_position);

            flag_option = HIGH;

            break;
          }
        }
        if (menu_line_1 == PUMPS)
        { // Actions for the PUMPS menu related options
          Serial.println(F("SELECTION MENU: PUMPS"));

          switch (cursor_position)
          {
          case 0:
            Serial.println(F("PUMP(s) SELECTED: 1"));
            menu[menu_line_1].option_selected = PUMP1;
            flag_option = HIGH;
            break;
          case 1:
            Serial.println(F("PUMP(s) SELECTED: 2"));
            menu[menu_line_1].option_selected = PUMP2;
            flag_option = HIGH;
            break;
          case 2:
            Serial.println(F("PUMP(s) SELECTED: 3"));
            menu[menu_line_1].option_selected = PUMP3;
            flag_option = HIGH;
            break;
          }
        }
        if (menu_line_1 == PUMP_DIR)
        { // Actions for the DIR menu related options
          Serial.println(F("SELECTION MENU: DIRECTION"));

          switch (cursor_position)
          {
          case 0:
            menu[menu_line_1].option_selected = 0;
            flag_option = HIGH;
            break;
          case 1:
            menu[menu_line_1].option_selected = 1;
            flag_option = HIGH;
            break;
          }
          Serial.print(F("DIRECTION SETTING: "));
          Serial.println(menu[menu_line_1].option_selected);
          set_direction(PUMP1_DIR, menu[menu_line_1].option_selected);
          set_direction(PUMP2_DIR, menu[menu_line_1].option_selected);
          set_direction(PUMP3_DIR, menu[menu_line_1].option_selected);
          Serial.print("PUMP1: ");
          Serial.println(set_direction(PUMP1_DIR, menu[menu_line_1].option_selected));
          Serial.print("PUMP2: ");
          Serial.println(set_direction(PUMP2_DIR, menu[menu_line_1].option_selected));
          Serial.print("PUMP3: ");
          Serial.println(set_direction(PUMP3_DIR, menu[menu_line_1].option_selected));
        }
        if (menu_line_1 == MODE)
        { // Actions for the PUMPS menu related options
          Serial.println(F("SELECTION MENU: MODE"));

          switch (cursor_position)
          {
          case 0:
            menu[menu_line_1].option_selected = MENU_MODE_SEQ;
            flag_option = HIGH;
            break;
          case 1:
            menu[menu_line_1].option_selected = MENU_MODE_CAL;
            flag_option = HIGH;
            break;
          case 2:
            menu[menu_line_1].option_selected = MENU_MODE_DOSE;
            flag_option = HIGH;
            break;
          }
          Serial.print(F("MODE: "));
          Serial.print(menu[menu_line_1].option_selected);
          Serial.print(F(" --> "));
          Serial.println(menu[menu_line_1].options[menu[menu_line_1].option_selected]);
        }
        menu[menu_line_1].option_selected = cursor_position; // When pressing OK for the second time, use the cursor position to record the selected OPTION
        leave();
      }
    }

    if (selected_action)
    { // If I've previously selected an action, then this is the cycle where it is executed.

      if (menu_line_1 == START)
      {

        clear_sensor_info(); // clear sensor flags and LED's from Calibration sequence once any type of action is started.

        switch (menu[1].option_selected)
        {
        case 0:
          Serial.println(F("SEQUENTIAL: PUMP + TRAY"));

          if (tray_flag_calibration != LOW) // only perform tray related operations if calibration is done.
          {
            Serial.println(F("Operation will start now ..."));
            BlinkLED(LED_START, 125, 50);

            //  Move to filling position 0
            //  rotate_tray(Tray_Speed, (2 * tray_position_start_stop_delta), TRAY_STEP, TRAY_ENABLE);

            delay(SECONDS1);
            flag_sequence = HIGH;
            Serial.print(F("FLAG SEQUENCE: "));
            Serial.println(flag_sequence);
            set_direction(TRAY_DIR, CW);
            delay(SECONDS1);

            for (int i = 1; i <= tray_max_positions; i++)
            {
              set_tray_position(i);
              Serial.println("Dispensing ...");
              delay(SECONDS1);
              dispense(volume_calibration_steps(menu[2].value), (menu[4].option_selected + 1));
              delay(SECONDS1);
            }

            delay(SECONDS1);
            // Return to HOME position
            set_tray_position(tray_home_position);
            delay(SECONDS1);
            clear_position_info();
            flag_sequence = LOW;
            Serial.print(F("FLAG SEQUENCE: "));
            Serial.println(flag_sequence);
            Serial.println(F("SEQUENCE COMPLETED!"));
          }
          else
          {
            Serial.println(F("Calibration ERROR!"));
            BlinkLED(LED_STOP, 125, 50);
          }
          break;

        case 1:
          Serial.println(F("CALIBRATION"));
          // DISPENSE A CALIBRATED DOSAGE ON THE SELECTED PUMP
          dispense(volume_calibration_steps(menu[3].value), (menu[4].option_selected + 1)); // Under test
          break;
        case 2:
          Serial.println(F("DOSE"));
          // DISPENSE A CALIBRATED DOSAGE ON THE SELECTED PUMP
          dispense(volume_calibration_steps(menu[2].value), (menu[4].option_selected + 1)); // Under test
          break;
        }
      }
      if (menu_line_1 == SAVE_SETTINGS)
      {
        eeprom_savesettings();
      }
      if (menu_line_1 == RESET_SETTINGS)
      {
        eeprom_resetsettings();
      }

      menu[menu_line_1].option_selected = cursor_position; // When pressing OK for the second time, use the cursor position to record the selected OPTION
      leave();
    }
  }

  if (auxButton.isClicked())
  {
    // *************************************************
    Serial.println(F("-------  BUTTON PRESSED: -> AUX <-  -------"));
    // *************************************************
  }

  // UPDATE LCD if any value has changed
  update_lcd();
}