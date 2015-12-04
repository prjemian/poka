// file: DCMotorServer.ino
// author: Pete Jemian
//
// derived from:
//   * Adafruit Motor & Servo Shield v1.0
//   * https://github.com/prjemian/cmd_response

#define LED_PIN 13      // select the pin for the LED

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// cmd_response protocol preparation
#define SOFTWARE_VERSION       "2015.1207.0"
#define SOFTWARE_ID            "DCMotorServer"

#define USB_BAUD                115200
#define BUFFER_LENGTH           40
#define COMMAND_LENGTH          16
#define EOS_TERMINATOR_CHAR     '\n'

#define UNDEFINED              -1

#define ADC_MAX_VALUE 1023
#define ADC_MIN_VALUE 0
#define PWM_MAX_VALUE 255
#define PWM_MIN_VALUE 0

#define PERIOD_DEFAULT_ms  1000
#define PERIOD_MIN_ms      5
#define PERIOD_MAX_ms      3600000

#define MULTIPLIER_DEFAULT  1000
#define MULTIPLIER_MIN      1
#define MULTIPLIER_MAX      1000000

char inputString[BUFFER_LENGTH+1];
char strPtr;
boolean stringComplete;
long arg1;
long arg2;
char baseCmd[COMMAND_LENGTH+1];

long period = PERIOD_DEFAULT_ms;
long multiplier = MULTIPLIER_DEFAULT;
int ai_watched[NUM_ANALOG_INPUTS];
long ai_sums[NUM_ANALOG_INPUTS];
float ai_mean[NUM_ANALOG_INPUTS];
long count, rate;
long nextUpdate;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// motor-specific parameters
// assumes only one motor moves at a time
int dcmotor_moving = false;      // flag to indicate motor is moving
int permit_move = true;  // flag to allow motion
#define V_TOP_MAX 255    // maximum allowed rotation speed
int v_top = V_TOP_MAX;   // maximum selected motor speed (absolute value)
int v_base = 0;          // starting motor speed (absolute value)

int v_step_ms = 5;       // stepping interval in vmove()
int ramp_time_ms = 500;  // time to ramp velocity up to v_top
#define V_STEP_MAX 45    // maximum allowed step for rotation rate (NOT USED)
#define MAX_STEP 10      // maximum allowed steps to ramp up to v_top (NOT USED)

// create instances for each motor
#include <AFMotor.h>
AF_DCMotor m1(1, MOTOR12_1KHZ);
AF_DCMotor m2(2, MOTOR12_1KHZ);
AF_DCMotor m3(3, MOTOR34_1KHZ);
AF_DCMotor m4(4, MOTOR34_1KHZ);

// an array makes it easier to generalize the API
#define NUM_MOTORS 4
AF_DCMotor dc_motor[] = {0L, m1, m2, m3, m4};


void setup_dc_motors() {
  // turn on motors
  for (uint8_t i=1; i<=NUM_MOTORS; i++) {
    dc_motor[i].setSpeed(v_top);
    dc_motor[i].run(RELEASE);
  }
}


// FIXME: the STOP algorithm does not work!!!

/**
 ** dc_vmove (velocity controlled ramp move of DC motor)
 ** 
 ** @time_ms (int): total time for move, negative for reverse direction
 **
 ** Only move one motor at a time
 ** Will not move motor if abs(time_ms) < v_step_ms
 ** 
 **     -----------------------------------
 ** vt |        +-------------+            |
 **    |       /               \           |
 **    |      /                 \          |
 **    |     /                   \         |
 **    |    /                     \        |
 ** v0 |   +                       +       |
 **    |   |                       |       |
 ** 0  +---+----+-------------+----+-------
 **        t0   t1            t2   t3
 **/
void dc_vmove(AF_DCMotor m, int time_ms) {
  long  t_steady_ms = abs(time_ms) - 2 * ramp_time_ms;
  if (abs(time_ms) < v_step_ms) return;
  unsigned long  t_ramp_ms = (t_steady_ms > 0) ? ramp_time_ms : abs(time_ms)/2;
  unsigned long  t0 = millis();
  unsigned long  t1 = t0 + t_ramp_ms;
  unsigned long  t2 = t1 + ((t_steady_ms < 0) ? 0 : t_steady_ms);
  unsigned long  t3 = t0 + abs(time_ms);
  int n = t_ramp_ms / v_step_ms;
  float v_step = float(v_top-v_base)/n;
  float rate = v_base;

  if (time_ms >= 0)
    m.run(FORWARD);
  else
    m.run(BACKWARD);
  
  dcmotor_moving = true;
  // ramp up
  while (millis() < t1 && permit_move) {
    m.setSpeed(rate);
    delay(v_step_ms);
    rate = constrain(rate + v_step, 0, v_top);
  }

  // flat top
  while (millis() < t2 && permit_move) {
    m.setSpeed(rate);
    delay(v_step_ms);
    rate = constrain(rate + v_step, 0, v_top);
  }

  // ramp down
  while (millis() < t3 && permit_move) {
    rate = constrain(rate - v_step, 0, v_top);
    m.setSpeed(rate);
    delay(v_step_ms);
  }

  // turn motor off
  m.run(RELEASE);
  dcmotor_moving = false;
}

void start_move() {
  // initialize and start the move (called from cmd_response handler)
}

void end_move() {
  // m.run(RELEASE);
  permit_move = true;   // re-enable
  dcmotor_moving = false;
}

void motion_handler() {
  // TODO: handle various phases of the motion protocol
  if (!permit_move) {
    end_move();
  }
  if (dcmotor_moving) {
    // operate the move, terminate if appropriate
    /*
     * t = millis();
     * if (t0 <= t && t < t1) ramp speed up
     * if (t1 <= t && t < t2) hold speed at V_top
     * if (t2 <= t && t < t3) ramp speed down
     * if (t23 <= t) {end_move(); and then clear the move variables}
     */
  }
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


void blink_LED() {
  int led_state = millis()/500 % 2;
  digitalWrite(LED_PIN, led_state);
}


void setup() {
  pinMode(LED_PIN, OUTPUT);
  resetBuffer();
  Serial.begin(USB_BAUD);    // set up Serial library at 9600 bps
  Serial.print(F(SOFTWARE_ID));
  Serial.print(F(" started: "));
  Serial.println(freeRam());
  for (int i = 0; i < NUM_ANALOG_INPUTS; i++) {
    ai_watched[i] = 0;    // by default, do not monitor
    ai_sums[i] = 0;
    ai_mean[i] = 0;
  }
  count = 0;
  rate = 0;
  nextUpdate = millis() + period;

  setup_dc_motors();
}


void loop() {
  blink_LED();
  cmd_response();   // TODO: do not allow certain changes while moving
  motion_handler();
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// cmd_response protocol
// https://github.com/prjemian/cmd_response


void cmd_response() {
  signalGathering();
  readBuffer();
  processCmd();
}

void signalGathering() {
  // periodically, average the watched ADC channels
  if (millis() >= nextUpdate) {
    // Serial.print(F"count"); Serial.print(F"  "); Serial.println(count);
    for (int i = 0; i < NUM_ANALOG_INPUTS; i++) {
      if (ai_watched[i] && count > 0) {    // if monitoring
        // Serial.print(i); Serial.print(F"  "); Serial.println(float(ai_sums[i]));
        ai_mean[i] = float(ai_sums[i]) / count;
        ai_sums[i] = 0;
      }
    }
    nextUpdate = millis() + period;
    rate = 1000 * count / period;    // loops per second
    count = 0;
  }
  // this is where the ADC channels are read
  for (int i = 0; i < NUM_ANALOG_INPUTS; i++) {
    if (ai_watched[i])    // if monitoring
      ai_sums[i] += analogRead(i);
  }
  count++;
}

/**
  * Finds the amount of free memory by calculating the difference
  * between the address of a new stack variable and the address of 
  * the end of the heap.
  */
int freeRam()
{   
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void readBuffer() {
  while (Serial.available()) {
    if (strPtr == BUFFER_LENGTH) {
      Serial.println(F("ERROR_BUFFER_OVERFLOW"));
      resetBuffer();    // discard the buffer contents
    } else {
      char inChar = (char)Serial.read();
      if (inChar == EOS_TERMINATOR_CHAR) {
        stringComplete = true;
        break;
      }
      inputString[strPtr++] = inChar;
      inputString[strPtr] = 0;
    }
  }
}

void processCmd() {
  if (stringComplete) {
    executeCommand();    // process the command
    resetBuffer();    // clear for the next command
  }
}

void resetBuffer() {
  inputString[0] = 0;    // discard the buffer contents
  strPtr = 0;
  stringComplete = false;
  baseCmd[0] = 0;
  arg1 = UNDEFINED;
  arg2 = UNDEFINED;
}

//  USB command interface of form: baseCmd [arg1 [arg2]]
//
// char* baseCmd (lower case)
// long arg1, arg2 (no octal or hex interpreted)

void executeCommand() {
  dissectCommand(inputString);
  if (strlen(baseCmd)) {
    if      (0 == strcmp(baseCmd, "?ai"))       readAI(inputString);
    else if (0 == strcmp(baseCmd, "?bi"))       readBI(inputString);
    else if (0 == strcmp(baseCmd, "!bo"))       writeBO(inputString);
    else if (0 == strcmp(baseCmd, "!dcm"))      move_DC_motor(inputString);
    else if (0 == strcmp(baseCmd, "!dcm:stop")) stop_DC_motor(inputString);
    else if (0 == strcmp(baseCmd, "?dcm:moving"))  is_DC_motor_moving(inputString);
    else if (0 == strcmp(baseCmd, "!dcm:v:top"))   set_DC_vtop(inputString);
    else if (0 == strcmp(baseCmd, "?dcm:v:top"))   get_DC_vtop(inputString);
    else if (0 == strcmp(baseCmd, "!dcm:v:base"))  set_DC_vbase(inputString);
    else if (0 == strcmp(baseCmd, "?dcm:v:base"))  get_DC_vbase(inputString);
    else if (0 == strcmp(baseCmd, "!dcm:t:ramp"))  set_DC_ramp_time(inputString);
    else if (0 == strcmp(baseCmd, "?dcm:t:ramp"))  get_DC_ramp_time(inputString);
    else if (0 == strcmp(baseCmd, "!dcm:t:step"))  set_DC_step_time(inputString);
    else if (0 == strcmp(baseCmd, "?dcm:t:step"))  get_DC_step_time(inputString);
    else if (0 == strcmp(baseCmd, "!pwm"))      writePWM(inputString);
    else if (0 == strcmp(baseCmd, "!pin"))      setPinMode(inputString);
    else if (0 == strcmp(baseCmd, "?#ai"))      get_num_ai_channels(inputString);
    else if (0 == strcmp(baseCmd, "?#bi"))      get_num_bi_channels(inputString);
    else if (0 == strcmp(baseCmd, "!t"))        set_period(inputString);
    else if (0 == strcmp(baseCmd, "?t"))        get_period(inputString);
    else if (0 == strcmp(baseCmd, "?t:min"))    get_period_min(inputString);
    else if (0 == strcmp(baseCmd, "?t:max"))    get_period_max(inputString);
    else if (0 == strcmp(baseCmd, "!k"))        set_multiplier(inputString);
    else if (0 == strcmp(baseCmd, "?k"))        get_multiplier(inputString);
    else if (0 == strcmp(baseCmd, "?k:min"))    get_multiplier_min(inputString);
    else if (0 == strcmp(baseCmd, "?k:max"))    get_multiplier_max(inputString);
    else if (0 == strcmp(baseCmd, "!ai:watch")) watch_ai_channel(inputString);
    else if (0 == strcmp(baseCmd, "?ai:mean"))  readAI_mean(inputString);
    else if (0 == strcmp(baseCmd, "?v"))        get_software_version(inputString);
    else if (0 == strcmp(baseCmd, "?id"))       get_software_id(inputString);
    else if (0 == strcmp(baseCmd, "?rate"))     get_loop_rate(inputString);
    else {
      Serial.print(F("ERROR_UNKNOWN_COMMAND:"));
      finalizeError(inputString);
    }
  }
}

/** dissectCommand
 *
 * parse the input "baseCmd arg1 [arg2]"
 */
void dissectCommand(char *source_string) {
  char *cmd;
  char buf[BUFFER_LENGTH+1];
  
  strcpy(buf, source_string);  // copy locally so we don't modify source
  
  cmd = strtok(buf, " ");
  strcpy(baseCmd, cmd);
  
  cmd = strtok(NULL, " ");
  if (cmd) {
    arg1 = atoi(cmd);

    cmd = strtok(NULL, " ");
    if (cmd) {
      arg2 = atoi(cmd);
      
      cmd = strtok(NULL, " ");
      if (cmd) {
        Serial.print(F("ERROR_TOO_MANY_ARGUMENTS:"));
        Serial.println(source_string);
        resetBuffer();
      }
    }
  }
}

void finalizeError(char *in) {
  Serial.println(in);
  resetBuffer();
}

void readAI(char* in) {
  if (arg1 < 0 || arg1 > NUM_ANALOG_INPUTS) {
    Serial.print(F("ERROR_AI_PIN_NOT_AVAILABLE:"));
    finalizeError(in);
  } else {
    Serial.println(analogRead(arg1));
  }
}

void readBI(char* in) {
  if (arg1 < 0 || arg1 > NUM_DIGITAL_PINS) {
    Serial.print(F("ERROR_BI_PIN_NOT_AVAILABLE:"));
    finalizeError(in);
  } else {
    Serial.println(digitalRead(arg1));
  }
}

void writeBO(char* in) {
  if (arg2 < 0 || arg2 > 1) {
    Serial.print(F("ERROR_BINARY_RANGE:"));
    finalizeError(in);
  } else if (arg1 < 0 || arg1 > NUM_DIGITAL_PINS) {
    // TODO: but did we set the pin for output?
    Serial.print(F("ERROR_BO_PIN_NOT_AVAILABLE:"));
    finalizeError(in);
  } else {
    digitalWrite(arg1, arg2);
    Serial.println(F("Ok"));
  }
}

//  USB command: !dcm motor_number time_ms
void move_DC_motor(char* in) {
  if (dcmotor_moving) {
    Serial.print(F("ERROR_CANNOT_MOVE_NOW:"));
    finalizeError(in);
  } else {
    if (0 < arg1 && arg1 <= NUM_MOTORS)
      dc_vmove(dc_motor[arg1], arg2);
  }
}

//  USB command: !dcm:stop
//
// stops any DC motor motion
void stop_DC_motor(char* in) {
  permit_move = false;
}

//  USB command: ?dcm:moving
void is_DC_motor_moving(char* in) {
  int state = dcmotor_moving ? 1 : 0;
  Serial.println(state);
}

//  USB command: !dcm:v:top
void set_DC_vtop(char* in) {
  if (dcmotor_moving) {
    Serial.print(F("ERROR_CANNOT_CHANGE_NOW:"));
    finalizeError(in);
  } else {
    v_top = constrain(arg1, 0, V_TOP_MAX);
  }
}

void get_DC_vtop(char* in) {
  Serial.println(v_top);
}

//  USB command: !dcm:v:base
void set_DC_vbase(char* in) {
  if (dcmotor_moving) {
    Serial.print(F("ERROR_CANNOT_CHANGE_NOW:"));
    finalizeError(in);
  } else {
    v_base = constrain(arg1, 0, V_TOP_MAX);
  }
}

void get_DC_vbase(char* in) {
  Serial.println(v_base);
}

//  USB command: !dcm:t:ramp
void set_DC_ramp_time(char* in) {
  if (dcmotor_moving) {
    Serial.print(F("ERROR_CANNOT_CHANGE_NOW:"));
    finalizeError(in);
  } else {
    ramp_time_ms = arg1;
  }
}

void get_DC_ramp_time(char* in) {
  Serial.println(ramp_time_ms);
}

//  USB command: !dcm:t:step
void set_DC_step_time(char* in) {
  if (dcmotor_moving) {
    Serial.print(F("ERROR_CANNOT_CHANGE_NOW:"));
    finalizeError(in);
  } else {
    v_step_ms = arg1;
  }
}

void get_DC_step_time(char* in) {
  Serial.println(v_step_ms);
}

void writePWM(char* in) {
  if (arg2 < PWM_MIN_VALUE || arg2 > PWM_MAX_VALUE) {
    Serial.print(F("ERROR_PWM_RANGE:"));
    finalizeError(in);
  } else if (!digitalPinHasPWM(arg1)) {
    // TODO: but did we set the pin for output?
    Serial.print(F("ERROR_PIN_NOT_PWM:"));
    finalizeError(in);
  } else {
    analogWrite(arg1, arg2);
    Serial.println(F("Ok"));
  }
}

void get_num_ai_channels(char* in) {
  Serial.println(NUM_ANALOG_INPUTS);
}

void get_num_bi_channels(char* in) {
  Serial.println(NUM_DIGITAL_PINS);
}

void set_period(char* in) {
  if (arg1 < PERIOD_MIN_ms || arg1 > PERIOD_MAX_ms) {
    Serial.print(F("ERROR_PERIOD_RANGE:"));
    finalizeError(in);
  } else {
    period = arg1;
    Serial.println(F("Ok"));
    nextUpdate = millis();    // trigger a new update immediately
  }
}

void get_period(char* in) {
  Serial.println(period);
}

void get_period_min(char* in) {
  Serial.println(PERIOD_MIN_ms);
}

void get_period_max(char* in) {
  Serial.println(PERIOD_MAX_ms);
}

void set_multiplier(char* in) {
  if (arg1 < MULTIPLIER_MIN || arg1 > MULTIPLIER_MAX) {
    Serial.print(F("ERROR_MULTIPLIER_RANGE:"));
    finalizeError(in);
  } else {
    multiplier = arg1;
    Serial.println(F("Ok"));
  }
}

void get_multiplier(char* in) {
  Serial.println(multiplier);
}

void get_multiplier_min(char* in) {
  Serial.println(MULTIPLIER_MIN);
}

void get_multiplier_max(char* in) {
  Serial.println(MULTIPLIER_MAX);
}

void get_software_version(char* in) {
  Serial.println(SOFTWARE_VERSION);
}

void get_software_id(char* in) {
  // this is the ONLY string output that is not an error
  Serial.println(F(SOFTWARE_ID));
}

void get_loop_rate(char* in) {
  Serial.println(rate);
}

void watch_ai_channel(char* in) {
  // configure pin to be averaged (or not depending on arg2)
  if (arg1 < 0 || arg1 > NUM_ANALOG_INPUTS) {
    Serial.print(F("ERROR_AI_PIN_NOT_AVAILABLE:"));
    finalizeError(in);
  } else {
    ai_watched[arg1] = (arg2 == 0 ? 0 : 1);
    Serial.println(F("Ok"));
  }
}

void readAI_mean(char* in) {
  if (arg1 < 0 || arg1 > NUM_ANALOG_INPUTS) {
    Serial.print(F("ERROR_AI_PIN_NOT_AVAILABLE:"));
    finalizeError(in);
  } else if (!ai_watched[arg1]) {
    Serial.print(F("ERROR_AI_PIN_NOT_WATCHED:"));
    finalizeError(in);
  } else {
    Serial.println(long(ai_mean[arg1] * multiplier));
  }
}

void not_implemented_yet(char* in) {
  Serial.print(F("ERROR_NOT_IMPLEMENTED_YET:"));
  finalizeError(in);
}

void setPinMode(char* in) {
  if (arg1 < 0 || arg1 > NUM_DIGITAL_PINS) {
    // TODO: but did we set the pin for output?
    Serial.print(F("ERROR_DIGITAL_PIN_NOT_AVAILABLE:"));
    finalizeError(in);
  } else {
    pinMode(arg1, (arg2 == 1) ? OUTPUT : INPUT);
    Serial.println(F("Ok"));
  }
}

