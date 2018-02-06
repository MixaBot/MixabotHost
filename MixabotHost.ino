#include <Adafruit_LiquidCrystal.h>
#include <SparkFunESP8266WiFi.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Wire.h>
#include <AccelStepper.h>
#include <EEPROM.h>

#define __ASSERT_USE_STDERR
#include <assert.h>


#define EPSILON 0.0000001

// handle diagnostic informations given by assertion and abort program execution:
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
  Serial.println("ASSERTION FAILURE");
  // transmit diagnostic informations through serial link. 
  Serial.println(__func);
  Serial.println(__file);
  Serial.println(__lineno, DEC);
  Serial.println(__sexp);
  Serial.flush();
  // abort program execution.
  abort();
}

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
// NOTE: motor shield must be powered at 9 volts, separately from the Arduino.


int homing_complete = 0;
int csw_x_motion_pin = 22;
int stepper_position = 0;
int csw_z_motion_upper_pin = 23;
int csw_z_motion_lower_pin = 24;
int hall_sensor_pin = 41;
int ir_sensor_pin = A8;

const double FULL_POUR_DURATION = 5.0;//seconds
const double CHAMBER_REFILL_DURATION = 5.0;//seconds
const int POURER_BACKOFF_STEPS = 1000;//steps

const double PANIC_STOP_ACCELERATION = 10000.0;
const double NORMAL_ACCELERATION = 200.0;

const int IR_SENSOR_THRESHOLD = 512;

#define DISABLE_MOTORS 0

enum {
  EEPROM_INGREDIENT_NAME_SIZE = 32,
  NUM_INGREDIENTS = 10,
};

char ingredients[NUM_INGREDIENTS][EEPROM_INGREDIENT_NAME_SIZE] = {0};

//////////////////////////////
// WiFi Network Definitions //
//////////////////////////////
// Replace these two character strings with the name and
// password of your WiFi network.
//const char mySSID[] = "WestColeman-2.4_EXT";
//const char myPSK[] = "HannaLouise1";
const char mySSID[] = "BG_Mixed";
const char myPSK[] = "uptotheelbow";


ESP8266Server server = ESP8266Server(80);


//////////////////
// HTTP Strings //
//////////////////
const char destServer[] = "example.com";
const String htmlHeader = "HTTP/1.1 200 OK\r\n"
                          "Content-Type: text/html\r\n"
                          "Connection: close\r\n\r\n"
                          "<!DOCTYPE HTML>\r\n"
                          "<html>\r\n";

const String httpRequest = "GET / HTTP/1.1\n"
                           "Host: example.com\n"
                           "Connection: close\n\n";

const String http_OK = 
"HTTP/1.1 200 OK\n"
"Access-Control-Allow-Origin: *\n\n";

const String http_ERROR =
"HTTP/1.1 503 Service Unavailable\n"
"Access-Control-Allow-Origin: *\n\n";

String httpReply = http_OK;


// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int I2C_LCD_address = 0;
Adafruit_LiquidCrystal lcd(I2C_LCD_address);
                           
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Creates the motorshield object
Adafruit_StepperMotor *x_motor= AFMS.getStepper(200, 2);
Adafruit_StepperMotor *pourer_motor = AFMS.getStepper(200, 1);

//Wrapper functions to support accel stepping
void x_motor_forward_step() {
  x_motor->onestep(BACKWARD, DOUBLE);//This motor got reversed
}

void x_motor_backward_step() {
  x_motor->onestep(FORWARD, DOUBLE);//This motor got reversed
}

void pourer_motor_forward_step() {
  pourer_motor->onestep(FORWARD, DOUBLE);
}

void pourer_motor_backward_step() {
  pourer_motor->onestep(BACKWARD, DOUBLE);
}

//Acceleration stepper objects. Use these instead of the raw motor objects to command controlled motions.
AccelStepper x_motor_profile(x_motor_forward_step, x_motor_backward_step);
AccelStepper pourer_motor_profile(pourer_motor_forward_step, pourer_motor_backward_step);


enum motor_id {
  X_MOTOR,
  POURER_MOTOR,
  NO_MOTOR,
};
void runMotor(/*enum motor_id*/int mid, int nsteps, int direction);

//200 steps per rev, 66mm per rev, ~10cm between bottles
unsigned int bottle_position_nsteps[NUM_INGREDIENTS] = {
    20,
    187,
    354,
    521,
    688,
    855,
    1022,
    1189,
    1356,
    1523,
    };


void load_name_from_eeprom(int idx) {
  assert(idx < NUM_INGREDIENTS && idx >= 0);
  for (int ii = 0; ii < EEPROM_INGREDIENT_NAME_SIZE; ii++) {
    ingredients[idx][ii] = EEPROM[idx*EEPROM_INGREDIENT_NAME_SIZE + ii];
  }
}

void store_name_to_eeprom(int idx) {
  assert(idx < NUM_INGREDIENTS && idx >= 0);
  for (int ii = 0; ii < EEPROM_INGREDIENT_NAME_SIZE; ii++) {
    EEPROM[idx*EEPROM_INGREDIENT_NAME_SIZE + ii] = ingredients[idx][ii];
  }
}

void save_name_if_different(int idx, char * new_name) {
  assert(idx < NUM_INGREDIENTS && idx >= 0);

  //No matter what, update our local RAM copy
  strncpy(ingredients[idx], new_name, EEPROM_INGREDIENT_NAME_SIZE - 1);
  ingredients[idx][EEPROM_INGREDIENT_NAME_SIZE - 1] = '\0';
  int ingredient_length = strlen(ingredients[idx]);
  bool different = false;
  
  Serial.print("Checking position ");
  Serial.print(idx);
  Serial.print(" to see if it's different from ");
  Serial.println(new_name);
  
  for (int ii = 0; ii < EEPROM_INGREDIENT_NAME_SIZE && ii < ingredient_length; ii++) {
    if (ingredients[idx][ii] != EEPROM[idx*EEPROM_INGREDIENT_NAME_SIZE + ii]) {
      different = true;
    }
  }

  if (different) {
    Serial.println("it is! Storing the new name");
    store_name_to_eeprom(idx);
  } else {
    Serial.println("it's not. Doing nothing. ");
  }
}

void load_all_names() {
  for (int ii = 0; ii < NUM_INGREDIENTS; ii++) {
    load_name_from_eeprom(ii);
  }
}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
Serial.println("Starting up...");
Serial.flush();
  pinMode(A8, INPUT);
  pinMode(hall_sensor_pin, INPUT);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Startup message.
  lcd.print("SirMixabot!");

  load_all_names();
  
  // Setup the motors
  AFMS.begin(); // setsup the motor code
  x_motor_profile.setMaxSpeed(2000);
  x_motor_profile.setAcceleration(NORMAL_ACCELERATION);
  pourer_motor_profile.setMaxSpeed(2000);
  pourer_motor_profile.setAcceleration(NORMAL_ACCELERATION);
  
//  serialTrigger(F("Press any key to begin."));

  // initializeESP8266() verifies communication with the WiFi
  // shield, and sets it up.
  initializeESP8266();

  // connectESP8266() connects to the defined WiFi network.
  connectESP8266();

  // displayConnectInfo prints the Shield's local IP
  // and the network it's connected to.
  displayConnectInfo();

  serialTrigger(F("Press any key to connect client."));
  clientDemo();
  
  serialTrigger(F("Press any key to test server."));
  serverSetup();

  do_homing();
}


void panic_stop(int pos) {
    x_motor->setSpeed(0);
    x_motor_profile.setAcceleration(PANIC_STOP_ACCELERATION);
    x_motor_profile.stop();
    x_motor_profile.setAcceleration(NORMAL_ACCELERATION);
    x_motor_profile.setCurrentPosition(pos);
}

void do_safe_x_motor_move(int pos) {
    bool home_safe = digitalRead(csw_x_motion_pin);
    bool hit_hall = !digitalRead(hall_sensor_pin);
      
    Serial.print("PRE_MOVE distanceToGo: ");
    Serial.println(x_motor_profile.distanceToGo());

    int counter = 0;
    int distanceToGo = x_motor_profile.distanceToGo();
    while (home_safe && x_motor_profile.distanceToGo() != 0/*stepper_position != pos*/) {
      runMotor(X_MOTOR, 0, 0);
      home_safe = digitalRead(csw_x_motion_pin);
      hit_hall = !digitalRead(hall_sensor_pin);
      counter++;
    }

    Serial.print("AFTER MOVE distanceToGo: ");
    Serial.println(x_motor_profile.distanceToGo());
    Serial.print("AFTER MOVE home safe: ");
    Serial.println(home_safe);
    Serial.print("AFTER MOVE counter: ");
    Serial.println(counter);
    
    if (!home_safe) {
        //stepper_position = 0;
        Serial.println("Hit the HOME switch!");
        panic_stop(0);
        x_motor_profile.moveTo(10);//->step(10,BACKWARD,DOUBLE);
        while (x_motor_profile.distanceToGo() != 0) {
          runMotor(X_MOTOR, 0, 0);
        }
        stepper_position = 10;
    }

    if (hit_hall && x_motor_profile.distanceToGo() > 0) {
      Serial.print("Hit the hall sensor - ");
      Serial.print(x_motor_profile.distanceToGo());
      Serial.println(" steps left to go");
      panic_stop(pos);
    } else if (!hit_hall && x_motor_profile.distanceToGo() <= 0) {
      Serial.println("Didn't hit hall - but stepper says we're there");
    } else {
      Serial.println("How did this happen????");
    }
}

void do_homing() {
  if (DISABLE_MOTORS) {
    Serial.println("Faking being home...");
    homing_complete = false;
    stepper_position = 0;
    return;
  }
  if (!homing_complete) {
    Serial.println("Homing...");
    if (digitalRead(csw_x_motion_pin)) {
      float old_max_speed = x_motor_profile.maxSpeed();
      x_motor_profile.setMaxSpeed(old_max_speed / 2.0);
      x_motor_profile.moveTo(-10000000);
      while (digitalRead(csw_x_motion_pin)) {
        runMotor(X_MOTOR, 0, 0);
      }
      x_motor_profile.setMaxSpeed(old_max_speed);
    }
    while (digitalRead(csw_z_motion_lower_pin)) {
      runMotor(POURER_MOTOR, 15, BACKWARD);//pourer_motor->step(POURER_BACKOFF_STEPS, BACKWARD, DOUBLE);
    }
    Serial.println("Homing Complete!");
    stepper_position = 0;
    homing_complete = true;
    panic_stop(0);
    x_motor_profile.moveTo(10);
    while (x_motor_profile.distanceToGo() != 0) {
      runMotor(X_MOTOR, 0, 0);
    }
    stepper_position = 10;
  }

  homing_complete = false;
  x_motor->release();
}


void go_to_position(int pos) {
    int n_steps = bottle_position_nsteps[pos] - x_motor_profile.currentPosition();
    Serial.print("bottle position: ");
    Serial.print(bottle_position_nsteps[pos]);
    Serial.print(" stepper_pos: ");
    Serial.print(x_motor_profile.currentPosition());
    Serial.print("   n_steps: ");
    Serial.println(n_steps);
    x_motor_profile.moveTo(bottle_position_nsteps[pos]);
    do_safe_x_motor_move(bottle_position_nsteps[pos]);
    Serial.print("Endof move stepper pos: ");
    Serial.println(x_motor_profile.currentPosition());
}


//param portion Number of shots to pour
void pour(double portion) {
  int hall_sensor_active = !digitalRead(hall_sensor_pin);
  if (!hall_sensor_active) {
    Serial.println("Can't pour because there's no magnet at this location!");
    //homing_complete = false;
    //do_homing();
    return;
  }
  
  Serial.println("Starting pour");
    while (portion > 0.0) {
      // Pourer motor calibrates
      while (digitalRead(csw_z_motion_upper_pin)) {
          runMotor(POURER_MOTOR, 15, FORWARD);//pourer_motor->step(15, FORWARD, DOUBLE);
      }
      Serial.println("Pouring...");
      delay(1000 * (int)fmin(FULL_POUR_DURATION, (portion * FULL_POUR_DURATION)));//Pour for either a full shot or the fraction of a shot that remains
      portion -= 1.0;//Always step down by whole shots, no problem going negative

      Serial.println("Backing off...");
      while (digitalRead(csw_z_motion_lower_pin)) {
        runMotor(POURER_MOTOR, 15, BACKWARD);//pourer_motor->step(POURER_BACKOFF_STEPS, BACKWARD, DOUBLE);
      }
      
      if (portion > 0.0) {//More than 1 shot requested, let the chamber fill back up before we pour again
          delay(1000 * (int)fmin(CHAMBER_REFILL_DURATION, (portion * CHAMBER_REFILL_DURATION)));
      }
    }

    pourer_motor->release();
}

// Create a drink based on positions of 4 potential mixes
void dispenseIngredients(int *booze_positions, float *amounts, int num_ingredients) {
  if (DISABLE_MOTORS) {
    Serial.println("Motors are disabled");
    return;
  }
  if (analogRead(ir_sensor_pin) > IR_SENSOR_THRESHOLD) {
    lcd.print("Feed me a glass!");
    Serial.println("Feed me a glass!");
  }
  while (analogRead(ir_sensor_pin) > IR_SENSOR_THRESHOLD) {
    //FIXME this is a bad idea.
  }
  homing_complete = false;
  do_homing();
  
  //x_motor_profile.enableOutputs();
  int idx = 0; 
  for (idx = 0; idx < num_ingredients; idx++) {
    go_to_position(booze_positions[idx] - 1);
    pour(amounts[idx]);
  }

  homing_complete = false;
  do_homing();
}


unsigned long global_step_counter = 0;
void runMotor(int mid, int nsteps, int direction) {
  if (X_MOTOR == mid) {
    x_motor_profile.run();
  } else if (POURER_MOTOR == mid) {
    pourer_motor->step(nsteps, direction, DOUBLE);
  } else if (NO_MOTOR == mid) {
    //Do nothing
  } else {
    Serial.println("What other motor are you running????");
  }

  global_step_counter++;
}




/////////////////////////////////
//// WiFi code
/////////////////////////////////

void initializeESP8266()
{
  // esp8266.begin() verifies that the ESP8266 is operational
  // and sets it up for the rest of the sketch.
  // It returns either true or false -- indicating whether
  // communication was successul or not.
  // true
  int test = esp8266.begin();
  if (test != true)
  {
    Serial.println(F("Error talking to ESP8266."));
    errorLoop(test);
  }
  Serial.println(F("ESP8266 Shield Present"));
}

void connectESP8266()
{
  // The ESP8266 can be set to one of three modes:
  //  1 - ESP8266_MODE_STA - Station only
  //  2 - ESP8266_MODE_AP - Access point only
  //  3 - ESP8266_MODE_STAAP - Station/AP combo
  // Use esp8266.getMode() to check which mode it's in:
  int retVal = esp8266.getMode();
  if (retVal != ESP8266_MODE_STA)
  { // If it's not in station mode.
    // Use esp8266.setMode([mode]) to set it to a specified
    // mode.
    retVal = esp8266.setMode(ESP8266_MODE_STA);
    if (retVal < 0)
    {
      Serial.println(F("Error setting mode."));
      errorLoop(retVal);
    }
  }
  Serial.println(F("Mode set to station"));

  // esp8266.status() indicates the ESP8266's WiFi connect
  // status.
  // A return value of 1 indicates the device is already
  // connected. 0 indicates disconnected. (Negative values
  // equate to communication errors.)
  retVal = esp8266.status();
  if (retVal <= 0)
  {
    Serial.print(F("Connecting to "));
    Serial.println(mySSID);
    // esp8266.connect([ssid], [psk]) connects the ESP8266
    // to a network.
    // On success the connect function returns a value >0
    // On fail, the function will either return:
    //  -1: TIMEOUT - The library has a set 30s timeout
    //  -3: FAIL - Couldn't connect to network.
    retVal = esp8266.connect(mySSID, myPSK);
    if (retVal < 0)
    {
      Serial.println(F("Error connecting"));
      errorLoop(retVal);
    }
  }
}

void displayConnectInfo()
{
  char connectedSSID[24];
  memset(connectedSSID, 0, 24);
  // esp8266.getAP() can be used to check which AP the
  // ESP8266 is connected to. It returns an error code.
  // The connected AP is returned by reference as a parameter.
  int retVal = esp8266.getAP(connectedSSID);
  if (retVal > 0)
  {
    Serial.print(F("Connected to: "));
    Serial.println(connectedSSID);
  }

  // esp8266.localIP returns an IPAddress variable with the
  // ESP8266's current local IP address.
  IPAddress myIP = esp8266.localIP();
  Serial.print(F("My IP: ")); Serial.println(myIP);
  lcd.println(F("My IP: "));
  lcd.print(myIP);
}

void clientDemo()
{
  // To use the ESP8266 as a TCP client, use the 
  // ESP8266Client class. First, create an object:
  ESP8266Client client;

  // ESP8266Client connect([server], [port]) is used to 
  // connect to a server (const char * or IPAddress) on
  // a specified port.
  // Returns: 1 on success, 2 on already connected,
  // negative on fail (-1=TIMEOUT, -3=FAIL).
  int retVal = client.connect(destServer, 80);
  if (retVal <= 0)
  {
    Serial.println(F("Failed to connect to server."));
    return;
  }

  // print and write can be used to send data to a connected
  // client connection.
  client.print(httpRequest);

  delay(100);

  // available() will return the number of characters
  // currently in the receive buffer.
  while (client.available())
    Serial.write(client.read()); // read() gets the FIFO char
  
  // connected() is a boolean return value - 1 if the 
  // connection is active, 0 if it's closed.
  if (client.connected())
    client.stop(); // stop() closes a TCP connection.
}

void serverSetup()
{
  // begin initializes a ESP8266Server object. It will
  // start a server on the port specified in the object's
  // constructor (in global area)
  server.begin();
  Serial.print(F("Server started! Go to "));
  Serial.println(esp8266.localIP());
  Serial.println();
}

void analyzeDrinkRequest(const char * drinkRequest) {
  char * url = strstr(drinkRequest, "?");
  url++;//skip the question mark
  char * end_of_line = strstr(url, "\n");
  end_of_line[1] = '\0';
  char * HTTP_version = strstr(url, " HTTP");
  if (HTTP_version) {
    HTTP_version[0] = '\0';//Cut off the HTTP version if it exists.
  }
  char * tok = strtok(url, "p=&");
  int token_num = 0;
  int booze_positions[NUM_INGREDIENTS] = {0};
  float amounts[NUM_INGREDIENTS] = {0.0};
  int idx = 0;
  while (tok) {
    if (token_num++ % 2 == 0) {
      int val = atoi(tok);
      if (val < 1 || val > NUM_INGREDIENTS) {
        Serial.print("Parsed a nonsensical pour position number: ");
        Serial.println(val);
        Serial.print("From ");
        Serial.println(tok);
        httpReply = http_ERROR;
        httpReply.concat(__FUNCTION__);
        httpReply.concat("() - parsed a nonsensical pour position number ");
        httpReply.concat(val);
        httpReply.concat("\nOriginal string: ");
        httpReply.concat(tok);
        httpReply.concat("\n\n");
        return;
      }
      Serial.print("Dispensing position ");
      Serial.print(val);
      Serial.print(" - ");
      booze_positions[idx] = val;
    } else {
      float val = atof(tok);
      if (val < EPSILON) {
        Serial.print("Parsed a nonsensical pour amount: ");
        Serial.println(val);
        Serial.print("From: ");
        Serial.println(tok);
        httpReply = http_ERROR;
        httpReply.concat(__FUNCTION__);
        httpReply.concat("() - parsed a nonsensical pour amount ");
        httpReply.concat(val);
        httpReply.concat("\nOriginal string: ");
        httpReply.concat(tok);
        httpReply.concat("\n\n");
        return;
      }
      Serial.print(val);
      Serial.println(" shots");
      amounts[idx++] = val;
      
    }
    tok = strtok(NULL, "p=& ");
  }
  //FIXME send the client reply before moving, probably...
  dispenseIngredients(booze_positions, amounts, idx);
  httpReply = http_OK;
}

void analyzeIngredientRequest(const char * ingredient_request) {
  char * set_ingredient = strstr(ingredient_request, "GET /ingredients?p");
  if (set_ingredient) {
    Serial.println("Setting ingredients!");
    char * url = strstr(ingredient_request, "?");
    url++;//skip the question mark
    char * end_of_line = strstr(url, "\n");
    if (!end_of_line) {
      Serial.println("Got an HTTP request without a newline, bailing...");
      httpReply = http_ERROR;
      httpReply.concat(__FUNCTION__);
      httpReply.concat("() - got an HTTP request without a newline, bailing...\n\n");
      return;
    }
    end_of_line[1] = '\0';//Don't let further string functions search past the newline
    char * HTTP_version = strstr(url, " HTTP");
    if (HTTP_version) {
      HTTP_version[0] = '\0';//Cut off the HTTP version if it exists.
    }
    char * tok = strtok(url, "=&");
    int token_num = 0;
    int booze_positions[NUM_INGREDIENTS] = {0};
    char * names[NUM_INGREDIENTS] = {NULL};
    char safe_name[EEPROM_INGREDIENT_NAME_SIZE*3] = {0};//Since every "character" can be a space, which translates to %20 in a URL. Ugh.
    int idx = 0;
    while (tok) {
      if (token_num++ %2 == 0) {
        int val = atoi(tok+1);//don't try to use the p in p1, p3, p10, etc...
        if (val < 0 || val > NUM_INGREDIENTS) {//FIXME return this to 1 and fix line 667
          Serial.print("Tried to parse nonsensical store ingredient position ");
          Serial.println(val);
          Serial.print("From original string ");
          Serial.println(tok);
          httpReply = http_ERROR;
          httpReply.concat(__FUNCTION__);
          httpReply.concat("() - tried to parse nonsensical store ingredient position ");
          httpReply.concat(val);
          httpReply.concat("\nFrom original string ");
          httpReply.concat(tok);
          httpReply.concat("\n\n");
          return;
        }
        val += 1;//FIXME remove this and fix line 653
        Serial.print("Setting position ");
        Serial.print(val);
        Serial.print(" to ");
        booze_positions[idx] = val;
      } else {
        strncpy(safe_name, tok, EEPROM_INGREDIENT_NAME_SIZE*3 - 1);
        //Change %20 to space
        char * percent_twenty = strstr(safe_name, "%20");
        int number_spaces = 0;
        while (percent_twenty) {
          percent_twenty[0] = ' ';
          number_spaces++;
          while (percent_twenty[3]) {//As long as there's another character, move the string up
            percent_twenty[1] = percent_twenty[3];
            percent_twenty++;
          }
          percent_twenty = strstr(safe_name, "%20");
        }
        if (number_spaces != 0) {
          //Cut off the end of this string that's now garbage
          int last_char = strlen(safe_name);
          last_char -= number_spaces*2;
          if (last_char > 0) {
            safe_name[last_char] = '\0';
          }
        }
        Serial.println(safe_name);
        save_name_if_different(booze_positions[idx++] - 1, safe_name);// - 1 for zero-indexing
      }
      tok = strtok(NULL, "=&");
    }
    httpReply = http_OK;
  } else {//Get an ingredients list from arduino
    Serial.println("Getting ingredients!");
    httpReply = http_OK;
    httpReply.concat("{\"ingredients\": [");
    for (int ii = 0; ii < NUM_INGREDIENTS; ii++) {
      httpReply.concat("{\"position\": ");
      httpReply.concat(ii + 1);
      httpReply.concat(",\"name\": \"");
      httpReply.concat(ingredients[ii]);
      httpReply.concat("\"}");
      if (ii != NUM_INGREDIENTS - 1) {
        httpReply.concat(",");
      }
    }
    httpReply.concat("]}\n\n");
    Serial.println(httpReply);
  }
}

//For getting a list of ingredients:
//GET /ingredients
//For setting an ingredient:
//GET /ingredients?p1=vodka&...
//For making a drink:
//GET /drinks/make?p1=1.0&p5=0.5&...
//
//Post-Condition: This function, and any functions it calls, are 
//responsible for setting httpReply to a sensible value before returning.
void analyzeGetRequest(const char * httpRequest) {
  Serial.println("Reprinting request:");
  Serial.println(httpRequest);
  Serial.println("Request END");
  
  char * drink_request = strstr(httpRequest, "GET /drinks/make?");
  if (drink_request) {
    analyzeDrinkRequest(drink_request);
    return;
  }

  char * ingredient_request = strstr(httpRequest, "GET /ingredients");
  if (ingredient_request) {
    analyzeIngredientRequest(ingredient_request);
    return;
  }

  //If we got here, the request was malformed by the time we got to analyze it
  httpReply = http_ERROR;
  httpReply.concat(__FUNCTION__);
  httpReply.concat("() - no matching URL was recognized\n\n");
}



void serverDemo()
{
  // available() is an ESP8266Server function which will
  // return an ESP8266Client object for printing and reading.
  // available() has one parameter -- a timeout value. This
  // is the number of milliseconds the function waits,
  // checking for a connection.
  ESP8266Client client = server.available(5000);
  
  if (client) 
  {
    Serial.println("Client Connected!");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    char * found_string = client.searchBuffer("");
    
    if (found_string) {
      analyzeGetRequest(found_string);
    } else {
      httpReply = http_ERROR;
      httpReply.concat(__FUNCTION__);
      httpReply.concat("() - no found_string!\n\n");
    }

    //Reply
    client.print(httpReply);
      
    // give the web browser time to receive the data
    delay(100);
    
    // close the connection:
    client.stop();
    Serial.println(F("Client disconnected"));
    delay(20000);
  }
  
}

// errorLoop prints an error code, then loops forever.
void errorLoop(int error)
{
  Serial.print(F("Error: ")); Serial.println(error);
  Serial.println(F("Looping forever."));
  for (;;)
    ;
}

// serialTrigger prints a message, then waits for something
// to come in from the serial port.
void serialTrigger(String message)
{
  Serial.println();
  Serial.println(message);
  Serial.println();
  while (!Serial.available())
    ;
  while (Serial.available())
    Serial.read();
}


void loop() {
  // Normal Drink Maker
  //

  serverDemo();
}
