#include <SparkFunESP8266WiFi.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Wire.h>
#include <AccelStepper.h>
#include <LiquidCrystal.h>


// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
// NOTE: motor shield must be powered at 9 volts, separately from the Arduino.


int homing_complete = 0;
int csw_x_motion_pin = 22;
int stepper_position = 0;
int csw_z_motion_pin = 23;
int hall_sensor_pin = 41;
int ir_sensor_pin = A8;

const double FULL_POUR_DURATION = 5.0;//seconds
const double CHAMBER_REFILL_DURATION = 5.0;//seconds
const int POURER_BACKOFF_STEPS = 1000;//steps

const double PANIC_STOP_ACCELERATION = 10000.0;
const double NORMAL_ACCELERATION = 200.0;

const int IR_SENSOR_THRESHOLD = 512;

enum {
  WEBSERVER_SLOW_TIMEOUT_MSEC = 500,//milliseconds to wait on an incoming connection when nothing is going on
  WEBSERVER_FAST_TIMEOUT_MSEC = 1,  //milliseconds to wait on an incoming connection when a motor is moving
  WEBSERVER_TIMELINESS_CONST  = 200,//Number of steps between checking the web server. Adjust up to make motion smoother, adjust down to make server more responsive.
  
};


//////////////////////////////
// WiFi Network Definitions //
//////////////////////////////
// Replace these two character strings with the name and
// password of your WiFi network.
const char mySSID[] = "HOME-C129-5";
const char myPSK[] = "HarrisonNelson1";


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

const String httpReply =
"HTTP/1.1 200 OK\n"
"Access-Control-Allow-Origin: *\n\n" 
"{\n"
  "\"drinks\": [\n"
    "{\n"
      "\"name\": \"Martinez\",\n"
      "\"ingredients\": [\n"
        "{\"name\": \"Gin\", \"quantity\": \"1.5 oz\"},\n"
        "{\"name\": \"Vermouth\", \"quantity\": \"1.5 oz\"},\n"
        "{\"name\": \"Maraschino\", \"quantity\": \"1 tsp\"}\n"
      "]\n"
    "},\n"
  "]\n"
"}\n\n";


// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 26, en = 28, d4 = 30, d5 = 32, d6 = 34, d7 = 36;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
                           
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Creates the motorshield object
Adafruit_StepperMotor *x_motor= AFMS.getStepper(200, 2);
Adafruit_StepperMotor *pourer_motor = AFMS.getStepper(200, 1);

//Wrapper functions to support accel stepping
void x_motor_forward_step() {
  x_motor->onestep(FORWARD, DOUBLE);
}

void x_motor_backward_step() {
  x_motor->onestep(BACKWARD, DOUBLE);
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


typedef enum bottle_pos{
    BOTTLE_POS_1,
    BOTTLE_POS_2,
    BOTTLE_POS_3,
    BOTTLE_POS_4,
    BOTTLE_POS_5,
    BOTTLE_POS_6,
} BottlePosition;

enum motor_id {
  X_MOTOR,
  POURER_MOTOR,
  NO_MOTOR,
};
void runMotor(/*enum motor_id*/int mid, int nsteps, int direction);

//200 steps per rev, 66mm per rev, ~10cm between bottles
unsigned int bottle_position_nsteps[6] = {
    (3),
    280,//(10) + ((10 * 10 * 200) / 66),
    560,//(10) + ((20 * 10 * 200) / 66),
    840,//(10) + ((30 * 10 * 200) / 66),
    1120,//(10) + ((40 * 10 * 200) / 66),
    1525,//(10) + ((50 * 10 * 200) / 66),
    };




void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  pinMode(A8, INPUT);
  pinMode(hall_sensor_pin, INPUT);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Startup message.
  lcd.print("SirMixabot!");
  
  // Setup the motors
  AFMS.begin(); // setsup the motor code
  x_motor_profile.setMaxSpeed(2000);
  x_motor_profile.setAcceleration(NORMAL_ACCELERATION);
  pourer_motor_profile.setMaxSpeed(2000);
  pourer_motor_profile.setAcceleration(NORMAL_ACCELERATION);
  
  serialTrigger(F("Press any key to begin."));

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
//    bool direction = pos > stepper_position;

    int counter = 0;
    int distanceToGo = x_motor_profile.distanceToGo();
    while (home_safe && x_motor_profile.distanceToGo() != 0/*stepper_position != pos*/) {
      runMotor(X_MOTOR, 0, 0);
//      x_motor->step(1, direction ? FORWARD : BACKWARD, DOUBLE);
      home_safe = digitalRead(csw_x_motion_pin);
      hit_hall = !digitalRead(hall_sensor_pin);
      counter++;
//      stepper_position += direction ? 1 : -1;
//      int distance = x_motor_profile.distanceToGo();
//      if (distance != distanceToGo) {
//        Serial.print("New distance to go: ");
//        Serial.println(x_motor_profile.distanceToGo());
//        Serial.print("At counter ");
//        Serial.println(counter);
//      }
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
  if (!homing_complete) {
    Serial.println("Homing...");
    if (digitalRead(csw_x_motion_pin)) {
      x_motor_profile.moveTo(-10000000);
      while (digitalRead(csw_x_motion_pin)) {
        runMotor(X_MOTOR, 0, 0);
      }
    }
    Serial.println("Homing Complete!");
    stepper_position = 0;
    homing_complete = 1;
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
        while (digitalRead(csw_z_motion_pin)) {
            runMotor(POURER_MOTOR, 15, FORWARD);//pourer_motor->step(15, FORWARD, DOUBLE);
//            Serial.println("Pourer Calibrate");
        }
        Serial.println("Pouring...");
        delay(1000 * (int)fmin(FULL_POUR_DURATION, (portion * FULL_POUR_DURATION)));//Pour for either a full shot or the fraction of a shot that remains
        portion -= 1.0;//Always step down by whole shots, no problem going negative

        Serial.println("Backing off...");
        runMotor(POURER_MOTOR, POURER_BACKOFF_STEPS, BACKWARD);//pourer_motor->step(POURER_BACKOFF_STEPS, BACKWARD, DOUBLE);
        if (portion > 0.0) {//More than 1 shot requested, let the chamber fill back up before we pour again
            delay(1000 * (int)fmin(CHAMBER_REFILL_DURATION, (portion * CHAMBER_REFILL_DURATION)));
        }
    }

    pourer_motor->release();
}

// Create a drink based on positions of 4 potential mixes
void martini() {
  while (analogRead(ir_sensor_pin) > IR_SENSOR_THRESHOLD) {
    Serial.println("Feed me a glass!");
    //Set the LCD screen to "feed me a glass"
  }
  homing_complete = false;
  do_homing();
  
  //x_motor_profile.enableOutputs();
  go_to_position(BOTTLE_POS_1);
  pour(1);
  go_to_position(BOTTLE_POS_3);
  pour(1);
  go_to_position(BOTTLE_POS_5);
  pour(1);

  homing_complete = false;
  do_homing();
}


unsigned long global_step_counter = 0;
void runMotor(int mid, int nsteps, int direction) {
  if (X_MOTOR == mid) {
    x_motor_profile.run();
  } else if (POURER_MOTOR == mid) {
//  pourer_motor_profile.run()
    pourer_motor->step(nsteps, direction, DOUBLE);
  } else if (NO_MOTOR == mid) {
    //Do nothing
  } else {
    Serial.println("What other motor are you running????");
  }
/*
  if (NO_MOTOR == mid) {
    serverCheck(WEBSERVER_SLOW_TIMEOUT_MSEC);
  } else if (global_step_counter % WEBSERVER_TIMELINESS_CONST == 0) {
    serverCheck(WEBSERVER_FAST_TIMEOUT_MSEC);
  } else {
    //Do nothing with the server this step
  }*/

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
  
}

void analyzeGetRequest(const char * getRequest) {
  Serial.println("Reprinting request:");
  Serial.println(getRequest);
  Serial.println("Request END");
  char * drink_request = strstr(getRequest, "GET /drinks/make");
  if (drink_request) {
    martini();
    homing_complete = false;//Go back home now.
    Serial.println("Found a drink request!");
    drink_request = strstr(drink_request, "?");
    Serial.println(drink_request+1);
  }
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
    //while (client.connected()) 
    //{
      char * found_string = client.searchBuffer("");
      if (found_string) {
        analyzeGetRequest(found_string);
      }

      //Reply
      client.print(httpReply);
      /*
      if (client.available()) 
      {
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) 
        {
          Serial.println(F("Sending HTML page"));
          Serial.println(F("Here's the GET request:"));
          getRequest[sizeof(getRequest) - 1] = 0;
//          Serial.println(getRequest);
          analyzeGetRequest();
          getRequestIndex = 0;
          // send a standard http response header:
          client.print(htmlHeader);
          String htmlBody;
          // output the value of each analog input pin
          for (int a = 0; a < 6; a++)
          {
            htmlBody += "A";
            htmlBody += String(a);
            htmlBody += ": ";
            htmlBody += String(analogRead(a));
            htmlBody += "<br>\n";
          }
          htmlBody += "time: ";
          htmlBody += String(millis());
          htmlBody += "<br>\n";
          htmlBody += "NBytes: ";
          htmlBody += String(getRequestIndex);
          htmlBody += "<br>\n";
          htmlBody += "Received: ";
          htmlBody += getRequest;
          htmlBody += "<br>\n";
          htmlBody += "</html>\n";
          client.print(htmlBody);
          memset(getRequest, 0, sizeof(getRequest));
          break;
        }
        getRequest[getRequestIndex++] = c;
        if (getRequestIndex >= sizeof(getRequest)) {
          getRequestIndex = 0;
        }
        if (c == '\n') 
        {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') 
        {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }*/
    //}
    // give the web browser time to receive the data
    delay(100);
    
    // close the connection:
    client.stop();
    Serial.println(F("Client disconnected"));
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
