/*
Bot programming code: Written on CPP modified to work on Arduino IDE
Written for and by Electronics & Robotics Club, IIT Bombay with assistance from internet
resources.

For: Raspberry Pi Pico W to navigate a 4-wheeled bot based on differential
drive(INCOMPLETE) NOTE: ALL LIBRARIES COME PREINSTALLED IN YOUR ARDUINO IDE, YOU AREN'T
EXPECTED TO INSTALL ANYTHING EXTRA OTHER THAN WHAT WAS TAUGHT IN GET CODIFIED. ©ERC //
MIT License 2024
*/

/*//////////////////////////////////////////////////////////////////////////////

# ASSUMPTIONS

L289D takes 12V VCC (+ve) and GND (-ve)
rpi takes power from 5V VOUT of L289D (+ve) and GND (-ve)

GPIO connections:
  GPIO 4 (6 from top)   - SERVOPIN (PWM)
  GPIO 8 (11 from top)  - ENA (enables motor A)
  GPIO 9 (12 from top)  - ENB (enables motor B)
  GPIO 10-14            - IN1-4

L289N connections:
  ENA, IN 1-2, OUT 1-2  - motor A
  ENB, IN 3-4, OUT 3-4  - motor B

  ENA/B pins read [0-255] input
  IN 1-4 pins read LOW/HIGH input
  SERVOPIN reads angle input

Choice of direction:
  motor A - left
  motor B - right

Angle [-90, 90] +ve => right, -ve => left, 0 => straight (steering angle basically)
Speed [-100, 100] +ve forward -ve backward

Constants are defined below

//////////////////////////////////////////////////////////////////////////////*/

#include <ArduinoJson.h>
#include <Servo.h>
#include <WiFi.h>

#define _USE_AP_

const char* ssidAP = "LevitasPicoW2_AP";
const char* passwordAP = "lightness";
const char* ssidExt = "echo";
const char* passwordExt = "qwertyui";
const char* hostname = "LevitasPicoW2";
const short port = 80;
const char* localIP;
const short dataReadingTimeout = 10;
const short dataPollingTimeout = 1;

const float CF = 3.14159265 / 180;  // this is as good as float can be
const short MAX_SERVO_ANGLE = 60;
const short MAX_STEERING_ANGLE = 45;
const float L = 11;   // Shaft to shaft length from front to back in cm
const float W = 14.5;   // Shaft to shaft width from left to right in cm
const short DEL = 0;  // Distance between axes of universal joint and tyre

typedef enum {
    SERVOPIN = 4,
    ENA = 8,
    ENB = 9,
    IN1 = 10,
    IN2 = 11,
    IN3 = 12,
    IN4 = 13
} OutPin;

typedef enum { MOTOR_A = 1, MOTOR_B = 2 } MotorMap;

////////////////////////////////////////////////////////////////////////////////

// create a wifi server and wifi client object for the TCP server
WiFiServer server(port);
WiFiClient client;

// declare servo object
Servo servo;

// define global variables from relevant functions
short servo_angle;
short status;
short speed, angle, _angle;
float r1, r2, rr, lsig, rsig;
bool state;

////////////////////////////////////////////////////////////////////////////////

float sinD(float deg) { return sin(deg * CF); }
float cosD(float deg) { return cos(deg * CF); }
float tanD(float deg) { return tan(deg * CF); }
float asinD(float val) { return asin(val) / CF; }
template <typename T>
short sgn(T val) {
    return ((val > 0) ? 1 : -1);
}

////////////////////////////////////////////////////////////////////////////////

short clamp_angle_servo(short angle) {
    return ((abs(angle) <= MAX_SERVO_ANGLE)
                ? angle
                : ((angle < 0) ? -MAX_SERVO_ANGLE : MAX_SERVO_ANGLE));
}

float scale_speed(short speed) { return ((speed / 100.0) * 255); }

short convert_to_servo_angle(short angle) {
    if (angle > 0)
        angle *= 4 / 3;
    else if (angle < 0)
        angle *= 5 / 3;
    return (clamp_angle_servo(angle) * 16 / 8 + 62);
}

////////////////////////////////////////////////////////////////////////////////

// sets the radii of the path left and right wheels will follow
void set_radii(const short _angle) {
    float r3 = (L / tanD(_angle)) - (W / 2.0);
    r1 = sqrt(L * L + r3 * r3);
    r2 = sqrt(L * L + (r3 + W) * (r3 + W));
    rr = L / sinD(_angle);
}

// sets the signals of the left and right motors
void set_signals(const short _angle) {
    if (!angle) {
        rsig = lsig = scale_speed(100);
        return;
    }
    rsig = (r1 * (lsig * rr * cosD(asinD(W * sinD(_angle) / (2 * r2)))) / r2) /
           (rr * cosD(asinD(W * sinD(_angle) / (2 * r1))));
}

// corrects for axis of tyres and universal joint being different
void correct_radii(const short _angle) {
    r1 -= DEL * cosD(asinD(W * sinD(_angle) / (2 * r1)));
    r2 += DEL * cosD(asinD(W * sinD(_angle) / (2 * r2)));
}

// used when steering angle greater than maximum set axle steering angle
void set_signals_diff_steering() { rsig *= 2 - float(angle) / MAX_STEERING_ANGLE; }

////////////////////////////////////////////////////////////////////////////////

void reset_instr() {
    lsig = rsig = 0;
    servo_angle = convert_to_servo_angle(0);
}

short process_response(String response) {
    // for debugging
    // Serial.print("received response @ ");
    // Serial.print(millis());
    // Serial.print("ms : ");
    // Serial.println(response);

    // deserialize json
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, response);

    // catch error
    if (error) {
        // Serial.printf("Deserialization failed:\n  error=%s\n", error.c_str());
        return -1;
    }

    state = (doc["state"] == "on") ? true : false;
    speed = doc["speed"].as<short>();
    angle = doc["angle"].as<short>();

    // digitalWrite(LED_BUILTIN, ((state) ? HIGH : LOW));

    // if state is off OR speed = 0
    if (!state || !speed) {
        reset_instr();
        return 0;
    }

    // calculate servo angle, and lsig as base value
    servo_angle = convert_to_servo_angle(angle);
    // Serial.printf("angle = %d, servo_angle = %d, clamped_angle = %d\n", angle,
    //               servo_angle, clamp_angle_servo(angle));
    lsig = scale_speed(speed);

    // convert to absolute values
    short speed_sgn = sgn(speed);
    short angle_sgn = sgn(angle);
    lsig *= speed_sgn;
    angle *= angle_sgn;

    _angle = ((angle > MAX_STEERING_ANGLE) ? MAX_STEERING_ANGLE : angle);

    // perform calculations
    set_radii(_angle);
    correct_radii(_angle);
    set_signals(_angle);

    if (angle > MAX_STEERING_ANGLE) {
        set_signals_diff_steering();
    }

    // reconvert to actual values
    lsig *= speed_sgn;
    rsig *= speed_sgn;
    angle *= angle_sgn;
    if (angle_sgn == -1) {
        float tmp = lsig;
        lsig = rsig;
        rsig = tmp;
    }

    lsig = short(lsig);
    rsig = short(rsig);

    return 0;
}

short write_motor_speed(short motor, short signal) {
    // A = 1, B = 2
    short in_f, in_b, en;
    switch (motor) {
    case 1:
        in_f = IN1;
        in_b = IN2;
        en = ENA;
        break;
    case 2:
        in_f = IN3;
        in_b = IN4;
        en = ENB;
        break;
    default:
        break;  // hope control never reaches here
    }

    // forward
    if (signal > 0) {
        digitalWrite(in_f, HIGH);
        digitalWrite(in_b, LOW);
    }
    // backward
    else if (signal < 0) {
        digitalWrite(in_f, LOW);
        digitalWrite(in_b, HIGH);
    }
    // stop
    else {
        digitalWrite(in_f, LOW);
        digitalWrite(in_b, LOW);
    }

    // write to en pin
    analogWrite(en, abs(signal));

    return 0;
}

short send_instructions() {
    // Serial.printf("Sending %f %f %d", lsig, rsig, servo_angle);

    // write to servo
    servo.write(servo_angle);

    // write to left motors
    write_motor_speed(1, lsig);

    // write to right motors
    write_motor_speed(2, rsig);

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

void setup() {
    // init serial for debugging
    // Serial.begin(115200);
    // while (!Serial);

    // setup servo pin
    servo.attach(SERVOPIN);
    servo.write(convert_to_servo_angle(0));  // start from center posn

    // setup motor pins
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH);

#ifdef _USE_AP_

    // set up the access point
    // Serial.print("Setting up WiFi AP ");
    WiFi.softAP(ssidAP, passwordAP);

    // print hostname ip port
    localIP = WiFi.softAPIP().toString().c_str();
    // Serial.printf("as '%s' @ %s:%d\n", hostname, localIP, port);

#else  // use external network to connect

    // set up wifi ext
    // Serial.print("Setting up WiFi ext ");
    // Serial.printf("[ ssid=%s, password=%s ]\n", ssidExt, passwordExt);
    // start wifi connection
    WiFi.begin(ssidExt, passwordExt);
    // wait for establishing connection
    while (WiFi.status() != WL_CONNECTED) {
    }
    // print hostname ip port
    localIP = WiFi.localIP().toString().c_str();
    // Serial.printf("Connected as '%s' @ %s:%d\n", hostname, localIP, port);

#endif

    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostname);

    // start the server
    server.begin();
    while (WiFi.status() != WL_CONNECTED) {
    }
}

void loop() {
    // open connection to client
    client = server.accept();
    if (!client) {
        return;
    }

    // read bytes till non-zero bytes are received
    while (!client.available()) {
        delay(dataPollingTimeout);
    }

    digitalWrite(LED_BUILTIN, HIGH);

    // read client response
    String response = client.readStringUntil('\n');

    // catch error
    if (status = process_response(response)) {
        // Serial.println("caught error in processing response. continuing loop.");
        return;
    }

    if (status = send_instructions()) {
        // Serial.println("caught error in sending instructions. continuing loop.");
        return;
    }

    // Serial.println();
    digitalWrite(LED_BUILTIN, LOW);
    delay(dataReadingTimeout);
}

#ifdef _USE_AP_
#undef _USE_AP_
#endif
