/*
 Name:		Apto_Final.ino
 Author:	Mohammad Ibrahim Nasir, Omid Ghayouri, Ayush Dave, Hetvi Patel
*/

// the setup function runs once when you press reset or power the board
//#include <U8x8lib.h>
//#include <U8g2lib.h>
//#include <U8g2_for_Adafruit_GFX.h>
//#include <u8g2_fonts.h>
//#include <U8glib.h>
#include <TeensyThreads.h>
#include <Servo.h>
#include <gfxfont.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <splash.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <USBHost_t36.h>
#include <antplusdefs.h>
#include <DRV8835MotorShield.h>



//=============================================================================
// --------- START of BT Variable Declaration ---------------------------------
//=============================================================================
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
USBHIDParser hid4(myusb);
USBHIDParser hid5(myusb);
JoystickController joystick1(myusb);
BluetoothController bluet(myusb);   // version assumes it already was paired (PS3 Controller is already paired)
int user_axis[64];
uint32_t buttons_prev = 0;
uint32_t buttons;
RawHIDController rawhid1(myusb);
RawHIDController rawhid2(myusb, 0xffc90004);

USBDriver* drivers[] = { &hub1, &hub2, &joystick1, &bluet, &hid1, &hid2, &hid3, &hid4, &hid5 };

#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char* driver_names[CNT_DEVICES] = { "Hub1", "Hub2", "JOY1D", "Bluet", "HID1" , "HID2", "HID3", "HID4", "HID5" };

bool driver_active[CNT_DEVICES] = { false, false, false, false };

// Lets also look at HID Input devices
USBHIDInput* hiddrivers[] = { &joystick1, &rawhid1, &rawhid2 };

#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char* hid_driver_names[CNT_DEVICES] = { "Joystick1", "RawHid1", "RawHid2" };

bool hid_driver_active[CNT_DEVICES] = { false, false, false };

BTHIDInput* bthiddrivers[] = { &joystick1 };
#define CNT_BTHIDDEVICES (sizeof(bthiddrivers)/sizeof(bthiddrivers[0]))
const char* bthid_driver_names[CNT_HIDDEVICES] = { "joystick" };
bool bthid_driver_active[CNT_HIDDEVICES] = { false };


bool show_changed_only = false;
bool show_raw_data = false;
bool show_changed_data = false;

uint8_t joystick_left_trigger_value = 0;
uint8_t joystick_right_trigger_value = 0;
uint64_t joystick_full_notify_mask = (uint64_t)-1;

int psAxis[64];
bool first_joystick_message = true;
uint8_t last_bdaddr[6] = { 0, 0, 0, 0, 0, 0 };

// ps3 motion on USB does not do much, but see if we can pair it and maybe change
// color of bulb... 
uint32_t PS3_MOTION_timer = 0;
uint8_t  PS3_MOTION_tried_to_pair_state = 0;
#define PS3_MOTION_PERIOD 2500 // not sure yet what would be good period for this..

static const uint32_t PS3_MOTION_colors[] = { 0, 0xff, 0xff00, 0xff0000, 0xffff, 0xff00ff, 0xffff00, 0xffffff };
uint8_t PS3_MOTION_colors_index = 0;
//=============================================================================
// --------- END of BT Variable Declaration -----------------------------------
//=============================================================================

//=============================================================================
// --------- START of Motor Shield Variable Declaration  ----------------------
//=============================================================================
DRV8835MotorShield motors;
//=============================================================================
// --------- END of Motor Shield Variable Declaration  ------------------------
//=============================================================================

//=============================================================================
// --------- START of NeoPixel Variable Declaration  --------------------------
//=============================================================================
#define PIXEL_PIN   30    // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 10     // Number of LEDs in Strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
int toggle = 0;

//=============================================================================
// --------- END of NeoPixel Variable Declaration  ----------------------------
//=============================================================================

//=============================================================================
// ---------- START of BOOT UP Hardware declaration ---------------------------
//=============================================================================
#define PWR_LED_PIN 31
//=============================================================================
// ---------- END of BOOT UP Hardware declaration -----------------------------
//=============================================================================

//=============================================================================
// ---------- START of OLED screen declaration ---------------------------------
//=============================================================================
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };
unsigned char PROGMEM bluetooth_icon16x16[] =
{
  0b00000000, 0b00000000, //                 
  0b00000001, 0b10000000, //        ##       
  0b00000001, 0b11000000, //        ###      
  0b00000001, 0b01100000, //        # ##     
  0b00001001, 0b00110000, //     #  #  ##    
  0b00001101, 0b00110000, //     ## #  ##    
  0b00000111, 0b01100000, //      ### ##     
  0b00000011, 0b11000000, //       ####      
  0b00000001, 0b10000000, //        ##       
  0b00000011, 0b11000000, //       ####      
  0b00000111, 0b01100000, //      ### ##     
  0b00001101, 0b00110000, //     ## #  ##    
  0b00001001, 0b00110000, //     #  #  ##    
  0b00000001, 0b01100000, //        # ##     
  0b00000001, 0b11000000, //        ###      
  0b00000001, 0b10000000, //        ##       
};

unsigned char PROGMEM empty_square16x16[] =
{
  0b00000000, 0b00000000,
  0b00000001, 0b10000000,     
  0b00000001, 0b11000000,     
  0b00000001, 0b01100000,   
  0b00001001, 0b00110000,   
  0b00001101, 0b00110000,  
  0b00000111, 0b01100000,   
  0b00000011, 0b11000000,    
  0b00000001, 0b10000000,     
  0b00000011, 0b11000000,
  0b00000111, 0b01100000,    
  0b00001101, 0b00110000,   
  0b00001001, 0b00110000,  
  0b00000001, 0b01100000, 
  0b00000001, 0b11000000,   
  0b00000001, 0b10000000,  
};

unsigned char PROGMEM wifi1_icon16x16[] =
{
  0b00000000, 0b00000000, //                 
  0b00000111, 0b11100000, //      ######     
  0b00011111, 0b11111000, //    ##########   
  0b00111111, 0b11111100, //   ############  
  0b01110000, 0b00001110, //  ###        ### 
  0b01100111, 0b11100110, //  ##  ######  ## 
  0b00001111, 0b11110000, //     ########    
  0b00011000, 0b00011000, //    ##      ##   
  0b00000011, 0b11000000, //       ####      
  0b00000111, 0b11100000, //      ######     
  0b00000100, 0b00100000, //      #    #     
  0b00000001, 0b10000000, //        ##       
  0b00000001, 0b10000000, //        ##       
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
};

unsigned char PROGMEM signal4_icon16x16[] =
{
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000100, //              #  
  0b00000000, 0b00001100, //             ##  
  0b00000000, 0b00011100, //            ###  
  0b00000000, 0b00011100, //            ###  
  0b00000000, 0b01011100, //          # ###  
  0b00000000, 0b11011100, //         ## ###  
  0b00000001, 0b11011100, //        ### ###  
  0b00000001, 0b11011100, //        ### ###  
  0b00000101, 0b11011100, //      # ### ###  
  0b00001101, 0b11011100, //     ## ### ###  
  0b00011101, 0b11011100, //    ### ### ###  
  0b00011101, 0b11011100, //    ### ### ###  
  0b01011101, 0b11011100, //  # ### ### ###  
  0b11011101, 0b11011100, // ## ### ### ###  
  0b11011101, 0b11011100, // ## ### ### ###  
};

unsigned char PROGMEM bat2_icon16x16[] =
{
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00111111, 0b11111000, //   ###########   
  0b01111111, 0b11111100, //  #############  
  0b11111111, 0b11111110, // ############### 
  0b11101101, 0b10000110, // ### ## ##    ## 
  0b11101101, 0b10000111, // ### ## ##    ###
  0b11101101, 0b10000111, // ### ## ##    ###
  0b11101101, 0b10000110, // ### ## ##    ## 
  0b11111111, 0b11111110, // ############### 
  0b01111111, 0b11111100, //  #############  
  0b00111111, 0b11111000, //   ###########   
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
};
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);
//=============================================================================
// ---------- END of LCD screen declaration -----------------------------------
//=============================================================================

//=============================================================================
// ---------- START of Speaker declaration ---------------------------------
//=============================================================================
//SPEAKER
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

// change this to make the song slower or faster
int tempo = 114;
// change this to whichever pin you want to use
int buzzer = 41;

// notes of the moledy followed by the duration.
// a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
// !!negative numbers are used to represent dotted notes,
// so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
int melody[] = {

    // Mii Channel theme 
    // Score available at https://musescore.com/user/16403456/scores/4984153
    // Uploaded by Catalina Andrade 

    NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, //1
    NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, REST,4, REST,8, NOTE_CS4,8,
    NOTE_D4,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
    NOTE_E5,-4, NOTE_DS5,8, NOTE_D5,8, REST,8, REST,4,

};

// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;
 
// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;
int divider = 0, noteDuration = 0;

//=============================================================================
// ---------- START of LEG declaration ----------------------------------------
//=============================================================================
// Define Servo Structure
struct servo_struct {
    uint16_t number;
    uint16_t type;
    int position;
    Servo motor;
};

// Define Body Structure
struct body {
    float x;
    float y;
    float z;
    float facing;
};

//Define Foot as a vector
struct footPoint {
    // might need angle theta
        //float theta;

    float x;
    float y;
    float z;
};

// Defining Leg parameters
struct leg {
    uint8_t number;
    bool move;
    float phi;
    float z;
    float x;
    float y;
    servo_struct joint[3];
};

int led = 13;
Servo legServo1a;
Servo legServo1b;
Servo legServo1c;

Servo legServo2a;
Servo legServo2b;
Servo legServo2c;

Servo legServo3a;
Servo legServo3b;
Servo legServo3c;

Servo legServo4a;
Servo legServo4b;
Servo legServo4c;

servo_struct legServo4c_test;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position 
char input;
String command;
leg legs[4];

#define ARM0_MAX 180
#define ARM1_MAX 0
#define ARM2_MAX 180
#define ARM3_MAX 0

#define ARM0_MIN 0
#define ARM1_MIN 180
#define ARM2_MIN 0
#define ARM3_MIN 180

#define CLAW0_MAX 0
#define CLAW1_MAX 180
#define CLAW2_MAX 0
#define CLAW3_MAX 180

#define CLAW0_MIN 180
#define CLAW1_MIN 0
#define CLAW2_MIN 180
#define CLAW3_MIN 0

#define SHLR0_MAX 115
#define SHLR1_MAX 65
#define SHLR2_MAX 115
#define SHLR3_MAX 65

#define SHLR0_MIN 180
#define SHLR1_MIN 0
#define SHLR2_MIN 180
#define SHLR3_MIN 0

//=============================================================================
// ---------- END of LEG declaration ------------------------------------------
//=============================================================================


//=============================================================================
// Setup (is ran once)
//=============================================================================
void setup()
{
	//1.) PWR LED PinMode declaration
	pinMode(PWR_LED_PIN, OUTPUT);
	digitalWrite(PWR_LED_PIN, HIGH);

    //2.) NeoPixel setup code
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    uint16_t i;
    for (i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(50, 50, 75)); // white
    }
    strip.show();

    //3.) OLED Display setup code
	Serial1.begin(2000000);
    //3.1) set I2C ports
    Wire.setSDA(17);
    Wire.setSCL(16);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    delay(2000);
    // 3.2) Clear display, set textsize, and set cursor for location of text
    display.clearDisplay();
    display.setTextSize(4);
    display.setTextColor(WHITE);
    display.setCursor(19, 15);
    //3.3) Display static text
    display.println("APTO");
    display.display();
    delay(1000);
    //3.4) Display starting UI
    screen1();

    ////4.) LEG connection code
    //brute_force_connections();
    //connections();

    //5.) Bluetooth Setup code
	//while (!Serial); // wait for Arduino Serial Monitor (Remove once final code is available)
	Serial.println("\n\nBluetooth Connection starting.......");
	Serial.println(sizeof(USBHub), DEC);
	myusb.begin();
	delay(2000);
	rawhid1.attachReceive(OnReceiveHidData);
	rawhid2.attachReceive(OnReceiveHidData);


}

//=============================================================================
// Loop (Runs indefinitely)
//=============================================================================
void loop()
{
    //Serial.printf("Start of Loop");
    myusb.Task();

    if (Serial.available()) {
        int ch = Serial.read(); // get the first char.
        while (Serial.read() != -1);
        if ((ch == 'b') || (ch == 'B')) {
            Serial.println("Only notify on Basic Axis changes");
            joystick1.axisChangeNotifyMask(0x3ff);
        }
        else if ((ch == 'f') || (ch == 'F')) {
            Serial.println("Only notify on Full Axis changes");
            joystick1.axisChangeNotifyMask(joystick_full_notify_mask);

        }
        else {
            if (show_changed_only) {
                show_changed_only = false;
                Serial.println("\n*** Show All fields mode ***");
            }
            else {
                show_changed_only = true;
                Serial.println("\n*** Show only changed fields mode ***");
            }
        }
    }
    // check to see if the device list has changed:
    UpdateActiveDeviceInfo();
    processPS3MotionTimer();

    if (joystick1.available()) {
        if (first_joystick_message) {
            Serial.printf("*** First Joystick message %x:%x ***\n",
                joystick1.idVendor(), joystick1.idProduct());
            first_joystick_message = false;

            const uint8_t* psz = joystick1.manufacturer();
            if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
            psz = joystick1.product();
            if (psz && *psz) Serial.printf("  product: %s\n", psz);
            psz = joystick1.serialNumber();
            if (psz && *psz) Serial.printf("  Serial: %s\n", psz);

            // lets try to reduce number of fields that update
            joystick1.axisChangeNotifyMask(0xFFFFFl);
        }

        for (uint8_t i = 0; i < 64; i++) {
            psAxis[i] = joystick1.getAxis(i);
        }

        // Library works for different controllers but it will automatically detect PS3 controller once the BT dongle is plugged in
        // And will pick the PS3 case statement
        switch (joystick1.joystickType()) {
        case JoystickController::UNKNOWN:
        case JoystickController::PS3:
            PS3Commands();
            break;
        case JoystickController::PS3_MOTION:
            displayPS3MotionData();
            break;
        default:
            displayRawData();
            break;
        }

        delay(100);
        joystick1.joystickDataClear();
    }

    // See if we have some RAW data
    if (rawhid1) {
        int ch;
        uint8_t buffer[64];
        uint8_t count_chars = 0;
        memset(buffer, 0, sizeof(buffer));
        if (Serial.available()) {
            while (((ch = Serial.read()) != -1) && (count_chars < sizeof(buffer))) {
                buffer[count_chars++] = ch;
            }
            rawhid1.sendPacket(buffer);
        }
    }
}


//=============================================================================
//                 ----  START OF BT CONNECTION CODE ----
//=============================================================================

// Used connecting devices to the USB Host
//=============================================================================
// UpdateActiveDeviceInfo
//=============================================================================
void UpdateActiveDeviceInfo() {
    for (uint8_t i = 0; i < CNT_DEVICES; i++) {
        if (*drivers[i] != driver_active[i]) {
            if (driver_active[i]) {
                Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
                driver_active[i] = false;
            }
            else {
                Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
                // Need to setup lights or speaker noise to notify of connection
                PlayBuzz();
                driver_active[i] = true;

                const uint8_t* psz = drivers[i]->manufacturer();
                if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
                psz = drivers[i]->product();
                if (psz && *psz) Serial.printf("  product: %s\n", psz);
                psz = drivers[i]->serialNumber();
                if (psz && *psz) Serial.printf("  Serial: %s\n", psz);

                if (drivers[i] == &bluet) {
                    const uint8_t* bdaddr = bluet.myBDAddr();
                    // remember it...
                    Serial.printf("  BDADDR: %x:%x:%x:%x:%x:%x\n", bdaddr[0], bdaddr[1], bdaddr[2], bdaddr[3], bdaddr[4], bdaddr[5]);
                    for (uint8_t i = 0; i < 6; i++) last_bdaddr[i] = bdaddr[i];
                }
            }
        }
    }
    // Prints out information pertaining to the controller after it has connected to the USB Host via Bluetooth
    for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
        if (*hiddrivers[i] != hid_driver_active[i]) {
            if (hid_driver_active[i]) {
                Serial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
                hid_driver_active[i] = false;
            }
            else {
                Serial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
                hid_driver_active[i] = true;

                const uint8_t* psz = hiddrivers[i]->manufacturer();
                if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
                psz = hiddrivers[i]->product();
                if (psz && *psz) Serial.printf("  product: %s\n", psz);
                psz = hiddrivers[i]->serialNumber();
                if (psz && *psz) Serial.printf("  Serial: %s\n", psz);

                // See if this is our joystick object...
                if (hiddrivers[i] == &joystick1) {
                    Serial.printf("  Joystick type: %d\n", joystick1.joystickType());
                    if (joystick1.joystickType() == JoystickController::PS3_MOTION) {
                        Serial.println("  PS3 Motion detected");
                        PS3_MOTION_timer = millis();  // set time for last event
                        PS3_MOTION_tried_to_pair_state = 0;
                    }
                }

            }
        }
    }
    // Then Bluetooth devices
    // Connecting Bluetooth Dongle to the USB Host
    for (uint8_t i = 0; i < CNT_BTHIDDEVICES; i++) {
        if (*bthiddrivers[i] != bthid_driver_active[i]) {
            if (bthid_driver_active[i]) {
                Serial.printf("*** BTHID Device %s - disconnected ***\n", hid_driver_names[i]);
                bthid_driver_active[i] = false;
            }
            else {
                Serial.printf("*** BTHID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
                bthid_driver_active[i] = true;

                const uint8_t* psz = bthiddrivers[i]->manufacturer();
                if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
                psz = bthiddrivers[i]->product();
                if (psz && *psz) Serial.printf("  product: %s\n", psz);
                psz = bthiddrivers[i]->serialNumber();
                if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
            }
        }
    }
}

// This function is used
//=============================================================================
// PS3Commands
//=============================================================================
void PS3Commands()
{
    Serial.printf("In PS3Commands\n");
    buttons = joystick1.getButtons();

    // Use L3 (Left joystick button) to toggle Show Raw or not...
    if ((buttons & 0x02) && !(buttons_prev & 0x02)) show_raw_data = !show_raw_data;
    if ((buttons & 0x04) && !(buttons_prev & 0x04)) show_changed_data = !show_changed_data;

    // This is used for pairing the controller to the Bluetooth Dongle (Already Done)
    if ((buttons & 0x10000) && !(buttons_prev & 0x10000) && (buttons & 0x0C01)) {
        // PS button just pressed and select button pressed act like PS4 share like...
        // Note: you can use either R1 or L1 with the PS button, to work with Sony Move Navigation...
        Serial.print("\nPS3 Pairing Request");
        if (!last_bdaddr[0] && !last_bdaddr[1] && !last_bdaddr[2] && !last_bdaddr[3] && !last_bdaddr[4] && !last_bdaddr[5]) {
            Serial.println(" - failed - no Bluetooth adapter has been plugged in");
        }
        else if (!hiddrivers[0]) {  // Kludge see if we are connected as HID?
            Serial.println(" - failed - PS3 device not plugged into USB");
        }
        else {
            Serial.printf(" - Attempt pair to: %x:%x:%x:%x:%x:%x\n", last_bdaddr[0], last_bdaddr[1], last_bdaddr[2], last_bdaddr[3], last_bdaddr[4], last_bdaddr[5]);

            if (!joystick1.PS3Pair(last_bdaddr)) {
                Serial.println("  Pairing call Failed");
            }
            else {
                Serial.println("  Pairing complete (I hope), make sure Bluetooth adapter is plugged in and try PS3 without USB");
            }
        }
    }


    if (show_raw_data) {
        displayRawData();
    }
    else {
        // Displays the x,y coordinates of the right and left analog sticks as you move them
        Serial.printf("LX: %d, LY: %d, RX: %d, RY: %d \r\n", psAxis[0], psAxis[1], psAxis[2], psAxis[5]);
        Serial.printf("L-Trig: %d, R-Trig: %d\r\n", psAxis[3], psAxis[4]);

        drive_motorL(psAxis[1]);
        drive_motorR(psAxis[5]);


        // PS3 Controller Button to Hex Mapping

        // Case 1: Up Arrow button pressed
        if (buttons & 0x10) {
            Serial.printf("Buttons: Up\n");                                              //0x10 = Up
            Serial.println("Executing Up_Down_Shimmy()\n");
            up_down_shimmy();
        }

        // Case 2: Right Arrow button pressed
        else if (buttons & 0x20) { 
            Serial.printf("Buttons: Right\n");                                           //0x20 = Right
            PlayMiiTheme();
        }

        // Case 3: Down Arrow button pressed
        else if (buttons & 0x40) {
            Serial.printf("Buttons: Down\n");                                            //0x40 = Down
        }

        // Case 4: Left Arrow button pressed
        else if (buttons & 0x80) {
            Serial.printf("Buttons: Left\n");                                           //0x80 = Left
            Serial.println("Executing Wave_left()\n");
            wave_left();
        }

        // Case 5: Left Analog Stick moved
        else if (buttons & 0x2) {
            Serial.printf("Buttons: Left Analog Stick Click\n");                        //0x2 = Left Analog Stick click
        }

        // Case 6: Right Analog Stick moved
        else if (buttons & 0x4) {
            Serial.printf("Buttons: Right Analog Stick Click\n");                       //0x4 = Right Analog Stick Click
        }

        // Case 7: Triangle button pressed
        else if (buttons & 0x1000) {
            Serial.printf("Buttons: Triangle\n");                                       //0x1000 = Triangle
            Serial.println("Executing Stance2(), servos go to walking stance position\n");
            stance2();
        } 

        // Case 8: Circle button pressed
        else if (buttons & 0x2000) {
            Serial.printf("Buttons: Circle\n");                                         //0x2000 = Circle
            Serial.println("Executing demo();");
            demo();
        }

        // Case 9: X button pressed
        else if (buttons & 0x4000) {
            Serial.printf("Buttons: X\n");                                             //0x4000 = X
            Serial.println("Executing Stance1(), servos go to closed position\n");
            stance1();
        }

        // Case 10: Square button pressed
        else if (buttons & 0x8000) {
            Serial.printf("Buttons: Square\n");                                       //0x8000 = Square
            //screen1();
        }

        // Case 11: L1 bumper pressed
        else if (buttons & 0x400) {
            Serial.printf("Buttons: L1\n");                                             //0x400 = L1
        }

        // Case 12: R1 bumper pressed
        else if (buttons & 0x800) {
            Serial.printf("Buttons: R1\n");                                              //0x800 = R1
        }  

        // Case 13: select button pressed
        else if (buttons & 0x01) {
            Serial.printf("Buttons: select\n");                                          //0x8 = select
            Serial.printf("Toggling Blue Light...\n");
            blueLight_Trigger();
        }

        // Case 14: start button pressed
        else if (buttons & 0x08) {
            Serial.printf("Buttons: start\n");                                          //0x1 = start
            Serial.printf("Starting Servo Connections...\n");
            Serial.println("Calling brute force connections:");
            brute_force_connections();
            Serial.println("Calling final connections:");
            connections();
            Serial.println(" finished servo connections:");

        }

        else {
            Serial.printf("Buttons: %x\r\n", buttons);
        }
    }
    Serial.printf("Joystick If Statements completed\n");
    uint8_t ltv;
    uint8_t rtv;

    ltv = psAxis[3];
    rtv = psAxis[4];

    if ((ltv != joystick_left_trigger_value) || (rtv != joystick_right_trigger_value)) {
        joystick_left_trigger_value = ltv;
        joystick_right_trigger_value = rtv;
        Serial.printf("Rumbling: %d, %d\r\n", ltv, rtv);
        joystick1.setRumble(ltv, rtv);
    }

    if (buttons != buttons_prev) {
        uint8_t leds = 0;
        if (buttons & 0x8000) leds = 1;   //Srq
        if (buttons & 0x2000) leds = 2;   //Cir
        if (buttons & 0x1000) leds = 3;   //Tri
        //Cross = 2
        joystick1.setLEDs(leds);
        buttons_prev = buttons;
    }
}

// BELOW NOT USED
//=============================================================================
// displayPS3MotionData
//=============================================================================
void displayPS3MotionData()

{
    buttons = joystick1.getButtons();

    // Hard to know what is best here. for now just copy raw data over... 
    // will do this for now... Format of thought to be data.
    //  data[1-3] Buttons (mentioned 4 as well but appears to be counter
    // axis[0-1] data[5] Trigger, Previous trigger value
    // 2-5 Unknown probably place holders for Axis like data for other PS3
    // 6 - Time stamp
    // 7 - Battery
    // 8-19 - Accel: XL, XH, YL, YH, ZL, ZH, XL2, XH2, YL2, YH2, ZL2, ZH2
    // 20-31 - Gyro: Xl,Xh,Yl,Yh,Zl,Zh,Xl2,Xh2,Yl2,Yh2,Zl2,Zh2
    // 32 - Temp High
    // 33 - Temp Low (4 bits)  Maybe Magneto x High on other?? 

    // Use Select button to choose raw or not
    if ((buttons & 0x01) && !(buttons_prev & 0x01)) show_raw_data = !show_raw_data;
    if ((buttons & 0x04) && !(buttons_prev & 0x04)) show_changed_data = !show_changed_data;

    if (show_raw_data) {
        displayRawData();
    }
    else {
        uint64_t changed_mask = joystick1.axisChangedMask();
        Serial.printf("Changed: %08x Buttons: %x: Trig: %d\r\n", (uint32_t)changed_mask, buttons, psAxis[0]);
        Serial.printf("Battery Status: %d\n", psAxis[7]);
        printPS3MotionAngles();
        Serial.println();
    }

    uint8_t ltv = psAxis[0];

    if ((ltv != joystick_left_trigger_value)) {
        joystick_left_trigger_value = ltv;
        Serial.printf("Rumbling: %d\r\n", ltv);
        joystick1.setRumble(ltv, 0);
    }

    if (buttons != buttons_prev) {
        uint8_t ledsR = (buttons & 0x8000) ? 0xff : 0;   //Srq
        uint8_t ledsG = (buttons & 0x2000) ? 0xff : 0;   //Cir
        uint8_t ledsB = (buttons & 0x1000) ? 0xff : 0;   //Tri
        Serial.printf("Set Leds %x %x %x\r\n", ledsR, ledsG, ledsB);
        joystick1.setLEDs(ledsR, ledsG, ledsB);
        buttons_prev = buttons;
    }
}

//=============================================================================
// displayRawData
//=============================================================================
void displayRawData() {
    uint64_t axis_mask = joystick1.axisMask();
    uint64_t changed_mask = joystick1.axisChangedMask();

    buttons = joystick1.getButtons();

    if (!changed_mask && (buttons == buttons_prev)) return;

    if (show_changed_data) {
        if (!changed_mask) return;
        changed_mask &= 0xfffffffffL; // try reducing which ones show...
        Serial.printf("%0x - ", joystick1.getButtons());

        for (uint16_t index = 0; changed_mask; index++) {
            if (changed_mask & 1) {
                Serial.printf("%d:%02x ", index, psAxis[index]);
            }
            changed_mask >>= 1;
        }

    }
    else {
        axis_mask &= 0xffffff;
        Serial.printf("%06x%06x: %06x - ", (uint32_t)(changed_mask >> 32), (uint32_t)(changed_mask & 0xffffffff), joystick1.getButtons());

        for (uint16_t index = 0; axis_mask; index++) {
            Serial.printf("%02x ", psAxis[index]);
            axis_mask >>= 1;
        }
    }
    Serial.println();
    buttons_prev = buttons;

}

//=============================================================================
// OnReceiveHidData
//=============================================================================
bool OnReceiveHidData(uint32_t usage, const uint8_t* data, uint32_t len) {
    // Called for maybe both HIDS for rawhid basic test.  One is for the Teensy
    // to output to Serial. while still having Raw Hid...
    if (usage == 0xffc90004) {
        // Lets trim off trailing null characters.
        while ((len > 0) && (data[len - 1] == 0)) {
            len--;
        }
        if (len) {
            Serial.print("RawHid Serial: ");
            Serial.write(data, len);
        }
    }
    else {
        Serial.print("RawHID data: ");
        Serial.println(usage, HEX);
        while (len) {
            uint8_t cb = (len > 16) ? 16 : len;
            const uint8_t* p = data;
            uint8_t i;
            for (i = 0; i < cb; i++) {
                Serial.printf("%02x ", *p++);
            }
            Serial.print(": ");
            for (i = 0; i < cb; i++) {
                Serial.write(((*data >= ' ') && (*data <= '~')) ? *data : '.');
                data++;
            }
            len -= cb;
            Serial.println();
        }
    }

    return true;
}

// BELOW NOT USED
//=============================================================================
// processPS3MotionTimer
//=============================================================================
void processPS3MotionTimer() {
    // See if we have a PS3_MOTION connected and we have run for a certain amount of time

    if (PS3_MOTION_timer && ((millis() - PS3_MOTION_timer) >= PS3_MOTION_PERIOD)) {
        Serial.println("PS3 Motion Timer"); Serial.flush();
        if (joystick1) {
            PS3_MOTION_timer = millis(); // joystick not there any more...

            // We will first try to set feedback color for the PS3, maybe alternate colors
            if (++PS3_MOTION_colors_index >= sizeof(PS3_MOTION_colors) / sizeof(PS3_MOTION_colors[0])) PS3_MOTION_colors_index = 0;
            joystick1.setLEDs(PS3_MOTION_colors[PS3_MOTION_colors_index]);

            // Next see if we can try to pair.
            if (PS3_MOTION_tried_to_pair_state == 0) {
                Serial.println("PS3_MOTION Connected");
                if (!last_bdaddr[0] && !last_bdaddr[1] && !last_bdaddr[2] && !last_bdaddr[3] && !last_bdaddr[4] && !last_bdaddr[5]) {
                    Serial.println(" - No Bluetooth adapter has been plugged in - so will not try to pair");
                    PS3_MOTION_tried_to_pair_state = 1;
                }
            }
            if ((PS3_MOTION_tried_to_pair_state < 2) &&
                (last_bdaddr[0] || last_bdaddr[1] || last_bdaddr[2] || last_bdaddr[3] || last_bdaddr[4] || last_bdaddr[5])) {
                Serial.println("  - Bluetooth device detected, will try to pair");
                // Lets try to pair
                if (!joystick1.PS3Pair(last_bdaddr)) {
                    Serial.println(" - Pairing call Failed");
                }
                else {
                    Serial.println(" - Pairing complete (I hope), make sure Bluetooth adapter is plugged in and try PS3 without USB");
                }
                PS3_MOTION_tried_to_pair_state = 2; // don't try again...
            }
        }
        else {
            Serial.println("PS3 Motion Joystick no longer detected");
            PS3_MOTION_timer = 0; // joystick not there any more...
        }
    }
}

//=============================================================================
//                 ----  END OF BT CONNECTION CODE ----
//=============================================================================

//=============================================================================
//                 ----  START OF TANK DRIVE FUNCTIONS ----
//=============================================================================
void drive_motorL(int Lstick_Val) {
    // While Left Trigger > 0 drive left motor forward

    if ( 120 <= Lstick_Val <= 134) {
        Serial.printf(" Braking Left Motor (set speed to 0) (analog value = %d)\n", Lstick_Val);
        motors.setM1Speed(0);
    }
    else if ( 134 < Lstick_Val <= 255) {
        Serial.printf(" Max Reverse Left Motor (set speed to -300) (analog value = %d)\n", Lstick_Val);
        motors.setM1Speed(-300);
    }

    else if (120 < Lstick_Val <= 0) {
        Serial.printf(" Max Forward Left Motor (set speed to 400) (analog value = %d)\n", Lstick_Val);
        motors.setM1Speed(400);
    }
}

void drive_motorR(int Rstick_Val) {
    // while Right Trigger > 0 drive right motor forward
    if (120 <= Rstick_Val <= 134) {
        Serial.printf(" Braking Right Motor (set speed to 0) (analog value = %d)\n", Rstick_Val);
        motors.setM1Speed(0);
    }
    else if (134 < Rstick_Val <= 255) {
        Serial.printf(" Max Reverse Right Motor (set speed to -350) (analog value = %d)\n", Rstick_Val);
        motors.setM1Speed(-300);
    }

    else if (120 < Rstick_Val <= 0) {
        Serial.printf(" Max Forward Right Motor (set speed to 400) (analog value = %d)\n", Rstick_Val);
        motors.setM1Speed(400);
    }
}
//=============================================================================
//                 ----  END OF TANK DRIVE FUNCTIONS ----
//=============================================================================

//=============================================================================
//                 ----  START OF NEOPIXEL FUNCTIONS ----
//=============================================================================
void blueLight_Trigger() {
    uint16_t i;

    if (toggle == 0) {
        for (i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(0, 0, 128));
        }
        strip.show();
        toggle = 1;
    }

    else if (toggle == 1) {
        for (i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(0, 0, 0));
        }
        strip.show();
        toggle = 0;
    }

}

//=============================================================================
//                 ---- END OF NEOPIXEL FUNCTIONS ----
//=============================================================================

//=============================================================================
//                 ---- START OF SPEAKER FUNCTIONS ----
//=============================================================================
void PlayMiiTheme() {
    // iterate over the notes of the melody. 
    // Remember, the array is twice the number of notes (notes + durations)
    for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

        // calculates the duration of each note
        divider = melody[thisNote + 1];
        if (divider > 0) {
            // regular note, just proceed
            noteDuration = (wholenote) / divider;
        }
        else if (divider < 0) {
            // dotted notes are represented with negative durations!!
            noteDuration = (wholenote) / abs(divider);
            noteDuration *= 1.5; // increases the duration in half for dotted notes
        }

        // we only play the note for 90% of the duration, leaving 10% as a pause
        tone(buzzer, melody[thisNote], noteDuration * 0.9);

        // Wait for the specief duration before playing the next note.
        delay(noteDuration);

        // stop the waveform generation before the next note.
        noTone(buzzer);
    }
 }

void PlayBuzz() {
        tone(buzzer, NOTE_F2, 5000);
}

void PlayStartTheme() {

}
//=============================================================================
//                 ----  START OF SPEAKER FUNCTIONS ----
//=============================================================================

//=============================================================================
//                 ----  START OF OLED (128x64) FUNCTIONS ----
//=============================================================================
void BT_connected_OLED() {

}

// Normal exploration dirivng mode with low hazard level
void screen1() {

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    // Setting the cursor to the top left corner of the display
    display.setCursor(0, 2);
    // Printing out APTO in top left corner

    display.print("APTO");
    // Displaying Wifi, Bluetooth and Battery icons at the top of the display
    display.drawBitmap(24, 2, wifi1_icon16x16, 16, 16, 1);
    display.drawBitmap(40, 2, bluetooth_icon16x16, 16, 16, 1);
    display.drawBitmap(112, 2, bat2_icon16x16, 16, 16, 1);

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 20);
    display.println("Mode: Driving");
    display.println("Terrain Level: Flat");
    display.println("Hazard Level: Low");

    // Displaying the Apto Website at the bottom of display
    display.setCursor(0, 50);
    display.print("aptorobot.github.io");
    display.display();
    display.startscrollright(0x06, 0x07);
    delay(5000);

}

// Hazard Level is High, the screen will blink multiple times indicating high Hazard
void screen2() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    // Setting the cursor to the top left corner of the display
    display.setCursor(0, 2);
    // Printing out APTO in top left corner
    display.print("APTO");
    // Displaying Wifi, Bluetooth and Battery icons at the top of the display
    display.drawBitmap(24, 2, wifi1_icon16x16, 16, 16, 1);
    display.drawBitmap(40, 2, bluetooth_icon16x16, 16, 16, 1);
    display.drawBitmap(112, 2, bat2_icon16x16, 16, 16, 1);

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 20);
    display.println("Mode: Walking");
    display.println("Terrain Level: Rocky");
    display.setTextColor(BLACK, WHITE);
    display.println("Hazard Level: High");

    // Displaying the Apto Website at the bottom of display
    display.setCursor(0, 45);
    display.setTextColor(WHITE);
    display.print("aptorobot.github.io");
    display.display();
    display.startscrollright(0x05, 0x07);
    delay(2000);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    // Setting the cursor to the top left corner of the display
    display.setCursor(0, 2);
    // Printing out APTO in top left corner
    display.print("APTO");
    // Displaying Wifi, Bluetooth and Battery icons at the top of the display
    display.drawBitmap(24, 2, wifi1_icon16x16, 16, 16, 1);
    display.drawBitmap(40, 2, bluetooth_icon16x16, 16, 16, 1);
    display.drawBitmap(112, 2, bat2_icon16x16, 16, 16, 1);

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(10, 15);
    display.println("Mode: Walking");
    display.println("Terrain Level: Rocky");
    display.setTextColor(BLACK, WHITE);
    display.println("Hazard Level: High");

    // Displaying the Apto Website at the bottom of display
    display.setCursor(0, 60);
    display.setTextColor(WHITE);
    display.print("aptorobot.github.io");
    display.display();
    display.startscrollright(0x07, 0x07);
    delay(5000);
}

//=============================================================================
//                 ---- END OF OLED FUNCTIONS ----
//=============================================================================

//=============================================================================
//                 ----  START OF LEG FUNCTIONS ----
//=============================================================================
void brute_force_connections() {
    legServo1a.attach(2);
    legServo1b.attach(3);
    legServo1c.attach(4);

    legServo2a.attach(10);
    legServo2b.attach(11);
    legServo2c.attach(12);

    legServo3a.attach(33);
    legServo3b.attach(36);
    legServo3c.attach(37);

    legServo4a.attach(13);
    legServo4b.attach(14);
    legServo4c.attach(15);
    //legServo4c_test.motor.attach(15);

    legServo1a.write(180);
    legServo1b.write(0);
    legServo1c.write(180);

    legServo2a.write(0);
    legServo2b.write(180);
    legServo2c.write(0);

    legServo3a.write(180);
    legServo3b.write(0);
    legServo3c.write(180);

    legServo4a.write(0);
    legServo4b.write(180);
    legServo4c.write(0);
    //legServo4c_test.motor.write(0);
}

void brute_force_connections2() {
    legServo1a.attach(2);
    legServo1b.attach(3);
    legServo1c.attach(4);

    legServo2a.attach(10);
    legServo2b.attach(11);
    legServo2c.attach(12);

    legServo3a.attach(33);
    legServo3b.attach(36);
    legServo3c.attach(37);

    legServo4a.attach(13);
    legServo4b.attach(14);
    legServo4c.attach(15);
    //legServo4c_test.motor.attach(15);

}

void connections() {
    for (int i = 0; i < 4; i++) {
        //delcare servo type as either Horizontal, Vertical, or Sensor
        legs[i].number = i;
        legs[i].move = false;
        legs[i].phi = 0;
        legs[i].z = 0;

        for (int j = 0; j < 3; j++) {
            legs[i].joint[j].number = i * j;
            legs[i].joint[j].type = j;
        }
        Serial.println(legs[i].number);
        Serial.println(legs[i].move);

    }

    legs[0].joint[0].motor = legServo1a;
    legs[0].joint[1].motor = legServo1b;
    legs[0].joint[2].motor = legServo1c;

    legs[1].joint[0].motor = legServo2a;
    legs[1].joint[1].motor = legServo2b;
    legs[1].joint[2].motor = legServo2c;

    legs[2].joint[0].motor = legServo3a;
    legs[2].joint[1].motor = legServo3b;
    legs[2].joint[2].motor = legServo3c;

    legs[3].joint[0].motor = legServo4a;
    legs[3].joint[1].motor = legServo4b;
    legs[3].joint[2].motor = legServo4c;

}

void stance1() {

    legServo1a.write(180);
    legServo1b.write(0);
    legServo1c.write(180);

    legServo2a.write(0);
    legServo2b.write(180);
    legServo2c.write(0);

    legServo3a.write(180);
    legServo3b.write(0);
    legServo3c.write(180);

    legServo4a.write(0);
    legServo4b.write(180);
    legServo4c.write(0);
}

void stance2() {
    Serial.printf("start of stance2() \n");
    legServo1a.write(150);
    legServo1b.write(170);
    legServo1c.write(180 - 150);

    legServo2a.write(30);
    legServo2b.write(180 - 170);
    legServo2c.write(150);

    legServo3a.write(150);
    legServo3b.write(170);
    legServo3c.write(180 - 150);

    legServo4a.write(30);
    legServo4b.write(180 - 170);
    legServo4c.write(150);
    Serial.printf("end of stance2() \n");

    //Serial.println(sizeof(USBHub), DEC);
    //myusb.begin();
    //delay(2000);
    //rawhid1.attachReceive(OnReceiveHidData);
    //rawhid2.attachReceive(OnReceiveHidData);


}

void stance65() {

    legServo1a.write(115);
    legServo1b.write(170);
    legServo1c.write(180 - 130);

    legServo2a.write(65);
    legServo2b.write(180 - 170);
    legServo2c.write(130);

    legServo3a.write(115);
    legServo3b.write(170);
    legServo3c.write(180 - 130);

    legServo4a.write(65);
    legServo4b.write(180 - 170);
    legServo4c.write(130);
}

void stance3() {

    legServo1a.write(115);
    legServo1b.write(180);
    legServo1c.write(0);

    legServo2a.write(65);
    legServo2b.write(0);
    legServo2c.write(180);

    legServo3a.write(115);
    legServo3b.write(180);
    legServo3c.write(0);

    legServo4a.write(65);
    legServo4b.write(0);
    legServo4c.write(180);
}

void stance1_test() {
    legs[0].joint[0].motor.write(180);
    legs[0].joint[1].motor.write(0);
    legs[0].joint[2].motor.write(180);

    legs[1].joint[0].motor.write(0);
    legs[1].joint[1].motor.write(180);
    legs[1].joint[2].motor.write(0);

    legs[2].joint[0].motor.write(180);
    legs[2].joint[1].motor.write(0);
    legs[2].joint[2].motor.write(180);

    legs[3].joint[0].motor.write(0);
    legs[3].joint[1].motor.write(180);
    legs[3].joint[2].motor.write(0);

}

void wave_left() {
    legs[0].joint[0].motor.write(140);
    legs[0].joint[1].motor.write(160);
    legs[0].joint[2].motor.write(180 - 150);

    legs[1].joint[0].motor.write(40);
    legs[1].joint[1].motor.write(180 - 160);
    legs[1].joint[2].motor.write(150);

    legs[2].joint[0].motor.write(140);
    legs[2].joint[1].motor.write(20);
    legs[2].joint[2].motor.write(180 - 150);

    legs[3].joint[0].motor.write(0);
    legs[3].joint[1].motor.write(180 - 160);
    legs[3].joint[2].motor.write(150);

    //delay(2000);

    for (pos = 0; pos < 180; pos += 5) { // goes from 0 degrees to 180 degrees, 1 degree steps
        legs[2].joint[2].motor.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 1; pos -= 5) {   // goes from 180 degrees to 0 degrees
        legs[2].joint[2].motor.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }

}

void wave_right() {
    legs[0].joint[0].motor.write(180);
    legs[0].joint[1].motor.write(160);
    legs[0].joint[2].motor.write(180 - 150);

    legs[1].joint[0].motor.write(40);
    legs[1].joint[1].motor.write(180 - 20);
    legs[1].joint[2].motor.write(150);

    legs[2].joint[0].motor.write(140);
    legs[2].joint[1].motor.write(160);
    legs[2].joint[2].motor.write(180 - 150);

    legs[3].joint[0].motor.write(40);
    legs[3].joint[1].motor.write(180 - 160);
    legs[3].joint[2].motor.write(150);

    //delay(2000);

    for (pos = 0; pos < 180; pos += 5) { // goes from 0 degrees to 180 degrees, 1 degree steps
        legs[1].joint[2].motor.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 1; pos -= 5) {   // goes from 180 degrees to 0 degrees
        legs[1].joint[2].motor.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }

}

void simple_walk() {
    // Walking Gait 

    for (int i = 0; i < 10; i++) {
        // pull legs 0 and 1 inward to each other, while pulling legs 2 and 3 outward 
        legs[0].joint[0].motor.write(180);
        legs[0].joint[1].motor.write(170);
        legs[0].joint[2].motor.write(180 - 150);

        legs[1].joint[0].motor.write(0);
        legs[1].joint[1].motor.write(180 - 170);
        legs[1].joint[2].motor.write(150);

        legs[2].joint[0].motor.write(180 - 35);
        legs[2].joint[1].motor.write(170);
        legs[2].joint[2].motor.write(180 - 150);

        legs[3].joint[0].motor.write(35);
        legs[3].joint[1].motor.write(180 - 170);
        legs[3].joint[2].motor.write(150);

        //for (pos = 0; pos < 60; pos += 5) { // goes from 0 degrees to 180 degrees, 1 degree steps
        //    legs[1].joint[0].motor.write(pos);              // tell servo to go to position in variable 'pos'
        //    delay(15);                       // waits 15ms for the servo to reach the position
        //}
        delay(1000);
        legs[1].joint[0].motor.write(55);

        // leg[0].joint[shoulder], leg[2].joint[shoulder], leg[3].joint[shoulder] all swerve backwards, moving the robot forwards
        delay(1000);
        legs[0].joint[0].motor.write(180 - 55);
        legs[2].joint[0].motor.write(180 - 0);
        legs[3].joint[0].motor.write(45);

        // leg[2] and leg[3] are pulled inward to each other.
        delay(1000);
        legs[2].joint[0].motor.write(180);
        legs[3].joint[0].motor.write(0);

        delay(1000);
        legs[2].joint[0].motor.write(55);

        delay(1000);
        legs[0].joint[0].motor.write(180 - 55);
        legs[1].joint[0].motor.write(0);
        legs[3].joint[0].motor.write(55);
    }
}

void simple_stretch_down() {
    // Walking Gait
    int temp0 = legs[0].joint[1].motor.read();
    int temp1 = legs[1].joint[1].motor.read();
    int temp2 = legs[2].joint[1].motor.read();
    int temp3 = legs[3].joint[1].motor.read();

    int temp_Claw0 = legs[0].joint[2].motor.read();
    int temp_Claw1 = legs[1].joint[2].motor.read();
    int temp_Claw2 = legs[2].joint[2].motor.read();
    int temp_Claw3 = legs[3].joint[2].motor.read();

    int increment = 1;

    while (temp0 != ARM0_MIN || temp2 != ARM2_MIN) {
        temp0--;
        temp1++;
        temp2--;
        temp3++;
        legs[0].joint[1].motor.write(temp0);
        legs[1].joint[1].motor.write(temp1);
        legs[2].joint[1].motor.write(temp2);
        legs[3].joint[1].motor.write(temp3);

        temp_Claw0++;
        temp_Claw1--;
        temp_Claw2++;
        temp_Claw3--;
        legs[0].joint[2].motor.write(temp_Claw0);
        legs[1].joint[2].motor.write(temp_Claw1);
        legs[2].joint[2].motor.write(temp_Claw2);
        legs[3].joint[2].motor.write(temp_Claw3);

        delay(10);
    }

}

void simple_stretch_up() {
    // Walking Gait
    int temp0 = legs[0].joint[1].motor.read();
    int temp1 = legs[1].joint[1].motor.read();
    int temp2 = legs[2].joint[1].motor.read();
    int temp3 = legs[3].joint[1].motor.read();

    int temp_Claw0 = legs[0].joint[2].motor.read();
    int temp_Claw1 = legs[1].joint[2].motor.read();
    int temp_Claw2 = legs[2].joint[2].motor.read();
    int temp_Claw3 = legs[3].joint[2].motor.read();

    int increment = 1;

    while (temp0 != ARM0_MAX || temp2 != ARM2_MAX) {
        temp0++;
        temp1--;
        temp2++;
        temp3--;
        legs[0].joint[1].motor.write(temp0);
        legs[1].joint[1].motor.write(temp1);
        legs[2].joint[1].motor.write(temp2);
        legs[3].joint[1].motor.write(temp3);

        temp_Claw0--;
        temp_Claw1++;
        temp_Claw2--;
        temp_Claw3++;
        legs[0].joint[2].motor.write(temp_Claw0);
        legs[1].joint[2].motor.write(temp_Claw1);
        legs[2].joint[2].motor.write(temp_Claw2);
        legs[3].joint[2].motor.write(temp_Claw3);

        delay(10);
    }


}

void up_down_shimmy() {
    //going down.....
    Serial.printf("start of up_down_shimmy() \n");
    int temp0 = legs[0].joint[1].motor.read();
    int temp1 = legs[1].joint[1].motor.read();
    int temp2 = legs[2].joint[1].motor.read();
    int temp3 = legs[3].joint[1].motor.read();

    int temp_Claw0 = legs[0].joint[2].motor.read();
    int temp_Claw1 = legs[1].joint[2].motor.read();
    int temp_Claw2 = legs[2].joint[2].motor.read();
    int temp_Claw3 = legs[3].joint[2].motor.read();

    int increment = 1;
    for (int i = 0; i < 4; i++) {
        while (temp0 != 70 || temp2 != 70) {
            temp0--;
            temp1++;
            temp2--;
            temp3++;
            legs[0].joint[1].motor.write(temp0);
            legs[1].joint[1].motor.write(temp1);
            legs[2].joint[1].motor.write(temp2);
            legs[3].joint[1].motor.write(temp3);

            temp_Claw0++;
            temp_Claw1--;
            temp_Claw2++;
            temp_Claw3--;
            legs[0].joint[2].motor.write(temp_Claw0);
            legs[1].joint[2].motor.write(temp_Claw1);
            legs[2].joint[2].motor.write(temp_Claw2);
            legs[3].joint[2].motor.write(temp_Claw3);

            delay(10);
        }

        while (temp0 != ARM0_MAX || temp2 != ARM2_MAX) {
            temp0++;
            temp1--;
            temp2++;
            temp3--;
            legs[0].joint[1].motor.write(temp0);
            legs[1].joint[1].motor.write(temp1);
            legs[2].joint[1].motor.write(temp2);
            legs[3].joint[1].motor.write(temp3);

            temp_Claw0--;
            temp_Claw1++;
            temp_Claw2--;
            temp_Claw3++;
            legs[0].joint[2].motor.write(temp_Claw0);
            legs[1].joint[2].motor.write(temp_Claw1);
            legs[2].joint[2].motor.write(temp_Claw2);
            legs[3].joint[2].motor.write(temp_Claw3);

            delay(10);
        }
    }
    Serial.printf("end of up_down_shimmy() \n");


}

void side_shimmy() {
    // get into stance where all shoulders are extended to max 65 degrees

    int temp_Shlr0 = legs[0].joint[0].motor.read();
    int temp_Shlr1 = legs[1].joint[0].motor.read();
    int temp_Shlr2 = legs[2].joint[0].motor.read();
    int temp_Shlr3 = legs[3].joint[0].motor.read();

    int temp_Arm0 = legs[0].joint[1].motor.read();
    int temp_Arm1 = legs[1].joint[1].motor.read();
    int temp_Arm2 = legs[2].joint[1].motor.read();
    int temp_Arm3 = legs[3].joint[1].motor.read();

    int temp_Claw0 = legs[0].joint[2].motor.read();
    int temp_Claw1 = legs[1].joint[2].motor.read();
    int temp_Claw2 = legs[2].joint[2].motor.read();
    int temp_Claw3 = legs[3].joint[2].motor.read();
    //1. turn leg1 and leg3 to their mins
    while (temp_Shlr1 != SHLR1_MIN || temp_Shlr3 != SHLR3_MIN) {
        temp_Shlr1--;
        temp_Shlr3--;
        legs[1].joint[0].motor.write(temp_Shlr1);
        legs[3].joint[0].motor.write(temp_Shlr3);
        delay(5);
    }

    //2. turn leg0 and leg2 to their mins and legs1 and legs3 to their max
    while (temp_Shlr0 != SHLR0_MIN || temp_Shlr2 != SHLR2_MIN) {
        temp_Shlr0++;
        temp_Shlr2++;
        legs[0].joint[0].motor.write(temp_Shlr0);
        legs[2].joint[0].motor.write(temp_Shlr2);

        temp_Shlr1++;
        temp_Shlr3++;
        legs[1].joint[0].motor.write(temp_Shlr1);
        legs[3].joint[0].motor.write(temp_Shlr3);

        delay(5);
    }

    //2. turn leg1 and leg3 to their mins and legs0 and legs2 to their max
    while (temp_Shlr1 != SHLR1_MIN || temp_Shlr3 != SHLR3_MIN) {
        temp_Shlr1--;
        temp_Shlr3--;
        legs[1].joint[0].motor.write(temp_Shlr1);
        legs[3].joint[0].motor.write(temp_Shlr3);

        temp_Shlr0--;
        temp_Shlr2--;
        legs[0].joint[0].motor.write(temp_Shlr0);
        legs[2].joint[0].motor.write(temp_Shlr2);
        delay(5);
    }

}

void turn_left() {
    int temp_Shlr0 = legs[0].joint[0].motor.read();
    int temp_Shlr1 = legs[1].joint[0].motor.read();
    int temp_Shlr2 = legs[2].joint[0].motor.read();
    int temp_Shlr3 = legs[3].joint[0].motor.read();

    int temp_Arm0 = legs[0].joint[1].motor.read();
    int temp_Arm1 = legs[1].joint[1].motor.read();
    int temp_Arm2 = legs[2].joint[1].motor.read();
    int temp_Arm3 = legs[3].joint[1].motor.read();

    int temp_Claw0 = legs[0].joint[2].motor.read();
    int temp_Claw1 = legs[1].joint[2].motor.read();
    int temp_Claw2 = legs[2].joint[2].motor.read();
    int temp_Claw3 = legs[3].joint[2].motor.read();

    //2. turn leg1 and leg3 to their mins and legs0 and legs2 to their max
    while (temp_Shlr1 != SHLR1_MIN || temp_Shlr3 != SHLR3_MIN) {
        temp_Shlr1--;
        temp_Shlr3--;
        legs[1].joint[0].motor.write(temp_Shlr1);
        legs[3].joint[0].motor.write(temp_Shlr3);

        temp_Shlr0--;
        temp_Shlr2--;
        legs[0].joint[0].motor.write(temp_Shlr0);
        legs[2].joint[0].motor.write(temp_Shlr2);
        delay(5);
    }

    while (temp_Shlr2 != SHLR2_MIN) {
        temp_Shlr2++;
        temp_Arm2--;
        temp_Claw2++;

        legs[2].joint[0].motor.write(temp_Shlr2);
        legs[2].joint[1].motor.write(temp_Arm2);
        legs[2].joint[2].motor.write(temp_Claw2);

        delay(5);
    }

}

void demo() {
    //brute_force_connections();
    brute_force_connections2();
    connections();

    //stance1();
    stance2();
    delay(3000);
    up_down_shimmy();

    delay(3000);
    stance2();
    wave_left();

    //delay(1500);
    //stance2();
    //stance1();

}


//=============================================================================
//                 ---- END OF LEG FUNCTIONS ----
//=============================================================================