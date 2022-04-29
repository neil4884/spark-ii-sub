// SMART TIMER CIRCUIT BOARD SOFTWARE V2
// Vivatsathorn Thitasirivit
// SPARK-II (Software version 0.0.1)
// SUB CONTROLLER (dev_id = 1)

// Imports
#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_ICM20948.h"
#include "utility/imumaths.h"

// Define constants
#define SERIAL_BAUD 115200

// Define pins
#define P_X A7
#define P_Y A8
#define P_Z A9

#define P_SD_CS 10
#define P_SD_MOSI 11
#define P_SD_MISO 12
#define P_SD_SCK 13

#define P_RX_TEENSY_MAIN 1
#define P_TX_TEENSY_MAIN 0

// Global objects, constants, variables
const float SEALEVELPRESSURE_HPA = 1013.25;
const float M_G = 9.81;

// Define functionality interval in ms
// RATE_IMU Default: 10
const u_int16_t RATE_IMU = 10;

// RATE_DATALOG Default: 100
const u_int16_t RATE_DATALOG = 100;

// RATE_DATACOM Default: 500
const u_int16_t RATE_DATACOM = 500;

// FIRE_HOLD Default: 2000
const u_int16_t FIRE_HOLD = 2000;

// Define launch acceleration threshold in g (Default: 4g)
const u_int16_t LAUNCH_THRESHOLD = 4;
const u_int16_t ACC_SCALE = 64;

// Define apogee in m, s
const u_int16_t APOGEE_HEIGHT = 900;
const float APOGEE_TIME = 13;
const float APOGEE_THRESHOLD = 0.95;

const String PK_HEADER = "SPARK2,";
const String F_NAME = "S2_SUB_";
const String F_EXT = ".csv";

// Define device id
const u_int8_t device_id = 1;

Adafruit_BNO055 bno055 = Adafruit_BNO055(-1, 0x28);
Adafruit_ICM20948 icm;

// os_state: 0 is default operation, 1 is wait for sd read, 2 is wait for user input
// 3 is sd read, 255 is end - restart to reset
u_int8_t os_state = 0;
u_int8_t dfu_state = 0;
u_int8_t trigger_launch = 0;
u_int8_t trigger_deployment_1 = 0;
u_int8_t trigger_deployment_2 = 0;
u_int32_t t_capture;
u_int32_t dt;
u_int32_t counter = 0;
u_int32_t last_millis_imu = 0;
u_int32_t last_millis_log = 0;
u_int32_t last_millis_com = 0;

float dir_X1 = 0, dir_Y1 = 0, dir_Z1 = 0;
float acc_X1 = 0, acc_Y1 = 0, acc_Z1 = 0;
float vel_X1 = 0, vel_Y1 = 0, vel_Z1 = 0;
float pos_X1 = 0, pos_Y1 = 0, pos_Z1 = 0;
float gyr_X1 = 0, gyr_Y1 = 0, gyr_Z1 = 0;

float acc_X2 = 0, acc_Y2 = 0, acc_Z2 = 0;
float vel_X2 = 0, vel_Y2 = 0, vel_Z2 = 0;
float pos_X2 = 0, pos_Y2 = 0, pos_Z2 = 0;
float gyr_X2 = 0, gyr_Y2 = 0, gyr_Z2 = 0;

float acc_X3 = 0, acc_Y3 = 0, acc_Z3 = 0;
float vel_X3 = 0, vel_Y3 = 0, vel_Z3 = 0;
float pos_X3 = 0, pos_Y3 = 0, pos_Z3 = 0;

char file_name[100];
char read_name[100];

String user_inp;
String p_log1 = "";
String p_log2 = "";
String p_log3 = "";
String p_log4 = "";
String p_com = "";

File root;
File sd_file;
File read_file;

// Functions
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

float accl(int raw_K);

float integrate(float inp_f, u_int32_t inp_dt);

String comma(const String &inp);

void printDirectory(File dir, uint8_t num_tabs);

void setup() {
    // PIN
    pinMode(P_X, INPUT);
    pinMode(P_Y, INPUT);
    pinMode(P_Z, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // SERIAL
    Serial.begin(SERIAL_BAUD);
    Serial1.begin(SERIAL_BAUD);

    // I2C
    Wire.begin();
    Wire.setClock(400000);

    // SPI
    SPI.setMISO(P_SD_MISO);
    SPI.setMOSI(P_SD_MOSI);
    SPI.setSCK(P_SD_SCK);
    SPI.setCS(P_SD_CS);

    // SD
    SD.begin(P_SD_CS);
    int sd_file_idx = 0;
    while (true) {
        String file_name_str = (F_NAME + String(sd_file_idx) + F_EXT);
        file_name_str.toCharArray(file_name, 100);
        sd_file_idx++;
        if (!SD.exists(file_name)) break;
    }

    // BNO055
    bno055.begin();
    bno055.setExtCrystalUse(true);

    // ICM20948
    icm.begin_I2C();
    icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
    icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    icm.setMagDataRate(AK09916_MAG_DATARATE_20_HZ);
    icm.setAccelRateDivisor(4095);
    icm.setGyroRateDivisor(255);

    delay(1000);
}

void loop() {
    if (Serial.available() > 0) {
        user_inp = Serial.readString();
        if (os_state == 2) {
            if (user_inp == "END_READ") {
                os_state = 255;
            } else {
                os_state = 3;
            }
        } else if (os_state == 254) {
            if (dfu_state != 0) {
                if (user_inp == "SYS_CMD_REQUEST_DEVICE_ID") {
                    dfu_state = 2;
                } else if (user_inp == "READ_EEPROM") {
                    dfu_state = 4;
                } else if (user_inp == "WRITE_EEPROM") {
                    dfu_state = 6;
                } else if (user_inp == "CLEAR_EEPROM") {
                    dfu_state = 8;
                } else if (user_inp == "EXIT_DFU") {
                    os_state = 0;
                    dfu_state = 0;
                }
            }

        } else if (user_inp == "START_OP") {
            os_state = 0;
        } else if (user_inp == "READ_SD") {
            os_state = 1;
        } else if (user_inp == "ENTER_DFU") {
            os_state = 254;
            dfu_state = 1;
        } else if (user_inp == "END_OP") {
            os_state = 255;
        }

    }

    // operational mode
    if (os_state == 0) {
        digitalWrite(LED_BUILTIN, HIGH);
        t_capture = millis();
        if (t_capture - last_millis_imu >= RATE_IMU) {
            dt = t_capture - last_millis_imu;

            sensors_event_t eventAcc1, eventGyr1, eventDir1;
            bno055.getEvent(&eventAcc1, Adafruit_BNO055::VECTOR_LINEARACCEL);
            bno055.getEvent(&eventGyr1, Adafruit_BNO055::VECTOR_GYROSCOPE);
            bno055.getEvent(&eventDir1, Adafruit_BNO055::VECTOR_EULER);

            dir_X1 = eventDir1.orientation.x;
            dir_Y1 = eventDir1.orientation.y;
            dir_Z1 = eventDir1.orientation.z;

            gyr_X1 = eventGyr1.gyro.x;
            gyr_Y1 = eventGyr1.gyro.y;
            gyr_Z1 = eventGyr1.gyro.z;

            acc_X1 = eventAcc1.acceleration.x;
            acc_Y1 = eventAcc1.acceleration.y;
            acc_Z1 = eventAcc1.acceleration.z;

            vel_X1 += integrate(acc_X1, dt);
            vel_Y1 += integrate(acc_Y1, dt);
            vel_Z1 += integrate(acc_Z1, dt);

            pos_X1 += integrate(vel_X1, dt);
            pos_Y1 += integrate(vel_Y1, dt);
            pos_Z1 += integrate(vel_Z1, dt);

            sensors_event_t eventAcc2, eventGyr2, eventTmp2, eventMag2;
            icm.getEvent(&eventAcc2, &eventGyr2, &eventTmp2, &eventMag2);

            gyr_X2 = eventGyr2.gyro.x;
            gyr_Y2 = eventGyr2.gyro.y;
            gyr_Z2 = eventGyr2.gyro.z;

            acc_X2 = eventAcc2.acceleration.x;
            acc_Y2 = eventAcc2.acceleration.y;
            acc_Z2 = eventAcc2.acceleration.z;

            vel_X2 += integrate(acc_X2, dt);
            vel_Y2 += integrate(acc_Y2, dt);
            vel_Z2 += integrate(acc_Z2, dt);

            pos_X2 += integrate(vel_X2, dt);
            pos_Y2 += integrate(vel_Y2, dt);
            pos_Z2 += integrate(vel_Z2, dt);

            acc_X3 = accl(analogRead(P_X));
            acc_Y3 = accl(analogRead(P_Y));
            acc_Z3 = accl(analogRead(P_Z));

            vel_X3 += integrate(acc_X3, dt);
            vel_Y3 += integrate(acc_Y3, dt);
            vel_Z3 += integrate(acc_Z3, dt);

            pos_X3 += integrate(vel_X3, dt);
            pos_Y3 += integrate(vel_Y3, dt);
            pos_Z3 += integrate(vel_Z3, dt);

            uint8_t t_sys, t_gyro, t_acc, t_mag = 0;
            bno055.getCalibration(&t_sys, &t_gyro, &t_acc, &t_mag);
            last_millis_imu = millis();
        }

        t_capture = millis();
        if (t_capture - last_millis_com >= RATE_DATACOM) {
            p_com = PK_HEADER;
            p_com += comma(String(counter));

            last_millis_com = millis();
        }

        t_capture = millis();
        if (t_capture - last_millis_log >= RATE_DATALOG) {
            p_log1 = PK_HEADER;
            p_log1 += comma(String(counter));
            p_log1 += comma(String(acc_X1, 2));
            p_log1 += comma(String(acc_Y1, 2));
            p_log1 += comma(String(acc_Z1, 2));
            p_log1 += comma(String(vel_X1, 2));
            p_log1 += comma(String(vel_Y1, 2));
            p_log1 += comma(String(vel_Z1, 2));
            p_log1 += comma(String(pos_X1, 2));
            p_log1 += comma(String(pos_Y1, 2));
            p_log1 += comma(String(pos_Z1, 2));

            p_log2 = "";
            p_log2 += comma(String(acc_X2, 2));
            p_log2 += comma(String(acc_Y2, 2));
            p_log2 += comma(String(acc_Z2, 2));
            p_log2 += comma(String(vel_X2, 2));
            p_log2 += comma(String(vel_Y2, 2));
            p_log2 += comma(String(vel_Z2, 2));
            p_log2 += comma(String(pos_X2, 2));
            p_log2 += comma(String(pos_Y2, 2));
            p_log2 += comma(String(pos_Z2, 2));

            p_log3 = "";
            p_log3 += comma(String(acc_X3, 2));
            p_log3 += comma(String(acc_Y3, 2));
            p_log3 += comma(String(acc_Z3, 2));
            p_log3 += comma(String(vel_X3, 2));
            p_log3 += comma(String(vel_Y3, 2));
            p_log3 += comma(String(vel_Z3, 2));
            p_log3 += comma(String(pos_X3, 2));
            p_log3 += comma(String(pos_Y3, 2));
            p_log3 += comma(String(pos_Z3, 2));

            p_log4 = "";
            p_log4 += comma(String(gyr_X1, 2));
            p_log4 += comma(String(gyr_Y1, 2));
            p_log4 += comma(String(gyr_Z1, 2));
            p_log4 += comma(String(gyr_X2, 2));
            p_log4 += comma(String(gyr_Y2, 2));
            p_log4 += comma(String(gyr_Z2, 2));
            p_log4 += comma(String(dir_X1, 2));
            p_log4 += comma(String(dir_Y1, 2));
            p_log4 += comma(String(dir_Z1, 2));

            Serial.println(p_log1);
            Serial.println(p_log2);
            Serial.println(p_log3);
            Serial.println(p_log4);

            sd_file = SD.open(file_name, FILE_WRITE);
            if (sd_file) {
                sd_file.print(p_log1);
                sd_file.print(p_log2);
                sd_file.print(p_log3);
                sd_file.println(p_log4);
                sd_file.close();
            }

            counter++;
            last_millis_log = millis();
        }
        digitalWrite(LED_BUILTIN, LOW);
    }

        // sd read wait mode
    else if (os_state == 1) {
        root = SD.open("/");
        Serial.println(F("-------------"));
        printDirectory(root, 0);
        Serial.println(F("-------------"));
        os_state = 2;
    }

        // sd read inp
    else if (os_state == 3) {
        if (read_file) {
            while (read_file.available()) {
                char inp_c_file = read_file.read();
                Serial.print(inp_c_file);
            }
            Serial.println();
            read_file.close();
        }
        os_state = 2;
    }

        // dfu
    else if (os_state == 254) {
        if (dfu_state == 2) {
            Serial.print("SYS_RESPOND_DEVICE_ID_");
            Serial.println(device_id);

        } else if (dfu_state == 4) {
            while (Serial.available() <= 0);
            user_inp = Serial.readString();
            if (user_inp.length() > 0) {
                int arg_read_addr = user_inp.toInt();
                int eeprom_read = EEPROM.read(arg_read_addr);
                Serial.println(eeprom_read);
            }

        } else if (dfu_state == 6) {
            while (Serial.available() <= 0);
            user_inp = Serial.readString();
            if (user_inp.length() > 0) {
                int arg_write_addr = user_inp.toInt();
                while (Serial.available() <= 0);
                user_inp = Serial.readString();
                if (user_inp.length() > 0) {
                    int arg_write_val = user_inp.toInt();
                    EEPROM.update(arg_write_addr, arg_write_val);
                }
            }

        } else if (dfu_state == 8) {
            // TEENSY 4.0 CLEAR 0 to 1079
            for (int idx_eepr_clear = 0; idx_eepr_clear < 1080; idx_eepr_clear++) {
                EEPROM.update(idx_eepr_clear, 0);
            }
        }
        dfu_state = 0;
    }
}

String comma(const String &inp) {
    return (inp + ",");
}

float accl(int raw_K) {
    float scaled_K;
    scaled_K = mapf((float) raw_K, 0, 1023, -ACC_SCALE, ACC_SCALE);
    return (M_G * scaled_K);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float integrate(float inp_f, u_int32_t inp_dt) {
    return inp_f * (float) inp_dt / 1000;
}

void printDirectory(File dir, uint8_t num_tabs) {
    File entry;
    while (true) {
        entry = dir.openNextFile();
        if (!entry) break;
        for (uint8_t i = 0; i < num_tabs; i++) {
            Serial.print("\t");
        }
        Serial.print(entry.name());
        if (entry.isDirectory()) {
            Serial.println("/");
            printDirectory(entry, num_tabs + 1);
        } else {
            Serial.print("\t\t");
            Serial.println(entry.size(), DEC);
        }
        entry.close();
    }
}
