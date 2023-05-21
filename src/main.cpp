/*
  Please add MCP_CAN_LIB to your library first........
  MCP_CAN_LIB file in M5stack lib examples -> modules -> COMMU ->
  MCP_CAN_lib.rar
*/

#include <M5Stack.h>
#include <mcp_can.h>
#include "m5_logo.h"

#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN CAN0(12);    // Set CS to pin 10

void init_can();
void test_can();

void setup() {
    M5.begin();
    M5.Power.begin();
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);

    M5.Lcd.pushImage(0, 0, 320, 240, (uint16_t *)gImage_logoM5);
    delay(500);
    M5.Lcd.setTextColor(BLACK);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(0xFFFF,0x0000);
    M5.Lcd.setCursor(10, 10);
    delay(500);

    init_can();
}

void loop() {
    test_can();
    M5.update();
}

void init_can() {
    unsigned char read_len;
    unsigned char read_buf[16];

    // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the
    // masks and filters disabled.
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
        Serial.println("MCP2515 Initialized Successfully!");
    else
        Serial.println("Error Initializing MCP2515...");

    CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515
    // sends acks to received data.

    pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input


    unsigned char registor_stmp[8] = {0x00, 0x00, 0, 0, 0, 0, 0, 0};
    CAN0.sendMsgBuf(0x109, 0, 8, registor_stmp);
    delay(10);

    unsigned char mode_stmp[8] = {0x02, 0x00, 0, 0, 0, 0, 0, 0};
    CAN0.sendMsgBuf(0x105, 0, 8, mode_stmp);
    delay(10);

    unsigned char feedback_stmp[8] = {0x80, 0x80, 0, 0, 0, 0, 0, 0};
    CAN0.sendMsgBuf(0x106, 0, 8, feedback_stmp);
    delay(10);
}

void test_can() {
    unsigned char test_stmp[8] = {(unsigned char) 0, 0, 0, 0, 0, 0, 0, 0};
    static int16_t velo = 0;
    static bool acc_flag = true;

    if(acc_flag)
        velo ++;
    else
        velo --;

    if(velo > 126) {
        acc_flag = false;
    } else if(velo < -127) {
        acc_flag = true;
    }


    test_stmp[0] = (int8_t)(velo >> 8);
    test_stmp[1] = (int8_t)(velo & 0x00FF);
    Serial.printf("%d\r\n", velo);

    M5.Lcd.clear();
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("%x, %x", test_stmp[0], test_stmp[1]);
    CAN0.sendMsgBuf(0x32, 0, 8, test_stmp);
    delay(100);
}