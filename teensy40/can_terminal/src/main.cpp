#include <Arduino.h>
#include <FlexCAN_T4.h>

#define TEENSY_NODE_ID  0x03
#define MESC_NODE_ID    0x0B
#define CAN_ID_TERMINAL 0x01   // adjust if different in MESC task_can.c

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

#define RX_BUF_SIZE 128
char rx_buf[RX_BUF_SIZE];
int rx_len = 0;

// --- Helper: pack MESC CAN ID (simple version: message_id | (sender<<8) | (receiver<<16)) ---
uint32_t CANhelper_packMESC_id(uint8_t message_id, uint8_t sender, uint8_t receiver) {
    return ((uint32_t)receiver << 16) | ((uint32_t)sender << 8) | (uint32_t)message_id;
}

// --- Helper: unpack MESC CAN ID ---
void CANhelper_unpackMESC_id(uint32_t id, uint8_t *message_id, uint8_t *sender, uint8_t *receiver) {
    *message_id = id & 0xFF;
    *sender     = (id >> 8) & 0xFF;
    *receiver   = (id >> 16) & 0xFF;
}

// --- Send terminal command to MESC ---
void send_terminal_command(const char *cmd) {
    size_t len = strlen(cmd);
    size_t sent = 0;

    while (sent < len) {
        CAN_message_t msg;
        msg.id  = CANhelper_packMESC_id(CAN_ID_TERMINAL, TEENSY_NODE_ID, MESC_NODE_ID);
        msg.len = min((size_t)8, len - sent);
        memcpy(msg.buf, cmd + sent, msg.len);
        Can0.write(msg);
        sent += msg.len;
    }
}

// --- CAN receive callback ---
void can_sniff(const CAN_message_t &msg) {
    uint8_t message_id, sender, receiver;
    CANhelper_unpackMESC_id(msg.id, &message_id, &sender, &receiver);

    if (message_id == CAN_ID_TERMINAL && sender == MESC_NODE_ID && receiver == TEENSY_NODE_ID) {
        for (int i = 0; i < msg.len; i++) {
            char c = (char)msg.buf[i];
	    Serial.print(c);
            if (rx_len < RX_BUF_SIZE - 1) {
                rx_buf[rx_len++] = c;
            }
            if (c == '\n') {
                rx_buf[rx_len] = '\0';
                Serial.print("MESC> ");
                Serial.print(rx_buf);
                rx_len = 0;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) ;  // wait for terminal

    Serial.println("Teensy CLI-to-MESC bridge ready. Type commands:");

    Can0.begin();
    Can0.setBaudRate(500000);  // typical CAN bus rate
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(can_sniff);
}

void loop() {
    static char cmd_buf[64];
    static int cmd_len = 0;

    // Read user input from serial terminal
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len++] = '\n';   // ensure newline
                cmd_buf[cmd_len] = '\0';
                send_terminal_command(cmd_buf);
                cmd_len = 0;
                Serial.print("Sent: ");
                Serial.print(cmd_buf);
            }
        } else {
            if (cmd_len < (int)sizeof(cmd_buf) - 2) {
                cmd_buf[cmd_len++] = c;
            }
        }
    }

    Can0.events(); // process CAN events
}
