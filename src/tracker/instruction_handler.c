#include "tracker/instruction_handler.h"

#include "common/dkvr_core.h"

#include "led/led_control.h"
#include "network/dkvr_client.h"
#include "tracker/behavior.h"
#include "tracker/calibration.h"
#include "tracker/statistic.h"
#include "tracker/status.h"


static uint8_t pearson_hash(uint8_t len, const uint8_t* ptr);

void tracker_instruction_handler(uint8_t opcode, union dkvr_byte_pack* payload)
{
    uint8_t hash;
    switch (opcode)
    {
    // miscellaneous
    case DKVR_OPCODE_LOCATE:
        dkvr_led_interrupt_for_locate();
        break;

    case DKVR_OPCODE_CLIENT_NAME:
        dkvr_client_send_instruction(DKVR_OPCODE_CLIENT_NAME, sizeof(DKVR_CLIENT_NAME), 1, DKVR_CLIENT_NAME);
        break;

    // configuration
    // all configuration instruction requires ACK response and 
    // those values are defined in tracker behavior sepecification
    case DKVR_OPCODE_BEHAVIOR:
        // BEHAVIOR is 1 byte instruction so just echo-back
        tracker_behavior_set(payload[0].uchar[0]);
        dkvr_client_send_instruction(DKVR_OPCODE_BEHAVIOR, 1, 1, &payload[0].uchar[0]); // ACK
        break;

    case DKVR_OPCODE_GYR_TRANSFORM:
        // all calibration inst are multi-byte, give back a hash of payload
        tracker_calib_set_gyr_transform((const float*)payload);
        hash = pearson_hash(DKVR_TRANSFORM_MATRIX_SIZE, (const uint8_t*)payload);
        dkvr_client_send_instruction(DKVR_OPCODE_GYR_TRANSFORM, 1, 1, &hash); // ACK
        break;

    case DKVR_OPCODE_ACC_TRANSFORM:
        tracker_calib_set_acc_transform((const float*)payload);
        hash = pearson_hash(DKVR_TRANSFORM_MATRIX_SIZE, (const uint8_t*)payload);
        dkvr_client_send_instruction(DKVR_OPCODE_ACC_TRANSFORM, 1, 1, &hash); // ACK
        break;

    case DKVR_OPCODE_MAG_TRANSFORM:
        tracker_calib_set_mag_transform((const float*)payload);
        hash = pearson_hash(DKVR_TRANSFORM_MATRIX_SIZE, (const uint8_t*)payload);
        dkvr_client_send_instruction(DKVR_OPCODE_MAG_TRANSFORM, 1, 1, &hash); // ACK
        break;

    case DKVR_OPCODE_NOISE_VARIANCE:
        tracker_calib_set_noise_variance((const float*)payload);
        hash = pearson_hash(DKVR_NOISE_VARIANCE_SIZE, (const uint8_t*)payload);
        dkvr_client_send_instruction(DKVR_OPCODE_NOISE_VARIANCE, 1, 1, &hash); // ACK
        break;


    // data transfer
    case DKVR_OPCODE_STATUS:
        dkvr_client_send_instruction(DKVR_OPCODE_STATUS,
                                     sizeof(struct tracker_status),
                                     1,
                                     tracker_status_get_struct_ptr());
        break;

    case DKVR_OPCODE_STATISTIC:
        dkvr_client_send_instruction(DKVR_OPCODE_STATISTIC,
                                     sizeof(struct tracker_statistic),
                                     1,
                                     tracker_statistic_get_struct_ptr());
        break;

    // network opcodes are already handled and never gonna happen
    //      DKVR_OPCODE_NETWORKING
    //      DKVR_OPCODE_HANDSHAKE1
    //      DKVR_OPCODE_HANDSHAKE2
    //      DKVR_OPCODE_HEARTBEAT
    //      DKVR_OPCODE_PING
    //      DKVR_OPCODE_PONG
    // not host-side opcode
    //      DKVR_OPCODE_RAW
    //      DKVR_OPCODE_ORIENTATION
    default:
        break;
    }
}

static uint8_t pearson_hash(uint8_t len, const uint8_t *ptr)
{
    static const uint8_t hash_table[256] PROGMEM = {
        251, 175, 119, 215, 81, 14, 79, 191, 103, 49, 181, 143, 186, 157, 0,
        232, 31, 32, 55, 60, 152, 58, 17, 237, 174, 70, 160, 144, 220, 90, 57,
        223, 59, 3, 18, 140, 111, 166, 203, 196, 134, 243, 124, 95, 222, 179,
        197, 65, 180, 48, 36, 15, 107, 46, 233, 130, 165, 30, 123, 161, 209, 23,
        97, 16, 40, 91, 219, 61, 100, 10, 210, 109, 250, 127, 22, 138, 29, 108,
        244, 67, 207, 9, 178, 204, 74, 98, 126, 249, 167, 116, 34, 77, 193,
        200, 121, 5, 20, 113, 71, 35, 128, 13, 182, 94, 25, 226, 227, 199, 75,
        27, 41, 245, 230, 224, 43, 225, 177, 26, 155, 150, 212, 142, 218, 115,
        241, 73, 88, 105, 39, 114, 62, 255, 192, 201, 145, 214, 168, 158, 221,
        148, 154, 122, 12, 84, 82, 163, 44, 139, 228, 236, 205, 242, 217, 11,
        187, 146, 159, 64, 86, 239, 195, 42, 106, 198, 118, 112, 184, 172, 87,
        2, 173, 117, 176, 229, 247, 253, 137, 185, 99, 164, 102, 147, 45, 66,
        231, 52, 141, 211, 194, 206, 246, 238, 56, 110, 78, 248, 63, 240, 189,
        93, 92, 51, 53, 183, 19, 171, 72, 50, 33, 104, 101, 69, 8, 252, 83, 120,
        76, 135, 85, 54, 202, 125, 188, 213, 96, 235, 136, 208, 162, 129, 190,
        132, 156, 38, 47, 1, 7, 254, 24, 4, 216, 131, 89, 21, 28, 133, 37, 153,
        149, 80, 170, 68, 6, 169, 234, 151};

    uint8_t hash = len;
    for (int i = len; i > 0;)
    {
        uint8_t offset = hash ^ ptr[--i];
        hash = pgm_read_byte(hash_table + offset);
    }
    return hash;
}
