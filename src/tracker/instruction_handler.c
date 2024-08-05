#include "tracker/instruction_handler.h"

#include "common/dkvr_core.h"

#include "led/led_control.h"
#include "network/dkvr_client.h"
#include "tracker/behavior.h"
#include "tracker/calibration.h"
#include "tracker/statistic.h"
#include "tracker/status.h"

void tracker_instruction_handler(uint8_t opcode, union dkvr_byte_pack* payload)
{
    switch (opcode)
    {
    // miscellaneous
    case DKVR_OPCODE_LOCATE:
        dkvr_led_interrupt_for_locate();
        break;

    case DKVR_OPCODE_CLIENTNAME:
        dkvr_client_send_instruction(DKVR_OPCODE_CLIENTNAME, sizeof(DKVR_CLIENT_NAME), 1, DKVR_CLIENT_NAME);
        break;

    // configuration
    // all configuration instruction requires ACK response
    case DKVR_OPCODE_BEHAVIOR:
        tracker_behavior_set(payload[0].uchar[0]);
        dkvr_client_send_instruction(DKVR_OPCODE_BEHAVIOR, 0, 0, NULL); // ACK
        break;

    case DKVR_OPCODE_GYR_TRANSFORM:
        tracker_calibration_set_gyr_transform((const float*)payload);
        dkvr_client_send_instruction(DKVR_OPCODE_GYR_TRANSFORM, 0, 0, NULL); // ACK
        break;

    case DKVR_OPCODE_ACC_TRANSFORM:
        tracker_calibration_set_acc_transform((const float*)payload);
        dkvr_client_send_instruction(DKVR_OPCODE_ACC_TRANSFORM, 0, 0, NULL); // ACK
        break;

    case DKVR_OPCODE_MAG_TRANSFORM:
        tracker_calibration_set_mag_transform((const float*)payload);
        dkvr_client_send_instruction(DKVR_OPCODE_MAG_TRANSFORM, 0, 0, NULL); // ACK
        break;

    case DKVR_OPCODE_NOISE_VARIANCE:
        tracker_calibration_set_noise_variance((const float*)payload);
        dkvr_client_send_instruction(DKVR_OPCODE_NOISE_VARIANCE, 0, 0, NULL); // ACK
        break;

    case DKVR_OPCODE_MAG_REFERENCE:
        tracker_calibration_set_mag_reference((const float*)payload);
        dkvr_client_send_instruction(DKVR_OPCODE_MAG_REFERENCE, 0, 0, NULL); // ACK
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