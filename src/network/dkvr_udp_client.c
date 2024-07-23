#include "network/dkvr_udp_client.h"

#include <string.h>

#include "common/dkvr_core.h"
#include "common/system_interface.h"
#include "common/udp_interface.h"
#include "common/wifi_interface.h"

typedef struct instruction_s
{
    uint8_t header;
    uint8_t length;
    uint8_t align;
    uint8_t opcode;
    uint32_t sequence;
    byte_pack_t payload[13];
} instruction_t;

static client_status_t client_status = CLIENT_DISCONNECTED;
static dispatcher_callback instruction_dispatcher = NULL;
static instruction_t* instruction = (instruction_t*)&udp_buffer.buffer;
static uint32_t recv_seq = 0;
static uint32_t send_seq = 0;
static uint32_t last_handshake_sent = 0;
static uint32_t last_heartbeat_sent = 0;
static uint32_t last_heartbeat_recv = 0;

static void reset_netstat();
static dkvr_err update_wifi_and_udp();
static dkvr_err internal_update_client_connection();

static int peek_client_recv();
static void dispatch_instruction();
static void do_bit_conversion_if_required();
static void load_inst(uint8_t opcode, uint8_t len, uint8_t align);
static void send_inst();

static void client_send_handshake1();
static void client_send_heartbeat();
static void client_send_pong();

client_status_t get_client_status()
{
    return client_status;
}

dkvr_err init_dkvr_client(dispatcher_callback callback)
{
    dkvr_udp_setup(DKVR_HOST_IP, DKVR_HOST_PORT, DKVR_NET_UDP_SERVER_PORT);
    reset_netstat();
    instruction_dispatcher = callback;
    return DKVR_OK;
}

dkvr_err update_client_connection()
{
    dkvr_err result = update_wifi_and_udp();
    if (result != DKVR_OK)
    {
        client_status = CLIENT_DISCONNECTED;
        return result;
    }

    result = internal_update_client_connection();
    if (result != DKVR_OK)
        return result;

    return DKVR_OK;
}

dkvr_err dispatch_received_inst()
{
    if (client_status == CLIENT_CONNECTED)
    {
        while (peek_client_recv())
            dispatch_instruction();
    }

    return DKVR_OK;
}

void send_client_inst(uint8_t opcode, uint8_t len, uint8_t align, const void* payload)
{
    load_inst(opcode, len, align);
    if (len && payload)
        memcpy(instruction->payload, payload, len);
    send_inst();
}

static void reset_netstat()
{
    recv_seq = 0;
    send_seq = 0;
    last_handshake_sent = 0;
    last_heartbeat_sent = 0;
    last_heartbeat_recv = 0;
}

static dkvr_err update_wifi_and_udp()
{
    if (is_wifi_status_changed())
    {
        stop_udp_server();
    }

    if (get_wifi_status() == WIFI_STATUS_DISCONNECTED)
    {
        dkvr_err result = dkvr_wifi_connect(DKVR_WIFI_SSID, DKVR_WIFI_PASSWORD);
        if (result != DKVR_OK)
            return result;
    }

    if (get_udp_server_status() == UDP_SERVER_STATUS_CLOSED)
    {
        dkvr_err result = begin_udp_server();
        if (result != DKVR_OK)
            return result;
    }
    return DKVR_OK;
}

static dkvr_err internal_update_client_connection()
{
    static int try_count = 0;
    uint32_t now = dkvr_get_time();

    switch (client_status)
    {
    case CLIENT_DISCONNECTED:
    {
        // try handshake
        try_count = 1;
        client_send_handshake1();
        client_status = CLIENT_HANDSHAKED;
        last_handshake_sent = now;
        break;
    }

    case CLIENT_HANDSHAKED:
    {
        // check timeout
        if (now > last_handshake_sent + DKVR_NET_RESPONSE_TIMEOUT)
        {
            if (try_count < DKVR_NET_RETRY_LIMIT)
            {
                try_count++;
                client_send_handshake1();
                last_handshake_sent = now;
            }
            else
            {
                client_status = CLIENT_DISCONNECTED;
                return DKVR_ERR_HOST_NO_RESPONSE;
            }
        }

        // check response
        if (peek_client_recv())
        {
            if (instruction->opcode == DKVR_OPCODE_HANDSHAKE2)
            {
                client_status = CLIENT_CONNECTED;
                client_send_heartbeat();
                last_heartbeat_sent = now;
                last_heartbeat_recv = now;
            }
            else
            {
                client_status = CLIENT_DISCONNECTED;
                return DKVR_ERR_WRONG_RESPONSE;
            }
        }
        break;
    }

    case CLIENT_CONNECTED:
    {
        // send heartbeat
        if (now > last_heartbeat_sent + DKVR_NET_HEARTBEAT_INTERVAL)
        {
            client_send_heartbeat();
            last_heartbeat_sent = now;
        }

        // recv heartbeat timeout
        if (now > last_heartbeat_recv + DKVR_NET_HEARTBEAT_TIMEOUT)
        {
            reset_netstat();
            client_status = CLIENT_DISCONNECTED;
            return DKVR_ERR_HEARTBEAT_TIMEOUT;
        }
        break;
    }
    }
    return DKVR_OK;
}

static int peek_client_recv()
{
    int len = peek_udp_recv();
    if (len < DKVR_NET_DGRAM_MIN_LEN)
        return 0;
    if (instruction->header != DKVR_NET_HEADER_VALUE)
        return 0;
    if (instruction->sequence <= recv_seq)
        return 0;

    recv_seq = instruction->sequence;
    do_bit_conversion_if_required();
    return 1;
}

static void dispatch_instruction()
{
    uint8_t opcode = instruction->opcode;

    if (DKVR_IS_NET_OPCODE(opcode)) // intercept network opcode
    {
        switch (opcode)
        {
        case DKVR_OPCODE_HEARTBEAT:
            last_heartbeat_recv = dkvr_get_time();
            break;

        case DKVR_OPCODE_PING:
            client_send_pong();
            break;

        default:
            break;
        }
    }
    else
    {
        instruction_dispatcher(opcode, instruction->payload);
    }
}

static void do_bit_conversion_if_required()
{
#if defined(__BYTE_ORDER__)&&(__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
    switch (instruction->align)
    {
    case 0:
    case 1:
    default:
        break;
    
    case 2:
    {
        uint8_t* ptr = (uint8_t*)&instruction->payload;
        for (int i = 0; i < instruction->length; i += 2) {
            uint8_t temp = ptr[i];
            ptr[i] = ptr[i + 1];
            ptr[i + 1] = temp;
        }
        break;
    }

    case 4:
    {
        uint8_t* ptr = (uint8_t*)&instruction->payload;
        for (int i = 0; i < instruction->length; i += 4) {
            uint8_t temp = ptr[i];
            ptr[i] = ptr[i + 3];
            ptr[i + 3] = temp;
            temp = ptr[i + 1];
            ptr[i + 1] = ptr[i + 2];
            ptr[i + 2] = temp;
        }
        break;
    }
    }
#endif
}

static void load_inst(uint8_t opcode, uint8_t len, uint8_t align)
{
    instruction->header = DKVR_NET_HEADER_VALUE;
    instruction->length = len;
    instruction->align = align;
    instruction->opcode = opcode;
}

static void send_inst()
{
    instruction->sequence = send_seq++;
    do_bit_conversion_if_required();
    send_udp(instruction, DKVR_NET_DGRAM_MIN_LEN + instruction->length);
}

static void client_send_handshake1()
{
    load_inst(DKVR_OPCODE_HANDSHAKE1, 0, 0);
    send_inst();
}

static void client_send_heartbeat()
{
    load_inst(DKVR_OPCODE_HEARTBEAT, 0, 0);
    send_inst();
}

static void client_send_pong()
{
    load_inst(DKVR_OPCODE_PONG, 0, 0);
    send_inst();
}