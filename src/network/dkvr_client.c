#include "network/dkvr_client.h"

#include <string.h>

#include "common/dkvr_core.h"
#include "common/system_interface.h"
#include "common/udp_interface.h"
#include "common/wifi_interface.h"

typedef enum
{
    DKVR_CLIENT_DISCONNECTED,
    DKVR_CLIENT_HANDSHAKED,
    DKVR_CLIENT_CONNECTED
} dkvr_client_status;

struct instruction
{
    uint8_t header;
    uint8_t length;
    uint8_t align;
    uint8_t opcode;
    uint32_t sequence;
    union dkvr_byte_pack payload[13];
};

struct network_statistics
{
    uint32_t recv_seq;
    uint32_t send_seq;
    uint32_t last_handshake_sent;
    uint32_t last_heartbeat_sent;
    uint32_t last_heartbeat_recv;
};

static dkvr_client_status client_status = DKVR_CLIENT_DISCONNECTED;
static dkvr_instruction_handler instruction_handler = NULL;
static struct instruction *instruction = (struct instruction*) &dkvr_udp_buffer;
static struct network_statistics netstat = {0};
static int retry_count = 0;

static dkvr_err uphold_wifi_and_udp();
static dkvr_err uphold_client_connection();
static dkvr_err on_client_disconnected();
static dkvr_err on_client_handshaked();
static dkvr_err on_client_connected();
static void reset_netstat();
static int peek_client_recv();
static void handle_received_instruction();
static void do_bit_conversion_if_required();


int dkvr_client_is_connected()
{
    return client_status == DKVR_CLIENT_CONNECTED;
}

dkvr_err dkvr_client_init(dkvr_instruction_handler callback)
{
    instruction_handler = callback;
    return DKVR_OK;
}

dkvr_err dkvr_client_update_connection()
{
    // updates the DKVR Client connection regardless of network disconnection
    // because on situation under 5 seconds connection lost, client can still stay
    // connected and get rid of undesired reconnecting action.
    dkvr_err network_result = uphold_wifi_and_udp();
    dkvr_err client_result = uphold_client_connection();

    // report network error first
    if (network_result) return network_result;
    if (client_result)  return client_result;
    
    return DKVR_OK;
}

dkvr_err dkvr_client_dispatch_received()
{
    while (dkvr_udp_peek_recv())
        handle_received_instruction();
    
    return DKVR_OK;
}

dkvr_err dkvr_client_send_instruction(uint8_t opcode, uint8_t len, uint8_t align, const void* payload)
{
    instruction->header = DKVR_NET_OPENER_VALUE;
    instruction->length = len;
    instruction->align = align;
    instruction->opcode = opcode;
    instruction->sequence = netstat.send_seq++;

    if (len && payload)
        memcpy(instruction->payload, payload, len);

    do_bit_conversion_if_required();
    dkvr_udp_send_dgram(DKVR_HOST_IP, DKVR_HOST_PORT, instruction, DKVR_NET_HEADER_LEN + instruction->length);

    return DKVR_OK;
}

static dkvr_err uphold_wifi_and_udp()
{
    // check WiFi connection
    dkvr_wifi_update_conection();
    if (!dkvr_wifi_is_connected())
    {
        dkvr_udp_server_stop();
        dkvr_err result = dkvr_wifi_connect(DKVR_WIFI_SSID, DKVR_WIFI_PASSWORD);
        if (result) 
            return result;
    }

    // check UDP server
    if (!dkvr_udp_is_server_up())
    {
        dkvr_err result = dkvr_udp_server_begin(DKVR_NET_UDP_SERVER_PORT);
        if (result)
            return result;
    }

    return DKVR_OK;
}

static dkvr_err uphold_client_connection()
{
    switch (client_status)
    {
    default:
    case DKVR_CLIENT_DISCONNECTED:
        return on_client_disconnected();

    case DKVR_CLIENT_HANDSHAKED:
        return on_client_handshaked();

    case DKVR_CLIENT_CONNECTED:
        return on_client_connected();
    }
}

static dkvr_err on_client_disconnected()
{
    // do nothing on network disconnection
    if (!dkvr_wifi_is_connected() || !dkvr_udp_is_server_up())
        return DKVR_OK;

    // try connect to host
    retry_count = 1;
    client_status = DKVR_CLIENT_HANDSHAKED;
    netstat.last_handshake_sent = dkvr_get_time();
    dkvr_client_send_instruction(DKVR_OPCODE_HANDSHAKE1, 0, 0, NULL);
    return DKVR_OK;
}

static dkvr_err on_client_handshaked()
{
    uint32_t now = dkvr_get_time();

    // handshake response timeout
    if (now > netstat.last_handshake_sent + DKVR_NET_RESPONSE_TIMEOUT)
    {
        // retry
        if (retry_count < DKVR_NET_RETRY_LIMIT)
        {
            retry_count++;
            netstat.last_handshake_sent = now;
            dkvr_client_send_instruction(DKVR_OPCODE_HANDSHAKE1, 0, 0, NULL);
            return DKVR_OK;
        }
        else
        {
            // failed
            client_status = DKVR_CLIENT_DISCONNECTED;
            return DKVR_ERR_HOST_NO_RESPONSE;
        }
    }

    // check handshake2 response
    if (peek_client_recv())
    {
        if (instruction->opcode == DKVR_OPCODE_HANDSHAKE2)
        {
            client_status = DKVR_CLIENT_CONNECTED;
            netstat.last_heartbeat_sent = now;
            netstat.last_heartbeat_recv = now;
            dkvr_client_send_instruction(DKVR_OPCODE_HEARTBEAT, 0, 0, NULL); // ACK of handshake2

            // send clinet name
            dkvr_client_send_instruction(DKVR_OPCODE_CLIENT_NAME, sizeof(DKVR_CLIENT_NAME), 1, DKVR_CLIENT_NAME);
        }
    }

    return DKVR_OK;
}

static dkvr_err on_client_connected()
{
    uint32_t now = dkvr_get_time();

    // send heartbeat
    if (now > netstat.last_heartbeat_sent + DKVR_NET_HEARTBEAT_INTERVAL)
    {
        netstat.last_heartbeat_sent = now;
        dkvr_client_send_instruction(DKVR_OPCODE_HEARTBEAT, 0, 0, NULL);
    }

    // recv heartbeat timed-out
    if (now > netstat.last_heartbeat_recv + DKVR_NET_HEARTBEAT_TIMEOUT)
    {
        reset_netstat();
        client_status = DKVR_CLIENT_DISCONNECTED;
        return DKVR_ERR_HEARTBEAT_TIMEOUT;
    }

    return DKVR_OK;
}

static void reset_netstat()
{
    memset(&netstat, 0, sizeof(struct network_statistics));
}

static int peek_client_recv()
{
    // validate format
    int len = dkvr_udp_peek_recv();
    if (len < DKVR_NET_HEADER_LEN)
        return 0;
    if (instruction->header != DKVR_NET_OPENER_VALUE)
        return 0;
    if (instruction->sequence <= netstat.recv_seq)
        return 0;

    // accept inst
    netstat.recv_seq = instruction->sequence;
    do_bit_conversion_if_required();
    return 1;
}

static void handle_received_instruction()
{
    uint8_t opcode = instruction->opcode;

    // intercept networking opcode
    if ((opcode & DKVR_OPCODE_CLASS_MASK) == DKVR_OPCODE_NETWORKING)
    {
        switch (opcode)
        {
        case DKVR_OPCODE_HEARTBEAT:
            netstat.last_heartbeat_recv = dkvr_get_time();
            break;

        case DKVR_OPCODE_PING:
            dkvr_client_send_instruction(DKVR_OPCODE_PONG, 0, 0, NULL);
            break;

        // ignore other notworking opcode
        default: 
            break;
        }
    }
    else
    {
        // delegate to handler
        instruction_handler(opcode, instruction->payload);
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
