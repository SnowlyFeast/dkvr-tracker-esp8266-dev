#include "tracker/tracker_main.h"

#include "common/dkvr_core.h"
#include "common/esp8266_util.h"
#include "common/system_interface.h"

#include "filter/ekf.h"
#include "filter/eskf_sensor_fusion.h"

#include "imu/imu_control.h"

#include "network/dkvr_udp_client.h"

#include "led/led_control.h"

#include "tracker/error_logger.h"
#include "tracker/tracker_config.h"
#include "tracker/tracker_statistic.h"
#include "tracker/tracker_status.h"

// #define DKVR_EKF

// result assertion macro
#define ERR_LOG_FUNC            log_dkvr_error
#define LOG_IF_ERR(result)       \
    do                           \
    {                            \
        dkvr_err res = result; \
        if (res != DKVR_OK)      \
            ERR_LOG_FUNC(res);   \
    } while (0);

volatile int gpio_interrupt = 0;

static quaternion_t imu_quat = {};

static void instruction_dispatcher(uint8_t opcode, byte_pack_t* payload);
static void handle_gpio_interrupt();
static void send_imu_data();
static void send_client_inst_status();
static void send_status_on_error();
static void tracker_update_led();

void init_tracker()
{
    dkvr_err net_init_result = init_dkvr_client(instruction_dispatcher);
    dkvr_err imu_init_result = init_imu();

    if (net_init_result != DKVR_OK)
    {
        // if the implementation is correct, no init errors will occur under any circumstances
        PRINTLN("DKVR udp client init failed : 0x", (net_init_result));
        return;
    }

    if (imu_init_result != DKVR_OK)
    {
        PRINTLN("IMU init failed: 0x", (imu_init_result));
        LOG_IF_ERR(imu_init_result);
    }
    set_tracker_init_result(imu_init_result);

    // etc
    turn_off_led();
    turn_on_led();
    reset_tracker_config();
    update_tracker_status();

    // wait for imu result and init filter
    while (1)
    {
        dkvr_delay(50);
        if (handle_interrupt()) continue;
        if (!is_imu_data_ready()) continue;
        if (update_imu_readings()) continue;

        // TODO: implement NSD getter for IMU
#ifdef DKVR_EKF
        ekf_set_nsd(7.6e-7f, 1.6e-5f, 4.0e-6f);
        // ekf_set_nsd(0.05f, 0.004f, 0.002f);
        ekf_init(&imu_raw.accel_out, &imu_raw.mag_out);
#else
        struct eskf_vector3 accel, mag;
        memcpy(accel.data, &imu_raw.accel_out, sizeof(accel));
        memcpy(mag.data, &imu_raw.mag_out, sizeof(mag));
        
        // eskf_configure(DKVR_IMU_SAMPLING_PERIOD, 0.5f, 0.5f, 2.74e-7f, 4.0e-6f, 4.0e-6f, 2.5e-5f, 2.0e-5f);
        // eskf_configure(DKVR_IMU_SAMPLING_PERIOD, 0.5f, 0.5f, 7.6e-7f, 1.6e-5f, 4.0e-6f, 3.8e-5f, 2.0e-5f);
        eskf_init(accel, mag);
#endif
        break;
    }
}

void update_tracker()
{
    handle_gpio_interrupt();
    
    LOG_IF_ERR(update_client_connection());
    switch (get_client_status())
    {
        case CLIENT_DISCONNECTED:
        {
            PRINTLN("Host connection failed. Retry after 3 secs.");
            turn_on_led();          // indication of waiting
            dkvr_delay(3000);
            break;
        }

        case CLIENT_HANDSHAKED:
            tracker_update_led();
            return;

        case CLIENT_CONNECTED:
            dispatch_received_inst();
            send_imu_data();
            send_status_on_error();
            break;
    }
    update_tracker_status();
    tracker_update_led();

#ifdef DKVR_STATISTIC_ENABLE
    record_execution_time();
#endif
}

static void instruction_dispatcher(uint8_t opcode, byte_pack_t* payload)
{
    switch (opcode)
    {
    case DKVR_OPCODE_LOCATE:
        interrupt_led_for_locate();
        break;

    case DKVR_OPCODE_ACTIVE:
        set_behavior_active();
        send_client_inst(DKVR_OPCODE_BEHAVIOR, 0, 0, NULL); // ack
        break;

    case DKVR_OPCODE_INACTIVE:
        set_behavior_inactive();
        send_client_inst(DKVR_OPCODE_BEHAVIOR, 0, 0, NULL); // ack
        break;

    case DKVR_OPCODE_BEHAVIOR:
        set_behavior(payload[0].uchar[0]);
        if (get_behavior_led())
        {
            set_led_mode(LED_MODE_NORMAL);
        }
        else
        {
            set_led_mode(LED_MODE_MANUAL);
            turn_off_led();
        }
        send_client_inst(DKVR_OPCODE_BEHAVIOR, 0, 0, NULL); // ack
        break;

    case DKVR_OPCODE_CALIBRATION_GR:
        set_gyro_offset((const float*)payload);
        send_client_inst(DKVR_OPCODE_CALIBRATION_GR, 0, 0, NULL); // ack
        break;

    case DKVR_OPCODE_CALIBRATION_AC:
        set_accel_calib_mat((const float*)payload);
        send_client_inst(DKVR_OPCODE_CALIBRATION_AC, 0, 0, NULL); // ack
        break;

    case DKVR_OPCODE_CALIBRATION_MG:
        set_mag_calib_mat((const float*)payload);
        send_client_inst(DKVR_OPCODE_CALIBRATION_MG, 0, 0, NULL); // ack
        break;

    case DKVR_OPCODE_MAG_REF_RECALC:
#ifdef DKVR_EKF
        ekf_init(&imu_raw.accel_out, &imu_raw.mag_out);
#else
        {
            struct eskf_vector3 accel, mag;
            memcpy(accel.data, &imu_raw.accel_out, sizeof(accel));
            memcpy(mag.data, &imu_raw.mag_out, sizeof(mag));
            eskf_init(accel, mag);
        }
#endif
        break;

    case DKVR_OPCODE_STATUS:
        send_client_inst_status();
        break;

    case DKVR_OPCODE_STATISTIC:
    {
        tracker_statistic_t statistic = get_tracker_statistic();
        send_client_inst(DKVR_OPCODE_STATISTIC, sizeof(tracker_statistic_t), 1, &statistic);
        break;
    }
        
    // non-host side opcode
    case DKVR_OPCODE_HANDSHAKE1:
    case DKVR_OPCODE_PONG:
    case DKVR_OPCODE_IMU_RAW:
    case DKVR_OPCODE_IMU_QUAT:
    // already handled by dkvr client
    case DKVR_OPCODE_HANDSHAKE2:
    case DKVR_OPCODE_HEARTBEAT:
    case DKVR_OPCODE_PING:
    default:
        break;
    }
}

static void handle_gpio_interrupt()
{
    if (gpio_interrupt)
    {
        gpio_interrupt = 0;
        LOG_IF_ERR(handle_interrupt());
    }

    // data ready interrupt will not raise on handle_interrupt() returns error
    if (is_imu_data_ready())
    {   
        dkvr_err result = update_imu_readings();
        if (result)
        {
            log_dkvr_error(result);
            return;
        }
        calibrate_imu(&imu_raw.gyro_out, &imu_raw.accel_out, &imu_raw.mag_out);

#ifdef DKVR_EKF
        ekf_predict(&imu_raw.gyro_out, DKVR_IMU_SAMPLING_PERIOD);
        ekf_correct(&imu_raw.accel_out, &imu_raw.mag_out, &imu_quat);
#else
        struct eskf_vector3 gyro, accel, mag;
        memcpy(gyro.data, &imu_raw.gyro_out, sizeof(struct eskf_vector3));
        memcpy(accel.data, &imu_raw.accel_out, sizeof(struct eskf_vector3));
        memcpy(mag.data, &imu_raw.mag_out, sizeof(struct eskf_vector3));
        for (int i = 0; i < 3; i++)
            gyro.data[i] *= 0.017453293f;
        eskf_update(gyro, accel, mag);
        memcpy(&imu_quat, eskf_nominal.orientation, sizeof(imu_quat));
#endif
    }
}

static void send_imu_data()
{
    static uint32_t last_imu_sent = 0;
    if (get_behavior_active())
    {
        uint32_t now = dkvr_get_time();
        if (now > last_imu_sent + DKVR_IMU_SEND_INTERVAL)
        {
            if (get_behavior_raw())
                send_client_inst(DKVR_OPCODE_IMU_RAW, sizeof(imu_raw), 4, &imu_raw);
            else
                send_client_inst(DKVR_OPCODE_IMU_QUAT, sizeof(imu_quat), 4, &imu_quat);
            
            last_imu_sent = now;
        }
    }
}

static void send_client_inst_status()
{
    send_client_inst(DKVR_OPCODE_STATUS, get_tracker_status_size(), 4, get_tracker_status_ptr());
}

static void send_status_on_error()
{
    if (is_tracker_err_raised())
    {
        send_client_inst_status();
    }
}

static void tracker_update_led()
{
    if (get_client_status() == CLIENT_CONNECTED)
    {
        if (get_behavior_led())
        {
            if (get_behavior_active())
                set_led_mode(LED_MODE_SLOWEST);
            else
                set_led_mode(LED_MODE_NORMAL);
        }
        else
        {
            set_led_mode(LED_MODE_MANUAL);
            turn_off_led();
        }
    }
    else
    {
        set_led_mode(LED_MODE_FASTEST);
    }

    if (get_tracker_battery_perc() <= DKVR_BAT_LOW_THRESHOLD)
    {
        static uint32_t bat_warning_begin = 0;
        uint32_t now = dkvr_get_time();
        if (now > (bat_warning_begin + DKVR_BAT_LOW_LED_INTERVAL))
        {
            bat_warning_begin = now;
            interrupt_led_for_low_battery();
        }
    }

    update_led();
}
