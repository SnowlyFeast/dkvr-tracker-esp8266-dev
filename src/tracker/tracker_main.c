#include "tracker/tracker_main.h"

#include "common/dkvr_const.h"
#include "common/dkvr_core.h"
#include "common/system_interface.h"

#include "filter/eskf_sensor_fusion.h"
#include "imu/imu_control.h"
#include "led/led_control.h"
#include "network/dkvr_client.h"

#include "tracker/behavior.h"
#include "tracker/calibration.h"
#include "tracker/instruction_handler.h"
#include "tracker/logger.h"
#include "tracker/statistic.h"
#include "tracker/status.h"


static volatile int gpio_interrupted = 0;
static int last_interrupt = 0;
static int interrupt_missed = 0;
static int imu_read_successful = 0;

static struct eskf_configuration eskf_config = {0};

static void read_imu();
static void send_data();
static void report_error();
static void apply_new_configuration();
static void update_led();

void IRAM_ATTR tracker_main_set_gpio_interrupted()
{
    // if previous interrupt is not handled
    if (gpio_interrupted)
        interrupt_missed = 1;

    gpio_interrupted = 1;
    last_interrupt = dkvr_get_time();
}

int tracker_main_init()
{
    // init miscellaneous things
    dkvr_led_init();
    dkvr_led_off();
    dkvr_led_on();
    tracker_behavior_reset();
    tracker_calib_reset();

    // init DKVR client
    dkvr_err net_err = dkvr_client_init(tracker_instruction_handler);
    if (net_err != DKVR_OK) 
    {
        // actually never gonna happen as dkvr_client_init()
        // always returns DKVR_OK.. but maybe someday?
        tracker_status_set_init_result(net_err);
        PRINTLN("DKVR client init failed with code 0x", (net_err));
    }

    // init IMU
    dkvr_err imu_err = dkvr_imu_init();
    if (imu_err == DKVR_OK)
    {
        // configure filter
        eskf_config.time_step = DKVR_IMU_SAMPLING_PERIOD;
        tracker_calib_get_noise_variance(eskf_config.noise_gyro,
                                         eskf_config.noise_accel,
                                         eskf_config.noise_mag);
        eskf_config.uncertainty_linear_accel = DKVR_ESKF_UNCERTAIN_ACC;
        eskf_config.uncertainty_magnetic_dist = DKVR_ESKF_UNCERTAIN_MAG;
        eskf_config.uncertainty_orientaiton_low = DKVR_ESKF_UNCERTAIN_ORI_LOW;
        eskf_config.uncertainty_orientation_med = DKVR_ESKF_UNCERTAIN_ORI_MED;
        eskf_config.lpf_cutoff_linear_accel = DKVR_ESKF_LPF_CUTOFF_ACC;
        eskf_config.lpf_cutoff_magnetic_dist = DKVR_ESKF_LPF_CUTOFF_MAG;
        eskf_configure(&eskf_config);

        // some delay for stable IMU read
        dkvr_delay(200);

        // wait for initial sensor readings
        while (1)
        {
            dkvr_delay(50);
            if (dkvr_imu_handle_interrupt() != DKVR_OK) continue;   // interrupt reading failed
            if (!dkvr_imu_is_data_ready()) continue;                // IMU data is not ready
            if (dkvr_imu_read() != DKVR_OK) continue;               // imu reading failed

            tracker_calib_transform_readings(dkvr_imu_raw.gyr, dkvr_imu_raw.acc, dkvr_imu_raw.mag);
            eskf_init(dkvr_imu_raw.acc, dkvr_imu_raw.mag);
            break;
        }

        #ifdef DKVR_IMU_MAG_BOOT_TIME_SCALING
        // set scale factor
        float scale = 1.0f / sqrtf(SQUARE(dkvr_imu_raw.mag[0]) + SQUARE(dkvr_imu_raw.mag[1]) + SQUARE(dkvr_imu_raw.mag[2]));
        tracker_calib_set_scale_factor(scale);
        #endif
    }
    else
    {
        // overwrite the init result as imu error is more important
        tracker_status_set_init_result(imu_err);
        tracker_logger_push(imu_err, PSTR("IMU initialization failed"));
        PRINTLN("IMU init failed with code 0x", (imu_err));
    }

    return (imu_err != DKVR_OK) || (net_err != DKVR_OK);
}

void tracker_main_update()
{
    // spinlock to wait interrupt if < 1ms
    uint32_t next_int_estimated = last_interrupt + DKVR_IMU_SAMPLING_PERIOD * 1000; // ms
    if (dkvr_get_time() > next_int_estimated - 2)
    {
        // lock for +2ms of estimated interrupt timinig
        while (!gpio_interrupted)
        {
            if (dkvr_get_time() > next_int_estimated + 2)
                break;
        }
    }

    // handle GPIO interrupt
    if (gpio_interrupted)
    {
        read_imu();
        gpio_interrupted = 0;

        if (interrupt_missed)
        {
            tracker_statistic_record_interrupt_miss();
            interrupt_missed = 0;
        }

        tracker_statistic_record_cycle_end();
    }

    // update dkvr client connection
    dkvr_err net_err = dkvr_client_update_connection();
    if (net_err)
    {
        // error report to host side is not available, Serial is the only option
        PRINTLN("DKVR client connection failed with code 0x", (net_err));
    }

    if (dkvr_client_is_connected())
    {
        dkvr_client_dispatch_received();
        apply_new_configuration(); // apply new config if received
        send_data();
        report_error();
    }
    
    // update etc
    tracker_status_update();
    update_led();
}

static void read_imu()
{
    dkvr_err err = dkvr_imu_handle_interrupt();
    if (err)
    {
        tracker_logger_push(err, PSTR("IMU interrupt handling failed."));
        tracker_statistic_record_imu_miss();
        return;
    }

    if (dkvr_imu_is_data_ready())
    {
        // read IMU
        dkvr_err err = dkvr_imu_read();
        
        if (err)
        {
            tracker_logger_push(err, PSTR("IMU reading failed."));
            tracker_statistic_record_imu_miss();
            return;
        }
        imu_read_successful = 1;

        // calibrate
        tracker_calib_transform_readings(dkvr_imu_raw.gyr,
                                         dkvr_imu_raw.acc,
                                         dkvr_imu_raw.mag);
        // estimate orienation
        eskf_update(dkvr_imu_raw.gyr, dkvr_imu_raw.acc, dkvr_imu_raw.mag);

        // handle post-error
        if (isnanf(eskf_nominal.orientation[0]) || isnanf(eskf_nominal.orientation[1]) ||
            isnanf(eskf_nominal.orientation[2]) || isnanf(eskf_nominal.orientation[3]) )
        {
            eskf_init(dkvr_imu_raw.acc, dkvr_imu_raw.mag);
        }
    }
}

static void send_data()
{
    // send data
    if (tracker_behavior_get_active())
    {
        static uint32_t last_data_sent = 0;
        uint32_t now = dkvr_get_time();
        if (now > last_data_sent + DKVR_NET_IMU_SEND_INTERVAL)
        {
            // send raw
            if (tracker_behavior_get_raw() && imu_read_successful)
            {
                dkvr_client_send_instruction(
                    DKVR_OPCODE_RAW,
                    sizeof(dkvr_imu_raw),
                    sizeof(float),
                    &dkvr_imu_raw);
            }

            // send nominal
            if (tracker_behavior_get_nominal())
            {
                dkvr_client_send_instruction(
                    DKVR_OPCODE_NOMINAL,
                    sizeof(eskf_nominal),
                    sizeof(float),
                    &eskf_nominal);
            }
            
            
            {
                // TODO: interpolation
            }
            last_data_sent = now;
        }
    }

    // reset flag
    imu_read_successful = 0;
}

static void report_error()
{
    int count = tracker_logger_get_count();
    for (int i = 0; i < count; i++)
    {
        dkvr_client_send_instruction(
            DKVR_OPCODE_DEBUG,
            sizeof(struct dkvr_error_log),
            1,
            &(tracker_logger_get_list()[i]));
    }
    tracker_logger_flush();
}

static void apply_new_configuration()
{
    if (tracker_calib_is_noise_variance_updated())
    {
        tracker_calib_get_noise_variance(eskf_config.noise_gyro,
                                         eskf_config.noise_accel,
                                         eskf_config.noise_mag);
        eskf_configure(&eskf_config);
    }
}

static void update_led()
{
    // led mode is selected by DKVR client status and tracker behavior
    if (dkvr_client_is_connected())
    {
        // only update on behavior changed
        if (tracker_behavior_is_updated())
        {
            if (tracker_behavior_get_led())
            {
                if (tracker_behavior_get_active())
                    dkvr_led_set_mode(DKVR_LED_MODE_SLOWEST);
                else
                    dkvr_led_set_mode(DKVR_LED_MODE_NORMAL);
            }
            else
            {
                dkvr_led_set_mode(DKVR_LED_MODE_MANUAL);
                dkvr_led_off();
            }
        }
    }
    else
    {
        dkvr_led_set_mode(DKVR_LED_MODE_FASTEST);
    }

    // low battery indication
    if (tracker_status_get_battery_level() <= DKVR_BAT_LOW_THRESHOLD)
    {
        static uint32_t bat_warning_begin = 0;
        uint32_t now = dkvr_approximate_time;
        if (now > (bat_warning_begin + DKVR_BAT_LOW_LED_INTERVAL))
        {
            bat_warning_begin = now;
            dkvr_led_interrupt_for_low_battery();
        }
    }

    // update internal led logic
    dkvr_led_update();
}
