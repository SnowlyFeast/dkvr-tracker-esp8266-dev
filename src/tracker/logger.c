#include <tracker/logger.h>

#include <common/dkvr_const.h>
#include <common/dkvr_core.h>

#define CAPACITY    8

#if defined(DKVR_DEBUG_USE_LOGGER) 

static struct dkvr_error_log tracker_log[CAPACITY];
static int size = 0;

const struct dkvr_error_log* tracker_logger_get_list() { return tracker_log; }
int tracker_logger_get_count() { return size; }
void tracker_logger_flush() { size = 0; }

void tracker_logger_push(dkvr_err err, PGM_P msg)
{
    if (size > CAPACITY){
        PRINTLN("Tracker logger is full.");
        return;
    }
    
    int len = strlen_P(msg);
    if (len > DKVR_LOGGER_MSG_LEN)
        len = DKVR_LOGGER_MSG_LEN;

    tracker_log[size].timestamp = dkvr_get_time();
#if defined(__BYTE_ORDER__)&&(__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
    uint8_t* ptr = tracker_log[size].timestamp;
    uint8_t temp = ptr[0];
    ptr[0] = ptr[3];
    ptr[3] = temp;
    temp = ptr[1];
    ptr[1] = ptr[2];
    ptr[2] = temp;
#endif
    tracker_log[size].err = err;
    memset(tracker_log[size].msg, '\0', DKVR_LOGGER_MSG_LEN);
    memcpy_P(tracker_log[size].msg, msg, len);
    size++;
}

#else

const struct dkvr_error_log* tracker_logger_get_list() { return NULL; }
int tracker_logger_get_count() { return 0; }
void tracker_logger_flush() {}

void tracker_logger_push(dkvr_err err, PGM_P msg) {}

#endif