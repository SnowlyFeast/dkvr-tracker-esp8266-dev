#include "common/i2c_interface.h"

#include <cstdint>

#include <Wire.h>

#include "common/dkvr_const.h"
#include "common/dkvr_core.h"

#define WIRE_SPEED          400000

dkvr_err_t dkvr_i2c_init()
{
    Wire.begin();
    Wire.setClock(WIRE_SPEED);
    return DKVR_OK;
}

uint8_t dkvr_i2c_read(uint8_t addr, uint8_t reg, uint8_t length, uint8_t* buffer, uint32_t timeout)
{
    TwoWire& wire = Wire;
    uint32_t begin = millis();
    
    wire.beginTransmission(addr);
    wire.write(reg);
    uint8_t result = wire.endTransmission(addr);

    // wire lib defined error
    if (result)
    {
#ifdef DKVR_DEBUG_I2C
        switch (result)
        {
        case 1:
            PRINTLN("BUFFER_LENGTH(32 byte) limit exceeded.");
            break;
        case 2:
            PRINTLN("received NACK on transmit of address.");
            break;
        case 3:
            PRINTLN("received NACK on transmit of data.");
            break;
        case 4:
            PRINTLN("unknown twi error.");
            break;
        case 5:
            PRINTLN("timeout.");
            break;
        default:
        case 0:
            break;
        }
#endif
        return DKVR_ERR_I2C + result;
    }

    // check timeout
    if (timeout != 0 && (millis() - begin > timeout))
        return DKVR_ERR_I2C_TIMEOUT;

    // read rx
    uint8_t size = wire.requestFrom(addr, length);
    result = (size == length) ? DKVR_OK : DKVR_ERR_END_OF_DATA;
#ifdef DKVR_DEBUG_I2C
    PRINT("Dev : 0x", (addr), ", Reg : 0x", (reg), ", Data : 0x");
#endif
    for (int i = 0; i < size; i++)
    {
        // rx buffer is uint8_t but return type is int
        buffer[i] = static_cast<uint8_t>(wire.read());
    }
#ifdef DKVR_DEBUG_I2C
    for (int i = 0; i < size; i++)
    {
        PRINT((buffer[i]), " ");
    }
    PRINT("<<< End of I2C read.");
#endif
#ifdef DKVR_DEBUG_I2C
    if (result)
        PRINTLN("The data read from the device was shorter than the request.");
#endif
    return result;
}

uint8_t dkvr_i2c_write(uint8_t addr, uint8_t reg, uint8_t length, const uint8_t* buffer, uint32_t timeout)
{
    TwoWire &wire = Wire;
    uint32_t begin = millis();

    wire.beginTransmission(addr);
    wire.write(reg);

#ifdef DKVR_DEBUG_I2C
    PRINT("Dev : 0x", (addr), ", Reg : 0x", (reg), ", Data : 0x");
#endif

    wire.write(buffer, length);
#ifdef DKVR_DEBUG_I2C
    for (int i = 0; i < length; i++)
    {
        PRINT((buffer[i]), " ");
    }
    PRINTLN("<<< End of I2C write.");
#endif
    uint8_t result = wire.endTransmission();

    // check timeout
    if (timeout != 0 && (millis() - begin > timeout))
        result = 5;
    
    if (result) {
#ifdef DKVR_DEBUG_I2C
        switch (result)
        {
        case 1:
            PRINTLN("BUFFER_LENGTH(32 byte) limit exceeded.");
            break;
        case 2:
            PRINTLN("received NACK on transmit of address.");
            break;
        case 3:
            PRINTLN("received NACK on transmit of data.");
            break;
        case 4:
            PRINTLN("unknown twi error.");
            break;
        case 5:
            PRINTLN("timeout.");
            break;
        default:
        case 0:
            break;
        }
#endif
        return DKVR_ERR_I2C + result;
    }

    return DKVR_OK;
}
