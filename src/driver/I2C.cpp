#include "I2C.h"

namespace DKVR
{
    namespace Driver
    {
        uint8_t I2C::ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint32_t timeout)
        {
            // prepare
            TwoWire &wire = Wire;
            uint32_t begin = millis();

            // begin i2c com
            wire.beginTransmission(devAddr);
            wire.write(regAddr);
            uint8_t ret = wire.endTransmission(devAddr);

            if (ret)
            {
#ifdef DKVR_ENABLE_I2C_DEBUG
                switch (ret)
                {
                case 1:
                    PRINTLN(F("BUFFER_LENGTH(32 byte) limit exceeded."));
                    break;
                case 2:
                    PRINTLN(F("received NACK on transmit of address."));
                    break;
                case 3:
                    PRINTLN(F("received NACK on transmit of data."));
                    break;
                case 4:
                    PRINTLN(F("unknown twi error."));
                    break;
                case 5:
                    PRINTLN(F("timeout."));
                    break;
                default:
                case 0:
                    break;
                }
#endif
                return ret;
            }

            // check timeout
            if (timeout != 0 && (millis() - begin > timeout))
                return 5;

            // read rx
            uint8_t size = wire.requestFrom(devAddr, length);
            ret = (size == length) ? 0 : 6;
#ifdef DKVR_ENABLE_I2C_DEBUG
            PRINT(F("Dev : 0x"));
            PRINT(String(devAddr, HEX));
            PRINT(F(", Reg : 0x"));
            PRINT(String(regAddr, HEX));
            PRINT(F(", Data : 0x"));
#endif
            for (int i = 0; i < size; i++)
            {
                data[i] = wire.read();
#ifdef DKVR_ENABLE_I2C_DEBUG
                PRINT(String(data[i], HEX));
                i == (size - 1) ? PRINTLN(F(" <<< End of I2C read.")) : PRINT(F(" "));
#endif
            }
#ifdef DKVR_ENABLE_I2C_DEBUG
            if (ret)
                PRINTLN(F("The data read from the device was shorter than the request."));
#endif
            return ret;
        }

        uint8_t I2C::WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t *data, uint32_t timeout)
        {
            // prepare
            TwoWire &wire = Wire;
            uint32_t begin = millis();

            // begin i2c com
            wire.beginTransmission(devAddr);
            wire.write(regAddr);

#ifdef DKVR_ENABLE_I2C_DEBUG
            PRINT(F("Dev : 0x"));
            PRINT(String(devAddr, HEX));
            PRINT(F(", Reg : 0x"));
            PRINT(String(regAddr, HEX));
            PRINT(F(", Data : 0x"));
#endif

            for (int i = 0; i < length; i++)
            {
                wire.write(data[i]);
#ifdef DKVR_ENABLE_I2C_DEBUG
                PRINT(String(data[i], HEX));
                i == (length - 1) ? PRINTLN(F(" <<< End of I2C write.")) : PRINT(F(" "));
#endif
            }
            uint8_t ret = wire.endTransmission();

            // check timeout
            if (timeout != 0 && (millis() - begin > timeout))
                ret = 5;
#ifdef DKVR_ENABLE_I2C_DEBUG
            switch (ret)
            {
            case 1:
                PRINTLN(F("BUFFER_LENGTH(32 byte) limit exceeded."));
                break;
            case 2:
                PRINTLN(F("received NACK on transmit of address."));
                break;
            case 3:
                PRINTLN(F("received NACK on transmit of data."));
                break;
            case 4:
                PRINTLN(F("unknown twi error."));
                break;
            case 5:
                PRINTLN(F("timeout."));
                break;
            default:
            case 0:
                break;
            }
#endif
            return ret;
        }

        uint8_t I2C::WriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t pos, uint8_t data, uint32_t timeout)
        {
            uint8_t buf;
            ReadBytes(devAddr, regAddr, 1, &buf);
            data ? (buf |= ((uint8_t)0x01 << pos)) : (buf &= ~((uint8_t)0x01 << pos));
            return WriteBytes(devAddr, regAddr, 1, &buf);
        }

    } // namespace driver

} // namespace dkvr
