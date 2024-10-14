#pragma once

#include <plog/Log.h>
#include <termios.h>

namespace Serial
{

/**
 * @brief Parity enum class.
 * @details This enum class is used to force the parity for serial communication, regardless of the set of options
 * applied. None is no parity (PARENB), Odd is odd parity (PARENB, PARODD), Even is even parity (PARENB, ~PARODD).
 */
enum class Parity : uint8_t
{
    None = 0,
    Odd = 1,
    Even = 2
};

/**
 * @brief Set serial port options for serial communication.
 * @details This function sets the serial port options for serial communication. Notably,
 * non-canonical mode is used, 8 data bits, 1 stop bit, and no parity.
 * @param options The termios struct to set the options for.
 */
inline void
set_serial_options(termios& options)
{
    PLOG_DEBUG.printf("Setting serial options.");
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~OPOST;
}

}  // namespace Serial
