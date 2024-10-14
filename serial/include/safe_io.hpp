#pragma once

#include <iomanip>
#include <sstream>
#include <string>

#include <message.hpp>

inline std::string
to_string(const std::vector<uint8_t>::const_iterator& begin, const std::vector<uint8_t>::const_iterator& end)
{
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');

    for (auto it = begin; it != end; ++it)
    {
        ss << "0x" << std::setw(2) << static_cast<int>(*it) << " ";
    }

    return ss.str();
}

inline std::string
to_string(const std::vector<uint8_t>& message)
{
    return to_string(message.cbegin(), message.cend());
}

namespace Serial
{

enum class TimeoutSeverity
{
    Debug,
    Info,
    Warning,
    Error
};

/**
 * @brief Write a message to the serial port.
 * @param serial_fd The file descriptor of the serial port.
 * @param message The message to write.
 * @param status The status of the write operation.
 * @param drain Whether to drain the output buffer.
 * @param byte_time_estimate_ns The estimated time to transmit a single byte in nanoseconds (ignored if <= 0).
 * @return True if the write was successful, false otherwise.
 */
bool
safe_write(
    const int serial_fd,
    const Message& message,
    Status& status,
    const bool drain = false,
    const int byte_time_estimate_ns = 0
);

/**
 * @brief Read a message from the serial port.
 * @details This function will read from the serial port until a complete message is received or a timeout occurs. When
 * the server mode is enabled, the function will read until the maximum message length is reached or a timeout occurs.
 * Otherwise, the maximum read size is determined by the size of the message.
 * @param serial_fd The file descriptor of the serial port.
 * @param timeout_ms The timeout in milliseconds (minimum 100).
 * @param message The message to read.
 * @param status The status of the read operation.
 * @param timeout_severity The severity of the timeout.
 * @param server Whether this call is being made by the server.
 * @return True if the read was successful, false otherwise.
 */
bool
safe_read(
    const int serial_fd,
    const unsigned int timeout_ms,
    Message& message,
    Status& status,
    const TimeoutSeverity timeout_severity = TimeoutSeverity::Warning,
    const bool server_mode = false
);

}  // namespace Serial
