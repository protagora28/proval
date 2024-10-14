#include <asm/termbits.h> /* Definition of constants */
#include <fcntl.h>
#include <plog/Log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <thread>

#include <safe_io.hpp>

namespace Serial
{

bool
safe_write(
    const int serial_fd,
    const Message& message,
    Status& status,
    const bool drain,
    const int byte_time_estimate_ns
)
{
    // TODO Investigate 3ms sleep before writing... THIS MUST BE REMOVED.
    // Probably the EP is transmitting for too long after the message is sent.
    // So we need to wait a bit longer before writing characters to the serial port.
    std::this_thread::sleep_for(std::chrono::milliseconds(3));

    status = Status::NoError;

    const std::vector<uint8_t> encoded_message = message.encode();

    // Print the encoded message size.
    PLOG_DEBUG.printf("Encoded message size: %lu", encoded_message.size());

    // Print the encoded message bytes in a single line.
    PLOG_DEBUG.printf("Encoded message bytes: %s", to_string(message).c_str());

    // Write the encoded message to the serial port.
    const long int n_write = write(serial_fd, encoded_message.data(), encoded_message.size());

    if (n_write <= 0)
    {
        PLOG_ERROR.printf("Failed write call: %s", strerror(errno));
        status = Status::WriteError;
        return false;
    }

    PLOG_DEBUG.printf("Wrote %li bytes to serial port.", n_write);

    // Fast serial port drain.
    // Code inspired by https://linux-serial.vger.kernel.narkive.com/zKYVsX3z/tcdrain-tcsbrk-wait-until-sent-delay
    if (drain)
    {
        const auto t0 = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::milliseconds(1000);

        unsigned int lsr = 0;
        unsigned int bit = TIOCM_RTS;
        if (ioctl(serial_fd, TIOCMBIS, &bit) < 0)
        {
            PLOG_DEBUG.printf("Failed TIOCMBIS call: %s", strerror(errno));
            // Not an error since in socat it is not supported.
        }

        if (byte_time_estimate_ns > 0)
        {
            // Estimate the time to transmit the message.
            const auto estimated_time =
                std::chrono::nanoseconds(static_cast<size_t>(byte_time_estimate_ns) * encoded_message.size());
            PLOG_DEBUG.printf(
                "Estimated time to transmit message: %f ms.",
                std::chrono::duration<double, std::milli>(estimated_time).count()
            );

            // Wait for the estimated time to transmit the message.
            std::this_thread::sleep_for(estimated_time);
        }

        do
        {
            if (ioctl(serial_fd, TIOCSERGETLSR, &lsr) < 0)
            {
                PLOG_DEBUG.printf("Failed TIOCSERGETLSR call: %s", strerror(errno));
                // Not an error since in socat and some USB to RS485 adapters it is not supported.
                break;
            }
            if (std::chrono::steady_clock::now() - t0 >= timeout)
            {
                PLOG_DEBUG.printf("Timeout expired.");
                // Not an error since later we will call tcdrain() anyway.
                break;
            }
        }
        while (!lsr & TIOCSER_TEMT);

        // Now the output buffer should be empty as all characters have been sent.
        // This may not be true if the previous loop was broken due to a timeout or an ioctl() error.
        // Subsequent calls to tcdrain() should be faster.

        // Equivalent to tcdrain();
        if (ioctl(serial_fd, TCSBRK, 1) < 0)
        {
            PLOG_ERROR.printf("Failed TCSBRK call: %s", strerror(errno));
            status = Status::DrainError;
            return false;
        }

        PLOG_DEBUG.printf(
            "Drained output buffer in %f ms.",
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count()
        );
    }

    return true;
}

bool
safe_read(
    const int serial_fd,
    const unsigned int timeout_ms,
    Message& message,
    Status& status,
    const TimeoutSeverity timeout_severity,
    const bool server_mode
)
{
    status = Status::NoError;
    std::vector<uint8_t> encoded_message;
    encoded_message.reserve(server_mode ? Message::c_max_length : message.size());

    {
        const auto start = std::chrono::steady_clock::now();
        std::chrono::nanoseconds timeout_ns{static_cast<size_t>(timeout_ms) * 1'000'000llu}, elapsed{0};
        bool is_read_complete{false};
        enum class ReadState
        {
            SeekStart,
            SeekEnd
        } read_state{ReadState::SeekStart};

        while (status == Status::NoError && !is_read_complete)
        {
            std::vector<uint8_t> partial_message(
                server_mode ? Message::c_max_length : (message.size() - encoded_message.size())
            );

            if (partial_message.empty())
            {
                PLOG_WARNING.printf("Partial message is empty, breaking loop.");
                break;
            }

            PLOG_DEBUG.printf("Reading partial message of size: %lu", partial_message.size());

            const ssize_t n_read = read(serial_fd, partial_message.data(), partial_message.size());

            elapsed = std::chrono::steady_clock::now() - start;

            if (n_read < 0)
            {
                PLOG_ERROR.printf("Failed read call: %s", strerror(errno));
                status = Status::ReadError;
                return false;
            }

            if (n_read == 0 || elapsed >= timeout_ns)
            {
                PLOG_ERROR_IF(timeout_severity == TimeoutSeverity::Error).printf("Timeout expired.");
                PLOG_WARNING_IF(timeout_severity == TimeoutSeverity::Warning).printf("Timeout expired.");
                PLOG_INFO_IF(timeout_severity == TimeoutSeverity::Info).printf("Timeout expired.");
                PLOG_DEBUG_IF(timeout_severity == TimeoutSeverity::Debug).printf("Timeout expired.");
                status = Status::Timeout;
                return false;
            }

            partial_message.resize(n_read);

            PLOG_DEBUG.printf("Read %ld bytes.", n_read);
            PLOG_DEBUG.printf("Read bytes: %s", ::to_string(partial_message.cbegin(), partial_message.cend()).c_str());

            size_t start_discard_count{0}, end_discard_count{0};

            if (read_state == ReadState::SeekStart)
            {
                PLOG_DEBUG.printf("Seeking start symbol.");
                const auto it_start =
                    std::find(partial_message.cbegin(), partial_message.cend(), Message::c_start_symbol);

                const bool start_symbol_found = it_start != partial_message.cend();
                PLOG_DEBUG_IF(!start_symbol_found).printf("Start symbol not found.");
                PLOG_DEBUG_IF(start_symbol_found).printf("Start symbol found.");

                start_discard_count = std::distance(partial_message.cbegin(), it_start);
                read_state = start_symbol_found ? ReadState::SeekEnd : ReadState::SeekStart;
            }

            if (read_state == ReadState::SeekEnd)
            {
                PLOG_DEBUG.printf("Seeking end symbol.");
                const auto it_end = std::find(partial_message.cbegin(), partial_message.cend(), Message::c_end_symbol);
                is_read_complete = it_end != partial_message.cend();
                end_discard_count = is_read_complete ? std::distance(it_end, partial_message.cend()) - 1 : 0;

                PLOG_DEBUG_IF(!is_read_complete).printf("End symbol not found.");
                PLOG_DEBUG_IF(is_read_complete).printf("End symbol found.");
            }

            const size_t discard_count = start_discard_count + end_discard_count;
            PLOG_DEBUG_IF(discard_count)
                .printf(
                    "Discarding %lu bytes from message beginning and %lu bytes from the end.",
                    start_discard_count,
                    end_discard_count
                );

            if (static_cast<size_t>(n_read - discard_count) + encoded_message.size() >
                (server_mode ? Message::c_max_length : message.size()))
            {
                PLOG_ERROR.printf(
                    "Read %ld bytes, expected %lu at most.",
                    n_read + encoded_message.size() - discard_count,
                    server_mode ? Message::c_max_length : message.size()
                );
                status = Status::InvalidSize;
                return false;
            }

            encoded_message.resize(encoded_message.size() + static_cast<size_t>(n_read - discard_count));
            const auto copy_location = encoded_message.end() - static_cast<size_t>(n_read - discard_count);
            std::copy(
                partial_message.cbegin() + start_discard_count,
                partial_message.cend() - end_discard_count,
                copy_location
            );
        }
    }

    PLOG_DEBUG.printf("Encoded message size: %lu", encoded_message.size());
    PLOG_DEBUG.printf(
        "Encoded message bytes: %s",
        ::to_string(encoded_message.cbegin(), encoded_message.cend()).c_str()
    );

    if (!server_mode && encoded_message.size() != message.size())
    {
        PLOG_ERROR.printf("Invalid message size, expected %lu got %lu.", message.size(), encoded_message.size());
        status = Status::InvalidSize;
        return false;
    }

    message = Message{encoded_message};

    return true;
}

}  // namespace Serial
