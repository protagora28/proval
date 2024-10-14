#pragma once

#include <stdio.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace Serial
{

enum class Status : uint32_t
{
    NoError = 0,
    InvalidTimeout = 1,
    OpenError = 1 << 1,
    TCGetAttrError = 1 << 2,
    CFSetISpeedError = 1 << 3,
    CFSetOSpeedError = 1 << 4,
    TCSetAttrError = 1 << 5,
    TCFlushError = 1 << 6,
    DrainError = 1 << 7,
    NAck = 1 << 8,
    Timeout = 1 << 9,
    WriteError = 1 << 10,
    ReadError = 1 << 11,
    InvalidSize = 1 << 12,
    InvalidStartSymbol = 1 << 13,
    InvalidEPID = 1 << 14,
    InvalidTRM16ID = 1 << 15,
    InvalidCmdID = 1 << 16,
    InvalidPayloadLength = 1 << 17,
    InvalidOutcome = 1 << 18,
    CRCMismatch = 1 << 19,
    InvalidEndSymbol = 1 << 20,
    EPIDMismatch = 1 << 21,
    TRM16IDMismatch = 1 << 22,
    CmdIDMismatch = 1 << 23,
    CRCCmdMismatch = 1 << 24,
    AlreadyConnectedError = 1 << 25,
    ISpeedMismatch = 1 << 26,
    OSpeedMismatch = 1 << 27,
    OptionsMismatch = 1 << 28,
    InvalidBeamRowID = 1 << 29,
    GPIOError = 1 << 30,
    InvalidBeamCoefficientValue = 1lu << 31lu
};

inline Status&
operator|=(Status& lhs, const Status& rhs)
{
    Status* result = &lhs;
    *result = static_cast<Status>(static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs));
    return *result;
}

std::string
status_message_to_string(const Status& error_code);

constexpr uint8_t c_7_bit_bitmask = 0x7F;

/**
 * @brief Extract the upper 7 bits of a uint16.
 * @details Extract the upper 7 bits of a uint16. Used to split long fields of the serial messages into two symbols.
 * It is implicitly assumed that the uint16 is instead a uint14 (MSBs are 0).
 * @param value Value to be split. Assumed to be a uint14.
 * @return Upper 7 bits of the value.
 */
inline uint8_t
upper_7_bits_0_msb(const uint16_t value)
{
    return static_cast<uint8_t>((value >> 7) & c_7_bit_bitmask);
}

/**
 * @brief Extract the lower 7 bits of a uint16.
 * @details Extract the lower 7 bits of a uint16. Used to split long fields of the serial messages into two symbols.
 * It is implicitly assumed that the uint16 is instead a uint14 (MSBs are 0).
 * @param value Value to be split. Assumed to be a uint14.
 * @return Lower 7 bits of the value.
 */
inline uint8_t
lower_7_bits_0_msb(const uint16_t value)
{
    return static_cast<uint8_t>(value & c_7_bit_bitmask);
}

/**
 * @brief Split a uint16 in two 7-bit symbols with MSB 0.
 * @details Split a uint16 into the lower and the upper 7 bits of the uint16. Used to split long fields of the
 * serial messages into two symbols. It is implicitly assumed that the uint16 is instead a uint14 (MSB are 0).
 * @param byte Value to be split. Assumed to be a uint14.
 * @return Lower and upper 7 bits of the value.
 */
template<typename T>
inline std::pair<uint8_t, uint8_t>
split_into_two_symbols(const T value)
{
    static_assert(std::is_unsigned<T>::value, "Value must be an integral type.");
    static_assert(sizeof(T) <= sizeof(uint16_t), "Value must be at most a uint16.");
    const uint8_t value_lower = static_cast<uint8_t>(value) & c_7_bit_bitmask;
    const uint8_t value_upper = static_cast<uint8_t>(value >> 7) & c_7_bit_bitmask;
    return std::make_pair(value_lower, value_upper);
}

/**
 * @brief Merge two 7-bit symbols with MSB 0 into a uint16.
 * @details Merge two 7-bit symbols with MSB 0 into a uint16. Used to merge long fields of the serial messages into
 * a single symbol. It is implicitly assumed that the uint16 is instead a uint14 (MSB are 0).
 * @param lower Lower 7 bits of the byte.
 * @param upper Upper 7 bits of the byte.
 * @return Merged value.
 */
inline uint16_t
merge_two_symbols(const uint8_t value_lower, const uint8_t value_upper)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(value_upper) << 7) | static_cast<uint16_t>(value_lower));
}

uint16_t
calculate_crc14(const std::vector<uint8_t>& data);

class Message
{
public:

    /**
     * @brief Message constructor.
     * @details Create an empty (invalid) message with the specified number of bytes in the payload. Typically used to
     * create an empty message to be filled with the data received through the serial port.
     * @param payload_length Payload length.
     */
    explicit Message(const size_t payload_length);

    /**
     * @brief Message constructor.
     * @details Create a message from the EP ID, TRM16 ID, command ID and payload. Constructs the header adding the
     * start symbol and the payload length, and the footer adding the CRC and the end symbol. Typically used to create
     * a message to be sent through the serial port.
     * @param ep_id EP ID.
     * @param trm16_id TRM16 ID.
     * @param cmd_id Command ID.
     */
    Message(
        const uint8_t ep_id,
        const uint8_t trm16_id,
        const uint8_t cmd_id,
        const std::vector<uint8_t>& payload = {}
    );

    /**
     * @brief Message constructor.
     * @details Create a message from a vector of bytes. The vector of bytes is expected to contain valid header and
     * footer, and the payload. The header and footer are stripped and stored in the corresponding fields. The payload
     * is stored as a vector of bytes. The CRC is calculated and stored in the corresponding field. It is checked
     * whether the message is valid, and if not an error message is logged. Typically used to create a Message from a
     * serialized message received through the serial port.
     * @param message Message as vector of bytes.
     */
    explicit Message(const std::vector<uint8_t>& message);

    size_t
    size() const
    {
        return sizeof(m_header) + m_payload.size() + sizeof(m_footer);
    }

    uint8_t
    get_start_symbol() const
    {
        return m_header.start_symbol;
    }

    uint8_t
    get_ep_id() const
    {
        return m_header.ep_id;
    }

    uint8_t
    get_trm16_id() const
    {
        return m_header.trm16_id;
    }

    uint8_t
    get_cmd_id() const
    {
        return m_header.cmd_id;
    }

    uint8_t
    get_payload_len_lsb() const
    {
        return m_header.payload_len_lsb;
    }

    uint8_t
    get_payload_len_msb() const
    {
        return m_header.payload_len_msb;
    }

    uint16_t
    get_payload_len() const
    {
        return merge_two_symbols(m_header.payload_len_lsb, m_header.payload_len_msb);
    }

    uint8_t
    get_outcome() const
    {
        return static_cast<uint8_t>(m_footer.outcome);
    }

    uint8_t
    get_crc_cmd_lsb() const
    {
        return m_footer.crc_cmd_lsb;
    }

    uint8_t
    get_crc_cmd_msb() const
    {
        return m_footer.crc_cmd_msb;
    }

    uint16_t
    get_crc_cmd() const
    {
        return merge_two_symbols(m_footer.crc_cmd_lsb, m_footer.crc_cmd_msb);
    }

    uint8_t
    get_crc_lsb() const
    {
        return m_footer.crc_lsb;
    }

    uint8_t
    get_crc_msb() const
    {
        return m_footer.crc_msb;
    }

    uint16_t
    get_crc() const
    {
        return merge_two_symbols(m_footer.crc_lsb, m_footer.crc_msb);
    }

    uint8_t
    get_end_symbol() const
    {
        return m_footer.end_symbol;
    }

    const std::vector<uint8_t>&
    get_payload() const
    {
        return m_payload;
    }

    /**
     * @brief Check if the message is valid.
     * @details Check if the message is valid. The message is considered valid if it starts with the start symbol, ends
     * with the end symbol, the EP and TRM16 ID are valid, the payload length is correct, the outcome is valid, and
     * that the CRC is correct.
     */
    bool
    is_valid(Status& status) const;

    /**
     * @brief Encode the message for serial communication.
     * @details Encode a message to be sent through the serial port as a vector of unsigned characters.
     * @return Encoded message as vector of bytes.
     */
    std::vector<uint8_t>
    encode() const;

    /**
     * @brief Check integrity of a message.
     * @details It is checked if the message is valid, if the CRC is correct and if the EP, TRM16 and command IDs match
     * the original message IDs.
     * @param original_message Original message.
     * @return True if the message is valid, the CRC is correct and the IDs match, false otherwise.
     */
    bool
    check_integrity(const Message& original_message, Status& status) const;

    /**
     * @brief Check the CRC of a message.
     * @details The CRC is calculated as the sum of all the bytes in the message, except the CRC itself and the original
     * command CRC (which are overwritten to 0). It is compared to the CRC contained in the message.
     * @return True if the two CRC match, false otherwise.
     */
    bool
    check_crc() const;

    /**
     * @brief Check the command CRC of a message against the original command.
     * @details Compare the command CRC fields of the message against the CRC of the original command.
     * @return True if the command CRC match, false otherwise.
     */
    bool
    check_cmd_crc(const Message& original_message) const;

private:

    struct Header
    {
        uint8_t start_symbol;
        uint8_t ep_id;
        uint8_t trm16_id;
        uint8_t cmd_id;
        uint8_t payload_len_lsb;
        uint8_t payload_len_msb;
    };

    enum class Outcome : uint8_t
    {
        Ack = 0x00,
        Nack = 0x7F,
        CRCMismatch = 0x55,
        NA = 0x41
    };

    struct Footer
    {
        Outcome outcome;
        uint8_t crc_cmd_lsb;
        uint8_t crc_cmd_msb;
        uint8_t crc_lsb;
        uint8_t crc_msb;
        uint8_t end_symbol;
    };

    Header m_header;
    std::vector<uint8_t> m_payload;
    Footer m_footer;

    /**
     * @brief Calculate the CRC of the message.
     * @details Calculate the CRC of a message. The CRC calculation skips the outcome field, the CRC itself and the
     * original command CRC. @see Serial::calculate_crc14.
     * @return CRC of the message.
     */
    uint16_t
    calculate_crc14() const;

public:

    static constexpr uint8_t c_start_symbol{0xA4};
    static constexpr uint8_t c_end_symbol{0x80};
    static constexpr uint8_t c_min_length{sizeof(Serial::Message::Header) + sizeof(Serial::Message::Footer)};
    static constexpr uint8_t c_max_pgt_rows{100};
    static constexpr uint16_t c_max_length{c_min_length + c_max_pgt_rows * 50};
};

std::string
to_string(const Message& message);

}  // namespace Serial