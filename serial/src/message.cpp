#include <plog/Log.h>

#include <iomanip>
#include <sstream>

#include <message.hpp>

namespace Serial
{

uint16_t
calculate_crc14(const std::vector<uint8_t>& data)
{
    const size_t length = data.size();
    const uint16_t polynomial = 0x202D;  // CRC-14/ITU polynomial.
    uint16_t crc = 0;  // Initial CRC value

    for (size_t i = 0; i < length; ++i)
    {
        crc = static_cast<uint16_t>(crc ^ static_cast<uint16_t>(data[i] << 6u));  // XOR in the next 6 bits.

        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x4000)
            {  // Check if the MSB is set.
                crc = static_cast<uint16_t>(crc << 1) ^ polynomial;
            }
            else
            {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc & 0x3FFF;  // Mask to keep only the lower 14 bits.
}

std::string
status_message_to_string(const Status& error_code)
{
    switch (error_code)
    {
        case Status::NoError:
            return "NoError";
        case Status::InvalidTimeout:
            return "InvalidTimeout";
        case Status::OpenError:
            return "OpenError";
        case Status::TCGetAttrError:
            return "TCGetAttrError";
        case Status::CFSetISpeedError:
            return "CFSetISpeedError";
        case Status::CFSetOSpeedError:
            return "CFSetOSpeedError";
        case Status::TCSetAttrError:
            return "TCSetAttrError";
        case Status::TCFlushError:
            return "TCFlushError";
        case Status::NAck:
            return "NAck";
        case Status::Timeout:
            return "Timeout";
        case Status::WriteError:
            return "WriteError";
        case Status::ReadError:
            return "ReadError";
        case Status::InvalidSize:
            return "InvalidSize";
        case Status::InvalidStartSymbol:
            return "InvalidStartSymbol";
        case Status::InvalidEPID:
            return "InvalidEPID";
        case Status::InvalidTRM16ID:
            return "InvalidTRM16ID";
        case Status::InvalidCmdID:
            return "InvalidCmdID";
        case Status::InvalidPayloadLength:
            return "InvalidPayloadLength";
        case Status::InvalidOutcome:
            return "InvalidOutcome";
        case Status::CRCMismatch:
            return "CRCMismatch";
        case Status::InvalidEndSymbol:
            return "InvalidEndSymbol";
        case Status::EPIDMismatch:
            return "EPIDMismatch";
        case Status::TRM16IDMismatch:
            return "TRM16IDMismatch";
        case Status::CmdIDMismatch:
            return "CmdIDMismatch";
        case Status::CRCCmdMismatch:
            return "CRCCmdMismatch";
        case Status::DrainError:
            return "DrainError";
        case Status::AlreadyConnectedError:
            return "AlreadyConnectedError";
        case Status::ISpeedMismatch:
            return "ISpeedMismatch";
        case Status::OSpeedMismatch:
            return "OSpeedMismatch";
        case Status::OptionsMismatch:
            return "OptionsMismatch";
        case Status::InvalidBeamRowID:
            return "InvalidBeamRowID";
        case Status::GPIOError:
            return "GPIOError";
        case Status::InvalidBeamCoefficientValue:
            return "InvalidBeamCoefficientValue";
        default:
            return "Unknown";
    }
}

Message::Message(const uint8_t ep_id, const uint8_t trm16_id, const uint8_t cmd_id, const std::vector<uint8_t>& payload)
    : m_header{c_start_symbol, ep_id, trm16_id, cmd_id, 0, 0}
    , m_payload(payload)
    , m_footer{Outcome::NA, 0, 0, 0, 0, c_end_symbol}
{
    const auto [payload_len_lower, payload_len_upper] = split_into_two_symbols(static_cast<uint16_t>(m_payload.size()));
    m_header.payload_len_lsb = payload_len_lower;
    m_header.payload_len_msb = payload_len_upper;

    const auto [crc_lower, crc_upper] = split_into_two_symbols(calculate_crc14());
    m_footer.crc_lsb = crc_lower;
    m_footer.crc_msb = crc_upper;
    m_footer.crc_cmd_lsb = crc_lower;
    m_footer.crc_cmd_msb = crc_upper;
}

Message::Message(const size_t payload_length)
    : m_payload(payload_length)
{}

Message::Message(const std::vector<uint8_t>& message)
{
    if (message.size() < c_min_length)
    {
        PLOG_ERROR.printf("Message too short.");
        return;
    }

    std::memcpy(&m_header, message.data(), sizeof(m_header));
    uint16_t payload_len = merge_two_symbols(m_header.payload_len_lsb, m_header.payload_len_msb);
    m_payload.resize(payload_len);
    std::copy(message.cbegin() + sizeof(m_header), message.cend() - sizeof(m_footer), m_payload.begin());
    std::memcpy(&m_footer, message.data() + message.size() - sizeof(m_footer), sizeof(m_footer));
}

uint16_t
Message::calculate_crc14() const
{
    std::vector<uint8_t> serialized_message = this->encode();
    serialized_message[serialized_message.size() - 3] = 0;
    serialized_message[serialized_message.size() - 2] = 0;
    return Serial::calculate_crc14(serialized_message);
}

std::vector<uint8_t>
Message::encode() const
{
    std::vector<uint8_t> message(sizeof(Header) + m_payload.size() + sizeof(Footer));
    std::memcpy(message.data(), &m_header, sizeof(Header));
    std::copy(m_payload.cbegin(), m_payload.cend(), message.begin() + sizeof(Header));
    std::memcpy(message.data() + sizeof(Header) + m_payload.size(), &m_footer, sizeof(Footer));

    return message;
}

bool
Message::is_valid(Status& status) const
{
    bool validity = true;
    status = Status::NoError;

    // The minimum length of a message is 12 bytes (header + footer).
    if (this->size() < c_min_length)
    {
        PLOG_ERROR.printf("Invalid message size.");
        validity = false;
        status |= Status::InvalidSize;
    }

    // The start symbol should be 0xA4.
    if (m_header.start_symbol != c_start_symbol)
    {
        PLOG_ERROR.printf("Start symbol mismatch.");
        validity = false;
        status |= Status::InvalidStartSymbol;
    }

    // EP ID should range from 0 to 8.
    if (m_header.ep_id > 8)
    {
        PLOG_ERROR.printf("Invalid EP ID.");
        validity = false;
        status |= Status::InvalidEPID;
    }

    // TRM16 ID should range from 0 to 64.
    if (m_header.trm16_id > 64)
    {
        PLOG_ERROR.printf("Invalid TRM16 ID.");
        validity = false;
        status |= Status::InvalidTRM16ID;
    }

    // Command ID should range from 1 to 33 and from 80 to 84.
    const bool is_valid_cmd_id = m_header.cmd_id > 0 && m_header.cmd_id <= 33;
    const bool is_valid_cmd_id_debug = m_header.cmd_id >= 80 && m_header.cmd_id <= 84;
    if (!is_valid_cmd_id && !is_valid_cmd_id_debug)
    {
        PLOG_ERROR.printf("Invalid command ID.");
        validity = false;
        status |= Status::InvalidCmdID;
    }

    // Payload length should match the actual payload length.
    if (this->get_payload_len() != (this->size() - c_min_length))
    {
        PLOG_ERROR.printf("Invalid payload length.");
        validity = false;
        status |= Status::InvalidPayloadLength;
    }

    // Outcome should be 0, 0x7F, 0x55 or 0x41.
    if (m_footer.outcome != Outcome::Ack && m_footer.outcome != Outcome::Nack &&
        m_footer.outcome != Outcome::CRCMismatch && m_footer.outcome != Outcome::NA)
    {
        PLOG_ERROR.printf("Invalid outcome.");
        validity = false;
        status |= Status::InvalidOutcome;
    }

    // CRC should match the calculated CRC.
    if (!check_crc())
    {
        PLOG_ERROR.printf("CRC mismatch.");
        validity = false;
        status |= Status::CRCMismatch;
    }

    // End symbol should be 0x80.
    if (m_footer.end_symbol != c_end_symbol)
    {
        PLOG_ERROR.printf("End symbol mismatch.");
        validity = false;
        status |= Status::InvalidEndSymbol;
    }

    return validity;
}

bool
Message::check_integrity(const Message& original_message, Status& status) const
{
    bool is_integral = true;

    if (m_header.ep_id != original_message.get_ep_id())
    {
        PLOG_ERROR.printf("EP ID mismatch.");
        is_integral = false;
        status |= Status::EPIDMismatch;
    }

    if (m_header.trm16_id != original_message.get_trm16_id())
    {
        PLOG_ERROR.printf("TRM16 ID mismatch.");
        is_integral = false;
        status |= Status::TRM16IDMismatch;
    }

    if (m_header.cmd_id != original_message.get_cmd_id())
    {
        PLOG_ERROR.printf("Command ID mismatch.");
        is_integral = false;
        status |= Status::CmdIDMismatch;
    }

    if (!check_cmd_crc(original_message))
    {
        PLOG_ERROR.printf("Command CRC mismatch.");
        is_integral = false;
        status |= Status::CRCCmdMismatch;
    }

    return is_integral;
}

bool
Message::check_crc() const
{
    const bool res = this->get_crc() == this->calculate_crc14();
    PLOG_DEBUG_IF(!res).printf("CRC mismatch: %04X != %04X", this->get_crc(), this->calculate_crc14());
    return res;
}

bool
Message::check_cmd_crc(const Message& original_message) const
{
    const bool res = this->get_crc_cmd() == original_message.get_crc();
    PLOG_DEBUG_IF(!res).printf("CMD CRC mismatch: %04X != %04X", this->get_crc(), this->calculate_crc14());
    return res;
}

std::string
to_string(const Message& message)
{
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');

    for (const auto& byte : message.encode())
    {
        ss << "0x" << std::setw(2) << static_cast<int>(byte) << " ";
    }

    return ss.str();
}

}  // namespace Serial
