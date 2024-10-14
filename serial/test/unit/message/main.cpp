#include <cstdlib>
#include <ctime>

#include <client.hpp>
#include <message.hpp>
#include <test_utils.hpp>

using namespace Serial;

void
test_bitmask();
void
test_upper_7_bits_0_msb();
void
test_lower_7_bits_0_msb();
void
test_split_into_two_symbols();
void
test_merge_two_symbols();
void
test_encode();
void
test_is_valid();
void
test_check_integrity();

int
main()
{
    // Initialize fixed seed
    std::srand(static_cast<unsigned int>(std::time(0)));

    // Run tests
    test_bitmask();
    test_upper_7_bits_0_msb();
    test_lower_7_bits_0_msb();
    test_split_into_two_symbols();
    test_merge_two_symbols();
    test_encode();

    return 0;
}

void
test_bitmask()
{
    check(c_7_bit_bitmask == 0b1111111);
    check((c_7_bit_bitmask & 0b11111111) == 0b1111111);
}

void
test_upper_7_bits_0_msb()
{
    // Special cases.
    check(upper_7_bits_0_msb(0) == 0);
    check(upper_7_bits_0_msb(1) == 0);
    check(upper_7_bits_0_msb(0b1111111) == 0);
    check(upper_7_bits_0_msb(0b10000000) == 1);
    check(upper_7_bits_0_msb(0b11111111) == 0b1);
    check(upper_7_bits_0_msb(0b111111111) == 0b11);
    check(upper_7_bits_0_msb(0b1111111111) == 0b111);
    check(upper_7_bits_0_msb(0b11111111111) == 0b1111);
    check(upper_7_bits_0_msb(0b111111111111) == 0b11111);
    check(upper_7_bits_0_msb(0b1111111111111) == 0b111111);
    check(upper_7_bits_0_msb(0b11111111111111) == c_7_bit_bitmask);

    for (int i = 0; i < 100; ++i)
    {
        uint16_t value = static_cast<uint16_t>(std::rand()) & c_7_bit_bitmask;
        check(upper_7_bits_0_msb(value) == ((value >> 7) & c_7_bit_bitmask));
    }
}

void
test_lower_7_bits_0_msb()
{
    // Special cases.
    check(lower_7_bits_0_msb(0) == 0);
    check(lower_7_bits_0_msb(1) == 1);
    check(lower_7_bits_0_msb(0b10000000) == 0);
    check(lower_7_bits_0_msb(0b11111111) == c_7_bit_bitmask);
    check(lower_7_bits_0_msb(0b111111111) == c_7_bit_bitmask);
    check(lower_7_bits_0_msb(0b1111111111) == c_7_bit_bitmask);
    check(lower_7_bits_0_msb(0b11111111111) == c_7_bit_bitmask);
    check(lower_7_bits_0_msb(0b111111111111) == c_7_bit_bitmask);
    check(lower_7_bits_0_msb(0b1111111111111) == c_7_bit_bitmask);
    check(lower_7_bits_0_msb(0b11111111111111) == c_7_bit_bitmask);

    for (int i = 0; i < 100; ++i)
    {
        uint16_t value = static_cast<uint16_t>(std::rand()) & c_7_bit_bitmask;
        check(lower_7_bits_0_msb(value) == (value & c_7_bit_bitmask));
    }
}

void
test_split_into_two_symbols()
{
    // Special cases.
    check(split_into_two_symbols((uint8_t)0) == std::make_pair<uint8_t, uint8_t>(0, 0));
    check(split_into_two_symbols((uint8_t)1) == std::make_pair<uint8_t, uint8_t>(1, 0));
    check(split_into_two_symbols((uint8_t)0x7F) == std::make_pair<uint8_t, uint8_t>(0x7F, 0));
    check(split_into_two_symbols((uint16_t)0x80) == std::make_pair<uint8_t, uint8_t>(0, 1));
    check(split_into_two_symbols((uint16_t)0xFF) == std::make_pair<uint8_t, uint8_t>(0x7F, 1));
    check(split_into_two_symbols((uint16_t)0xFFF) == std::make_pair<uint8_t, uint8_t>(0x7F, 31));
    check(split_into_two_symbols((uint16_t)0x3FF) == std::make_pair<uint8_t, uint8_t>(0x7F, 7));

    for (int i = 0; i < 100; ++i)
    {
        uint16_t value = static_cast<uint16_t>(std::rand()) & c_7_bit_bitmask;
        const auto [lower, upper] = split_into_two_symbols(value);
        check(lower == (value & c_7_bit_bitmask));
        check(upper == ((value >> 7) & c_7_bit_bitmask));
    }
}

void
test_merge_two_symbols()
{
    // Special cases.
    check(merge_two_symbols(0, 0) == 0);
    check(merge_two_symbols(1, 0) == 1);
    check(merge_two_symbols(0x7F, 0) == 0x7F);
    check(merge_two_symbols(0, 1) == 0x80);
    check(merge_two_symbols(0x7F, 1) == 0xFF);
    check(merge_two_symbols(0x7F, 0x7F) == 0x3FFF);

    for (int i = 0; i < 100; ++i)
    {
        uint16_t value = static_cast<uint16_t>(std::rand()) & c_7_bit_bitmask;
        const auto [lower, upper] = split_into_two_symbols(value);
        const auto merged = merge_two_symbols(lower, upper);
        check(merged == value);
    }
}

void
test_encode()
{
    // Case empty payload.
    Serial::Message message(0x1, 0x2, 0x3);

    std::vector<uint8_t> test_message = message.encode();

    std::vector<uint8_t> expected_message{
        Serial::Message::c_start_symbol,
        1,
        2,
        3,
        0,
        0,
        static_cast<uint8_t>(0x41),  // Serial::Message::Outcome::NA
        0,
        0,
        0,
        0,
        Serial::Message::c_end_symbol};

    const uint16_t expected_crc = calculate_crc14(expected_message);
    const auto [expected_crc_lower, expected_crc_upper] = split_into_two_symbols(expected_crc);
    expected_message[7] = expected_crc_lower;
    expected_message[8] = expected_crc_upper;
    expected_message[9] = expected_crc_lower;
    expected_message[10] = expected_crc_upper;

    check(test_message == expected_message);

    // Case non-empty payload.
    for (uint16_t payload_len = 1; payload_len < 100; ++payload_len)
    {
        // Payload with random data (fixed seed).
        std::vector<uint8_t> payload(payload_len);
        for (uint16_t i = 0; i < payload_len; ++i)
        {
            payload[i] = static_cast<uint16_t>(std::rand() & c_7_bit_bitmask);
        }

        Serial::Message test_message(1, 2, 3, payload);

        std::vector<uint8_t> test_message_encoded = test_message.encode();

        std::vector<uint8_t> expected_message_encoded(
            6 +  // sizeof(Serial::Message::Header)
            6 +  // sizeof(Serial::Message::Footer)
            payload_len
        );
        const auto [expected_payload_len_lower, expected_payload_len_upper] = split_into_two_symbols(payload_len);

        std::array<uint8_t, 6> header{// Serial::Message::Header
                                      Serial::Message::c_start_symbol,
                                      1,
                                      2,
                                      3,
                                      expected_payload_len_lower,
                                      expected_payload_len_upper};

        std::memcpy(expected_message_encoded.data(), &header, sizeof(header));

        std::copy(payload.begin(), payload.end(), expected_message_encoded.begin() + sizeof(header));

        std::array<uint8_t, 6> footer{
            // Serial::Message::Footer
            0x41,  // Serial::Message::Outcome::NA
            0,
            0,
            0,
            0,
            0x80  // Serial::Message::c_end_symbol
        };

        std::memcpy(expected_message_encoded.data() + sizeof(header) + payload_len, &footer, sizeof(footer));

        const uint16_t expected_crc = calculate_crc14(expected_message_encoded);
        const auto [expected_crc_lower, expected_crc_upper] = split_into_two_symbols(expected_crc);

        footer[1] = expected_crc_lower;
        footer[2] = expected_crc_upper;
        footer[3] = expected_crc_lower;
        footer[4] = expected_crc_upper;

        std::memcpy(expected_message_encoded.data() + sizeof(header) + payload_len, &footer, sizeof(footer));

        check(test_message_encoded == expected_message_encoded);
    }
}

/*
This test is meant for the constructor that takes a vector of bytes as argument.
*/
void
test_is_valid()
{
    // Passing test for size() < c_min_length
    {
        Status status{Status::NoError};
        Message message{0, 0, 1};
        check(message.is_valid(status));
        check(status == Status::NoError);
    }

    // Failing test for size() < c_min_length
    {
        Status status{Status::NoError};
        std::vector<uint8_t> raw_message(Serial::Message::c_min_length - 1);
        Message message{raw_message};
        check(!message.is_valid(status));
        check(status == Status::InvalidSize);
    }

    // Passing test for check_id_validity()
    {
        Status status{Status::NoError};
        for (uint8_t ep_id = 0; ep_id < 8; ++ep_id)
        {
            for (uint8_t trm16_id = 0; trm16_id < 64; ++trm16_id)
            {
                for (uint8_t cmd_id = 1; cmd_id < 30; ++cmd_id)
                {
                    Message message{ep_id, trm16_id, cmd_id};
                    check(message.is_valid(status));
                    check(status == Status::NoError);
                }
            }
        }
    }

    // Failing test for check_id_validity()
    {
        Status status{Status::NoError};
        {
            Message message{8, 0, 1};
            check(!message.is_valid(status));
            check(status == Status::InvalidEPID);
        }
        {
            Message message{0, 64, 1};
            check(!message.is_valid(status));
            check(status == Status::InvalidTRM16ID);
        }
        {
            Message message{0, 0, 0};
            check(!message.is_valid(status));
            check(status == Status::InvalidCmdID);
        }
        {
            Message message{0, 0, 30};
            check(!message.is_valid(status));
            check(status == Status::InvalidCmdID);
        }
    }

    // Passing test for start_symbol != c_start_symbol
    {
        Status status{Status::NoError};
        Message message{0, 0, 1};
        // Set up message with incorrect start symbol
        check(!message.is_valid(status));
        check(status == Status::InvalidStartSymbol);
    }

    // Failing test for start_symbol != c_start_symbol
    {
        Status status{Status::NoError};
        std::vector<uint8_t> raw_message(Serial::Message::c_min_length);
        raw_message[0] = Serial::Message::c_start_symbol + 1;
        Message message{raw_message};
        // Set up message with correct start symbol
        check(message.is_valid(status));
        check(status == Status::NoError);
    }

    // Passing test for payload_len == (size() - c_min_length)
    {
        Status status{Status::NoError};
        Message message{0, 0, 1, {1, 2, 3}};
        // Set up message with correct payload length
        check(message.is_valid(status));
        check(status == Status::NoError);
    }

    // Failing test for payload_len == (size() - c_min_length)
    {
        Status status{Status::NoError};
        Message message{0, 0, 1, {1, 2, 3}};
        {
            // Set up message with incorrect payload length
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[4] = 55;
            Message message{raw_message};
            check(!message.is_valid(status));
            check(status == Status::InvalidPayloadLength);
        }
        {
            // Set up message with incorrect payload length
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[5] = 0;
            Message message{raw_message};
            check(!message.is_valid(status));
            check(status == Status::InvalidPayloadLength);
        }
    }

    // Passing test for end_symbol != c_end_symbol
    {
        Status status{Status::NoError};
        Message message{0, 0, 1, {1, 2, 3}};
        // Set up message with correct end symbol
        check(message.is_valid(status));
        check(status == Status::NoError);
    }

    // Failing test for end_symbol != c_end_symbol
    {
        Status status{Status::NoError};
        Message message{0, 0, 1, {1, 2, 3}};
        {
            // Set up message with incorrect end symbol
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[raw_message.size() - 1] = Serial::Message::c_end_symbol + 1;
            Message message{raw_message};
            check(!message.is_valid(status));
            check(status == Status::InvalidEndSymbol);
        }
    }
}

void
test_check_integrity()
{
    // Skipping validity check.

    // Test EP ID mismatch.
    {
        Message message{0, 0, 1, {1, 2, 3}};
        {
            // Set up message with incorrect EP ID
            Status status = Status::NoError;
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[1] = 55;
            Message message{raw_message};
            check(!message.check_integrity(message, status));
            check(status == Status::InvalidEPID);
        }
    }

    // Test TRM16 ID mismatch.
    {
        Message message{0, 0, 1, {1, 2, 3}};
        {
            // Set up message with incorrect TRM16 ID
            Status status = Status::NoError;
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[2] = 55;
            Message message{raw_message};
            check(!message.check_integrity(message, status));
            check(status == Status::InvalidTRM16ID);
        }
    }

    // Test command ID mismatch.
    {
        Message message{0, 0, 1, {1, 2, 3}};
        {
            // Set up message with incorrect command ID
            Status status = Status::NoError;
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[3] = 55;
            Message message{raw_message};
            check(!message.check_integrity(message, status));
            check(status == Status::InvalidCmdID);
        }
    }

    // Test CRC mismatch.
    {
        Message message{0, 1, 1, {1, 2, 3}};
        {
            // Set up message with incorrect CRC
            Status status = Status::NoError;
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[raw_message.size() - 5] = 55;
            Message message{raw_message};
            check(!message.check_integrity(message, status));
            check(status == Status::CRCMismatch);
        }
    }

    // Passing test for check_crc()
    {
        Message message{0, 0, 1, {1, 2, 3}};
        {
            // Set up message with correct CRC
            Status status = Status::NoError;
            std::vector<uint8_t> raw_message = message.encode();
            Message message{raw_message};
            check(message.is_valid(status));
            check(status == Status::NoError);
        }
        {
            // Set up message with correct CRC, setting the CRC fields to 0.
            Status status = Status::NoError;
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[raw_message.size() - 2] = 0;
            raw_message[raw_message.size() - 1] = 0;
            Message message{raw_message};
            check(message.is_valid(status));
            check(status == Status::NoError);
        }
    }

    // Failing test for check_crc()
    {
        Message message{0, 0, 1, {1, 2, 3}};
        {
            // Set up message with incorrect CRC lower
            Status status = Status::NoError;
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[raw_message.size() - 5] = 55;
            Message message{raw_message};
            check(!message.is_valid(status));
            check(status == Status::CRCMismatch);
        }
        {
            // Set up message with incorrect CRC upper
            Status status = Status::NoError;
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[raw_message.size() - 4] = 55;
            Message message{raw_message};
            check(!message.is_valid(status));
            check(status == Status::CRCMismatch);
        }
        {
            // Set up message with incorrect CRC command lower
            Status status = Status::NoError;
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[raw_message.size() - 3] = 55;
            Message message{raw_message};
            check(!message.is_valid(status));
            check(status == Status::CRCMismatch);
        }
        {
            // Set up message with incorrect CRC command upper
            Status status = Status::NoError;
            std::vector<uint8_t> raw_message = message.encode();
            raw_message[raw_message.size() - 2] = 55;
            Message message{raw_message};
            check(!message.is_valid(status));
            check(status == Status::CRCMismatch);
        }
    }
}
