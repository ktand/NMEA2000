#include <catch.hpp>
#include <N2kMsg.h>

#include <cstdint>
#include <cstring>
#include <vector>

// tN2kMsg's bit-field access (read_value/write_value and the readers built
// on them), checked against a plain bit-by-bit reference: NMEA 2000 packs
// fields least significant bit first, byte after byte.

namespace
{
// A copy: k_max_data_len has no out-of-class definition, so it can't
// be bound by reference (as Catch's macros do).
constexpr int k_max_data_len = tN2kMsg::MaxDataLen;

// Bit `bit` of the data, LSB-first within each byte.
bool reference_bit(const unsigned char *data, unsigned bit)
{
  return (data[bit / 8] >> (bit % 8)) & 1;
}

uint64_t reference_read(const unsigned char *data, unsigned offset, unsigned width)
{
  uint64_t value = 0;
  for (unsigned i = 0; i < width; ++i)
  {
    value |= static_cast<uint64_t>(reference_bit(data, offset + i)) << i;
  }
  return value;
}

// A message whose data bytes are a fixed, irregular pattern.
tN2kMsg patterned_message(int length = k_max_data_len)
{
  tN2kMsg msg;
  msg.DataLen = length;
  for (int i = 0; i < k_max_data_len; ++i)
  {
    msg.Data[i] = static_cast<unsigned char>(i * 37 + 11);
  }
  return msg;
}

int64_t sign_extended(uint64_t value, unsigned width)
{
  if (width < 64 && (value >> (width - 1)) & 1)
  {
    value |= ~((uint64_t{1} << width) - 1);
  }
  return static_cast<int64_t>(value);
}
} // namespace

TEST_CASE("read_value matches the reference at every width and bit offset")
{
  const tN2kMsg msg = patterned_message();

  for (unsigned offset = 0; offset < 24; ++offset)
  {
    for (unsigned width = 1; width <= 8; ++width)
    {
      uint16_t index = offset;
      REQUIRE(msg.read_value<uint8_t>(index, width) == reference_read(msg.Data, offset, width));
      REQUIRE(index == offset + width);
      index = offset;
      REQUIRE(msg.read_value<int8_t>(index, width) == sign_extended(reference_read(msg.Data, offset, width), width));
    }
    for (unsigned width = 1; width <= 16; ++width)
    {
      uint16_t index = offset;
      REQUIRE(msg.read_value<uint16_t>(index, width) == reference_read(msg.Data, offset, width));
      index = offset;
      REQUIRE(msg.read_value<int16_t>(index, width) == sign_extended(reference_read(msg.Data, offset, width), width));
    }
    for (unsigned width = 1; width <= 32; ++width)
    {
      uint16_t index = offset;
      REQUIRE(msg.read_value<uint32_t>(index, width) == reference_read(msg.Data, offset, width));
      index = offset;
      REQUIRE(msg.read_value<int32_t>(index, width) == sign_extended(reference_read(msg.Data, offset, width), width));
    }
    for (unsigned width = 1; width <= 64; ++width)
    {
      uint16_t index = offset;
      REQUIRE(msg.read_value<uint64_t>(index, width) == reference_read(msg.Data, offset, width));
      index = offset;
      REQUIRE(msg.read_value<int64_t>(index, width) == sign_extended(reference_read(msg.Data, offset, width), width));
    }
  }
}

TEST_CASE("read_value of known fields")
{
  tN2kMsg msg;
  const unsigned char bytes[] = {0x34, 0x12, 0x00, 0x08, 0xab};
  std::memcpy(msg.Data, bytes, sizeof(bytes));
  msg.DataLen = sizeof(bytes);

  uint16_t index = 0;
  CHECK(msg.read_value<uint16_t>(index, 16) == 0x1234); // little-endian
  index = 16;
  CHECK(msg.read_value<int16_t>(index, 12) == -2048); // 0x800 as 12-bit two's complement
  index = 36;
  CHECK(msg.read_value<uint8_t>(index, 4) == 0x0a); // high nibble of 0xab
}

TEST_CASE("write_value puts the bits where read_value finds them, and leaves the others")
{
  for (unsigned offset = 0; offset < 16; ++offset)
  {
    for (unsigned width = 1; width <= 64; ++width)
    {
      tN2kMsg msg = patterned_message();
      const tN2kMsg before = msg;
      const uint64_t value = 0x9e3779b97f4a7c15ull >> (64 - width); // irregular bits

      uint16_t index = offset;
      msg.write_value<uint64_t>(index, width, value);
      REQUIRE(index == offset + width);
      REQUIRE(reference_read(msg.Data, offset, width) == value);

      for (unsigned bit = 0; bit < k_max_data_len * 8; ++bit)
      {
        if (bit < offset || bit >= offset + width)
        {
          REQUIRE(reference_bit(msg.Data, bit) == reference_bit(before.Data, bit));
        }
      }
    }
  }
}

TEST_CASE("write_value of a signed value and of an enum")
{
  enum class mode : uint8_t
  {
    off = 0,
    standby = 5,
  };
  tN2kMsg msg;
  uint16_t index = 3;
  msg.write_value<int16_t>(index, 10, -3);
  msg.write_value<mode>(index, 3, mode::standby);
  CHECK(msg.DataLen == 2);

  index = 3;
  CHECK(msg.read_value<int16_t>(index, 10) == -3);
  CHECK(msg.read_value<mode>(index, 3) == mode::standby);
}

TEST_CASE("fields past the received data read as not available")
{
  tN2kMsg msg = patterned_message(4);

  uint16_t index = 24; // the last byte, then three missing ones
  CHECK(msg.read_value<uint32_t>(index, 32) == (0xffffff00u | msg.Data[3]));
  index = 40;
  CHECK(msg.read_number<uint8_t>(index, 8, 7) == 7); // all ones: the default
}

TEST_CASE("the last bytes of a full message: nothing read or written past the buffer")
{
  tN2kMsg msg = patterned_message();
  msg.MsgTime = 0x12345678;

  // 32 bits from 4 bits before the end: 4 real bits, then not available.
  uint16_t index = k_max_data_len * 8 - 4;
  CHECK(msg.read_value<uint32_t>(index, 32) == (0xfffffff0u | (msg.Data[k_max_data_len - 1] >> 4)));

  index = k_max_data_len * 8 - 4;
  msg.write_value<uint64_t>(index, 64, 0);
  CHECK((msg.Data[k_max_data_len - 1] >> 4) == 0);
  CHECK(msg.MsgTime == 0x12345678u);
  CHECK(msg.DataLen == k_max_data_len);
}

TEST_CASE("read_float reads 32 bits")
{
  tN2kMsg msg;
  const float value = 1.5f;
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  uint16_t index = 0;
  msg.write_value<uint32_t>(index, 32, bits);
  msg.write_value<uint8_t>(index, 8, 0x42);

  index = 0;
  CHECK(msg.read_float(index) == 1.5f);
  CHECK(index == 32);
  CHECK(msg.read_value<uint8_t>(index, 8) == 0x42);

  index = 0;
  msg.write_value<uint32_t>(index, 32, 0x7fffffffu); // not available
  index = 0;
  CHECK(msg.read_float(index, -1.0f) == -1.0f);
}

TEST_CASE("read_string: fixed width, ends at NUL, '@' or 0xff, and at the data's end")
{
  tN2kMsg msg;
  const char text[] = "ABC@@@HELLO";
  std::memcpy(msg.Data, text, 11);
  msg.DataLen = 9;

  uint16_t index = 0;
  CHECK(msg.read_string(index, 48) == "ABC");
  CHECK(index == 48);
  CHECK(msg.read_string(index, 40) == "HEL"); // 5 bytes wanted, 3 received
  CHECK(index == 88);
}

TEST_CASE("read_string: STRING_LAU's length counts its two header bytes")
{
  tN2kMsg msg;
  const unsigned char bytes[] = {5, 1, 'a', 'b', 'c', 0x42, 2, 1, 0x43};
  std::memcpy(msg.Data, bytes, sizeof(bytes));
  msg.DataLen = sizeof(bytes);

  uint16_t index = 0;
  CHECK(msg.read_string(index) == "abc");
  CHECK(index == 40);
  CHECK(msg.read_value<uint8_t>(index, 8) == 0x42);
  CHECK(msg.read_string(index) == ""); // empty: header only
  CHECK(msg.read_value<uint8_t>(index, 8) == 0x43);

  index = 16; // 'a' as a length: no valid header, skip the rest
  msg.Data[3] = 7; // encoding 7
  CHECK(msg.read_string(index) == "");
  CHECK(index == msg.DataLen * 8);
}
