#include <catch.hpp>
#include <NMEA2000.h>
#include <N2kMessages.h>

#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

// tNMEA2000::SetSentMsgHandler(): every message SendMsg() accepts reaches
// the handler once, whole and with its source set; refused ones don't.

namespace
{
// The library's own clock (N2kTimer.cpp: the monotonic clock on Linux):
// opening waits 200 ms, an address claim 250 ms.
void wait_a_little()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// A node on a bus nobody else is on: frames sent are kept, none arrive.
class test_node : public tNMEA2000
{
public:
  std::vector<unsigned long> sent_frame_ids;

protected:
  bool CANSendFrame(unsigned long id, unsigned char, const unsigned char *, bool) override
  {
    sent_frame_ids.push_back(id);
    return true;
  }
  bool CANOpen() override { return true; }
  bool CANGetFrame(unsigned long &, unsigned char &, unsigned char *) override { return false; }
};

struct sent_log
{
  std::vector<tN2kMsg> messages;
};

void record_sent(const tN2kMsg &message, void *context)
{
  static_cast<sent_log *>(context)->messages.push_back(message);
}

// Opens the node (it waits for its open scheduler).
bool open(test_node &node)
{
  for (int step = 0; step < 100; ++step)
  {
    wait_a_little();
    if (node.Open())
    {
      return true;
    }
  }
  return false;
}

// Opens the node and lets its address claim finish (250 ms of no
// objection).
void open_node(test_node &node)
{
  node.SetMode(tNMEA2000::N2km_NodeOnly, 22);
  REQUIRE(open(node));
  for (int step = 0; step < 100; ++step)
  {
    wait_a_little();
    node.ParseMessages();
  }
}

size_t count_pgn(const sent_log &log, unsigned long pgn)
{
  size_t count = 0;
  for (const auto &message : log.messages)
  {
    count += message.PGN == pgn ? 1 : 0;
  }
  return count;
}
} // namespace

TEST_CASE("The sent message handler gets every message the node sends")
{
  test_node node;
  sent_log log;
  node.SetSentMsgHandler(record_sent, &log);
  open_node(node);

  // The library's own address claim went through SendMsg too.
  CHECK(count_pgn(log, 60928UL) >= 1);
  log.messages.clear();

  SECTION("single frame, with the node's source address")
  {
    tN2kMsg message;
    SetN2kRateOfTurn(message, 1, 0.01);
    REQUIRE(node.SendMsg(message));
    REQUIRE(log.messages.size() == 1);
    CHECK(log.messages[0].PGN == 127251UL);
    CHECK(log.messages[0].Source == 22);
    CHECK(log.messages[0].DataLen == message.DataLen);
  }

  SECTION("fast packet: once, whole")
  {
    tN2kMsg message;
    SetN2kNavigationInfo(message, 1, 1000, N2khr_true, false, false, N2kdct_GreatCircle, 0, 0, 0.5, 0.5, 1, 2,
                         59.0, 18.0, 2.5);
    const size_t frames_before = node.sent_frame_ids.size();
    REQUIRE(node.SendMsg(message));
    CHECK(node.sent_frame_ids.size() - frames_before > 1);
    REQUIRE(log.messages.size() == 1);
    CHECK(log.messages[0].PGN == 129284UL);
    CHECK(log.messages[0].DataLen == message.DataLen);
    for (int index = 0; index < message.DataLen; ++index)
    {
      CHECK(log.messages[0].Data[index] == message.Data[index]);
    }
  }

  SECTION("removed: no more calls")
  {
    node.SetSentMsgHandler(nullptr);
    tN2kMsg message;
    SetN2kRateOfTurn(message, 1, 0.01);
    REQUIRE(node.SendMsg(message));
    CHECK(log.messages.empty());
  }
}

TEST_CASE("A message SendMsg refuses doesn't reach the sent message handler")
{
  test_node node;
  sent_log log;
  node.SetSentMsgHandler(record_sent, &log);
  node.SetMode(tNMEA2000::N2km_ListenOnly, 22);
  REQUIRE(open(node));
  tN2kMsg message;
  SetN2kRateOfTurn(message, 1, 0.01);
  CHECK_FALSE(node.SendMsg(message));
  CHECK(log.messages.empty());
}
