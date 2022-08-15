#include "gtest/gtest.h"
#include <memory>
#include "hans_cute_driver/hans_cute_driver.h"
#include "hans_cute_driver/hans_cute_const.h"
#include "serial_port/dummy_serial_port.h"

class ServoDriverTest : public ::testing::Test
{
protected:
  virtual void SetUp() override
  {
    std::string port = "/dev/ttyUSB0";
    unsigned int baud_rate = 250000;

    serial_port_ = std::make_shared<DummySerialPort>(port, baud_rate);
    hans_robot_.setSerialPort(serial_port_);
    hans_robot_.open();

    correct_status_packet_ = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
    corrupted_status_packet_ = {0xFA, 0xFB, 0x00};
  };

  virtual void TearDown() override{

  };

protected:
  std::shared_ptr<SerialPortInterface> serial_port_;
  HansCuteRobot::ServoDriver hans_robot_;

  std::vector<uint8_t> correct_status_packet_;
  std::vector<uint8_t> corrupted_status_packet_;
};

TEST_F(ServoDriverTest, test_calcChecksum)
{
  ASSERT_EQ(correct_status_packet_.back(), hans_robot_.calcCheckSum(correct_status_packet_));
}

TEST_F(ServoDriverTest, test_readResponseCorrectStatusPacket)
{
  serial_port_->setReadDataStream(correct_status_packet_);
  std::vector<uint8_t> response;
  bool status = hans_robot_.readResponse(response);
  ASSERT_TRUE(status);
  ASSERT_EQ(response, correct_status_packet_);
}

// TODO: Maybe it's better to move ground truth into individual tests
TEST_F(ServoDriverTest, test_readResponseCorruptedStatusPacket)
{
  serial_port_->setReadDataStream(corrupted_status_packet_);
  std::vector<uint8_t> response;
  bool status = hans_robot_.readResponse(response);
  ASSERT_FALSE(status);
}

TEST_F(ServoDriverTest, test_read)
{
  uint8_t id = 0x01;
  uint8_t address = 0x2B;
  uint8_t length = 0x01;
  std::vector<uint8_t> ground_truth_read_io_stream = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0xCC};
  std::vector<uint8_t> read_io_stream;
  
  std::vector<uint8_t> ground_truth_read_return_stream = {0xFF, 0xFF,0x01,0x03,0x00,0x20,0xDB};
  std::vector<uint8_t> read_return_stream;

  unsigned long timestamp;

  serial_port_->setReadDataStream(ground_truth_read_return_stream);
  bool status = hans_robot_.read(id, address, length, read_return_stream, timestamp);
  serial_port_->getWriteDataStream(read_io_stream);

  ASSERT_TRUE(status);
  ASSERT_EQ(read_io_stream, ground_truth_read_io_stream);
}

TEST_F(ServoDriverTest, test_write)
{
  uint8_t id = 0x01;
  uint8_t address = 0x03;
  std::vector<uint8_t> ground_truth_write = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x03, 0x00, 0xF4};
  std::vector<uint8_t> raw_data = {0x00};
  std::vector<uint8_t> write_io_stream;
  std::vector<uint8_t> write_return_stream;
  unsigned long timestamp;

  hans_robot_.write(id, address, raw_data, write_return_stream, timestamp);
  serial_port_->getWriteDataStream(write_io_stream);
  ASSERT_EQ(write_io_stream, ground_truth_write);
}

TEST_F(ServoDriverTest, test_ping)
{
  uint8_t id = 0x01;
  std::vector<uint8_t> ground_truth_ping = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  std::vector<uint8_t> ping_io_stream;
  std::vector<uint8_t> ping_return_stream;

  hans_robot_.ping(id,ping_return_stream);
  serial_port_->getWriteDataStream(ping_io_stream);
  ASSERT_EQ(ping_io_stream, ground_truth_ping);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}