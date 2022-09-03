#include "gtest/gtest.h"
#include <memory>
#include "hans_cute_driver/hans_cute_driver.h"
#include "hans_cute_driver/hans_cute_const.h"
#include "serial_port/dummy_serial_port.h"

class ServoSerialTest : public ::testing::Test
{
protected:
  virtual void SetUp() override
  {
    std::string port = "/dev/ttyUSB0";
    unsigned int baud_rate = 250000;

    serial_port_ = std::make_shared<DummySerialPort>(port, baud_rate);
    hans_serial_comms_.setSerialPort(serial_port_);
    hans_serial_comms_.open();

    hans_serial_comms_.setSamplePacket(HansCuteRobot::ServoDriver::DXL_PACKET);
  };

  virtual void TearDown() override{

  };

protected:
  std::shared_ptr<SerialPortInterface> serial_port_;
  HansCuteRobot::ServoSerialComms hans_serial_comms_;
};

TEST_F(ServoSerialTest, test_calcChecksum)
{
  std::vector<uint8_t> correct_status_packet = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
  ASSERT_EQ(correct_status_packet.back(), hans_serial_comms_.calcCheckSum(correct_status_packet));
}

TEST_F(ServoSerialTest, test_readResponseCorrectStatusPacket)
{
  std::vector<uint8_t> correct_status_packet = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
  serial_port_->setReadDataStream(correct_status_packet);
  std::vector<uint8_t> response;
  int status = hans_serial_comms_.readResponse(response);
  ASSERT_TRUE(status == 0);
  ASSERT_EQ(response, correct_status_packet);
}

TEST_F(ServoSerialTest, test_readResponseStatusPacketWrongHeader)
{
  std::vector<uint8_t> wrong_header_packet = {0xFA, 0xFB, 0x00};
  serial_port_->setReadDataStream(wrong_header_packet);
  std::vector<uint8_t> response;
  int status = hans_serial_comms_.readResponse(response);
  ASSERT_TRUE(status == (int)SerialCommandError::WRONG_HEADER);
}

TEST_F(ServoSerialTest, test_readResponseStatusPacketNoResponse)
{
  std::vector<uint8_t> wrong_header_packet;
  serial_port_->setReadDataStream(wrong_header_packet);
  std::vector<uint8_t> response;
  int status = hans_serial_comms_.readResponse(response);
  ASSERT_TRUE(status == (int)SerialCommandError::NO_RESPONSE);
}

TEST_F(ServoSerialTest, test_readResponseStatusPacketWrongCheckSum)
{
  std::vector<uint8_t> wrong_header_packet = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFA};
  serial_port_->setReadDataStream(wrong_header_packet);
  std::vector<uint8_t> response;
  int status = hans_serial_comms_.readResponse(response);
  ASSERT_TRUE(status == (int)SerialCommandError::WRONG_CHECKSUM);
}

TEST_F(ServoSerialTest, test_read)
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
  bool status = hans_serial_comms_.read(id, address, length, read_return_stream, timestamp);
  serial_port_->getWriteDataStream(read_io_stream);

  ASSERT_TRUE(status);
  ASSERT_EQ(read_io_stream, ground_truth_read_io_stream);
}

TEST_F(ServoSerialTest, test_write)
{
  uint8_t id = 0x01;
  uint8_t address = 0x03;
  std::vector<uint8_t> ground_truth_write = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x03, 0x00, 0xF4};
  std::vector<uint8_t> raw_data = {0x00};
  std::vector<uint8_t> write_io_stream;
  std::vector<uint8_t> write_return_stream;
  unsigned long timestamp;

  hans_serial_comms_.write(id, address, raw_data, write_return_stream, timestamp);
  serial_port_->getWriteDataStream(write_io_stream);
  ASSERT_EQ(write_io_stream, ground_truth_write);
}

TEST_F(ServoSerialTest, test_ping)
{
  uint8_t id = 0x01;
  std::vector<uint8_t> ground_truth_ping = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  std::vector<uint8_t> ping_io_stream;
  std::vector<uint8_t> ping_return_stream;

  hans_serial_comms_.ping(id,ping_return_stream);
  serial_port_->getWriteDataStream(ping_io_stream);
  ASSERT_EQ(ping_io_stream, ground_truth_ping);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}