#include "gtest/gtest.h"

#include "hans_cute_driver/hans_cute_driver.h"
#include "hans_cute_driver/hans_cute_const.h"
#include "custom_serial_port/dummy_serial_port.h"

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

    ping_return_data_ = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
  };

  virtual void TearDown() override{

  };

protected:
  std::shared_ptr<SerialPortInterface> serial_port_;
  HansCuteRobot::ServoDriver hans_robot_;

  std::vector<uint8_t> ping_return_data_;
};

TEST_F(ServoDriverTest, test_calcChecksum)
{
  ASSERT_EQ(ping_return_data_.back(), hans_robot_.calcCheckSum(ping_return_data_));
}

TEST_F(ServoDriverTest, test_write)
{
  std::vector<uint8_t> ground_truth_write = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x03, 0x00, 0xF4};
  std::vector<uint8_t> raw_data = {0x00};
  std::vector<uint8_t> write_data;
  unsigned long timestamp;

  hans_robot_.write(0x01, 0x03, raw_data, timestamp);
  serial_port_->getWriteDataStream(write_data);
  EXPECT_EQ(write_data, ground_truth_write);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}