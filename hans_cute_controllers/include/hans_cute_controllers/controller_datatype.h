#ifndef _CONTROLLER_DATATYPE_H_
#define _CONTROLLER_DATATYPE_H_

#include <mutex>
#include <vector>

namespace HansCuteController
{
  class Data
  {
  public:
    Data(){};
    virtual ~Data(){};

    virtual void get(std::vector<double> &data)
    {
      std::unique_lock<std::mutex> lock(mtx_);
      lock.lock();
      data = data_;
    }

    virtual void set(const std::vector<double> &data)
    {
      std::unique_lock<std::mutex> lock(mtx_);
      lock.lock();
      data_ = data;
    }

  protected:
    std::mutex mtx_;
    std::vector<double> data_;
  };

  struct JointParams
  {
    // Raw data
    unsigned int raw_origin;
    unsigned int raw_min;
    unsigned int raw_max;
    static const unsigned int raw_full_sweep = 4096;

    // Radian
    double min;
    double max;

    // Utils
    double rad_per_enc_tick;
    double enc_tick_per_rad;
  };
};

#endif