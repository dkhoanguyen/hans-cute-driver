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
      data = data_;
    }

    virtual void set(const std::vector<double> &data)
    {
      std::unique_lock<std::mutex> lock(mtx_);
      data_ = data;
    }

  protected:
    std::mutex mtx_;
    std::vector<double> data_;
  };
};

#endif