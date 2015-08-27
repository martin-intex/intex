#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <cassert>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/types.h>
#include <sys/stat.h>

extern "C" {
#include <linux/spi/spidev.h>
}
#include "IntexHardware.h"

namespace intex {
namespace hw {

struct ADS1248::Impl {
public: 
 Impl() {}

  /*return true if device is present, false if communication is not possible*/
  bool selftest() {}
};


ADS1248::ADS1248() :
d(std::make_unique<Impl>())
 {
  /*Why can I not use the std::make_unique<Impl> way here?*/
//  d = new Impl(config, reset);
}

ADS1248::~ADS1248() {}

bool ADS1248::selftest() {

  d->selftest();
  std::cout << "Dont reach this line " << std::endl;
  return false;
}




} /*namespace hw*/
} /*namspace intex*/
#pragma clang diagnostic ignored "-Wundefined-reinterpret-cast"
#include "IntexHardware.moc"
//#include "moc_IntexHardware.cpp"
