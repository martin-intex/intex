#include <iostream>
#include <memory>

#include "IntexHardware.h"

struct ADS1248::Impl {
public: 
 Impl() {}

  /*return true if device is present, false if communication is not possible*/
  bool selftest() {}
};


ADS1248::ADS1248() :
d(new Impl())
 {
}

ADS1248::~ADS1248() {}

bool ADS1248::selftest() {

  d->selftest();
  std::cout << "Dont reach this line " << std::endl;
  return false;
}



