#include <iostream>
#include <memory>

#include "IntexHardware.h"
int main() {
  class ADS1248 ads1248;

 ads1248.selftest();
 std::cout << "Selftest done main " << std::flush;

  return 0;
}
