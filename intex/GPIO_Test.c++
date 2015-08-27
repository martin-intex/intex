#include <QDebug>
#include <QTimer>
#include <QString>

#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono> // std::chrono::seconds
#include <thread> // std::this_thread::sleep_for

#include "IntexHardware.h"
int main() {
  class ::intex::hw::ADS1248 ADS1248(intex::hw::config::ads1248,intex::hw::config::ads1248_reset);

 ADS1248.selftest();
 std::cout << "Selftest done main " << std::flush;

  return 0;
}
