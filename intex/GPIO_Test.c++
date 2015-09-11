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
  class ::intex::hw::BurnWire BurnWire(intex::hw::config::burnwire);
  class ::intex::hw::Valve Valve0(intex::hw::config::valve0);
  class ::intex::hw::Valve Valve1(intex::hw::config::valve1);
  class ::intex::hw::ADS1248 ADS1248(intex::hw::config::ads1248,intex::hw::config::ads1248_reset);

  class ::intex::hw::HSCSensors PressureSensor_1(intex::hw::config::hscxxxn1bar6sa5_1,true);
  class ::intex::hw::HSCSensors PressureSensor_2(intex::hw::config::hscxxxn1bar6sa5_2,false);
  class ::intex::hw::HSCSensors PressureSensor_3(intex::hw::config::hscxxxn12barsa5_1,true);

  //sensor 2 high Pressure
  //

  Valve0.set(false);
  Valve1.set(false);

  while(true){
/* */
    std::cout<<"mittelwert 0   ->"<<double(ADS1248.selftest(0))<<std::endl;
    std::cout<<"##############################################"<<std::endl;

    std::cout<<"mittelwert 1   ->"<<double(ADS1248.selftest(1))<<std::endl;
    std::cout<<"##############################################"<<std::endl;

    std::cout<<"mittelwert 2   ->"<<double(ADS1248.selftest(2))<<std::endl;
    std::cout<<"##############################################"<<std::endl;





 std::cout << "Pressure Sensor 1: " << PressureSensor_1.get_pressure() << endl;
 std::cout << "Pressure Sensor 2: " << PressureSensor_2.get_pressure() << endl;
 std::cout << "Pressure Sensor 3: " << PressureSensor_3.get_pressure() << endl;
}
/*std::cout << "Turn Burnwire on ... burning" << std::flush;
  BurnWire.actuate();
  std::cout << "done" << std::endl;


  std::cout << "Valve 0 on ... " << std::flush;
  Valve0.set(true);
  std::this_thread::sleep_for(std::chrono::seconds(15));
  Valve0.set(false);
  std::cout << "off" << std::endl;

  while(1)
  {
  std::this_thread::sleep_for(std::chrono::seconds(10));
  Valve0.set(true);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  Valve0.set(false);
  }

  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::cout << "Valve 1 on ... " << std::flush;
  Valve1.set(true);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  Valve1.set(false);
  std::cout << "off" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(3));
/*
  std::cout << "Turn Burnwire on ... burning" << std::flush;
  BurnWire.actuate();
  std::cout << "done" << std::endl;
*/


  return 0;
}
