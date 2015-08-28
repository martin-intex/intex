#pragma once

#include <memory>

#include <QObject>
#include <QString>

namespace intex {
namespace hw {

namespace config {

/*
 * GPIO14 BURNWIRE1_EN
 * GPIO15 AUX1_24V_CTRL
 * GPIO18 ADS1248_CS0
 * GPIO23 ADS1248_CS1
 * GPIO24 RTC_CS
 * GPIO21 WD_INPUT
 * GPIO04 PRESSURE_CS0
 * GPIO17 PRESSURE_CS1
 * GPIO27 PRESSURE_CS2
 * GPIO10 SPI_MOSI
 * GPIO09 SPI_MISO
 * GPIO11 SPI_CLK
 * GPIO05 VALVE1_OPEN
 * GPIO06 VAVLE2_OPEN
 * GPIO13 VALVE3_OPEN
 * GPIO19 HEATER1_EN
 * GPIO26 HEATER2_EN
 */

struct gpio {
  enum class direction { in, out };
  int pinno;
  const char *const name;
  enum direction direction;
  bool active_low;
};


struct spi {
  uint32_t speed; /*bits per second*/
  const char *const device; /*device file*/
  const char *const name; /*Human readable description to device*/
  uint8_t bpw; /*bits per word*/
  uint16_t delay;
  bool loopback;
  bool cpha; /*clock phase*/
  bool cpol; /*clock polarity*/
  bool lsb_first; /*least significant bit first*/
  bool cs_high; /*chip select active high*/
  bool threewire; /*SI/SO signals shared*/
  bool no_cs; /*no internal chip select control*/
  const gpio &cs_pin; /*pin used for chip select*/
};

static constexpr gpio valve0{5, "VALVE1", gpio::direction::out, false};
static constexpr gpio valve1{6, "VALVE2", gpio::direction::out, false};
static constexpr gpio ads1248_cs{18, "ADS1248_CS", gpio::direction::out, true};
static constexpr gpio ads1248_reset{23, "ADS1248_CS", gpio::direction::out, true};
static constexpr gpio burnwire{14, "BURNWIRE", gpio::direction::out, false};

static constexpr spi ads1248{50000, "/dev/spidev0.0", "ADS1248 Temperature ADC", 8, 0, false, false, true, false, false, false, true, ads1248_cs};

}

class Valve : public QObject {
  Q_OBJECT

  struct Impl;
  std::unique_ptr<Impl> d;

public:
  Valve(const config::gpio &config);
  ~Valve();

  void set(const bool state);

  // clang-format off
Q_SIGNALS:
  void log(QString msg);
  // clang-format on
};

class Heater : public QObject {
  Q_OBJECT

  struct Impl;
  std::unique_ptr<Impl> d;

public:
  /* add timeout on temperatureChanged */
  Heater(const config::gpio &config, int low, int high);
  ~Heater();

  void set(const bool on);
  void temperatureChanged(int temperature);
};

class BurnWire : public QObject {
  Q_OBJECT

  struct Impl;
  std::unique_ptr<Impl> d;

public:
  /* add timeout on temperatureChanged */
  BurnWire(const config::gpio &config);
  ~BurnWire();

  void actuate();
};

class ADS1248 : public QObject {
  Q_OBJECT

  struct Impl;
  Impl *d;

  public:
  ADS1248(const config::spi &config, const config::gpio &reset);
  bool selftest();


};

}
}
