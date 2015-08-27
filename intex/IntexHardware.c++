#include <QDebug>
#include <QTimer>
#include <QString>
#include <QThread>

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

using namespace std::chrono;
using namespace std::literals::chrono_literals;

namespace intex {
namespace hw {

static constexpr int retries = 3;

class gpio {
public:
  enum class attribute { active_low, direction, edge, value };
  gpio(const config::gpio &config);
  gpio(const gpio &) = delete;
  gpio(gpio &&) = default;
  gpio &operator=(const gpio &) = delete;
  gpio &operator=(gpio &&) = default;

  void init();

  void on();
  void off();
  bool isOn();

private:
  config::gpio config_;
  void set(const bool on);
  bool isinitialized;
};

static const char *to_string(const enum config::gpio::direction &direction) {
  switch (direction) {
  case config::gpio::direction::in:
    return "in";
  case config::gpio::direction::out:
    return "out";
  }
}

static std::ostream &operator<<(std::ostream &os,
                                const enum config::gpio::direction &direction) {
  return os << to_string(direction);
}

static QDebug &operator<<(QDebug &os,
                          const enum config::gpio::direction &direction) {
  return os << to_string(direction);
}

static std::ostream &operator<<(std::ostream &os,
                                const gpio::attribute attribute) {
  switch (attribute) {
  case gpio::attribute::active_low:
    return os << "active_low";
  case gpio::attribute::direction:
    return os << "direction";
  case gpio::attribute::edge:
    return os << "edge";
  case gpio::attribute::value:
    return os << "value";
  }
}
static bool is_directory_present(const char *path) {
  struct stat st;
  if (stat(path, &st) == 0)
    return S_ISDIR(st.st_mode);
  else
    return false;
}

static void export_pin(int pin) {
  std::ofstream export_;
  /*Directory name for the gpio controll*/
  std::stringstream dname;
  dname << "/sys/class/gpio/gpio" << pin;
  /*Check if already present*/
  if (!is_directory_present(dname.str().c_str())) {
    export_.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    export_.open("/sys/class/gpio/export");
    export_ << pin << std::endl;
  }
}

static void sysfs_file(std::fstream &file, const gpio::attribute attr,
                       const int pin, const std::ios_base::openmode mode) {
  file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  std::ostringstream fname;
  fname << "/sys/class/gpio/gpio" << pin << "/" << attr;

  file.open(fname.str().c_str(), mode);
}

template <typename T>
static void set_attribute(const gpio::attribute attr, const int pin,
                          const T value) {
  std::fstream file;
  sysfs_file(file, attr, pin, std::ios_base::out);
  file << value << std::endl;
}

template <typename T>
static int get_attribute(const gpio::attribute attr, const int pin) {
  T value;
  std::fstream file;
  sysfs_file(file, attr, pin, std::ios_base::in);
  file >> value;
  return value;
}

gpio::gpio(const config::gpio &config)
    : config_(config), isinitialized(false) {}

void gpio::init() {
  std::cout << "Configuring GPIO " << config_.name << " (" << config_.pinno
            << ") as " << config_.direction
            << (config_.active_low ? "(active low)" : "") << "." << std::endl;

  export_pin(config_.pinno);
  isinitialized = true;
  set_attribute(attribute::active_low, config_.pinno, config_.active_low);
  set_attribute(attribute::direction, config_.pinno, config_.direction);
}

void gpio::on() { set(true); }
void gpio::off() { set(false); }
bool gpio::isOn() {
  assert(isinitialized);
  return get_attribute<int>(attribute::value, config_.pinno);
}

void gpio::set(const bool on) {

  assert(isinitialized);
  for (int retry = 0; retry < retries; ++retry) {
    set_attribute(attribute::value, config_.pinno, static_cast<int>(on));
    if (isOn() == on)
      return;
  }

  throw std::runtime_error("Could not set pin");
}

class debug_gpio {
public:
  debug_gpio(const config::gpio &config)
      : name_(config.name), pin_(config.pinno), direction_(config.direction),
        active_low_(config.active_low), state(false) {}
  debug_gpio(const debug_gpio &) = delete;
  debug_gpio(debug_gpio &&) = default;
  debug_gpio &operator=(const debug_gpio &) = delete;
  debug_gpio &operator=(debug_gpio &&) = default;

  void init() {
    qDebug() << "Initializing pin" << name_ << "(" << pin_ << ") as"
             << direction_ << (active_low_ ? "(active_low)" : "");
  }

  void on() { set(true); }
  void off() { set(false); }
  bool isOn() {
    qDebug() << "Reading pin" << name_ << "(" << pin_ << ") as" << state;
    return state;
  }

private:
  void set(const bool on) {
    state = on;
    qDebug() << "Setting pin" << name_ << "(" << pin_ << ")" << state;
  }
  QString name_;
  int pin_;
  enum config::gpio::direction direction_;
  bool active_low_;
  bool state;
};

#pragma clang diagnostic ignored "-Wweak-vtables"

class PWM : public QObject {
  Q_OBJECT

  milliseconds period_;
  float duty_;
  QTimer timer;
  bool state_;

  void cycle() {
    if (state_) {
      Q_EMIT on();
    } else {
      Q_EMIT off();
    }
    const auto factor = state_ ? duty_ : 1.0 - duty_;
    const auto timeout = period_ * factor;
    state_ = !state_;
    timer.setInterval(static_cast<int>(timeout.count()));
  }

public:
  template <class Rep, class Period>
  PWM(duration<Rep, Period> period, float duty)
      : period_(duration_cast<milliseconds>(period)), duty_(duty),
        state_(true) {
    connect(&timer, &QTimer::timeout, this, &PWM::cycle);
  }

  void setDuty(float duty) { duty_ = duty; }
  void start() {
    Q_EMIT on();
    timer.start();
  }
  void stop() {
    timer.stop();
    Q_EMIT off();
  }

  // clang-format off
Q_SIGNALS:
  void on();
  void off();
  // clang-format on
};

class GPIO : public QObject {
  Q_OBJECT

public:
  template <typename T>
  GPIO(T &&backend)
      : model_(std::make_unique<gpio_model<T>>(std::move(backend))) {}
  void init() {
    try {
      model_->init();
    } catch (const std::exception &e) {
      Q_EMIT log(QString::fromStdString(e.what()));
    }
  }
  void on() {
    try {
      model_->on();
    } catch (const std::exception &e) {
      Q_EMIT log(QString::fromStdString(e.what()));
    }
  }
  void off() {
    try {
      model_->off();
    } catch (const std::exception &e) {
      Q_EMIT log(QString::fromStdString(e.what()));
    }
  }

  // clang-format off
Q_SIGNALS:
  void log(QString);
  // clang-format on

private:
  struct gpio_concept {
    virtual ~gpio_concept() = default;
    virtual void init() = 0;
    virtual void on() = 0;
    virtual void off() = 0;
  };

  template <typename T> struct gpio_model final : gpio_concept {
    T backend_;
    gpio_model(T &&backend) : backend_(std::move(backend)) {}
    void init() override { backend_.init(); }
    void on() override { backend_.on(); }
    void off() override { backend_.off(); }
  };

  std::unique_ptr<gpio_concept> model_;
};

struct Valve::Impl {
  PWM pwm;
  GPIO pin_;
  Impl(const config::gpio &config)
      : pwm(2s, 0.1f),
#ifdef BUILD_ON_RASPBERRY
        pin_(::intex::hw::gpio(config))
#else
        pin_(::intex::hw::debug_gpio(config))
#endif
  {
    QObject::connect(&pwm, &PWM::on, &pin_, &GPIO::on);
    QObject::connect(&pwm, &PWM::off, &pin_, &GPIO::off);
  }
};

Valve::Valve(const config::gpio &config) : d(std::make_unique<Impl>(config)) {
  connect(&d->pin_, &GPIO::log, this, &Valve::log);
  d->pin_.init();
}
Valve::~Valve() = default;

void Valve::set(const bool state) {
  if (state)
    d->pwm.start();
  else
    d->pwm.stop();
}

struct Heater::Impl {
  PWM pwm;
  GPIO pin;
  QTimer timer;
  int low_;
  int high_;
  milliseconds timeout_;

public:
  template <class Rep, class Period>
  Impl(const config::gpio &config, int low, int high,
       duration<Rep, Period> timeout)
      : pwm(2s, 0.1f),
#ifdef BUILD_ON_RASPBERRY
        pin(::intex::hw::gpio(config)),
#else
        pin(::intex::hw::debug_gpio(config)),
#endif
        low_(low), high_(high), timeout_(duration_cast<milliseconds>(timeout)) {
    QObject::connect(&pwm, &PWM::on, &pin, &GPIO::on);
    QObject::connect(&pwm, &PWM::off, &pin, &GPIO::off);
    QObject::connect(&timer, &QTimer::timeout, [this]() { stop(); });
  }

  void start() {
    pwm.start();
    timer.start(static_cast<int>(timeout_.count()));
  }
  void stop() {
    pwm.stop();
    timer.stop();
  }
  void temperatureChanged(int temperature) {
    if (temperature < low_) {
      qDebug() << "Low setpoint (" << low_ << ") reached (" << temperature
               << ").";
      start();
    } else if (temperature > high_)
      qDebug() << "High setpoint (" << high_ << ") reached (" << temperature
               << ").";
    stop();
  }
};

Heater::Heater(const config::gpio &config, int low, int high)
    : d(std::make_unique<Impl>(config, low, high, 10s)) {}
Heater::~Heater() = default;

void Heater::set(const bool state) {
  if (state)
    d->start();
  else
    d->stop();
}

void Heater::temperatureChanged(int temperature) {
  d->temperatureChanged(temperature);
}

struct BurnWire::Impl {
  GPIO pin;

public:
  Impl(const config::gpio &config)
      :
#ifdef BUILD_ON_RASPBERRY
        pin(::intex::hw::gpio(config))
#else
        pin(::intex::hw::debug_gpio(config))
#endif
  {
  }

  void on() { pin.on(); }
  void off() { pin.off(); }
};

BurnWire::BurnWire(const config::gpio &config)
    : d(std::make_unique<Impl>(config)) {
  d->pin.init();
}
BurnWire::~BurnWire() = default;

void BurnWire::actuate() {
  d->on();
  /*XXX: please replace with an appropriate qt timer and remove #include
   * <unistd.h>*/
  sleep(3);
  d->off();
}

class spi {

public:
  static spi &get_instance(const config::spi &config) {
/*Todo - make some fancy stuff to create a new device, that has not been seen
before and store it
in a accosicative array*/

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wexit-time-destructors"
    static class spi spidev00("/dev/spidev0.0");
    static class spi spidev01("/dev/spidev0.1");
#pragma clang diagnostic pop

    if (strcmp(config.device, "/dev/spidev0.0") == 0)
      return spidev00;
    if (strcmp(config.device, "/dev/spidev0.1") == 0)
      return spidev01;

    throw std::runtime_error("Invalide SPI device");
  }

  void configure(config::spi &config, GPIO &cs_pin) {
    /*Bitmask SPI mode for ioctl*/
    uint32_t mode = 0;
    /*bits per word and speed readback*/
    uint32_t speed;
    uint8_t bpw;
    /*C call return value*/
    int ret;

    if (config.loopback)
      mode |= SPI_LOOP;
    if (config.cpha)
      mode |= SPI_CPHA;
    if (config.cpol)
      mode |= SPI_CPOL;
    if (config.lsb_first)
      mode |= SPI_LSB_FIRST;
    if (config.cs_high)
      mode |= SPI_CS_HIGH;
    if (config.threewire)
      mode |= SPI_3WIRE;
    if (config.no_cs)
      mode |= SPI_NO_CS;

    ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
    if (ret == -1)
      throw std::runtime_error("Could not set SPI mode");

    ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
    if (ret == -1)
      throw std::runtime_error("Could not get SPI mode");
    qDebug() << "Read back SPI config: " << mode;

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &config.bpw);
    if (ret == -1)
      throw std::runtime_error("Could not set SPI bits per word");

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bpw);
    if (ret == -1)
      throw std::runtime_error("Could not get SPI bits per word");
    qDebug() << "Read back SPI (fd: " << fd << ") BPW" << bpw << "was ("
             << config.bpw << ")";
    assert(bpw == config.bpw);

    /*
     * max speed hz
     */
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &config.speed);
    if (ret == -1)
      throw std::runtime_error("Could not set SPI speed");

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
      throw std::runtime_error("Could not get SPI speed");
    qDebug() << "Set SPI speed (fd: " << fd << ") from " << config.name
             << " to " << speed;

    /*We still need some values from the configuration during transfer*/
    _current_config = &config;

    /*store the current cs pin for following transfers*/
    _current_cs_pin = &cs_pin;
  }

  /*Performe one transfere with deassertes CS*/
  void transfer(uint8_t *tx, uint8_t *rx, size_t len) {
    int ret;

    struct spi_ioc_transfer tr;

    tr.tx_buf = reinterpret_cast<uint64_t>(tx);
    tr.rx_buf = reinterpret_cast<uint64_t>(rx);
    tr.len = static_cast<uint32_t>(len);
    tr.delay_usecs = _current_config->delay;
    tr.speed_hz = _current_config->speed;
    tr.bits_per_word = _current_config->bpw;

    if (_current_config->no_cs) {
      /*Disable CS - just in case ....*/
      _current_cs_pin->off();
    } else
      /*one transfere shall be completed with a deasserted CS*/
      tr.cs_change = 1;

    /*one transfere transferes without a CS deassert*/
    tr.tx_nbits = _current_config->bpw;
    tr.rx_nbits = _current_config->bpw;

    if (_current_config->no_cs)
      _current_cs_pin->on();
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
      throw std::runtime_error("Could not transfere SPI data");
    if (_current_config->no_cs)
      _current_cs_pin->off();
  }

private:
  spi(const char *device) : _current_config(nullptr), _current_cs_pin(nullptr) {
    fd = open(device, O_RDWR);
    if (fd < 0)
      throw std::runtime_error("Could not open SPI device");
    qDebug() << "Set up SPI device " << device;
  }

  ~spi() {
    /*C call return value*/
    int ret;

    ret = close(fd);
    if (ret)
      throw std::runtime_error("Could not close SPI device");
  }

  config::spi *_current_config;
  /*GPIO Pin for CS usage*/
  GPIO *_current_cs_pin;
  /*SPI mode bitfild*/

  int fd;
};

struct ADS1248::Impl {
public: 
 Impl() {}

  /*return true if device is present, false if communication is not possible*/
  bool selftest() {}
};


ADS1248::ADS1248(const config::spi &config, const config::gpio &reset) :
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
#include "moc_IntexHardware.cpp"
