#include <QDebug>
#include <QTimer>
#include <QString>
#include <QThread>
#include <QFileInfo>

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

static void export_pin(int pin) {
  std::ofstream export_;
  std::stringstream GPIO_Dir_Name;
  QFileInfo GPIO_Dir;

  GPIO_Dir_Name << "/sys/class/gpio/gpio" << pin;

  GPIO_Dir.setFile(QString::fromStdString(GPIO_Dir_Name.str()));

std::cout << "GPIO_Dir_Name: " << GPIO_Dir_Name.str() << " exists: " << GPIO_Dir.exists() << std::endl;
  if(!GPIO_Dir.exists())
{
  export_.exceptions(std::ofstream::failbit | std::ofstream::badbit);
  export_.open("/sys/class/gpio/export");
  export_ << pin << std::endl;
}
else
 assert(GPIO_Dir.isDir());

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

gpio::gpio(const config::gpio &config) : config_(config), isinitialized(false) {}

void gpio::init() {
  std::cout << "Configuring GPIO " << config_.name << " (" << config_.pinno
            << ") as " << config_.direction
            << (config_.active_low ? "(active low)" : "") << "." << std::endl;

  export_pin(config_.pinno);
  std::cout << "Export done" << std::endl;
  set_attribute(attribute::active_low, config_.pinno, config_.active_low);
  set_attribute(attribute::direction, config_.pinno, config_.direction);
  isinitialized=true;
}

void gpio::on() { set(true); }
void gpio::off() { set(false); }
bool gpio::isOn() { assert(isinitialized);
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
static spi& get_instance(const config::spi &config)
{
/*Todo - make some fancy stuff to create a new device, that has not been seen before and store it
in a accosicative array*/
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wexit-time-destructors"
static class spi spidev00("/dev/spidev0.0");
static class spi spidev01("/dev/spidev0.1");
#pragma clang diagnostic pop


if (strcmp(config.device,"/dev/spidev0.0")==0)
 return spidev00;
if (strcmp(config.device,"/dev/spidev0.1")==0)
 return spidev01;

throw std::runtime_error("Invalide SPI device");

}


  void configure(config::spi &config, GPIO &cs_pin) {
    /*Bitmask SPI mode for ioctl*/
    uint32_t mode=0;
    /*bits per word and speed readback*/
    uint32_t speed;
    uint8_t bpw;
    /*C call return value*/
    int ret;

    if (config.loopback){
      mode |= SPI_LOOP;
std::cout << "SPI loopback mode enabled" << std::endl;}

    if (config.cpha)
{ std::cout << "SPI CPHA=1" << std::endl;
      mode |= SPI_CPHA;
} else
 std::cout << "SPI CPHA=0" << std::endl;

    if (config.cpol){
      mode |= SPI_CPOL;
     std::cout << "SPI CPOL=1" << std::endl;

} else
 std::cout << "SPI CPOL=0" << std::endl;

    if (config.lsb_first) {
      mode |= SPI_LSB_FIRST;
 std::cout << "SPI LSB First" << std::endl;
} else
 std::cout << "SPI MSB First" << std::endl;


    if (config.cs_high){
      mode |= SPI_CS_HIGH;
 std::cout << "SPI CS High-Active" << std::endl;
}else
std::cout << "SPI CS Low-Active" << std::endl;

    if (config.threewire){
      mode |= SPI_3WIRE;
std::cout << "SPI 3-Wire mode" << std::endl;
} else
std::cout << "SPI 4-Wire mode" << std::endl;

    if (config.no_cs){
      mode |= SPI_NO_CS;
std::cout << "SPI external CS generation" << std::endl;
}
else
std::cout << "SPI internal CS generation" << std::endl;

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
    qDebug() << "Read back SPI BPW: " << bpw;
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
    qDebug() << "Set SPI speed from " << config.name << " to " << speed;

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
      /*one transfer shall be completed with a deasserted CS*/
      tr.cs_change = 1;

    /*one transfere transferes without a CS deassert*/
    tr.tx_nbits = _current_config->bpw;
    tr.rx_nbits = _current_config->bpw;

    if (_current_config->no_cs){
        _current_cs_pin->on();
        std::this_thread::sleep_for(2ms);

    }
//std::cout << "SPI Transfere with " << tr.len << " byte(s)"<<std::endl;
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
      throw std::runtime_error("Could not transfer SPI data");
    if (_current_config->no_cs)
      _current_cs_pin->off();
//std::cout << "CS deactivated (high)" << std::endl;
//std::cin.get();

  }


private:

  spi(const char *device)
  : _current_config(nullptr), _current_cs_pin(nullptr)
   {
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
  GPIO reset_pin;
  GPIO *cs_pin;
  spi &spi_bus;
  config::spi _config;
public:
  Impl(const config::spi &config, const config::gpio &reset)
      : reset_pin(::intex::hw::gpio(reset)), spi_bus(spi::get_instance(config)),
      _config(config) {

    reset_pin.init();
    if (config.no_cs)
    {
     cs_pin = new GPIO(::intex::hw::gpio(config.cs_pin));
     cs_pin->init();
     cs_pin->off();
     std::cout<<"###################### cs pin off "<<std::endl;
    };

  }

   void reset()
   {
   reset_pin.on();
   std::cout << "Reset active (low)"<< std::endl;
  // std::cin.get();

   reset_pin.off();
   std::cout << "Reset deactivated (high)" << std::endl;
  // std::cin.get();

   uint8_t tx[]={0x06};
   uint8_t rx[1];

   spi_bus.configure(_config,*cs_pin);
   spi_bus.transfer(tx,rx,1);
   std::this_thread::sleep_for(200ms);


   }

void selfOff()
{//perform selfoffset calibration
    uint8_t tx[3]={0x62};
    uint8_t rx[3]={0x0};
    spi_bus.transfer(tx,rx,1);
    std::this_thread::sleep_for(5ms);



}

/*return true if device is present, false if communication is not possible*/
void _init()
{
   reset();
   spi_bus.configure(_config,*cs_pin);
   //std::this_thread::sleep_for(5ms);

   bool ADP_ready=false;
   // hier schicke ich solange die Anfrage ein Register auszulesen 0x02B dessen Inhalt nach Reset bekannt ist
   //bis ich das Richtige Ergebnis zurückkriege 0xFF

   uint8_t tx_ready[3]={0x2B,0x0,0x0};
   uint8_t rx_ready[3]={0,0,0x0};

   while (!ADP_ready){
    spi_bus.transfer(tx_ready,rx_ready,3);
    std::cout << "Lese Register 0x0B -->"<< static_cast<unsigned int>(rx_ready[2]) <<std::endl;

    if (static_cast<unsigned int>(rx_ready[2])==0xFF)
        {
        ADP_ready=true;
        std::cout << "Register 0x0B korrekt ausgelesen"<<std::endl;
        }

    }

    // Dann schreibe ich in Register 0x0A ,0x08 rein um DRDY & DOUT Funktionalität zu setzen
    // mit anschließender Kontrolle ob es richtig gesetzt ist
    uint8_t tx_drdydout_write[3]={0x4A,0x0,0x00};
    uint8_t rx_drdydout_write[3]={0,0,0x0};
    uint8_t chip_id;

    //auf reset-werte setzen
    uint8_t tx_drdydout_rewrite[3]={0x4A,0x0,0x00};
    spi_bus.transfer(tx_drdydout_rewrite,rx_drdydout_write,3);

    //chip_id auslesen
    uint8_t tx_drdydout_read[3]={0x2A,0x0,0x00};
    uint8_t rx_drdydout_read[3]={0,0,0x0};
    spi_bus.transfer(tx_drdydout_read,rx_drdydout_read,3);
    chip_id=static_cast<uint8_t>(rx_drdydout_read[2]);

    //solange register beschreiben bis korrekter Wert drinne ist
    ADP_ready=false;
    while (!ADP_ready){

    //schreibe in Register 0x0A
    spi_bus.transfer(tx_drdydout_write,rx_drdydout_write,3);
    //lese aus Register 0x0A
    spi_bus.transfer(tx_drdydout_read,rx_drdydout_read,3);
    std::cout << "Lese Register 0x0A --> "<<static_cast<unsigned int>(rx_drdydout_read[2])<<std::endl;
    if (static_cast<unsigned int>(rx_drdydout_read[2])-chip_id==0)
        {
        ADP_ready=true;
        std::cout << "Register 0x0A korrekt ausgelesen"<<std::endl;
        }
    }

    //selfOff();
}

double binToTemp(uint8_t rx[3])
{//rechnet binäres Ergebnis in Temperatur um
 //zusammenfügen
 //zweier komplement bilden
 //Umrechnen über näherungsformel (Cbin*LSB)/(I*TR)
    //zusammenfügen
    double rbin=rx[0]*std::pow(2,16)+rx[1]*std::pow(2,8)+rx[2];
    //zweierkomplement weg rechnen
    //std::cout<< static_cast<unsigned int>(rx[0])<<static_cast<unsigned int>(rx[1])<<static_cast<unsigned int>(rx[2])<<std::endl;
    if (rbin>std::pow(2,23))
    {
           rbin=rbin-std::pow(2,23);
           std::cout<< "negativ"<<std::endl;
    }

    //Umrechnung
    double T=rbin*2.02097*std::pow(10,-9);
    T=T/(0.0005*0.385);
    return T;

}

double binToTemp_intern(uint8_t rx[3])
{
    double rbin=rx[0]*std::pow(2,16)+rx[1]*std::pow(2,8)+rx[2];


return rbin;

}


int wrReg(uint8_t reg, uint8_t val)
{
    if ((reg>0xE)||(val>0xFF))
    {
        std::cout<< "#### register or val to high, see IntexHardware::WRREG()"<<std::endl;
        return -1;
    }

    uint8_t tx[3]={0x0,0x0,val};
    tx[0]=0x40+reg;
    //std::cout<< static_cast<unsigned int>(tx[0])<<"+++"<<static_cast<unsigned int>(tx[1])<<"+++"<<static_cast<unsigned int>(tx[2])<<std::endl;
    uint8_t rx[3]={0,0,0};
    spi_bus.transfer(tx,rx,3);
    //control if new value in reg
    uint8_t txc[3]={0x0,0x0,0x0};
    txc[0]=0x20+reg;
    //std::cout<< static_cast<unsigned int>(txc[0])<<"+++"<<static_cast<unsigned int>(txc[1])<<"+++"<<static_cast<unsigned int>(txc[2])<<std::endl;
    uint8_t rxc[3]={0x0,0x0,0x0};
    //lese aus
    std::this_thread::sleep_for(30ms);
    spi_bus.transfer(txc,rxc,3);
    //std::cout<< std::hex<<std::hex<<static_cast<unsigned int>(rxc[2])<<std::endl;
    uint8_t val_regA=val+128;
    if ((reg==0xA)&&(rxc[2]=val_regA)){
            return 1;
        }
    if (rxc[2]!=val)
    {
        std::cout<< static_cast<unsigned int>(reg) <<std::endl;
        std::cout<< "#### Writing to register fails, see IntexHardware::WRREG() "<<std::endl;
        return -1;
    }
    return 1;

}
/*sensor_num - starting at sensor 0*/
void sensor_modus(uint8_t sensor_num)
{
//schalte auf pt100 1 um
    uint32_t error_array[6]={0,0,0,0,0,0};
    //std::cout<<"##### Start sensor modus #####"<<std::endl;
    //std::cout<<"##### sensor nummer : "<<std::dec<<static_cast<unsigned int>(sensor_num)<<std::endl;
    uint8_t parameter=((2*sensor_num)+1) | (2*sensor_num)<<3;
    //std::cout<<"##### first byte = "<<std::hex<<static_cast<unsigned int>(parameter)<<std::endl;

    //Eingänge für PGA setzen und SPS auf 160
    while(wrReg(0x0, ((2*sensor_num)+1) | (2*sensor_num)<<3 ) <0){
    error_array[0]++;
    }
    //VBIAS setzen
    while(wrReg(0x1,0x00)<0){
    error_array[1]++;
    }
    //i=wrReg(0x1,0x0);
    //internal reference on for IDAC
    while(wrReg(0x2,0x20)<0){
    error_array[2]++;
    }
    //sys control 0, PGAwert setzen und SPS
    while(wrReg(0x03,0x70)<0){
    error_array[3]++;
    }
    //IDAC reg1
    while(wrReg(0x0B, ((2*sensor_num)+1) | (2*sensor_num)<<4 )<0){
    error_array[4]++;
    }
    //IDAC reg0
    //while(wrReg(0x0A,0x0C)<0){
    while(wrReg(0x0A,0x04)<0){
    error_array[5]++;
    }
    //std::cout<<"### ERROR ARRAY-->" << error_array[5]<<"---"<<error_array[4]<<"---"<<error_array[3]<<"---"<<error_array[2]<<"---"<<error_array[1]<<"---"<<error_array[0]<<std::endl;
    //selfOffset calibration
    std::this_thread::sleep_for(300ms);
    // see manual page 35 for SPS 40 settling time < 8ms
    //std::cout<<"##### End sensor modus #####"<<std::endl;
}


void spi_loop()
{

    uint16_t modus=1; // 0 interne Diode, 1 RTD1

    while (modus==1){

        uint8_t tx3[1]={0x12};
        uint8_t rx3[1]={0x0};
        spi_bus.transfer(tx3,rx3,1);
        std::this_thread::sleep_for(5ms);
        uint8_t tx4[3]={0xFF,0xFF,0xFF};
        uint8_t rx4[3]={0x0,0x0,0x0};
        spi_bus.transfer(tx4,rx4,3);
        std::cout<<"conversion result-->" << static_cast<unsigned int>(rx4[0])<<"---"<<static_cast<unsigned int>(rx4[1])<<"---"<<static_cast<unsigned int>(rx4[2])<<std::endl;
        double temperature=binToTemp(rx4);
        std::cout<<"temperature -->" <<std::dec<< temperature <<"°C"<<std::endl;
   }
////////############ interne Dioden Messung #############

    // wenn nein schreibe in register
    uint8_t tx2[3]={0x42,0x0,0x33};
    uint8_t rx2[3]={0,0,0};
    spi_bus.transfer(tx2,rx2,3);
    std::cout << "Auf interne Dioden umgeschalten --> "<<std::endl;


   while (modus==0){
        uint8_t tx3[1]={0x12};
        uint8_t rx3[1]={0};
        spi_bus.transfer(tx3,rx3,1);
        std::this_thread::sleep_for(5ms);
        uint8_t tx4[3]={0xFF,0xFF,0xFF};
        uint8_t rx4[3]={0,0,0};
        spi_bus.transfer(tx4,rx4,3);
        std::cout << "interne Diodenmessung"<<std::endl;
        double temperature=binToTemp_intern(rx4);
        std::cout<<"temperature -->" << temperature <<"°C, Diode"<<std::endl;

   }


}

double messung_extern()
{
    uint8_t tx3[1]={0x12};
    uint8_t rx3[1]={0x0};
    spi_bus.transfer(tx3,rx3,1);
    std::this_thread::sleep_for(5ms);
    uint8_t tx4[3]={0xFF,0xFF,0xFF};
    uint8_t rx4[3]={0x0,0x0,0x0};
    spi_bus.transfer(tx4,rx4,3);
    //std::cout<<"conversion result-->" << static_cast<unsigned int>(rx4[0])<<"---"<<static_cast<unsigned int>(rx4[1])<<"---"<<static_cast<unsigned int>(rx4[2])<<std::endl;
    double temperature=binToTemp(rx4);
    std::cout<<"temperature -->" <<std::dec<< temperature<<"°C"<<std::endl;
    return temperature;
}

double manual_calibration(uint8_t sensor_select)
{//100 Messungen zur Mittelwertbestimmung
    double mittelwert;
    _init();
    sensor_modus(sensor_select);
    //for(int i=0;i<20;i++)
/*
    while(1)
    {
    mittelwert=messung_extern();
    std::this_thread::sleep_for(30ms);
    }
    return mittelwert;
*/
//    for(int i=0;i<100;i++){
        mittelwert=messung_extern();
//            std::this_thread::sleep_for(200ms);

    //}
    //std::cout<<"##############################################"<<std::endl;
    return mittelwert;

}

private:
};

ADS1248::ADS1248(const config::spi &config, const config::gpio &reset)
{
    /*Why can I not use the std::make_unique<Impl> way here?*/
    d=new Impl(config, reset);
    d->reset();
}

double ADS1248::selftest(uint8_t sensor_select)
{
double x=0;
d->_init();
return d->manual_calibration(sensor_select);
}



/*HSC Sensor part*/

struct HSCSensors::Impl {
  GPIO *cs_pin;
  spi &spi_bus;
  config::spi _config;
  bool _metric;
public:
  Impl(const config::spi &config,bool metric)
      : spi_bus(spi::get_instance(config)), _config(config), _metric(metric) {

    if (config.no_cs)
    {
     cs_pin = new GPIO(::intex::hw::gpio(config.cs_pin));
     cs_pin->init();
     cs_pin->off();
     std::cout<<"###################### cs pin off "<<std::endl;
    };
  }

double bin_to_double_pressure(uint8_t ax[4])
{//wandelt die binäre Ausgabe in dezimales Druckergebnis um

    uint32_t x = ax[0] & 0x3F;
    x = (x<<8)  + ax[1];
    //für 10 bar sensor ist umrechnung anders
    if(_metric)
    {
        double xx=x-1638;
        //std::cout<<x<<std::endl;
        xx=xx*1.6;
        //std::cout<<x<<std::endl;
        xx=xx/13107;
        //std::cout<<"x="<<x<<std::endl;
        return  xx;
    }
    else
    {
        double xx=x-1638;
        //std::cout<<x<<std::endl;
        xx=xx*150;
        //std::cout<<x<<std::endl;
        xx=xx/13107;
        //std::cout<<"x="<<x<<std::endl;
        return  xx/14.504;
    }

}



double bin_to_temp(double temp)
{
    return ((temp/2047)*200)-50;



}
  double get_pressure()
{
    spi_bus.configure(_config,*cs_pin);
    std::cout<<"######### Drucksensor am GPIO  :  "<<std::dec<<_config.cs_pin.pinno<<std::endl;

    for ( int i=0;i<10;i++){
        uint8_t tx[3]={0x0,0x0,0x0};
        uint8_t rx[3]={0x0,0x0,0x0};
        std::cout<<"### --- " << std::endl;
        spi_bus.transfer(tx,rx,3);
        std::cout<<"rx (0x ) : " << std::hex<<static_cast<unsigned int>(rx[0])<<" # "<< std::hex<<static_cast<unsigned int>(rx[1])<<" # "<< std::hex<<static_cast<unsigned int>(rx[2])<<std::endl;

        uint16_t status=rx[0]>>6;

        std::cout<<"Statusbit :  "<<std::hex<<status<<std::endl;

        //std::cout<<"Druckb :  "<<"0x"<<std::hex<<out_bin<<std::endl;
        std::cout<<"Druckd :  "<<std::dec<<bin_to_double_pressure(rx)<<" Bar"<<std::endl;


        double t1=double(uint16_t(rx[2])<<3);
        std::cout<<"Temperatur :  "<<bin_to_temp(t1)<<std::endl;


    }

    return 0;

}


  double get_temperature()
{



  return -300.0;
}



};

HSCSensors::HSCSensors(const config::spi &config,bool metric)
     {
    /*Why can I not use the std::make_unique<Impl> way here?*/
    d=new Impl(config,metric);
    }


 double HSCSensors::get_pressure()
 {

 return d->get_pressure();

 }


 double HSCSensors::get_temperature()
 {

 return d->get_temperature();

 }


} /*namespace hw*/
} /*namspace intex*/
#pragma clang diagnostic ignored "-Wundefined-reinterpret-cast"
#include "IntexHardware.moc"
#include "moc_IntexHardware.cpp"
