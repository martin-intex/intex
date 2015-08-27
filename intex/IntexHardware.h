#pragma once



class ADS1248  {
  struct Impl;
  std::unique_ptr<Impl> d;

  public:
  ADS1248();
  ~ADS1248();
  bool selftest();


};

