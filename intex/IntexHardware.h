#pragma once



class ADS1248  {
  struct Impl;
  Impl *d;

  public:
  ADS1248();
  ~ADS1248();
  bool selftest();


};

