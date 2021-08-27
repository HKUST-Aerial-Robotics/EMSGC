#ifndef EMSGC_TOOLS_TICTOK_H
#define EMSGC_TOOLS_TICTOK_H

//#pragma once
#include <ctime>
#include <cstdlib>
#include <chrono>

namespace emsgc
{
namespace tools
{
class TicToc
{
  public:
  TicToc()
  {
    tic();
  }

  void tic()
  {
    start = std::chrono::system_clock::now();
  }

  double toc()
  {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000; // returns millisec
  }

  private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
}
}

#endif //EMSGC_TOOLS_TICTOK_H