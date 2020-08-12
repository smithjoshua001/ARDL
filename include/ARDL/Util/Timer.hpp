#pragma once
#include "ARDL/Util/Logger.hpp"
#include <chrono>
#include <sys/time.h>
#include <map>
using namespace std::chrono;
using Clock = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;

inline double operator-(const struct timespec & t1,const struct timespec & t0)
{
   /* TODO: double check the double conversion from long (on 64x). */
   return double(t1.tv_sec - t0.tv_sec)+1e-9*double(t1.tv_nsec - t0.tv_nsec);
}
namespace ARDL {
    namespace Util {
        class Timer {
        private:
            TimePoint t1, t2;
            std::map<std::string, struct timespec> times;
            duration<double> time_span;
          struct timespec t3,t4;
          /* Return the time spent in secs. */


        public:
            void tic(const std::string &in = "") {
                if (times.find(in) != times.end()) {
                  clock_gettime(CLOCK_MONOTONIC,&t4);
                  //t2 = Clock::now();
                  // time_span = duration_cast<duration<double> >(t2 - times[in]);
                  //LOG_DEBUG_LEVEL1("{} tick: {} s", in, time_span.count());
                  LOG_DEBUG_LEVEL1("{} tick: {} s", in, (t4-times[in]));
                } else {
                    // times[in] = Clock::now();
                  clock_gettime(CLOCK_MONOTONIC,&times[in]);
                }
            }
            void tock(const std::string &in = "") {
              /*t2 = Clock::now();
                time_span = duration_cast<duration<double> >(t2 - times[in]);
                LOG_DEBUG_LEVEL1("{} tock: {} s", in, time_span.count());
                times[in] = Clock::now();*/
              clock_gettime(CLOCK_MONOTONIC,&t4);
              LOG_DEBUG_LEVEL1("{} toc: {} s", in, (t4-times[in]));
              clock_gettime(CLOCK_MONOTONIC,&times[in]);
            }
          void refresh(const std::string &in =""){
            clock_gettime(CLOCK_MONOTONIC,&times[in]);
          }
        };
        static std::shared_ptr<Timer> global_timer = std::make_shared<Timer>();
        #ifdef LOG_DEBUG_ON
    #define TIC(Input) global_timer->tic(Input)
    #define TOCK(Input) global_timer->tock(Input)
    #define TOC(Input) global_timer->tock(Input)
#define REFRESH(Input) global_timer->refresh(Input)
        #else
    #define TIC(Input) (void)0
    #define TOCK(Input) (void)0
    #define TOC(Input) (void)0
#define REFRESH(Input) (void)0
        #endif
    }
}
