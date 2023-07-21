#include <UnitreeCameraSDK.hpp>
#include <unistd.h>

#include <sstream>     // for stringstream
#include <iomanip>     // for put_time
#include <chrono>      // for timestamps
#include <iostream>    // for std::cout
#include <sys/stat.h>  // for mkdir

#include <thread> // needed for sleep_until() -> https://en.cppreference.com/w/cpp/thread/sleep_until


int main(int argc, char* argv[])
{
   // does seem to be code to change device id, ... by shell parameters
   // if(argc >= 2){
   //     deviceNode1 = std::atoi(argv[1]);
   //     if(argc >= 4){
   //         frameSize = cv::Size(std::atoi(argv[2]), std::atoi(argv[3]));
   //     }
   //     if(argc >=5)
   //         fps = std::atoi(argv[4]);
   // }

   std::chrono::time_point now = std::chrono::system_clock::now() // time points should be comparable -> https://en.cppreference.com/w/cpp/chrono/time_point

   return 0;
}
