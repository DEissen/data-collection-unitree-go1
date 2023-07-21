#include <UnitreeCameraSDK.hpp>
#include <unistd.h>

#include <sstream>     // for stringstream
#include <iomanip>     // for put_time
#include <chrono>      // for timestamps
#include <iostream>    // for std::cout
#include <sys/stat.h>  // for mkdir
#include <ctime>       // for std::localtime()
#include <string>      // for sscanf, ...
#include <thread>      // for sleep_for()


auto g_base_path = "./data/";
auto g_NameCam1 = "BellyCam";

std::string getMeasurementDirName(std::string base_path)
{
   // get timestamp for path
   auto const now = std::chrono::system_clock::now();
   auto in_time_t = std::chrono::system_clock::to_time_t(now);

   // create pathname and return it
   std::stringstream path;
   path << base_path << "measurement_" << std::put_time(std::localtime(&in_time_t), "%d_%m__%H_%M");
   return path.str();
}

std::string createMeasurementDir()
{
   // create path name based on timestamp for measurement
   auto measurement_path = getMeasurementDirName(g_base_path);
   auto cam1Path = measurement_path + "/" + g_NameCam1;

   // create measurement directory and report status
   if (mkdir(measurement_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
   {
      // dir alredy exists
      if (errno == EEXIST)
      {
         std::cout << "Cannot create directory " << measurement_path << " as it already exists!" << std::endl;
      }
      // not able to create due to other issue
      else
      {
         // try to crate base path which might resolve issue
         if (mkdir(g_base_path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
         {
            // even base path creation was not successfully, thus the path is not able to be created!
            std::cout << "Cannot create directory " << measurement_path << " due to another issue!" << std::endl;
         }
         else
         {
            // successfully created
            std::cout << "Created base directory " << g_base_path << " first!" << std::endl;
            // base path was created, thus try again
            createMeasurementDir();
         }
      }
   }
   // successfully created measurement dir
   else
   {
      // create dir for each camera
      mkdir(cam1Path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      std::cout << "Create measurement directory (including sub dirs) " << measurement_path << " successfully!"
                << std::endl;
   }

   return cam1Path;
}

std::string getCurrentLocalTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_time = std::localtime(&time);
    char buffer[9];
    std::strftime(buffer, sizeof(buffer), "%H:%M:%S", tm_time);
    return buffer;
}

int calculateTimeDifference(const std::string& targetTime) {
    std::string current_time = getCurrentLocalTime();

    int target_hour, target_minute, target_second, current_hour, current_minute, current_second;
    sscanf(targetTime.c_str(), "%d:%d:%d", &target_hour, &target_minute, &targetSecond);
    sscanf(currentTime.c_str(), "%d:%d:%d", &current_hour, &current_minute, &current_second);

    int target_total_seconds = target_hour * 3600 + target_minute * 60 + targetSecond;
    int current_total_seconds = current_hour * 3600 + current_minute * 60 + current_second;

    if (target_total_seconds <= current_total_seconds) {
        return -1; // provided time is not in the future
    }

    return (target_total_seconds - current_total_seconds) * 1000;
}

int main(int argc, char* argv[])
{
   std::string cam1Path = createMeasurementDir();

   int deviceNode1 = 0;  // 0 -> /dev/video0

   cv::Size frameSize(1856, 800);  // defalut image size: 1856 X 800
   int fps = 5;                    // set fps below 30 (at 30 the frame might be incomplete)

   // get starting time as argument
   if (argc < 2) {
      std::cerr << "Starting time is needed as argument in format 'HH:MM:SS'" << std::endl;
      return 1;
   }

   UnitreeCamera cam1(deviceNode1);

   // check whether both cams are available
   if (!cam1.isOpened())
      exit(EXIT_FAILURE);

   // configure camera parameters
   cam1.setRawFrameSize(frameSize);
   cam1.setRawFrameRate(fps);

   // Start camera capturing
   cam1.startCapture();


   // calculate time difference to target time 
   std::string targetTime(argv[1]);
   int time_difference_milliseconds = calculateTimeDifference(targetTime);

   if (time_difference_milliseconds < 0) {
      std::cerr << "Starting time is not in the future!" << std::endl;
      return 1;
   }

   std::cout << "Camera starts caputring, will wait for " << (time_difference_milliseconds / 1000) << "s to start with saving the images." << std::endl;
   std::this_thread::sleep_for(std::chrono::milliseconds(time_difference_milliseconds));
   std::cout << "Camera starts saving images now!";

   while (cam1.isOpened())
   {
      // get images or wait if they are not present yet
      cv::Mat leftCam1, rightCam1;
      std::chrono::microseconds t;
      if (!cam1.getStereoFrame(leftCam1, rightCam1, t))
      {  ///< get camera raw image
         usleep(1000);
         continue;
      }

      // get time for filename
      auto const now = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(now);
      auto transformed = now.time_since_epoch().count() / 1000000;
      auto millis = transformed % 1000;
      std::stringstream datetime;
      datetime << std::put_time(std::localtime(&in_time_t), "%H_%M_%S_") << std::setw(3) << std::setfill('0') << int(millis);

      cv::imwrite(cam1Path + "/Right_" + datetime.str() + ".jpg", rightCam1);
      cv::imwrite(cam1Path + "/Left_" + datetime.str() + ".jpg", leftCam1);
   }

   cam1.stopCapture();  ///< stop camera capturing

   return 0;
}
