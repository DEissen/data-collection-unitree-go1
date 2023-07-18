#include <UnitreeCameraSDK.hpp>
#include <unistd.h>

#include <sstream>     // for stringstream
#include <iomanip>     // for put_time
#include <chrono>      // for timestamps
#include <iostream>    // for std::cout
#include <sys/stat.h>  // for mkdir

auto g_base_path = "./data/";
auto g_NameCam1 = "ChinCam"; // "ChinCam" for 13 and "RightCam" for 14
auto g_NameCam2 = "HeadCam"; // "HeadCam" for 13 and "LeftCam" for 14

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
   auto cam2Path = measurement_path + "/" + g_NameCam2;

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
      mkdir(cam2Path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      std::cout << "Create measurement directory (including sub dirs) " << measurement_path << " successfully!"
                << std::endl;
   }

   return measurement_path;
}

int main(int argc, char* argv[])
{
   std::string measurement_path = createMeasurementDir();

   int deviceNode1 = 0;  // 0 -> /dev/video0
   int deviceNode2 = 1;  // 1 -> /dev/video1

   cv::Size frameSize(1856, 800);  // defalut image size: 1856 X 800
   int fps = 5;                    // set fps below 30 (at 30 the frame might be incomplete)

   // does seem to be code to change device id, ... by shell parameters
   // if(argc >= 2){
   //     deviceNode1 = std::atoi(argv[1]);
   //     if(argc >= 4){
   //         frameSize = cv::Size(std::atoi(argv[2]), std::atoi(argv[3]));
   //     }
   //     if(argc >=5)
   //         fps = std::atoi(argv[4]);
   // }

   UnitreeCamera cam1(deviceNode1);
   UnitreeCamera cam2(deviceNode2);

   // check whether both cams are available
   if (!cam1.isOpened())
      exit(EXIT_FAILURE);
   if (!cam2.isOpened())
      exit(EXIT_FAILURE);

   // configure camera parameters
   cam1.setRawFrameSize(frameSize);
   cam1.setRawFrameRate(fps);
   cam2.setRawFrameSize(frameSize);
   cam2.setRawFrameRate(fps);

   std::cout << "Cam1 Device Position Number:" << cam1.getPosNumber() << std::endl;
   std::cout << "Cam2 Device Position Number:" << cam2.getPosNumber() << std::endl;

   // Start camera capturing
   cam1.startCapture();
   cam2.startCapture();

   while (cam1.isOpened() && cam2.isOpened())
   {
      // get images or wait if they are not present yet
      cv::Mat frameCam1, frameCam2;
      std::chrono::microseconds t;
      if (!cam1.getRawFrame(frameCam1, t))
      {  ///< get camera raw image
         usleep(500);
         continue;
      }
      if (!cam2.getRawFrame(frameCam2, t))
      {  ///< get camera raw image
         usleep(500);
         continue;
      }

      // get time for filename
      auto const now = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(now);
      auto transformed = now.time_since_epoch().count() / 1000000;
      auto millis = transformed % 1000;
      std::stringstream datetime;
      datetime << std::put_time(std::localtime(&in_time_t), "%H_%M_%S_") << int(millis);

      cv::Mat leftCam1, rigthCam1;
      frameCam1(cv::Rect(0, 0, frameCam1.size().width / 2, frameCam1.size().height)).copyTo(rigthCam1);
      frameCam1(cv::Rect(frameCam1.size().width / 2, 0, frameCam1.size().width / 2, frameCam1.size().height))
          .copyTo(leftCam1);
      cv::imwrite(measurement_path + "/" + g_NameCam1 + "/Right_" + datetime.str() + ".jpg", rigthCam1);
      cv::imwrite(measurement_path + "/" + g_NameCam1 + "/Left_" + datetime.str() + ".jpg", leftCam1);

      cv::Mat leftCam2, rigthCam2;
      frameCam2(cv::Rect(0, 0, frameCam2.size().width / 2, frameCam2.size().height)).copyTo(rigthCam2);
      frameCam2(cv::Rect(frameCam2.size().width / 2, 0, frameCam2.size().width / 2, frameCam2.size().height))
          .copyTo(leftCam2);
      cv::imwrite(measurement_path + "/" + g_NameCam2 + "/Right_" + datetime.str() + ".jpg", rigthCam2);
      cv::imwrite(measurement_path + "/" + g_NameCam2 + "/Left_" + datetime.str() + ".jpg", leftCam2);
   }

   // stop camera capturing
   cam1.stopCapture();
   cam2.stopCapture();

   return 0;
}
