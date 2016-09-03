// This program captures video from a camera (e.g. a webcam) in periodic
// intervals. If motion was detected between the two images the frame and the
// motion will be displayed.
//
// Optionally, the user can add a command line argument that specifies a
// directory on the machine. If this is provided, the frames where motion was
// detected will be saved to that directory.

#include <ctime>
#include <iostream>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

static const cv::Size kFrameSize(400, 300);
static const int64 kMotionRegisterThresh = 1;
static const int kDelayBetweenFramesMS = 1000;

int main(int argc, char** argv) {
  // Process the argument for a save image path if one was provided.
  std::string save_image_path;  // Initially empty.
  if (argc > 1) {
    save_image_path = std::string(argv[1]);
    std::cout << save_image_path << std::endl;
  }

  cv::VideoCapture video_capture(0);
  if (!video_capture.isOpened()) {
    std::cerr << "Could not open camera." << std::endl;
    return -1;
  }

  // Get the first frame.
  cv::Mat previous_frame;
  video_capture >> previous_frame;
  cv::waitKey(kDelayBetweenFramesMS);
  cv::resize(previous_frame, previous_frame, kFrameSize);

  int image_counter = 0;
  while (true) {
    cv::Mat frame;
    video_capture >> frame;
    cv::resize(frame, frame, kFrameSize);

    // Compute the motion image and the amount of movement.
    cv::Mat motion;
    cv::absdiff(previous_frame, frame, motion);

    // Compute the amount of motion (thresholded).
    cv::Mat binary_motion;
    cv::threshold(motion, binary_motion, 100, 1, cv::THRESH_BINARY);
    const int64 motion_amount = cv::sum(binary_motion)[0];

    if (motion_amount >= kMotionRegisterThresh) {
      // Get the current time string.
      const time_t sys_time = time(0);
      const tm* time_now = localtime(&sys_time);
      std::string time_string(asctime(time_now));
      time_string.erase(time_string.end() - 1);  // Erase trailing newline.
      std::cout << "MOTION DETECTED: " << time_string << std::endl;

      // Save the frame to a file if a path was provided.
      if (!save_image_path.empty()) {
        // Path of the form "user/specified/path/img_0.jpg" where 0 is replaced
        // with the current image number.
        std::string path =
            save_image_path + "/img_" + std::to_string(image_counter) + ".jpg";
        cv::imwrite(path, frame);
        image_counter++;
        std::cout << "Saved image: " << path << std::endl;
      }
    }

    // Display the raw frame and the motion image for visualization.
    cv::Mat visualization;
    cv::hconcat(frame, motion, visualization);
    cv::imshow("Visualization", visualization);
    cv::waitKey(kDelayBetweenFramesMS);

    previous_frame = frame.clone();
  }

  std::cout << "Program finished." << std::endl;
  return 0;
}
