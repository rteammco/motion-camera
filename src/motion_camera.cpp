#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(save_image_path, "", "Where the movement frames get saved.");

static const cv::Size kFrameSize(400, 300);
static const int64 kMotionRegisterThresh = 1;
static const int kDelayBetweenFramesMS = 1000;

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  cv::VideoCapture video_capture(0);

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
      LOG(INFO) << "MOTION DETECTED";
      if (!FLAGS_save_image_path.empty()) {
        std::string path = FLAGS_save_image_path + "/img_"
                         + std::to_string(image_counter) + ".jpg";
        cv::imwrite(path, frame);
        image_counter++;
        LOG(INFO) << "Saved image " << path;
      }
    }

    // Display the images.
    cv::Mat visualization;
    cv::hconcat(frame, motion, visualization);
    cv::imshow("Visualization", visualization);
    cv::waitKey(kDelayBetweenFramesMS);

    previous_frame = frame.clone();
  }

  LOG(INFO) << "Program finished.";
  return EXIT_SUCCESS;
}
