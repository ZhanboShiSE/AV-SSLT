#include <iostream>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "facetracing.hpp"
#include "image_ros_read.hpp"

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {

    fioReadFaceFeature();

    msgRosSubscribe(argc, argv);

    freeFeatureList();

}
