#include <iostream>
#include <iomanip>

#include <posest/esm.hpp>
using namespace cv;

int main(int argc, char *argv[])
{
  if (argc != 3)
  {
    std::cout << "Format: run_esm [template_image] [test_folder]" << std::endl;
    exit(0);
  }

  string tplFilename = argv[1];
  string testFolder = argv[2];
  Mat templateImage = imread(tplFilename, 0);
  assert( !templateImage.empty() );

  HomoESM homoESM;
  homoESM.setTemplateImage(templateImage);

  const int nIters = 20;
  Mat H = Mat::eye(3, 3, CV_64FC1);
  H.at<double> (0, 2) = 220;
  H.at<double> (1, 2) = 200;
  const int lastFrameIdx = 200;

  for (int frameIdx = 0; frameIdx <= lastFrameIdx; frameIdx++)
  {
    std::stringstream filename;
    filename << testFolder << "/im" << std::setfill('0') << std::setw(3) << frameIdx << ".pgm";
    Mat frame = imread(filename.str(), 0);
    assert( !frame.empty() );

    homoESM.setTestImage(frame);
    double rmsError;
    homoESM.track(nIters, H, rmsError);

    Mat visualization;
    homoESM.visualizeTracking(H, visualization);
    imshow("Tracking", visualization);
    waitKey(10);
  }
  return 0;
}
