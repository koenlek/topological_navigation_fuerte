#include <posest/esm.hpp>
using namespace cv;

void rect2vertices(const Rect &rect, vector<Point2f> &vertices);

void HomoESM::setTemplateImage(const Mat &image)
{
  assert( image.type() == CV_8UC1 );
  templateImage = image;
  Mat templateImageDouble;
  image.convertTo(templateImageDouble, CV_64FC1);
  templateImageRowDouble = templateImageDouble.reshape(0, 1);

  Mat templateDx, templateDy;
  computeGradient(templateImage, templateDx, templateDy);
  templateDxRow = templateDx.reshape(0, 1);
  templateDyRow = templateDy.reshape(0, 1);

  xx.create(1, image.rows * image.cols, CV_64FC1);
  yy.create(1, image.rows * image.cols, CV_64FC1);
  for (int i = 0; i < image.rows * image.cols; i++)
  {
    xx.at<double> (0, i) = i % image.cols;
    yy.at<double> (0, i) = i / image.cols;
  }

  rect2vertices(Rect(0, 0, templateImage.cols, templateImage.rows), templateVertices);

  templatePoints.create(templateImage.size().area(), 1, CV_32FC2);
  for (int y = 0; y < templateImage.rows; y++)
  {
    for (int x = 0; x < templateImage.cols; x++)
    {
      templatePoints.at<Vec2f> (y * templateImage.cols + x, 0) = Vec2f(x, y);
    }
  }
}

void HomoESM::setTestImage(const Mat &image)
{
  assert( image.type() == CV_8UC1 );
  testImage = image;
}

double HomoESM::computeRMSError(const Mat &error)
{
  return norm(error) / sqrt((double)error.size().area());
}

void HomoESM::track(int nIters, Mat &H, double &rmsError, Ptr<LieAlgebra> lieAlgebra, bool saveComputations, vector<
    HomoESMState> *computations) const
{
  if (saveComputations)
  {
    assert( computations != 0 );
    computations->clear();
  }

  for (int iter = 0; iter < nIters; iter++)
  {
    Mat warpedHomogeneousPoints;
    transform(templatePoints, warpedHomogeneousPoints, H);
    vector<Point2f> warpedPoints;
    convertPointsHomogeneous(warpedHomogeneousPoints, warpedPoints);

    Mat warpedTemplateDouble;
    constructImage(testImage, warpedPoints, warpedTemplateDouble);

    Mat errorRow = warpedTemplateDouble.reshape(0, 1) - templateImageRowDouble;

    if (saveComputations)
    {
      HomoESMState state;
      H.copyTo(state.H);
      state.error = computeRMSError(errorRow);
      computations->push_back(state);
    }

    if (iter == nIters - 1)
    {
      rmsError = computeRMSError(errorRow);
      break;
    }

    Mat warpedTestImageDx, warpedTestImageDy;
    computeGradient(warpedTemplateDouble, warpedTestImageDx, warpedTestImageDy);

    Mat Jt;
    computeJacobian(templateDxRow + warpedTestImageDx.reshape(0, 1), templateDyRow + warpedTestImageDy.reshape(0, 1),
                    Jt, lieAlgebra);

    Mat JtJ;
    mulTransposed(Jt, JtJ, false);
    //TODO: Is JtJ always non-singular?
    Mat d = -2 * JtJ.inv(DECOMP_CHOLESKY) * Jt * errorRow.t();

    Mat delta = lieAlgebra->algebra2group(d);
    H = H * delta;
  }
}

void HomoESM::projectVertices(const cv::Mat &H, std::vector<cv::Point2f> &vertices) const
{
  vertices.clear();
  Mat transformedVertices;
  transform(Mat(templateVertices), transformedVertices, H);
  convertPointsHomogeneous(transformedVertices, vertices);
}

void HomoESM::visualizeTracking(const Mat &H, Mat &visualization) const
{
  vector<Point2f> vertices;
  projectVertices(H, vertices);

  cvtColor(testImage, visualization, CV_GRAY2RGB);
  Scalar color = Scalar(0, 255, 0);
  int thickness = 2;
  for (size_t i = 0; i < vertices.size(); i++)
  {
    line(visualization, vertices[i], vertices[(i + 1) % vertices.size()], color, thickness);
  }
}

void HomoESM::computeGradient(const Mat &image, Mat &dx, Mat &dy)
{
  Sobel(image, dx, CV_64FC1, 1, 0);
  Sobel(image, dy, CV_64FC1, 0, 1);

  //normalize to get derivative
  dx /= 8.;
  dy /= 8.;
}

void HomoESM::computeJacobian(const Mat &dx, const Mat &dy, Mat &Jt, Ptr<LieAlgebra> lieAlgebra) const
{
  assert( dx.rows == 1 );
  assert( dy.rows == 1 );
  Mat Ix = dx;
  Mat Iy = dy;

  const int dim = 3;
  Mat JiJw_t(dim * dim, Ix.cols, CV_64FC1);
  JiJw_t.row(0) = Ix.mul(xx);
  JiJw_t.row(1) = Ix.mul(yy);

  Mat tmpMat = JiJw_t.row(2);
  Ix.copyTo(tmpMat);

  JiJw_t.row(3) = Iy.mul(xx);
  JiJw_t.row(4) = Iy.mul(yy);

  tmpMat = JiJw_t.row(5);
  Iy.copyTo(tmpMat);

  JiJw_t.row(6) = -(xx.mul(JiJw_t.row(0)) + yy.mul(JiJw_t.row(3)));
  JiJw_t.row(7) = -(xx.mul(JiJw_t.row(1)) + yy.mul(JiJw_t.row(4)));
  JiJw_t.row(8) = -(xx.mul(JiJw_t.row(2)) + yy.mul(JiJw_t.row(5)));

  lieAlgebra->dot(JiJw_t, Jt);
}

void HomoESM::constructImage(const Mat &srcImage, const vector<Point2f> &points, Mat &intensity) const
{
  assert( srcImage.type() == CV_8UC1 );
  intensity.create(templateImage.size(), CV_64FC1);

  const double defaultValue = 0;
  //Nearest neighbor interpolation
  /*
  for (size_t i = 0; i < points.size(); i++)
  {
    double val;
    Point pt = points[i];
    if (0 <= pt.x && pt.x < srcImage.cols && 0 <= pt.y && pt.y < srcImage.rows)
      val = srcImage.at<uchar> (pt.y, pt.x);
    else
      val = defaultValue;
    intensity.at<double> (i / templateImage.cols, i % templateImage.cols) = val;
  }
  */

  //Bilinear interpolation
  for (size_t i = 0; i < points.size(); i++)
  {
    double val;
    Point2f pt = points[i];
    if (0 <= pt.x && pt.x < srcImage.cols - 1 && 0 <= pt.y && pt.y < srcImage.rows - 1)
    {
      Point tl = Point(floor(pt.x), floor(pt.y));
      double x1 = pt.x - tl.x;
      double y1 = pt.y - tl.y;
      double y2 = 1 - y2;
      val = (1 - x1) * (1 - y1) * srcImage.at<uchar> (tl.y, tl.x) + x1 * (1 - y1) * srcImage.at<uchar> (tl.y, tl.x + 1)
          + (1 - x1) * y1 * srcImage.at<uchar> (tl.y + 1, tl.x) + x1 * y1 * srcImage.at<uchar> (tl.y + 1, tl.x + 1);
    }
    else
      val = defaultValue;
    intensity.at<double> (i / templateImage.cols, i % templateImage.cols) = val;
  }
}

void HomoESM::checkAccuracy(vector<vector<HomoESMState> > &computations, string groundTruthFile, double &meanSSDError,
                            double &meanSpatialError) const
{
  FileStorage fs(groundTruthFile, FileStorage::READ);
  assert( fs.isOpened() );

  double ssdErrorSum = 0;
  double spatialErrorSum = 0;
  const double minSSDError = 1.;
  int iterationsCount = 0;
  int errorsCount = 0;
  for (size_t frameIdx = 0; frameIdx < computations.size(); frameIdx++)
  {
    for (size_t iterIdx = 0; iterIdx < computations[frameIdx].size(); iterIdx++)
    {
      std::stringstream frame, iter;
      frame << "frame_" << frameIdx;
      iter << "iter_" << iterIdx;
      double groundTruthError = fs["ESM_results"][frame.str()][iter.str()]["rms_error"];
      double error = computations[frameIdx][iterIdx].error;
      if (groundTruthError > minSSDError)
      {
        ssdErrorSum += error / groundTruthError;
        errorsCount++;
      }
      Mat groundTruthH;
      fs["ESM_results"][frame.str()][iter.str()]["warp_p"] >> groundTruthH;
      Mat H = computations[frameIdx][iterIdx].H;
      vector<Point2f> groundTruthVertices;
      vector<Point2f> vertices;
      projectVertices(groundTruthH, groundTruthVertices);
      projectVertices(H, vertices);
      double spatialError = norm(Mat(groundTruthVertices), Mat(vertices));
      spatialErrorSum += spatialError;
      iterationsCount++;
    }
  }
  fs.release();
  meanSSDError = ssdErrorSum / errorsCount;
  meanSpatialError = spatialErrorSum / iterationsCount;
}

LieAlgebraHomography::LieAlgebraHomography()
{
  basis.resize(8);
  basis[0].addPositive(2);
  basis[1].addPositive(5);
  basis[2].addPositive(1);
  basis[3].addPositive(3);

  basis[4].addPositive(0);
  basis[4].addNegative(4);
  basis[5].addNegative(4);
  basis[5].addPositive(8);

  basis[6].addPositive(6);
  basis[7].addPositive(7);
}

LieAlgebraRotation3d::LieAlgebraRotation3d()
{
  basis.resize(3);
  basis[0].addPositive(1);
  basis[0].addNegative(3);
  basis[1].addPositive(2);
  basis[1].addNegative(6);
  basis[2].addPositive(5);
  basis[2].addNegative(7);
}

LieAlgebraTranslation::LieAlgebraTranslation()
{
  basis.resize(2);
  basis[0].addPositive(2);
  basis[1].addPositive(5);
}

LieAlgebraEuclidean::LieAlgebraEuclidean()
{
  LieAlgebraBasisVector vector;
  vector.addPositive(1);
  vector.addNegative(3);

  basis.push_back(vector);
}

LieAlgebraSimilarity::LieAlgebraSimilarity()
{
  LieAlgebraBasisVector vector;
  vector.addPositive(0);
  vector.addPositive(4);

  basis.push_back(vector);
}

LieAlgebraAffine::LieAlgebraAffine()
{
  basis.resize(6);
  basis[0].addPositive(0);
  basis[1].addPositive(1);
  basis[2].addPositive(3);
  basis[3].addPositive(4);
  basis[4].addPositive(2);
  basis[5].addPositive(5);
}

void LieAlgebra::LieAlgebraBasisVector::dot(const Mat &src, Mat dst) const
{
  assert( dst.rows == 1 && dst.cols == src.cols );
  assert( dst.type() == src.type() );

  for (size_t i = 0; i < positives.size(); i++)
    dst += src.row(positives[i]);
  for (size_t i = 0; i < negatives.size(); i++)
    dst -= src.row(negatives[i]);
}

Mat LieAlgebra::LieAlgebraBasisVector::vector2mat(double coordinate) const
{
  const int dim = 3;
  Mat result(dim, dim, CV_64FC1, Scalar(0));

  for (size_t i = 0; i < positives.size(); i++)
  {
    int row = positives[i] / dim;
    int col = positives[i] % dim;
    result.at<double> (row, col) = coordinate;
  }

  for (size_t i = 0; i < negatives.size(); i++)
  {
    int row = negatives[i] / dim;
    int col = negatives[i] % dim;
    result.at<double> (row, col) = -coordinate;
  }

  return result;
}

void LieAlgebra::dot(const Mat &src, Mat &dst) const
{
  dst.create(basis.size(), src.cols, CV_64FC1);
  dst.setTo(0);

  for (size_t i = 0; i < basis.size(); i++)
  {
    basis[i].dot(src, dst.row(i));
  }
}

Mat LieAlgebra::algebra2group(const Mat &lieAlgebraCoords) const
{
  Mat coords = lieAlgebraCoords.reshape(0, 1);
  assert( coords.cols == basis.size() );
  assert( coords.type() == CV_64FC1 );

  const int dim = 3;
  Mat lieAlgebraMat(dim, dim, CV_64F, Scalar(0));
  for (size_t i = 0; i < basis.size(); i++)
  {
    lieAlgebraMat += basis[i].vector2mat(coords.at<double> (0, i));
  }

  Mat lieGroupMat = matrixExp(lieAlgebraMat);
  return lieGroupMat;
}

Mat LieAlgebra::matrixExp(const Mat &mat)
{
  assert( mat.rows == 3 );
  assert( mat.cols == 3 );
  assert( mat.type() == CV_64FC1 );

  Eigen::Matrix<double, 3, 3> eigenMat;
  cv2eigen(mat, eigenMat);
  Eigen::Matrix<double, 3, 3> matExp;
  matExp = eigenMat.exp();

  Mat result;
  eigen2cv(matExp, result);
  return result;
}

void rect2vertices(const Rect &rect, vector<Point2f> &vertices)
{
  vertices.clear();
  vertices.push_back(Point2f(rect.x, rect.y));
  vertices.push_back(Point2f(rect.x + rect.width, rect.y));
  vertices.push_back(Point2f(rect.x + rect.width, rect.y + rect.height));
  vertices.push_back(Point2f(rect.x, rect.y + rect.height));
}
