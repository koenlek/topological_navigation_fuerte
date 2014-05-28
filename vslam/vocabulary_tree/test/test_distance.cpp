#include <vocabulary_tree/distance.h>
#include <vector>
#include <boost/array.hpp>
#include <cstdio>

int main(int argc, char** argv)
{
  {
    typedef Eigen::Vector4i Feature;
    Feature a(0, 0, 0, 0), b(1, 1, 1, 1);
    vt::distance::L2<Feature> distance;
    printf("Distance = %d\n", distance(a, b));
  }
  {
    typedef Eigen::Vector4f Feature;
    Feature a(0, 0, 0, 0), b(1.6, 2.24, 5.3, -0.512);
    vt::distance::L2<Feature> distance;
    printf("Distance = %f\n", distance(a, b));
  }
  {
    typedef boost::array<double, 2> Feature;
    Feature a, b;
    a[0] = 0.0; a[1] = 3.0;
    b[0] = 4.0; b[1] = 0.0;
    vt::distance::L2<Feature> distance;
    printf("Distance = %f\n", distance(a, b));
  }
  {
    typedef std::vector<uint8_t> Feature;
    Feature a(2), b(2);
    a[0] = 0; a[1] = 3;
    b[0] = 4; b[1] = 0;
    vt::distance::L2<Feature> distance;
    printf("Distance = %d\n", distance(a, b));
  }
}
