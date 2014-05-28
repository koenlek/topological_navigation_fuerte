#include <kidnapped_robot/place_database.h>
#include <cstdio>

int main(int argc, char** argv)
{
  kidnapped_robot::PlaceDatabase place_db(argv[1], argv[2], argv[3]);

  printf("Should have created %s\n", argv[1]);
  return 0;
}
