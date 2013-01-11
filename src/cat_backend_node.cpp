
#include "cat_backend/cat_backend.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cat_backend_node");

  bool debug = false;
  for (int i = 1 ; i < argc ; ++i)
    if (strncmp(argv[i], "--debug", 7) == 0)
    {
      debug = true;
      break;
    }

  cat::CatBackend cb(debug);
  ros::spin();
  return 0;

}
