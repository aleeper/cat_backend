
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


  int threads = 2; // default 2 threads
  for (int i = 1 ; i < argc ; ++i)
    if (strncmp(argv[i], "--threads", 9) == 0)
    {
      threads = atoi(argv[i+1]);
      break;
    }

  ros::AsyncSpinner spinner(threads);
  cat::CatBackend cb(debug);
  ROS_INFO("Starting async spinner with %d threads.", threads);
  spinner.start();
  ros::waitForShutdown();
  ROS_INFO("Done, shutting down.");
  return 0;

}
