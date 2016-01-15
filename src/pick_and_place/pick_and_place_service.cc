#include "pr2_pick_and_place_service.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pick_and_place_service");
  bool save = false, read = false;
  std::string save_file, read_file;
  char c;
  ros::NodeHandle nh;
  while ((c = getopt(argc, argv, "s:r:")) != -1) {
    switch (c) {
      case 's':
        save_file = optarg;
        save = true;
        break;
      case 'r':
        read_file = optarg;
        read = true;
        break;
      case '?':
        if (isprint(optopt))
          printf("Unknown option: %d.\n", optopt);
        return 1;
      default:
        abort();
    }
  }

  // Create Pick Place object
  PickPlace pp("right_arm");

  if (read) {
    printf("Read File: %s\n", read_file.c_str());
    pp.ReadCalibration(read_file);
  } else {
    pp.CalibrateObjects();
  }

  pp.PostParameters();
  if (save) {
    printf("Save File: %s\n", save_file.c_str());
    pp.SaveCalibration(save_file);
  }


  // Advertise the service
  ros::ServiceServer service = nh.advertiseService("pick_and_place_object", &PickPlace::PickAndPlaceObject, &pp);
  printf("READY to PICK and Place\n");
  ros::spin();
  return 0;
}
