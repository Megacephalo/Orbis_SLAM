#include <cstdio>
#include <iostream>
#include <iomanip>



#include "orbis_slam/zed_setup_help_utils.h"
#include "orbis_slam/zed_wrapper.h"

int main(int argc, char ** argv) {
    std::cout << "Launching get_camera_parameters" << std::endl;

    Orbis::ZEDWrapper zed_wrapper;
    bool camera_ready = zed_wrapper.setup(argc, argv);
    if ( ! camera_ready ) {
        std::cerr << "Camera setup failed. Exit." << std::endl;
        return EXIT_FAILURE;
    }

    while (zed_wrapper.hasNewFrame()) {
        zed_wrapper.grabFrame();

        cv::imshow("ZED Stream", zed_wrapper.getStereoImage());
        cv::waitKey(1);
    }

    return EXIT_SUCCESS;
}
