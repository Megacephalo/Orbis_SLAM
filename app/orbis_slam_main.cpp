#include <cstdio>
#include <iostream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>

#include "orbis_slam/orbis_slam_pipeline.h"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(Orbis::OrbisSLAMPipeline::create());
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
