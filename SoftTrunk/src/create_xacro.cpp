//
// Created by yasu on 24/10/18.
//
#include "SoftTrunk_common_defs.h"

#include "fmt/core.h"
#include <fstream>

/**
 * @file create_xacro.cpp
 * @brief generates a URDF model of augmented rigid arm using parameters defined in SoftTrunk_common_defs.h. XACRO (and ROS) must be installed on the system.
 */
int main() {
    std::string xacro_filename = fmt::format("./urdf/{}.urdf.xacro", st_params::robot_name);
    std::string urdf_filename = fmt::format("./urdf/{}.urdf", st_params::robot_name);

    std::cout << "generating XACRO file:\t" << xacro_filename << std::endl;
    std::ofstream xacro_file;

    xacro_file.open(xacro_filename);

    xacro_file << "<?xml version='1.0'?>\n"
               << "<!-- This file has been generated automatically from create_xacro.cpp, do not edit by hand -->\n"
               << fmt::format("<robot xmlns:xacro='http://www.ros.org/wiki/xacro' name='{}'>\n", st_params::robot_name)
               << "<xacro:include filename='macro_definitions.urdf.xacro' />\n"
               << "<xacro:empty_link name='base_link'/>\n";

    // write out first PCC element
    // this is written outside for loop because parent of first PCC element must be called base_link
    xacro_file << fmt::format("<xacro:PCC id='0' parent='base_link' child='mid-0' length='{}' mass='{}'/>\n",
                              st_params::lengths[0], st_params::masses[0])
               << "<xacro:empty_link name='mid-0'/>\n";
    // iterate over all the other PCC elements
    for (int i = 1; i < st_params::num_segments; ++i) {
        xacro_file
                << fmt::format("<xacro:PCC id='{}' parent='mid-{}' child='mid-{}' length='{}' mass='{}'/>\n", i, i - 1,
                               i, st_params::lengths[i], st_params::masses[i])
                << fmt::format("<xacro:empty_link name='mid-{}'/>\n", i);
    }
    xacro_file << "</robot>";

    xacro_file.close();
    std::cout << "XACRO file generated." << std::endl;

    std::cout << "generating URDF file (ROS and XACRO must be installed):\t" << urdf_filename << std::endl;
    std::system(fmt::format("rosrun xacro xacro -o {} {}", urdf_filename, xacro_filename).c_str());
    std::cout << "URDF file generated." << std::endl;
}