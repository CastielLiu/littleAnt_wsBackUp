//
// Created by adam on 18-10-8.
//

#include "euclidean_cluster_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "euclidean_cluster");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    EuClusterCore core(nh , private_nh);
    
    return 0;
    
}
