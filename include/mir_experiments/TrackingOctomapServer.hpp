#pragma once

#include "octomap_server/OctomapServer.h"

namespace octomap_server {

class TrackingOctomapServer: public OctomapServer {
public:
    TrackingOctomapServer(const std::string& filename = "");
    virtual ~TrackingOctomapServer();

    void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

protected:
    void trackChanges();

    int min_change_pub;
    std::string change_id_frame;
    ros::Publisher pubChangeSet;
};

} 

