#include <mir_experiments/TrackingOctomapServer.hpp>

using namespace octomap;

namespace octomap_server {

TrackingOctomapServer::TrackingOctomapServer(const std::string& filename) :
	    OctomapServer()
{
    if (filename != "") {
        if (m_octree->readBinary(filename)) {
            ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), m_octree->size());
            m_treeDepth = m_octree->getTreeDepth();
            m_res = m_octree->getResolution();
            m_gridmap.info.resolution = m_res;

            publishAll();
        } 
        else {
            ROS_ERROR("Could not open requested file %s, exiting.", filename.c_str());
            exit(-1);
        }
    }

    original_tree = new OcTreeT(*m_octree);

    ros::NodeHandle private_nh("~");

    std::string changeSetTopic = "changes";
    std::string changeIdFrame = "/base_link";

    private_nh.param("topic_changes", changeSetTopic, changeSetTopic);
    private_nh.param("change_id_frame", change_id_frame, changeIdFrame);
    private_nh.param("min_change_pub", min_change_pub, 0);

    pubChangeSet = private_nh.advertise<sensor_msgs::PointCloud2>(changeSetTopic, 1);
    m_octree->enableChangeDetection(true);

}

TrackingOctomapServer::~TrackingOctomapServer() {}

void TrackingOctomapServer::insertScan(const tf::Point & sensorOrigin, const PCLPointCloud & ground, const PCLPointCloud & nonground) {
    OctomapServer::insertScan(sensorOrigin, ground, nonground);
    trackChanges();
}

void TrackingOctomapServer::trackChanges() {
    KeyBoolMap::const_iterator startPnt = m_octree->changedKeysBegin();
    KeyBoolMap::const_iterator endPnt = m_octree->changedKeysEnd();
    pcl::PointCloud<pcl::PointXYZI> changedCells = pcl::PointCloud<pcl::PointXYZI>();

    int c = 0;
    for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) {

        OcTreeNode *original_node = original_tree->search(iter->first);

        if (original_node) {
            c++;

            bool occupied = original_tree->isNodeOccupied(original_node);

            point3d center = original_tree->keyToCoord(iter->first);

            pcl::PointXYZI pnt;
            pnt.x = center(0);
            pnt.y = center(1);
            pnt.z = center(2);

            // Reversed the occupied colours because
            // now we are checking the original tree now.
            // ---
            // Red = now free (object missing)
            // Pink = now occupied (new object found)
            pnt.intensity = not occupied ? 1 : -1;

            changedCells.push_back(pnt);
        }
    }

    if (c > min_change_pub) {
        sensor_msgs::PointCloud2 changed;
        pcl::toROSMsg(changedCells, changed);
        changed.header.frame_id = change_id_frame;
        changed.header.stamp = ros::Time().now();
        pubChangeSet.publish(changed);
    }
}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "octomap_map_changes");
    std::string mapFilename("");

    if (argc == 2) {
        mapFilename = std::string(argv[1]);
    }

    try {
        octomap_server::TrackingOctomapServer ms(mapFilename);
        ros::spin();
    }
    catch(std::runtime_error& e){
        ROS_ERROR("octomap_map_changes exception: %s", e.what());
        return -1;
    }

    return 0;
}

