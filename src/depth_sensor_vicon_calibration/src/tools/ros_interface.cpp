
#include "tools/ros_interface.hpp"

namespace ri
{

void PublishMarker(const Eigen::Matrix3f R, const Eigen::Vector3f t,
        std_msgs::Header header,
        std::string object_model_path,
        const ros::Publisher &pub,
        int marker_id, float r, float g, float b, float a)
{

    Eigen::Quaternion<float> q(R);

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header =  header;
    pose.pose.pose.position.x = t(0);
    pose.pose.pose.position.y = t(1);
    pose.pose.pose.position.z = t(2);

    pose.pose.pose.orientation.x = q.x();
    pose.pose.pose.orientation.y = q.y();
    pose.pose.pose.orientation.z = q.z();
    pose.pose.pose.orientation.w = q.w();


    visualization_msgs::Marker marker;
    marker.header.frame_id =  pose.header.frame_id;
    marker.header.stamp =  pose.header.stamp;
    marker.ns = "object_pose_estimation";
    marker.id = marker_id;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose.pose.pose;

    marker.mesh_resource = object_model_path;

    pub.publish(marker);
}

void PublishMarker(const Eigen::Matrix4f H,
        std_msgs::Header header,
        std::string object_model_path,
        const ros::Publisher &pub,
        int marker_id, float r, float g, float b, float a)
{
    PublishMarker(H.topLeftCorner(3,3), H.topRightCorner(3,1), header, object_model_path, pub,
            marker_id, r, g, b, a);
}

void PublishPoints(const std_msgs::Header header,
                   const ros::Publisher &pub,
                   const std::vector<Eigen::Vector3f> points,
                   std::vector<float> colors,
                   const Eigen::Matrix3f R,
                   Eigen::Vector3f t)
{
    // if no color has been given we set it to some value -----------------------------
    if(colors.size() == 0)
        colors = std::vector<float> (points.size(), 1);

    // renormalize colors -----------------------------
    float max = -std::numeric_limits<float>::max();
    float min = std::numeric_limits<float>::max();
    for(int i = 0; i < int(colors.size()); i++)
    {
        min = colors[i] < min ? colors[i] : min;
        max = colors[i] > max ? colors[i] : max;
    }
    if(min == max) min = 0;

    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    point_cloud.header = header;
    point_cloud.width    = points.size(); point_cloud.height   = 1; point_cloud.is_dense = false;
    point_cloud.points.resize (point_cloud.width * point_cloud.height);

    for (int point_index = 0; point_index < int(points.size()); ++point_index)
    {
        Eigen::Vector3f new_point = R*points[point_index] + t;
        point_cloud.points[point_index].x = new_point(0);
        point_cloud.points[point_index].y = new_point(1);
        point_cloud.points[point_index].z = new_point(2);

        point_cloud.points[point_index].r = (colors[point_index]-min)/(max-min) * 255.;
        point_cloud.points[point_index].g = 0.;
        point_cloud.points[point_index].b = (1 - (colors[point_index]-min)/(max-min)) * 255.;
    }
    sensor_msgs::PointCloud2 point_cloud2;
    pcl::toROSMsg(point_cloud, point_cloud2);
    pub.publish(point_cloud2);
}

}
