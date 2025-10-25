#ifndef CUSTOM_COSTMAP_LAYER_HPP
#define CUSTOM_COSTMAP_LAYER_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <unordered_set>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <map>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>


#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


class MapLayer
{

    struct SegmentT
    {
        cv::Point2d start;
        cv::Point2d end;

        cv::Point2d representation() const
        {
            return end - start;
        }
    };

    struct Wall
    {
        typedef enum
        {
            None,
            Vertical,
            Horizonal,
        } Orinteation;

        unsigned int x_index;
        unsigned int y_index;
        Orinteation orientation;

        inline Wall next_wall() const
        {
            ROS_ASSERT(orientation != None);

            return Wall{ .x_index = x_index + int(orientation == Horizonal),
                         .y_index = y_index + int(orientation == Vertical),
                         .orientation = orientation };
        }

        bool operator<(const Wall &rhs) const
        {
            if (x_index < rhs.x_index)
                return true;

            if (x_index == rhs.x_index && y_index < rhs.y_index)
                return true;

            return false;
        }
    };

public:
    MapLayer();

    virtual void onInitialize();

private:
    inline double getResolution() const
    {
        return resolution_;
    }

    inline unsigned int getSizeInCellsX() const
    {
        return (unsigned int)(width_ / resolution_);
    }

    inline unsigned int getSizeInCellsY() const
    {
        return (unsigned int)(height_ / resolution_);
    }

    inline double getSizeInMetersX() const
    {
        return width_;
    }

    inline double getSizeInMetersY() const
    {
        return height_;
    }
    inline unsigned int getIndex(unsigned int x, unsigned int y) const
    {
        return y * getSizeInCellsX() + x;
    }

    inline int minmax(int x, int mn, int mx) const { return std::min(std::max(x, mn), mx); }

    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
    {
        // First, transform world coordinates to map frame
        double dx = wx - grid.info.origin.position.x;
        double dy = wy - grid.info.origin.position.y;
        
        // Apply rotation (if any) - most maps have identity rotation
        double map_x = dx * grid.info.origin.orientation.w + dy * grid.info.origin.orientation.z;
        double map_y = -dx * grid.info.origin.orientation.z + dy * grid.info.origin.orientation.w;
        
        // Convert to cell coordinates
        int _mx = static_cast<int>(std::floor(map_x / getResolution()));
        int _my = static_cast<int>(std::floor(map_y / getResolution()));
        
        bool condition = (_mx >= 0 && _mx < static_cast<int>(getSizeInCellsX()) && 
                          _my >= 0 && _my < static_cast<int>(getSizeInCellsY()));

        if(condition)
        {
            mx = static_cast<unsigned int>(_mx);
            my = static_cast<unsigned int>(_my);
        }

        // Check bounds
        return condition;
    }

    inline bool worldToImage(double wx, double wy, unsigned int &ix, unsigned int &iy) const
    {
        bool status = worldToMap(wx, wy, ix, iy);
        return status;
    }

    inline bool wallCoordinateToImage(Wall w, cv::Point &i) const
    {
        const double wx = w.x_index * cell_size_;
        const double wy = w.y_index * cell_size_;
        unsigned int ix, iy;

        ix = (unsigned int)(wx / getResolution());
        iy = (unsigned int)(wy / getResolution());
        if(ix < 0 || iy < 0 || ix >= getSizeInCellsX() || iy >= getSizeInCellsY())
            return false;

        i = cv::Point(ix, iy);
        return true;
    }

    inline bool isWallFitInMap(Wall w) const
    {
        cv::Point i;
        (void)i;

        return wallCoordinateToImage(w, i) && wallCoordinateToImage(w.next_wall(), i);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void updateWalls(const ros::TimerEvent& event);

    inline double vectorLengthSquared(cv::Point2d a) const { return std::pow(a.x, 2) + std::pow(a.y, 2); }
    inline double vectorLength(cv::Point2d a) const { return std::sqrt(vectorLengthSquared(a)); }

    inline bool projectPointOnSegment(cv::Point2d point, SegmentT segment, cv::Point2d &projected) const
    {
        const cv::Point2d v        = segment.representation();
        const cv::Point2d radius_v = point - segment.start;
        double t = radius_v.dot(v) / v.dot(v);

        t = std::min(std::max(t, 0.0), 1.0);

        projected = segment.start + t * v;
        return true;
    }

    inline double angleBeetweenSegments(SegmentT a, SegmentT b) const
    {
        const cv::Point2d av = a.representation();
        const cv::Point2d bv = b.representation();
        const double a_length = vectorLength(av);
        const double b_length = vectorLength(bv);

        ROS_ASSERT(a_length * b_length > 0.0);

        const double cosinus = av.dot(bv) / (a_length * b_length);

        return std::acos(cosinus);
    }

    bool areSegmentsClose(SegmentT a, SegmentT b) const;

    std::vector<Wall> allPossibleWalls() const;

    void publishWeightMap();
    void publishDetectedLinesMap();
    void publishDebugInformation(const ros::TimerEvent& event);

    ros::Subscriber laser_sub_;
    ros::Publisher map_pub_;
    image_transport::Publisher weight_map_pub_;
    image_transport::Publisher lines_map_pub_;

    ros::Timer debug_timer_;
    ros::Timer update_timer_;

    boost::mutex data_mutex_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string laser_topic_name_;
    std::string global_frame_;

    int debug_frequency_;
    int point_weight_;
    int update_frequency_;

    double fading_gradient_;
    double cell_size_;

    double max_segment_angle_;
    double max_segment_distance_;
    double min_segment_overlap_;
    double resolution_;
    double width_, height_;

    cv::Mat weighted_points_;
    boost::scoped_ptr<image_transport::ImageTransport> it_;

    std::vector<cv::Vec4i> detected_segments_;
    std::set<Wall> possible_walls_;
    laser_geometry::LaserProjection projector_;
    nav_msgs::OccupancyGrid grid;
};

#endif