#include <grid_maze_costmap/layer.hpp>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <algorithm>
#include <queue>

PLUGINLIB_EXPORT_CLASS(grid_maze_costmap::CostmapLayer, costmap_2d::Layer)

namespace grid_maze_costmap
{

CostmapLayer::CostmapLayer()
    : tf_listener_(tf_buffer_)
    , debug_frequency_(5)
    , update_frequency_(10)
    , point_weight_(25)
    , fading_gradient_(0.4)
    , cell_size_(1.0)
    , max_segment_angle_(10.0)
    , max_segment_distance_(0.1)
    , min_segment_overlap_(0.5)
{
}

void CostmapLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);

    it_.reset(new image_transport::ImageTransport(nh));

    global_frame_ = layered_costmap_->getGlobalFrameID();

    nh.param("laser_topic", laser_topic_name_, std::string("/scan"));
    nh.param("point_weight", point_weight_, 25);
    nh.param("fading_gradient", fading_gradient_, 0.4);
    nh.param("debug_frequency", debug_frequency_, 5);
    nh.param("update_frequency", update_frequency_, 10);
    nh.param("cell_size", cell_size_, 1.0);
    nh.param("max_segment_angle", max_segment_angle_, 10.0);
    nh.param("max_segment_distance", max_segment_distance_, 0.5);
    nh.param("min_segment_overlap", min_segment_overlap_, 0.5);

    max_segment_angle_ *= CV_PI / 180.0;

    debug_timer_ = nh.createTimer(
        ros::Duration(1.0 / double(debug_frequency_)),
        &CostmapLayer::publishDebugInformation, this);

    update_timer_ = nh.createTimer(
        ros::Duration(1.0 / double(update_frequency_)),
        &CostmapLayer::updateWalls, this);

    laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(
        laser_topic_name_, 5, 
        &CostmapLayer::laserScanCallback, this
    );

    weight_map_pub_ = it_->advertise(name_ + "/weight_map", 1);

    lines_map_pub_ = it_->advertise(name_ + "/lines_map", 1);
}

void CostmapLayer::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(data_mutex_);

    double range, angle;
    unsigned int ix, iy;
    geometry_msgs::TransformStamped transform;
    geometry_msgs::PointStamped laser_point, map_point;
    cv::Mat actual_weights;

    try
    {
        transform = tf_buffer_.lookupTransform(
            global_frame_, msg->header.frame_id, ros::Time(0)
        );
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Transform exception: %s", ex.what());
    }

    laser_point.header = msg->header;
    laser_point.header.stamp = transform.header.stamp;
    laser_point.point.z = 0.0;

    if(weighted_points_.rows == 0 || weighted_points_.cols == 0)
        weighted_points_ = cv::Mat::zeros(getSizeInCellsY(), getSizeInCellsX(), CV_8UC1);

    actual_weights = cv::Mat::zeros(getSizeInCellsY(), getSizeInCellsX(), CV_8UC1);

    if(weighted_points_.rows != actual_weights.rows || weighted_points_.cols != actual_weights.cols)
        cv::resize(weighted_points_, weighted_points_, cv::Size(getSizeInCellsY(), getSizeInCellsX()));

    for(size_t i = 0; i < msg->ranges.size(); ++i)
    {
        range = msg->ranges[i];

        if(std::isinf(range) || std::isnan(range) || 
           range > msg->range_max || range < msg->range_min)
        {
            continue;
        }

        angle = msg->angle_min + double(i) * msg->angle_increment;
        laser_point.point.x = range * cos(angle);
        laser_point.point.y = range * sin(angle);

        tf2::doTransform(laser_point, map_point, transform);

        if(worldToImage(laser_point.point.x, laser_point.point.y, ix, iy))
        {
            uchar &value = actual_weights.at<uchar>(iy, ix);
            value = uchar(minmax(int(value) + point_weight_, 0, 255));
        }
    }

    cv::addWeighted(weighted_points_, fading_gradient_,
                    actual_weights, 1.0 - fading_gradient_,
                    0.0, weighted_points_);
}

void CostmapLayer::updateWalls(const ros::TimerEvent& event)
{
    boost::mutex::scoped_lock lock(data_mutex_);

    detected_segments_.clear();
    possible_walls_.clear();

    const double cell_pixel_size  = cell_size_ / getResolution();
    const double pixel_resolution = std::max(cell_pixel_size * 0.01, 1.0);
    const double angle_resolution = CV_PI / 180.0;
    const double intersection_thr = int(std::max(cell_pixel_size * 0.2, 1.0));
    const double maximum_gap_size = std::max(cell_pixel_size * 0.05, 1.0);
    const double minimum_points   = std::max(cell_pixel_size * 0.3, 1.0);

    cv::HoughLinesP(weighted_points_, detected_segments_,
                    pixel_resolution, angle_resolution, intersection_thr,
                    minimum_points, maximum_gap_size);

    std::vector<Wall> all_walls = allPossibleWalls();
    Wall wall;
    cv::Point wall_start, wall_end;
    cv::Vec4i line;

    for(std::size_t i = 0; i < detected_segments_.size(); ++i)
    {
        line = detected_segments_[i];

        for(std::size_t j = 0; j < all_walls.size(); ++j)
        {
            wall = all_walls[j];

            ROS_ASSERT(wallCoordinateToImage(wall, wall_start) &&
                       wallCoordinateToImage(wall.next_wall(), wall_end));

            if(areSegmentsClose(SegmentT{ .start = cv::Point2d(line[0], line[1]), 
                                          .end   = cv::Point2d(line[2], line[3]) }, 
                                SegmentT{ .start = wall_start, .end = wall_end }))
            {
                possible_walls_.insert(wall);
            }
        }
    }
}

std::vector<CostmapLayer::Wall> CostmapLayer::allPossibleWalls() const
{
    std::vector<Wall> walls;
    Wall wall;
    unsigned int possibly_cells_x = (unsigned int)(getSizeInMetersX() / getResolution());
    unsigned int possibly_cells_y = (unsigned int)(getSizeInMetersY() / getResolution());

    for(unsigned int x_index = 0; x_index <= possibly_cells_x; ++x_index)
    {
        for(unsigned int y_index = 0; y_index <= possibly_cells_y; ++y_index)
        {
            wall = Wall{ .x_index = x_index,
                         .y_index = y_index,
                         .orientation = Wall::Horizonal };
            if(isWallFitInMap(wall))
                walls.push_back(wall);

            wall.orientation = Wall::Vertical;
            if(isWallFitInMap(wall))
                walls.push_back(wall);
        }
    }

    return walls;
}

void CostmapLayer::publishWeightMap()
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
        std_msgs::Header(), "mono8", weighted_points_
    ).toImageMsg();

    msg->header.stamp = ros::Time::now();

    weight_map_pub_.publish(msg);
}

void CostmapLayer::publishDetectedLinesMap()
{
    cv::Vec4i line;
    cv::Mat line_map;
    cv::Point wall_start, wall_end;
    std::vector<Wall> all_walls;
    Wall wall;

    line_map = cv::Mat::zeros(getSizeInCellsY(), getSizeInCellsX(), CV_8UC3);

    for(size_t i = 0; i < detected_segments_.size(); i++)
    {
        line = detected_segments_[i];
        cv::line(line_map, cv::Point(line[0], line[1]),
                           cv::Point(line[2], line[3]),
                           cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }

    all_walls = allPossibleWalls();
    for(std::size_t i = 0; i < all_walls.size(); ++i)
    {
        wall = all_walls[i];

        ROS_ASSERT(wallCoordinateToImage(wall, wall_start) &&
                   wallCoordinateToImage(wall.next_wall(), wall_end));

        cv::line(line_map, wall_start,
                           wall_end,
                           cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    }

    for(auto it = possible_walls_.begin();
        it != possible_walls_.end();
        ++it)
    {
        wall = (*it);

        ROS_ASSERT(wallCoordinateToImage(wall, wall_start) &&
                   wallCoordinateToImage(wall.next_wall(), wall_end));

        cv::line(line_map, wall_start,
                           wall_end,
                           cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
        std_msgs::Header(), "bgr8", line_map
    ).toImageMsg();

    msg->header.stamp = ros::Time::now();

    lines_map_pub_.publish(msg);
}

void CostmapLayer::publishDebugInformation(const ros::TimerEvent& event)
{
    boost::mutex::scoped_lock lock(data_mutex_);

    publishWeightMap();
    publishDetectedLinesMap();
}

bool CostmapLayer::areSegmentsClose(SegmentT a, SegmentT b) const
{
    const double angle = angleBeetweenSegments(a, b);
    double max_distance = 0.0;
    double overlap;
    int i = 0;
    cv::Point2d projected[2];

    auto project = [this, &i, &projected, &max_distance](cv::Point2d p, SegmentT s) -> void
    {
        if(i < 2 && projectPointOnSegment(p, s, projected[i]))
        {
            max_distance = std::max(vectorLength(p - projected[i]), max_distance);
            ++i;
        }
    };

    project(a.start, b);
    project(a.end,   b);

    ROS_ASSERT(i == 2 || i == 0);

    if(i == 0)
        return false;

    overlap = vectorLength(projected[0] - projected[1]);

    return abs(angle)   <= max_segment_angle_ &&
           max_distance <= (max_segment_distance_ / getResolution()) &&
           overlap > 0.0 &&
           (overlap / vectorLength(b.representation())) >= min_segment_overlap_;
}

void CostmapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double* min_x, double* min_y, double* max_x, double* max_y)
{
    costmap_2d::CostmapLayer::updateBounds(robot_x, robot_y, robot_yaw,
                                           min_x, min_y, max_x, max_y);
}

void CostmapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                               int min_i, int min_j, int max_i, int max_j)
{
    costmap_2d::CostmapLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
}

void CostmapLayer::matchSize()
{
    costmap_2d::CostmapLayer::matchSize();
}

}