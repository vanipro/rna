#include <grid_maze_map/layer.hpp>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <cmath>
#include <algorithm>
#include <queue>


MapLayer::MapLayer()
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
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    onInitialize();
}

void MapLayer::onInitialize()
{
    ros::NodeHandle nh("~/");

    it_.reset(new image_transport::ImageTransport(nh));

    nh.param("laser_topic", laser_topic_name_, std::string("/scan"));
    nh.param("point_weight", point_weight_, 25);
    nh.param("fading_gradient", fading_gradient_, 0.4);
    nh.param("debug_frequency", debug_frequency_, 5);
    nh.param("update_frequency", update_frequency_, 10);
    nh.param("cell_size", cell_size_, 0.5);
    nh.param("max_segment_angle", max_segment_angle_, 10.0);
    nh.param("max_segment_distance", max_segment_distance_, 0.5);
    nh.param("min_segment_overlap", min_segment_overlap_, 0.5);
    nh.param("global_frame", global_frame_, std::string("map"));
    nh.param("resolution", resolution_, 0.01);
    nh.param("width", width_, 8.0);
    nh.param("height", height_, 8.0);

    max_segment_angle_ *= CV_PI / 180.0;

    debug_timer_ = nh.createTimer(
        ros::Duration(1.0 / double(debug_frequency_)),
        &MapLayer::publishDebugInformation, this);

    update_timer_ = nh.createTimer(
        ros::Duration(1.0 / double(update_frequency_)),
        &MapLayer::updateWalls, this);

    laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(
        laser_topic_name_, 5, 
        &MapLayer::laserScanCallback, this
    );

    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

    weight_map_pub_ = it_->advertise("weight_map", 1);

    lines_map_pub_ = it_->advertise("lines_map", 1);
}

void MapLayer::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(data_mutex_);

    double range, angle;
    unsigned int ix, iy;
    geometry_msgs::TransformStamped transform;
    geometry_msgs::PointStamped laser_point, map_point;
    cv::Mat actual_weights;

    sensor_msgs::PointCloud2 cloud_in_laser, cloud_in_map;

    try {
        projector_.projectLaser(*msg, cloud_in_laser, 50.0, laser_geometry::channel_option::Index);

        tf_buffer_.transform(cloud_in_laser, cloud_in_map, global_frame_, ros::Duration(0.1));
            
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Transform failed: %s", ex.what());
    }

    laser_point.header = msg->header;
    laser_point.header.stamp = transform.header.stamp;
    laser_point.point.z = 0.0;

    if(weighted_points_.rows == 0 || weighted_points_.cols == 0)
        weighted_points_ = cv::Mat::zeros(getSizeInCellsY(), getSizeInCellsX(), CV_8UC1);

    actual_weights = cv::Mat::zeros(getSizeInCellsY(), getSizeInCellsX(), CV_8UC1);

    if(weighted_points_.rows != actual_weights.rows || weighted_points_.cols != actual_weights.cols)
        ROS_ASSERT("Bad size");

    bool has_x = false, has_y = false, has_z = false;
for (const auto& field : cloud_in_map.fields) {
	if(field.name == "x") has_x = true;
	if(field.name == "y") has_y = true;
	if(field.name == "z") has_z = true;
}
	if(!has_x || !has_y || !has_z) return;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in_map, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in_map, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in_map, "z");
    //for(size_t i = 0; i < msg->ranges.size(); ++i)
    //{
    for (;iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;

        //range = msg->ranges[i];

        //if(std::isinf(range) || std::isnan(range) || 
        //   range > msg->range_max || range < msg->range_min)
        //{
        //    continue;
        //}

        //angle = msg->angle_min + double(i) * msg->angle_increment;
        laser_point.point.x = x;
        laser_point.point.y = y;
	if(std::isinf(x) || std::isinf(y)) continue;
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

void MapLayer::updateWalls(const ros::TimerEvent& event)
{
    boost::mutex::scoped_lock lock(data_mutex_);

    detected_segments_.clear();
    possible_walls_.clear();

    const double cell_pixel_size  = cell_size_ / getResolution();
    const double pixel_resolution = std::max(cell_pixel_size * 0.01, 1.0);
    const double angle_resolution = CV_PI / 180.0;
    const double intersection_thr = int(std::max(cell_pixel_size * 0.1, 1.0));
    const double maximum_gap_size = std::max(cell_pixel_size * 0.3, 1.0);
    const double minimum_points   = std::max(cell_pixel_size * 0.1, 1.0);

    cv::Mat bin_points = cv::Mat::zeros(weighted_points_.rows, weighted_points_.cols, CV_8UC1);
    cv::threshold(weighted_points_, bin_points, 20, 255, cv::THRESH_BINARY);
    //cv::threshold(weighted_points_, bin_points, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    cv::HoughLinesP(bin_points, detected_segments_,
                    pixel_resolution, angle_resolution, intersection_thr,
                    minimum_points, maximum_gap_size);

    std::vector<Wall> all_walls = allPossibleWalls();
    Wall wall;
    cv::Point wall_start, wall_end;
    cv::Vec4i line;

    //ROS_INFO("%d", detected_segments_.size());
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

    if(possible_walls_.size() || grid.data.size() == 0)
    {
        grid.header.frame_id = global_frame_;
        grid.header.stamp = ros::Time::now();
        grid.info.resolution = getResolution();
        grid.info.width = getSizeInCellsX();
        grid.info.height = getSizeInCellsY();

        grid.info.origin.position.x = 0.0;
        grid.info.origin.position.y = 0.0;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;

        int map_size = grid.info.width * grid.info.height;
        ROS_ASSERT(grid.data.size() == 0 || grid.data.size() == map_size);

        if(grid.data.size() == 0)
            grid.data.resize(map_size, -1);

        for(auto it = possible_walls_.begin();
            it != possible_walls_.end();
            ++it)
        {
            wall = (*it);

            ROS_ASSERT(wallCoordinateToImage(wall, wall_start) &&
                       wallCoordinateToImage(wall.next_wall(), wall_end));

            if(wall.orientation == Wall::Horizonal)
            {
                for(unsigned int i = wall_start.x; i < wall_end.x; ++i)
                {
                    unsigned int index = getIndex(minmax(i, 0, getSizeInCellsX() - 1), minmax(wall_start.y, 0, getSizeInCellsY() - 1));
                    //ROS_INFO("%d %d", index, grid.data.size());
                    //ROS_ASSERT(index >= grid.data.size(), );
                    ROS_ASSERT_MSG(index < grid.data.size(), "%d %d %d %d %d", index, grid.data.size(), i, wall_start.y, getSizeInCellsX());
                    grid.data[index] = 100;
                }
            }
            else
            {
                for(unsigned int i = wall_start.y; i < wall_end.y; ++i)
                {
                    unsigned int index = getIndex(minmax(wall_start.x, 0, getSizeInCellsX() - 1), minmax(i, 0, getSizeInCellsY() - 1));
                    ROS_ASSERT_MSG(index < grid.data.size(), "%d %d %d %d", index, grid.data.size(), wall_start.x, i);
                    grid.data[index] = 100;
                }
            }
        }

        map_pub_.publish(grid);
    }
}

std::vector<MapLayer::Wall> MapLayer::allPossibleWalls() const
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

void MapLayer::publishWeightMap()
{
    cv::Mat bin_points;
    cv::threshold(weighted_points_, bin_points, 20, 255, cv::THRESH_BINARY);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
        std_msgs::Header(), "mono8", bin_points
    ).toImageMsg();

    msg->header.stamp = ros::Time::now();

    weight_map_pub_.publish(msg);
}

void MapLayer::publishDetectedLinesMap()
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

void MapLayer::publishDebugInformation(const ros::TimerEvent& event)
{
    boost::mutex::scoped_lock lock(data_mutex_);

    publishWeightMap();
    publishDetectedLinesMap();
}

bool MapLayer::areSegmentsClose(SegmentT a, SegmentT b) const
{
    const double angle = angleBeetweenSegments(a, b);
    double max_distance = 1000000.0;
    double overlap;
    int i = 0;
    cv::Point2d projected[4];

    auto project = [this, &i, &projected, &max_distance](cv::Point2d p, SegmentT s) -> void
    {
        if(i < 4 && projectPointOnSegment(p, s, projected[i]))
        {
            max_distance = std::min(vectorLength(p - projected[i]), max_distance);
            ++i;
        }
    };

    project(a.start, b);
    project(a.end,   b);
    project(b.start, a);
    project(b.end,   a);

    ROS_ASSERT(i == 4);

    /*
    if(abs(b.start.x - 150) <= 1.0 && abs(b.start.y - 150) <= 1.0 && abs(a.start.x - 150) <= 1.0 && abs(a.start.y - 150) <= 1.0)
    {
        ROS_INFO("===============");
        ROS_INFO("%f %f %f", std::min(abs(CV_PI - abs(angle)), abs(angle)), max_distance, overlap / vectorLength(b.representation()));
        ROS_INFO("a = (%f, %f), (%f, %f)", a.start.x, a.start.y, a.end.x, a.end.y);
        ROS_INFO("b = (%f, %f), (%f, %f)", b.start.x, b.start.y, b.end.x, b.end.y);
        ROS_INFO("proj = (%f, %f), (%f, %f)", projected[0].x, projected[0].y, projected[1].x, projected[1].y);
    }
    */


    if(i == 0)
        return false;

    overlap = vectorLength(projected[0] - projected[1]);

    return std::min(abs(CV_PI - abs(angle)), abs(angle))   <= max_segment_angle_ &&
           max_distance <= (max_segment_distance_ / getResolution()) &&
           overlap > 0.0 &&
           (overlap / vectorLength(b.representation())) >= min_segment_overlap_;
}

/*
void MapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double* min_x, double* min_y, double* max_x, double* max_y)
{
    boost::mutex::scoped_lock lock(data_mutex_);
    Wall wall;
    cv::Point wall_start, wall_end;

    for(auto it = possible_walls_.begin();
        it != possible_walls_.end();
        ++it)
    {
        wall = (*it);

        ROS_ASSERT(wallCoordinateToImage(wall, wall_start) &&
                   wallCoordinateToImage(wall.next_wall(), wall_end));

        double px, py;

        if(wall.orientation == Wall::Horizonal)
        {
            for(unsigned int i = wall_start.x; i < wall_end.x; ++i)
            {
                unsigned int index = getIndex(i, wall_start.y);
                costmap_[index] = costmap_2d::LETHAL_OBSTACLE;
                mapToWorld(i, wall_start.y, px, py);
                touch(px, py, min_x, min_y, max_x, max_y);
            }
        }
        else
        {
            for(unsigned int i = wall_start.y; i < wall_end.y; ++i)
            {
                unsigned int index = getIndex(wall_start.x, i);
                costmap_[index] = costmap_2d::LETHAL_OBSTACLE;
                mapToWorld(wall_start.x, i, px, py);
                touch(px, py, min_x, min_y, max_x, max_y);
            }
        }
    }
}

void MapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                               int min_i, int min_j, int max_i, int max_j)
{
    boost::mutex::scoped_lock lock(data_mutex_);
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

void MapLayer::matchSize()
{
    costmap_2d::MapLayer::matchSize();
}
*/
