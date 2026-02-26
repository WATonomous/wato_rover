GoalFinderNode::GoalFinderNode()
: Node("goal_finder_node")
{
    // subscribe to known topics
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&GoalFinderNode::mapCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoalFinderNode::odomCallback, this, std::placeholders::_1));

    object_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/yolo_detections", 10, std::bind(&GoalFinderNode::objectCallback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10);

    rotate_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    // timer with 100ms default period
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GoalFinderNode::timerCallback, this));
}

void GoalFinderNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
}

void GoalFinderNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_pose_ = *msg;
}

void GoalFinderNode::objectCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    if (state_ != State::SEARCHING_FOR_OBJECT) {
        return;
    }

    object_message_ = *msg;

    int best_idx = -1;
    float best_conf = 0.0f;

    for (size_t i = 0; i < object_message_.detections.size(); ++i) {
        const auto &detection = object_message_.detections[i];
        if (detection.results.empty()) continue;

        int class_id = detection.results[0].id;
        float conf = detection.results[0].score;

        if (class_id == desired_class_ && conf > best_conf && conf > 0.5f) {
            best_conf = conf;
            best_idx = static_cast<int>(i);
        }
    }

    if (best_idx < 0) {
        // no suitable detection
        return;
    }

    const auto &best_detection = object_message_.detections[best_idx];
    search_angle_ = cameraToAngle(best_detection.bbox);

    // run raytrace; only transition/publish if an occupied cell is found
    if (findGoalPoint(search_angle_)) {
        state_ = State::MOVING_TO_OBJECT;
        goal_pub_->publish(goal_point_);
    } else {
        // no occupied cell found along ray; wait for another detection
        return;
    }
}

void GoalFinderNode::timerCallback()
{
    if (state_ == State::SEARCHING_FOR_OBJECT) {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.5; // radians/sec
        rotate_pub_->publish(twist);
    } else if (state_ == State::MOVING_TO_OBJECT) {
        // noop here; planner/motion layer handles movement to goal
    } else {
        // waiting or other states
    }
}

double GoalFinderNode::cameraToAngle(const vision_msgs::msg::BoundingBox2D &bbox)
{
    // pixel offset from image center
    double dx = static_cast<double>(bbox.center.x) - static_cast<double>(image_width_) / 2.0;

    // compute focal length in pixels from horizontal FOV
    // max_angle_ is half-FOV in degrees (e.g. 45 => 90deg FOV)
    const double fov_rad = 2.0 * max_angle_ * M_PI / 180.0;
    const double focal_px = static_cast<double>(image_width_) / (2.0 * std::tan(fov_rad / 2.0));

    // angle relative to camera forward axis
    return std::atan2(dx, focal_px);
}

bool GoalFinderNode::findGoalPoint(double angle)
{
    // ensure we have a map
    if (map_.data.empty()) {
        return false;
    }

    double resolution = map_.info.resolution;
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;
    int width = static_cast<int>(map_.info.width);
    int height = static_cast<int>(map_.info.height);

    double rx = robot_pose_.pose.pose.position.x;
    double ry = robot_pose_.pose.pose.position.y;

    // extract yaw
    double qx = robot_pose_.pose.pose.orientation.x;
    double qy = robot_pose_.pose.pose.orientation.y;
    double qz = robot_pose_.pose.pose.orientation.z;
    double qw = robot_pose_.pose.pose.orientation.w;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double robot_yaw = std::atan2(siny_cosp, cosy_cosp);

    double theta = robot_yaw + angle;

    const double max_range = 30.0; // meters
    const double step = std::max(0.05, resolution * 0.5); // meters per sample

    for (double s = 0.0; s <= max_range; s += step) {
        double wx = rx + s * std::cos(theta);
        double wy = ry + s * std::sin(theta);

        int mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
        int my = static_cast<int>(std::floor((wy - origin_y) / resolution));

        if (mx < 0 || my < 0 || mx >= width || my >= height) {
            // out of map bounds
            return false;
        }

        int idx = my * width + mx;
        int8_t val = map_.data[idx];

        if (val >= 50) {
            goal_point_.header = map_.header;
            // publish center of the occupied cell
            goal_point_.point.x = origin_x + (mx + 0.5) * resolution;
            goal_point_.point.y = origin_y + (my + 0.5) * resolution;
            goal_point_.point.z = 0.0;
            return true;
        }
    }

    // no obstacle found along ray
    return false;
}


