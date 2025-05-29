#include <px4_safety_lib/PX4Safety.hpp>

using namespace std::chrono_literals;

namespace px4_safety_lib {
    PX4Safety::PX4Safety() {}
    PX4Safety::~PX4Safety() {
        delete node_;
        node_ = nullptr;
    }

    void PX4Safety::initialize(rclcpp::Node *set_node) {
        //Set the node so we can pub/sub
        node_ = set_node;

        //Get node parameters
        RCLCPP_WARN(node_->get_logger(), "Initializing PX4 safety library.");

        //Get safety parameters from ros params
        // geometry_msgs::msg::Point fence_min, fence_max;
        // double fence_a, fence_b, fence_p;
        // double obs_a, obs_b, obs_p;
        node_->declare_parameter("safety.min_x", 0.0);
        node_->declare_parameter("safety.max_x", 0.0);
        node_->declare_parameter("safety.min_y", 0.0);
        node_->declare_parameter("safety.max_y", 0.0);
        node_->declare_parameter("safety.min_z", 0.0);
        node_->declare_parameter("safety.max_z", 0.0);

        node_->declare_parameter("safety.fence_a", 0.0);
        node_->declare_parameter("safety.fence_b", 0.0);
        node_->declare_parameter("safety.fence_p", 0.0);
        node_->declare_parameter("safety.obs_a", 0.0);
        node_->declare_parameter("safety.obs_b", 0.0);
        node_->declare_parameter("safety.obs_p", 0.0);

        if (
            node_->get_parameter("safety.min_x", fence_min_.x) &&
            node_->get_parameter("safety.max_x", fence_max_.x) && 
            node_->get_parameter("safety.min_y", fence_min_.y) && 
            node_->get_parameter("safety.max_y", fence_max_.y) && 
            node_->get_parameter("safety.min_z", fence_min_.z) &&
            node_->get_parameter("safety.max_z", fence_max_.z) &&
            node_->get_parameter("safety.fence_a", fence_a_) && 
            node_->get_parameter("safety.fence_b", fence_b_) && 
            node_->get_parameter("safety.fence_p", fence_p_) && 
            node_->get_parameter("safety.obs_a", obs_a_) &&
            node_->get_parameter("safety.obs_b", obs_b_) && 
            node_->get_parameter("safety.obs_p", obs_p_)
        ) {
            RCLCPP_WARN(node_->get_logger(), "(PX4Safety) Virtual fence set to (%.4f, %.4f), (%.4f, %.4f), (%.4f, %.4f)", 
                fence_min_.x, fence_max_.x, fence_min_.y, fence_max_.y, fence_min_.z, fence_max_.z);
            RCLCPP_WARN(node_->get_logger(), "(PX4Safety) Safety influence gains set to: Fence=(%.4f, %.4f, %.4f), Obstacle=(%.4f, %.4f, %.4f)", 
                fence_a_, fence_b_, fence_p_, obs_a_, obs_b_, obs_p_);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "(PX4Safety) Safety parameters not set. Exiting.");
            rclcpp::shutdown();
            return; 
        }

        //Compute rho_min (activation distance)
        rho_min_fence_ = fence_p_*(1+fence_b_)/fence_b_;
        rho_min_obs_ = obs_p_*(1+obs_b_)/obs_b_;

        //Get list of obstacles for collision avoidance
        // std::vector<std::string> empty_vect;
        // node_->declare_parameter("obstacles", empty_vect);
        // if (node_->get_parameter("obstacles", obstacles_)) {
        //     std::string obstacle_str;
        //     geometry_msgs::msg::Pose empty_pose;
        //     for (int i = 0; i < (int)obstacles_.size(); i++) {
        //         obstacle_str.append(obstacles_[i]);

        //         if (i < (int)obstacles_.size()-1) {
        //             obstacle_str.append(", ");
        //         }

        //         //Push back empty obstacle pose
        //         obs_poses_.poses.push_back(empty_pose);
        //     }

        //     RCLCPP_INFO(node_->get_logger(), "(PX4Safety) Initialized with the following obstacles for obstacle avoidance: %s", obstacle_str.c_str());
        // } 
        //else {
            RCLCPP_WARN(node_->get_logger(), "(PX4Safety) No obstacles provided.");
        //}

        //Check if safety visualization is enabled
        node_->declare_parameter("safety.enable_viz", false);
        node_->get_parameter("safety.enable_viz", enable_viz_);

        if (enable_viz_ && obstacles_.size() > 0) {
            RCLCPP_INFO(node_->get_logger(), "(PX4Safety) Safety visualization enabled.");

            //Precompute post colors
            std_msgs::msg::ColorRGBA obs_color;
            obs_color.a = 0.5;
            obs_color.r = 100.0;
            obs_color.b = 0.0;
            obs_color.g = 0.0;

            //Precompute post scales
            geometry_msgs::msg::Vector3 obs_scale;
            obs_scale.x = obs_scale.y = rho_min_obs_;
            obs_scale.z = 5.0;

            rclcpp::Time now = node_->get_clock()->now();

            for (size_t i=0; i < obstacles_.size(); i++) {

                visualization_msgs::msg::Marker obs_marker;
                obs_marker.header.stamp = now;
                obs_marker.header.frame_id = "autonomy_park";
                obs_marker.action = visualization_msgs::msg::Marker::ADD;

                obs_marker.ns = "obstacles";
                obs_marker.id = i;
                obs_marker.type = visualization_msgs::msg::Marker::CYLINDER;
                obs_marker.scale = obs_scale;
                obs_marker.color = obs_color;

                obs_marker.pose.position.x = obs_poses_.poses[i].position.x;
                obs_marker.pose.position.y = obs_poses_.poses[i].position.y;
                obs_marker.pose.position.z = obs_scale.z/2.0;
                obs_marker.pose.orientation.w = 1.0;

                obs_markers_.markers.push_back(obs_marker);
            }

            obs_viz_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", rclcpp::QoS(1));

            // Publish obstacle visualization at 10 Hz
            obs_viz_timer_ = node_->create_wall_timer(100ms, std::bind(&PX4Safety::publish_obs_viz, this));
        }

        //Dynamically subscribe to obstacle poses
        for (int i = 0; i < (int)obstacles_.size(); i++) {
            std::string obs_sub_str = "/" + obstacles_[i] + "/autonomy_park/pose";
            std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr msg)> obs_callback_i = std::bind(&PX4Safety::obstacle_pose_callback, this, std::placeholders::_1, i);
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr obs_sub_i = node_->create_subscription<geometry_msgs::msg::PoseStamped>(obs_sub_str, 10, obs_callback_i);
            obs_pose_subs_.push_back(obs_sub_i);
        }

        //Hardcode maximum influence magnitude for now
        max_influence_ = 1.0;
    }

    void PX4Safety::obstacle_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg, int obs_id) {
         // RCLCPP_INFO(node_->get_logger(), "Got pose for obstacle %s", obstacles_[obs_id].c_str());

        obs_poses_.poses[obs_id] = pose_msg->pose;

        if (enable_viz_) {
            //Update visualization marker position if enabled
            obs_markers_.markers[obs_id].pose.position.x = pose_msg->pose.position.x;
            obs_markers_.markers[obs_id].pose.position.y = pose_msg->pose.position.y;
        }
    }

    void PX4Safety::publish_obs_viz() {
        obs_viz_publisher_->publish(obs_markers_);
    }

    // void PX4Safety::set_params(
    //         geometry_msgs::msg::Point set_fence_min, 
    //         geometry_msgs::msg::Point set_fence_max, 
    //         double set_fence_a, double set_fence_b, double set_fence_p,
    //         double set_obs_a, double set_obs_b, double set_obs_p
    // ) {
    //     fence_min_ = set_fence_min;
    //     fence_max_ = set_fence_max;
    //     fence_a_ = set_fence_a;
    //     fence_b_ = set_fence_b;
    //     fence_p_ = set_fence_p;
    //     obs_a_ = set_obs_a;
    //     obs_b_ = set_obs_b;
    //     obs_p_ = set_obs_p;

    //     char buf[128];
    //     sprintf(buf, "(PX4 Safety) Virtual fence set to (%.4f, %.4f), (%.4f, %.4f), (%.4f, %.4f)\n", 
    //         fence_min_.x, fence_max_.x, fence_min_.y, fence_max_.y, fence_min_.z, fence_max_.z);
    //     std::cout << std::string(buf) << std::endl;
    //     sprintf(buf, "(PX4 Safety) Influence gains set to: Fence=(a:%.4f, b:%.4f, p:%.4f), Obstacle=(a:%.4f, b:%.4f, p:%.4f)\n", 
    //         fence_a_, fence_b_, fence_p_, obs_a_, obs_b_, obs_p_);
    //     std::cout << std::string(buf) << std::endl;
    // }

    geometry_msgs::msg::Point PX4Safety::normalize_vector(geometry_msgs::msg::Point vector_in) {
        geometry_msgs::msg::Point vector_out;
        double norm_v = vector_norm(vector_in);
        vector_out.x = vector_in.x/norm_v;
        vector_out.y = vector_in.y/norm_v;
        vector_out.z = vector_in.z/norm_v;
        return vector_out;
    }

    double PX4Safety::vector_norm(geometry_msgs::msg::Point vector_in) {
        return sqrt(vector_in.x*vector_in.x + vector_in.y*vector_in.y + vector_in.z*vector_in.z);
    }

    double PX4Safety::euclidean_distance(geometry_msgs::msg::Point vector0, geometry_msgs::msg::Point vector1) {
        geometry_msgs::msg::Point vector_sub;
        vector_sub.x = vector1.x - vector0.x;
        vector_sub.y = vector1.y - vector0.y;
        vector_sub.z = vector1.z - vector0.z;
        return vector_norm(vector_sub);
    }

    geometry_msgs::msg::Point PX4Safety::vector_sum(geometry_msgs::msg::Point vector_0, geometry_msgs::msg::Point vector_1) {
        geometry_msgs::msg::Point vector_out;
        vector_out.x = vector_0.x + vector_1.x;
        vector_out.y = vector_0.y + vector_1.y;
        vector_out.z = vector_0.z + vector_1.z;
        return vector_out;
    }

    double PX4Safety::fence_influence_function(double dist) {
        double influence_magnitude;
        if (dist == 0) {
            influence_magnitude = max_influence_;
        } else if (dist > 0 && dist <= fence_p_) {
            influence_magnitude = fence_a_*pow(dist, -fence_b_);
        } else if (dist > fence_p_ && dist <= rho_min_fence_) {
            influence_magnitude = -fence_a_*fence_b_*pow(fence_p_, -(fence_b_+1))*dist + fence_a_*pow(fence_p_,-fence_b_)*(1+fence_b_);
        } else {
            influence_magnitude = 0.0;
        }

        influence_magnitude = std::min(influence_magnitude, max_influence_);

        return influence_magnitude;
    }

    double PX4Safety::obs_influence_function(double dist) {
        double influence_magnitude;
        if (dist == 0) {
            influence_magnitude = max_influence_;
        } else if (dist > 0 && dist <= obs_p_) {
            influence_magnitude = obs_a_*pow(dist, -obs_b_);
        } else if (dist > obs_p_ && dist <= rho_min_obs_) {
            influence_magnitude = -obs_a_*obs_b_*pow(obs_p_, -(obs_b_+1))*dist + obs_a_*pow(obs_p_,-obs_b_)*(1+obs_b_);
        } else {
            influence_magnitude = 0.0;
        }

        influence_magnitude = std::min(influence_magnitude, max_influence_);

        // if (influence_magnitude > 0.0) {
        //     std::cout << "Obstacle influence = " << std::to_string(influence_magnitude) << " @ " << std::to_string(dist) << " meters!" << std::endl;
        // }

        return influence_magnitude;
    }

    //Function to compute safe velocity commands
    geometry_msgs::msg::Twist PX4Safety::compute_safe_cmd_vel(geometry_msgs::msg::Pose agent_pose, geometry_msgs::msg::Twist cmd_vel_in) {

        std::vector<geometry_msgs::msg::Point> influence_vectors;

        //First, compute influence of virtual fence
        double norm_x_min = fabs(agent_pose.position.x - fence_min_.x);
        double norm_x_max = fabs(agent_pose.position.x - fence_max_.x);

        double norm_y_min = fabs(agent_pose.position.y - fence_min_.y);
        double norm_y_max = fabs(agent_pose.position.y - fence_max_.y);

        double norm_z_min = fabs(agent_pose.position.z - fence_min_.z);
        double norm_z_max = fabs(agent_pose.position.z - fence_max_.z);

        geometry_msgs::msg::Point x_min_inf, x_max_inf, y_min_inf, y_max_inf, z_min_inf, z_max_inf;
        if (agent_pose.position.x > fence_min_.x) {
            x_min_inf.x = ((agent_pose.position.x - fence_min_.x)/norm_x_min) * fence_influence_function(norm_x_min);
        } else {
            x_min_inf.x = max_influence_;
        }

        if (agent_pose.position.x < fence_max_.x) {
            x_max_inf.x = ((agent_pose.position.x - fence_max_.x)/norm_x_max) * fence_influence_function(norm_x_max);
        } else {
            x_min_inf.x = -max_influence_;
        }
        
        if (agent_pose.position.y > fence_min_.y) {
            y_min_inf.y = ((agent_pose.position.y - fence_min_.y)/norm_y_min) * fence_influence_function(norm_y_min);
        } else {
            y_min_inf.y = max_influence_;
        } 

        if (agent_pose.position.y < fence_max_.y) {
            y_max_inf.y = ((agent_pose.position.y - fence_max_.y)/norm_y_max) * fence_influence_function(norm_y_max);
        } else {
            y_min_inf.y = -max_influence_;
        }

        if (agent_pose.position.z > fence_min_.z) {
            z_min_inf.z = ((agent_pose.position.z - fence_min_.z)/norm_z_min) * fence_influence_function(norm_z_min);
        } else {
            z_min_inf.z = max_influence_;
        } 

        if (agent_pose.position.z < fence_max_.z) {
            y_max_inf.z = ((agent_pose.position.z - fence_max_.z)/norm_z_max) * fence_influence_function(norm_z_max);
        } else {
            y_min_inf.z = -max_influence_;
        }

        //Push virtual fence influences onto the stack
        influence_vectors.push_back(x_min_inf);
        influence_vectors.push_back(x_max_inf);

        influence_vectors.push_back(y_min_inf);
        influence_vectors.push_back(y_max_inf);

        influence_vectors.push_back(z_min_inf);
        influence_vectors.push_back(z_max_inf);

        //Loop through relevant poses and compute their influence
        for (int i = 0; i < (int)obs_poses_.poses.size(); i++) {
            geometry_msgs::msg::Point obs_influence;

            //Only include X/Y influence since we don't allow Z overlap
            geometry_msgs::msg::Point obs_to_agent;
            obs_to_agent.x = agent_pose.position.x - obs_poses_.poses[i].position.x;
            obs_to_agent.y = agent_pose.position.y - obs_poses_.poses[i].position.y;
            obs_to_agent.z = 0.0;

            double obs_to_agent_norm = vector_norm(obs_to_agent);
            double obs_influence_mag = obs_influence_function(obs_to_agent_norm);

            obs_influence.x = (obs_to_agent.x/obs_to_agent_norm)*obs_influence_mag;
            obs_influence.y = (obs_to_agent.y/obs_to_agent_norm)*obs_influence_mag;
            // obs_influence.z = (obs_to_agent.z/obs_to_agent_norm)*agent_influence_function(obs_to_agent_norm);
            obs_influence.z = 0.0;

            // ROS_WARN_STREAM(std::string("bebop pose:") << agent_pose << std::endl << std::string("agent pose:") << obs_poses.poses[i]);
            // ROS_WARN_STREAM(agent_influence);
            // ROS_WARN_STREAM(std::string("agent to bebop norm: ") << agent_to_bebop_norm);

            influence_vectors.push_back(obs_influence);
        }

        geometry_msgs::msg::Point influence_sum;
        for (int i = 0; i < (int)influence_vectors.size(); i++) {
            influence_sum = vector_sum(influence_sum, influence_vectors[i]);
        }

        //Add influence vectors to velocity command
        geometry_msgs::msg::Twist cmd_vel_out;
        cmd_vel_out.linear.x = cmd_vel_in.linear.x + influence_sum.x;
        cmd_vel_out.linear.y = cmd_vel_in.linear.y + influence_sum.y;
        cmd_vel_out.linear.z = cmd_vel_in.linear.z + influence_sum.z;

        cmd_vel_out.angular.z = cmd_vel_in.angular.z;

        //Return the safe velocity command to the controller
        return cmd_vel_out;
    }
}