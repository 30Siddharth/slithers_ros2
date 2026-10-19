/**
 * @file slithers_node.cpp
 * @brief ROS 2 node for the SLITHERS Lie-theory IK planner.
 *
 * C++/ROS 2 port of MATLAB's lie_theory/main.m.
 * All algorithm code lives in the slithers library (libslithers); this node
 * only handles parameters and publishing.
 * Loads a robot model, generates four reference SE(3) paths, steps a
 * slithers::Planner along each, and publishes the resulting poses and
 * joint velocity commands.
 *
 * Publishers
 * ----------
 *   /slithers/tool_pose        geometry_msgs/msg/PoseStamped
 *   /slithers/pose_goal        geometry_msgs/msg/PoseStamped
 *   /slithers/joint_velocities std_msgs/msg/Float64MultiArray
 *
 * Parameters (with defaults matching MATLAB main.m)
 * -------------------------------------------------
 *   robot_type     string   "holo" | "husky"           default: "holo"
 *   dt             double   integration time step [s]  default: 0.1
 *   time           double   total trial duration [s]   default: 20.0
 *   num_paths      int      number of path trials      default: 3
 *   lambda_e       double[] pose-error weights (6)     default: [25x6]
 *   lambda_j       double   jerk weight scalar         default: 0.001
 *   lambda_v_husky double[] per-joint vel weights (8)
 *   lambda_v_holo  double[] per-joint vel weights (9)
 */

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "slithers/slithers.hpp"

namespace slithers {

// ---------------------------------------------------------------------------
// Helper: SE(3) matrix  -->  PoseStamped
// ---------------------------------------------------------------------------
/**
 * @brief Convert a 4x4 SE(3) homogeneous transform to a ROS 2 PoseStamped.
 *
 * The upper-left 3x3 rotation block is extracted and converted to a unit
 * quaternion via Eigen::Quaterniond so that all four orientation fields are
 * populated consistently.
 *
 * @param T        4x4 SE(3) transform (rotation | translation)
 * @param frame_id TF frame name written into the message header
 * @param clock    Node clock used to generate the timestamp
 */
static geometry_msgs::msg::PoseStamped
matrix4d_to_pose_stamped(const Eigen::Matrix4d &        T,
                          const std::string &             frame_id,
                          const rclcpp::Clock::SharedPtr &clock)
{
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = clock->now();
    ps.header.frame_id = frame_id;

    // Translation
    ps.pose.position.x = T(0, 3);
    ps.pose.position.y = T(1, 3);
    ps.pose.position.z = T(2, 3);

    // Rotation: extract 3x3 block and convert to quaternion
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);
    q.normalize();
    ps.pose.orientation.w = q.w();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();

    return ps;
}

// ---------------------------------------------------------------------------
// SlithersNode
// ---------------------------------------------------------------------------
class SlithersNode : public rclcpp::Node
{
public:
    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------
    SlithersNode()
    : rclcpp::Node("slithers_node")
    {
        // ---- Declare ROS 2 parameters with MATLAB-equivalent defaults ------
        declare_parameter<std::string>("robot_type", "holo");
        declare_parameter<double>("dt",              0.1);
        declare_parameter<double>("time",            20.0);
        declare_parameter<int>("num_paths",          3);

        // Pose-error weight diagonal (6-vector)
        declare_parameter<std::vector<double>>(
            "lambda_e", {25.0, 25.0, 25.0, 25.0, 25.0, 25.0});

        // Jerk regularisation scalar (reserved for future use)
        declare_parameter<double>("lambda_j", 0.001);

        // Per-joint velocity weights — robot-type-specific sizes
        declare_parameter<std::vector<double>>(
            "lambda_v_husky", {1.0, 1.0, 0.25, 0.25, 0.1, 0.1, 0.1, 0.1});
        declare_parameter<std::vector<double>>(
            "lambda_v_holo",  {1.0, 1.0, 1.0,  0.25, 0.25, 0.1, 0.1, 0.1, 0.1});

        // ---- Read parameters into member variables -------------------------
        robot_type_ = get_parameter("robot_type").as_string();
        dt_         = get_parameter("dt").as_double();
        time_       = get_parameter("time").as_double();
        num_paths_  = get_parameter("num_paths").as_int();
        lambda_j_   = get_parameter("lambda_j").as_double();

        // lambda_e -> fixed 6-vector
        {
            auto v = get_parameter("lambda_e").as_double_array();
            if (v.size() != 6) {
                RCLCPP_WARN(get_logger(),
                            "lambda_e must have 6 elements (got %zu). "
                            "Using defaults.", v.size());
                v = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
            }
            lambda_e_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(v.data());
        }

        // lambda_v -> sized to match the selected robot's DOF
        {
            const std::string key = (robot_type_ == "husky")
                                    ? "lambda_v_husky" : "lambda_v_holo";
            auto v = get_parameter(key).as_double_array();
            lambda_v_ = Eigen::Map<Eigen::VectorXd>(
                v.data(), static_cast<Eigen::Index>(v.size()));
        }

        // ---- Create publishers --------------------------------------------
        pub_tool_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/slithers/tool_pose", 10);

        pub_pose_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/slithers/pose_goal", 10);

        pub_joint_vel_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/slithers/joint_velocities", 10);

        RCLCPP_INFO(get_logger(),
                    "SlithersNode initialised — robot_type=%s  dt=%.3f  "
                    "time=%.1f  num_paths=%d",
                    robot_type_.c_str(), dt_, time_, num_paths_);

        // ---- One-shot timer: run IK loop after node spin begins -----------
        // A 100 ms delay ensures publishers are advertised before we start.
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                timer_->cancel();   // fire only once
                run_ik_loop();
            });
    }

private:
    // -----------------------------------------------------------------------
    // run_ik_loop  --  equivalent to MATLAB lie_theory/main.m
    // -----------------------------------------------------------------------
    void run_ik_loop()
    {
        // ---- 1. Load robot model ------------------------------------------
        const RobotModel robot = (robot_type_ == "husky")
                                 ? husky_ur5e()
                                 : husky_ur5e_holo();
        const int total_dof = robot.base_dof + robot.arm_dof;

        // Validate that lambda_v matches the robot's total DOF
        if (lambda_v_.size() != total_dof) {
            RCLCPP_WARN(get_logger(),
                        "lambda_v size (%ld) != robot DOF (%d). "
                        "Padding / truncating.",
                        static_cast<long>(lambda_v_.size()), total_dof);
            Eigen::VectorXd tmp = Eigen::VectorXd::Ones(total_dof) * 0.1;
            int copy_n = static_cast<int>(
                std::min<long>(lambda_v_.size(), total_dof));
            tmp.head(copy_n) = lambda_v_.head(copy_n);
            lambda_v_ = tmp;
        }

        // ---- 2. Build the planner (library owns the IK loop) --------------
        PlannerConfig config;
        config.dt       = dt_;
        config.lambda_e = lambda_e_;
        config.lambda_j = lambda_j_;
        config.lambda_v = lambda_v_;
        Planner planner(robot, config);

        const int steps = static_cast<int>(std::round(time_ / dt_));
        RCLCPP_INFO(get_logger(),
                    "steps=%d  base_dof=%d  arm_dof=%d  total_dof=%d",
                    steps, robot.base_dof, robot.arm_dof, total_dof);

        // ---- 3. Generate reference paths ---------------------------------
        // Parameters mirror the MATLAB main.m defaults:
        //   helix            : a=0.5, b=0.05
        //   sine             : a=0.0 (unused), b=0.3
        //   horizontal_helix : a=0.3, b=0.15
        //   spiral_sine      : a=2.0, b=0.3
        const std::vector<std::vector<Eigen::Matrix4d>> all_paths = {
            generate_helix(           0.5,  0.05, dt_, time_),
            generate_sine(            0.0,  0.3,  dt_, time_),
            generate_horizontal_helix(0.3,  0.15, dt_, time_),
            generate_spiral_sine(     2.0,  0.3,  dt_, time_)
        };

        // ---- 4. IK trial loop -------------------------------------------
        for (int trial = 0; trial < num_paths_; ++trial)
        {
            // Select path (cycle through the four available paths)
            const std::vector<Eigen::Matrix4d> & path =
                all_paths[trial % static_cast<int>(all_paths.size())];

            const int path_steps = static_cast<int>(path.size());
            RCLCPP_INFO(get_logger(),
                        "Trial %d/%d — path steps = %d",
                        trial + 1, num_paths_, path_steps);

            // Reset joint state and base pose for each new trial
            planner.reset();

            for (int step = 0; step < path_steps && rclcpp::ok(); ++step)
            {
                const Eigen::Matrix4d & ee_desired = path[step];
                const StepResult r = planner.step(ee_desired);

                pub_tool_pose_->publish(
                    matrix4d_to_pose_stamped(r.tool_pose, "world", get_clock()));

                pub_pose_goal_->publish(
                    matrix4d_to_pose_stamped(ee_desired, "world", get_clock()));

                std_msgs::msg::Float64MultiArray jv_msg;
                jv_msg.data.assign(r.joint_velocities.data(),
                                   r.joint_velocities.data() + r.joint_velocities.size());
                pub_joint_vel_->publish(jv_msg);
            }  // step loop

            RCLCPP_INFO(get_logger(), "Trial %d complete.", trial + 1);

        }  // trial loop

        RCLCPP_INFO(get_logger(),
                    "All %d trials complete. Node remains alive.", num_paths_);
    }

    // -----------------------------------------------------------------------
    // Member variables
    // -----------------------------------------------------------------------

    // Parameters
    std::string                        robot_type_;
    double                             dt_;
    double                             time_;
    int                                num_paths_;
    Eigen::Matrix<double, 6, 1>        lambda_e_;
    double                             lambda_j_;
    Eigen::VectorXd                    lambda_v_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_tool_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_pose_goal_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joint_vel_;

    // One-shot startup timer
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace slithers

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<slithers::SlithersNode>());
    rclcpp::shutdown();
    return 0;
}
