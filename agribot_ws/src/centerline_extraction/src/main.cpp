#include "rclcpp/rclcpp.hpp"
// #include "centerline_extraction/corn_row_detector.hpp"
#include "centerline_extraction/pure_pursuit_controller.hpp"
#include "centerline_extraction/obstacle_detector.hpp"
#include "centerline_extraction/pid_controller.hpp"

#include <string>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string controller_type = "pid";
    bool enable_obstacle_detector = true;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--controller_type" && i + 1 < argc) {
            controller_type = argv[++i];
        } else if (std::string(argv[i]) == "--disable_obstacle_detector") {
            enable_obstacle_detector = false;
        }
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<ObstacleDetector> obstacle_detector;
    if (enable_obstacle_detector) {
        obstacle_detector = std::make_shared<ObstacleDetector>();
        executor.add_node(obstacle_detector);
    }

    std::shared_ptr<rclcpp::Node> controller_node;
    if (controller_type == "pure_pursuit") {
        controller_node = std::make_shared<PurePursuitController>();
        executor.add_node(controller_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Cornfield navigation system started with pure_pursuit controller");
    } else {
        controller_node = std::make_shared<PIDController>();
        executor.add_node(controller_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Cornfield navigation system started with pid controller");
    }
    if (!enable_obstacle_detector) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Obstacle detector disabled for path-tracking test");
    }

    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
