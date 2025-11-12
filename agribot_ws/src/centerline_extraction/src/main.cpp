#include "rclcpp/rclcpp.hpp"
// #include "centerline_extraction/corn_row_detector.hpp"
// #include "centerline_extraction/pure_pursuit_controller.hpp"
// #include "centerline_extraction/obstacle_detector.hpp"
#include "centerline_extraction/pid_controller.hpp"
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // 创建节点
    // auto corn_row_detector = std::make_shared<CornRowDetector>();
    // auto controller = std::make_shared<PurePursuitController>();
    // auto obstacle_detector = std::make_shared<ObstacleDetector>();
    auto pid_controller = std::make_shared<PIDController>();
    
    // 组成执行器并运行
    rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(corn_row_detector);
    // executor.add_node(controller);
    // executor.add_node(obstacle_detector);
    executor.add_node(pid_controller);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Cornfield navigation system started");
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
