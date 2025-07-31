#include <rclcpp/rclcpp.hpp>
#include "TURTLMap/PosegraphBackendOnline.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string config_file = "/path/to/config.yaml";
    if (argc > 1) {
        config_file = argv[1];
    }

    auto backend = std::make_shared<pose_graph_backend::PosegraphBackendOnline>(config_file);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(backend);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
