#include "imx_stereo/dual_camera.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
