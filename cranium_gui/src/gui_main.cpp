#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "imgui.h"

#include "cranium_gui/gui_menu.hpp"
#include "cranium_gui/gui_manager.hpp"
#include "cranium_gui/ros_listener_prototype.hpp"

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node
{
public:
    TalkerNode() : Node("talker_node")
    {
        count = 0;
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&TalkerNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto listener = std::make_shared<ListenerNode>();

    std::thread listener_thread([&listener]() {
        rclcpp::spin(listener);
    });

    GuiManager manager = GuiManager();
    GuiManager manager2 = GuiManager();

    manager2.addOnRenderCallback("menu1", []()
                                 {
        bool show = true;
        ImGui::ShowDemoWindow(&show); });

    Menu menu = Menu();
    manager.render();
    rclcpp::shutdown();

    return 0;
}