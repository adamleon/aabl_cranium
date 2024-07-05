#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "cranium_gui/imgui_joint_slider.hpp"


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
        bool show = false;
        ImGui::ShowDemoWindow(&show); 
        ImGui::Begin("Joints");
        static float f1 = 2.0f;
        static float s1 = 90.0f;
        static float f2 = -170.0f;
        s1++;
        if(s1 > 140.0f)
            s1 = -170.0f;
        ImGui::JointSlider("Joint 2", &f1, &s1, &f2, -170.0f, 140.0f);
        //ImGui::SameLine();
        ImGui::Text("Current: %f", f1);
        ImGui::SliderAngle("Joint 3", &f1, -180, 180);
        ImGui::End();
        });

    manager.render();
    rclcpp::shutdown();

    return 0;
}