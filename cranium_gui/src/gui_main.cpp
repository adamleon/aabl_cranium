#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "cranium_gui/gui_menu.hpp"
#include "cranium_gui/gui_manager.hpp"
#include "cranium_gui/ros_listener_prototype.hpp"

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node {
public:
    TalkerNode() : Node("talker_node") {
        count = 0;
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&TalkerNode::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<GuiManager> manager = std::make_shared<GuiManager>();

    auto talker = std::make_shared<TalkerNode>();
    auto listener = std::make_shared<ListenerNode>(manager);

    std::thread t([talker, listener]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(talker);
        executor.add_node(listener);
        executor.spin();
    });

    // manager->invokeLater([]() {
    //     std::cout << "Invoked" << std::endl;
    // }, 3.0f);


    manager->addOnRenderCallback("menu1", []() {
        ImGui::Begin("Test Window");
        static float f1 = 2.0f;
        ImGui::SliderFloat("float", &f1, 1.0f, 5.9f);

        ImGui::End();
    });
    std::shared_ptr<Menu> menu = std::make_shared<Menu>(manager);

    // auto scene = manager->addScene("default");
    // scene->background = threepp::Color::aqua;

    // auto camera = threepp::PerspectiveCamera::create(75, manager->getCanvas()->aspect(), 0.1f, 1000);
    // camera->position.z = 5;
    // manager->addCamera("default", camera);

    manager->run();

    rclcpp::shutdown();

    return 0;
}