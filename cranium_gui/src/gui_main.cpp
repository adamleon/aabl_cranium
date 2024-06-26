#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "imgui.h"
#include "threepp/threepp.hpp"


using namespace std::chrono_literals;
using namespace threepp;

class ThreePPNode : public rclcpp::Node {
public:
    ThreePPNode() : Node("threepp_node") {
        
    }
};

class TalkerNode : public rclcpp::Node {
public:
    TalkerNode() : Node("talker_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&TalkerNode::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::thread t([]() {
        rclcpp::spin(std::make_shared<TalkerNode>());
    });
    
    Canvas canvas("threepp demo");
        GLRenderer renderer(canvas.size());
        renderer.autoClear = false;

        auto scene = Scene::create();
        scene->background = Color::aliceblue;
        auto camera = PerspectiveCamera::create(75, canvas.aspect(), 0.1f, 1000);
        camera->position.z = 5;

        canvas.onWindowResize([&](WindowSize size) {
            camera->aspect = size.aspect();
            camera->updateProjectionMatrix();
            renderer.setSize(size);
        });

        Clock clock;
        canvas.animate([&]() {
            renderer.clear();
            renderer.render(*scene, *camera);
        });

    rclcpp::shutdown();
}