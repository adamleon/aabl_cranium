#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "cranium_gui/gui_menu.hpp"
#include "cranium_gui/gui_context.hpp"

using namespace std::chrono_literals;

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

    auto manager = GuiManager();
    
    manager.addOnRenderCallback("test", [](){
        static float f = 2.0f;

        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("File"))
            {
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Edit"))
            {
                if (ImGui::MenuItem("Undo", "CTRL+Z")) {}
                if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {}  // Disabled item
                ImGui::Separator();
                if (ImGui::MenuItem("Cut", "CTRL+X")) {}
                if (ImGui::MenuItem("Copy", "CTRL+C")) {}
                if (ImGui::MenuItem("Paste", "CTRL+V")) {}
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        ImGui::Begin("Test Window");

        ImGui::SliderFloat("float", &f, 1.0f, 5.9f);

        ImGui::End();

        ImGui::Begin("Test Window 2");

        ImGui::SliderFloat("float", &f, 1.0f, 5.9f);

        ImGui::End();
    });

    auto scene = manager.addScene("default");
    scene->background = threepp::Color::aqua;

    auto camera = threepp::PerspectiveCamera::create(75, manager.getCanvas()->aspect(), 0.1f, 1000);
    camera->position.z = 5;
    manager.addCamera("default", camera);

    manager.render();

    rclcpp::shutdown();

    return 0;
}