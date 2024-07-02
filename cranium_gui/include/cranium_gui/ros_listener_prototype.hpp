#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cranium_gui/gui_manager.hpp"

class ListenerNode : public rclcpp::Node
{
public:
    ListenerNode() : Node("listener_node")
    {
        m_manager = GuiManager();
        subscription_ = create_subscription<std_msgs::msg::String>(
            "/chatter",
            10, std::bind(&ListenerNode::listenCallback, this, std::placeholders::_1));

        m_manager.addOnRenderCallback("listener", std::bind(&ListenerNode::render, this));
    }

    void listenCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
        message = msg->data;
        m_manager.invokeLater([this, msg]() {
            std::cout << "Invoked Message" << std::endl;
            this->displayMessage = msg->data;
        });

    }

    void render() {
        ImGui::Begin("Listener");

        ImGui::Text("Message: %s", displayMessage.c_str());

        ImGui::End();
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string message;
    std::string displayMessage;
    GuiManager m_manager;
};