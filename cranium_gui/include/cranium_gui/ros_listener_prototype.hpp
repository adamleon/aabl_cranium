#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "cranium_gui/gui_manager.hpp"

class ListenerNode : public rclcpp::Node
{
public:
    ListenerNode(std::shared_ptr<GuiManager> manager) : Node("listener_node")
    {
        displayMessage = "No message received";
        this->m_manager = *manager;
        subscription_ = create_subscription<std_msgs::msg::String>(
            "/chatter",
            10, std::bind(&ListenerNode::listenCallback, this, std::placeholders::_1));

        m_manager.addOnRenderCallback("listener", std::bind(&ListenerNode::render, this));
    }

    void listenCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
        message = msg->data;
        // displayMessage = message;
        // m_manager.invokeLater([&]() {
        //     std::cout << "Invoked" << std::endl;
        //     displayMessage = "hey " + message;
        // });

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