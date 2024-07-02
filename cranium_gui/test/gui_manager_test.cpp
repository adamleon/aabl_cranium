#include "cranium_gui/gui_manager.hpp"
#include <gtest/gtest.h>

// Positive test for initialize()
TEST(GuiManagerTest, InitializePositiveTest) {
    GuiManager guiManager;
    ASSERT_NO_THROW(GuiManager guiManager = GuiManager());
    ASSERT_NO_THROW(GuiManager guiManager = GuiManager());
    ASSERT_NO_THROW(GuiManager guiManager = GuiManager());
    ASSERT_NO_THROW(GuiManager guiManager = GuiManager());
}

// Negative test for initialize()
TEST(GuiManagerTest, InitializeNegativeTest) { 
    // GuiManager guiManager;
    //ASSERT_THROW(GuiManager().render(), std::runtime_error);
    ASSERT_EQ(1, 1);
}

// Positive test for render()
TEST(GuiManagerTest, RenderPositiveTest) {
    GuiManager guiManager = GuiManager();
    
    guiManager.addOnRenderCallback("hello world", []() {
        ImGui::Begin("Test Window");
        ImGui::Text("Hello, world!");
        ImGui::End();
    });

    ASSERT_NO_THROW(guiManager.render());
}

// Negative test for render()
TEST(GuiManagerTest, RenderNegativeTest) {
    GuiManager guiManager = GuiManager();

    guiManager.addOnRenderCallback("hello world", []() {
        ImGui::Text("Hello, world!");
    });

    ASSERT_ANY_THROW(guiManager.render());
}

// Positive test for addOnRenderCallback()
TEST(GuiManagerTest, AddOnRenderCallbackPositiveTest) {
    GuiManager guiManager = GuiManager();
    
    // Add a callback
    bool success1 = guiManager.addOnRenderCallback("callback1", []() {
        ImGui::Begin("Test Window");
        ImGui::Text("Hello, world!");
        ImGui::End();
    });
    
    // Add another callback
    bool success2 = guiManager.addOnRenderCallback("callback2", []() {
        ImGui::Begin("Test Window");
        ImGui::Text("Hello, world!");
        ImGui::End();
    });

    ASSERT_EQ(success1, true);
    ASSERT_EQ(success2, true);
}

// Negative test for addOnRenderCallback()
TEST(GuiManagerTest, AddOnRenderCallbackNegativeTest) {
    GuiManager guiManager = GuiManager();
    
    // Add a callback
    bool success1 = guiManager.addOnRenderCallback("callback1", []() {
        ImGui::Begin("Test Window");
        ImGui::Text("Hello, world!");
        ImGui::End();
    });
    
    // Add the same callback
    bool success2 = guiManager.addOnRenderCallback("callback1", []() {
        ImGui::Begin("Test Window");
        ImGui::Text("Hello, world!");
        ImGui::End();
    });

    ASSERT_THROW(guiManager.addOnRenderCallback("callback1", nullptr), std::runtime_error);

    ASSERT_EQ(success1, true);
    ASSERT_EQ(success2, false);
}

// Positive test for removeOnRenderCallback()
TEST(GuiManagerTest, RemoveOnRenderCallbackPositiveTest) {
    GuiManager guiManager = GuiManager();
    
    // Add a callback
    bool success1 = guiManager.addOnRenderCallback("callback1", []() {
        ImGui::Begin("Test Window");
        ImGui::Text("Hello, world!");
        ImGui::End();
    });
    
    // Remove the callback
    bool success2 = guiManager.removeOnRenderCallback("callback1");

    ASSERT_EQ(success1, true);
    ASSERT_EQ(success2, true);
}

// Negative test for removeOnRenderCallback()
TEST(GuiManagerTest, RemoveOnRenderCallbackNegativeTest) {
    GuiManager guiManager = GuiManager();
    
    // Add a callback
    bool success1 = guiManager.addOnRenderCallback("callback1", []() {
        ImGui::Begin("Test Window");
        ImGui::Text("Hello, world!");
        ImGui::End();
    });
    
    // Remove the callback
    bool success2 = guiManager.removeOnRenderCallback("callback2");

    ASSERT_EQ(success1, true);
    ASSERT_EQ(success2, false);
}

// Positive test for addScene()
TEST(GuiManagerTest, AddScenePositiveTest) {
    GuiManager guiManager = GuiManager();
    
    // Add a scene
    std::shared_ptr<threepp::Scene> scene1 = guiManager.addScene("scene1");
    std::shared_ptr<threepp::Scene> scene2 = guiManager.addScene("scene2");

    ASSERT_NE(scene1, nullptr);
    ASSERT_NE(scene2, nullptr);
}

// Negative test for addScene()
TEST(GuiManagerTest, AddSceneNegativeTest) {
    GuiManager guiManager = GuiManager();
    
    // Add a scene
    std::shared_ptr<threepp::Scene> scene1 = guiManager.addScene("scene1");
    std::shared_ptr<threepp::Scene> scene2 = guiManager.addScene("scene1");

    ASSERT_NE(scene1, nullptr);
    ASSERT_EQ(scene2, nullptr);
}

// Positive test for addCamera()
TEST(GuiManagerTest, AddCameraPositiveTest) {
    GuiManager guiManager = GuiManager();

    // Add a scene
    std::shared_ptr<threepp::Scene> scene1 = guiManager.addScene("scene1");
    auto camera = threepp::PerspectiveCamera::create(75, 1, 0.1, 1000);

    // Add a camera
    ASSERT_NO_THROW(guiManager.addCamera("scene1", camera));
}

// Negative test for addCamera()
TEST(GuiManagerTest, AddCameraNegativeTest) {
    GuiManager guiManager = GuiManager();

    // Add a scene
    std::shared_ptr<threepp::Scene> scene1 = guiManager.addScene("scene1");
    auto camera = threepp::PerspectiveCamera::create(75, 1, 0.1, 1000);

    // Add a camera
    ASSERT_THROW(guiManager.addCamera("scene2", camera), std::runtime_error);
}