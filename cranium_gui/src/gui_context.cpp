#include "cranium_gui/gui_context.hpp"

void GuiManager::onAnimation() {
    s_renderer->clear();
    for (auto &[scene, camera] : s_cameras) {
        s_renderer->render(*scene, *camera);
    }
    
    glfwPollEvents();
    
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    ImGui::SetNextWindowPos({0, 0}, 0, {0, 0});
    ImGui::SetNextWindowSize({230, 0}, 0);

    for (auto &[name, callback] : s_onRenderCallbacks)
    {
        callback();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    ImGuiIO &io = ImGui::GetIO();

    // Update and Render additional Platform Windows
    // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
    //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        GLFWwindow* backup_current_context = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(backup_current_context);
    }
}

void GuiManager::render()
{   
    GuiManager::s_canvas->animate( std::bind(&GuiManager::onAnimation, this));
}

void GuiManager::initCanvas()
{
    GuiManager::s_canvas = std::make_shared<threepp::Canvas>("Canvas");
    GuiManager::s_renderer = std::make_shared<threepp::GLRenderer>(GuiManager::s_canvas->size());
    GuiManager::s_renderer->autoClear = false;

    GuiManager::s_canvas->onWindowResize([&](threepp::WindowSize size) {
        for (auto &[scene, camera] : GuiManager::s_cameras) {
            if(typeid(*camera) == typeid(threepp::PerspectiveCamera))
                (std::static_pointer_cast<threepp::PerspectiveCamera>(camera))->aspect = size.aspect();
            camera->updateProjectionMatrix();
        }
        GuiManager::s_renderer->setSize(size);
    });
}

void GuiManager::initImgui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking
    //io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // Enable Multi-Viewport / Platform Windows

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
    ImGuiStyle &style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    ImGui_ImplGlfw_InitForOpenGL((GLFWwindow *)s_canvas->windowPtr(), true);
#if EMSCRIPTEN
    ImGui_ImplOpenGL3_Init("#version 300 es");
#else
    ImGui_ImplOpenGL3_Init("#version 330 core");
#endif
}