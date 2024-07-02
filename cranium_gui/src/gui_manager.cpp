#include "cranium_gui/gui_manager.hpp"

void GuiManager::initialize() {
    // Create a canvas and renderer
    s_canvas = std::make_shared<threepp::Canvas>("Cranium");
    s_renderer = std::make_shared<threepp::GLRenderer>(s_canvas->size());
    s_renderer->autoClear = false;

    // Resize the renderer when the window is resized
    s_canvas->onWindowResize([&](threepp::WindowSize size) {
        for (auto &[scene, camera] : GuiManager::s_cameras) {
            if(typeid(*camera) == typeid(threepp::PerspectiveCamera))
                (std::static_pointer_cast<threepp::PerspectiveCamera>(camera))->aspect = size.aspect();
            camera->updateProjectionMatrix();
        }
        s_renderer->setSize(size);
    });
 
    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking
    // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // Enable Multi-Viewport / Platform Windows

    // Setup Dear ImGui style
    // ImGui::StyleColorsDark();
    ImGui::StyleColorsLight();

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
    ImGuiStyle &style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    ImGui_ImplGlfw_InitForOpenGL((GLFWwindow*)s_canvas->windowPtr(), true);
#if EMSCRIPTEN 
    ImGui_ImplOpenGL3_Init("#version 300 es");
#else
    ImGui_ImplOpenGL3_Init("#version 330 core");
#endif
}

void GuiManager::render() {
    if(!s_initialized) throw std::runtime_error("GUI has not been initialized");

    // Render the GUI
    s_canvas->animate([&]()
            {
        s_renderer->clear();
        s_taskManager.handleTasks();

        // Render all scenes and cameras
        for (auto &[scene, camera] : GuiManager::s_cameras)
        {
            s_renderer->render(*scene, *camera);
        }

        // Start the ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Run all callbacks for rendering
        for (auto &[name, callback] : GuiManager::s_onRenderCallbacks)
        {
            callback();
        }

        // Render ImGui
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    });
}