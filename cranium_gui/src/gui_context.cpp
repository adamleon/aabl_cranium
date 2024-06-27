#include "cranium_gui/gui_context.hpp"

void GuiContext::render()
{
    auto wrapperFunction = [](
            std::unordered_map<std::shared_ptr<threepp::Scene>, std::shared_ptr<threepp::Camera>> cameras, 
            std::shared_ptr<threepp::GLRenderer> renderer) {
                std::cout << "Amount of cameras in wrapper: " << cameras.size() << std::endl;
            return [&]() {
                renderer->clear();
                std::cout << "Amount of cameras: " << cameras.size() << std::endl;
                for (auto &[scene, camera] : cameras) {
                    std::cout << "Rendering scene" << std::endl;
                    renderer->render(*scene, *camera);
                }
        };
    };

    auto lambdaFunction = wrapperFunction(GuiContext::m_cameras, GuiContext::m_renderer);
    
    std::cout << "Amount of cameras in Renderer: " << GuiContext::m_cameras.size() << std::endl;

    GuiContext::m_canvas->animate(lambdaFunction);

    // ImGui_ImplOpenGL3_NewFrame();
    // ImGui_ImplGlfw_NewFrame();
    // ImGui::NewFrame();

    onRender();

    // ImGui::Render();
    // ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void GuiContext::initCanvas()
{
    GuiContext::m_canvas = std::make_unique<threepp::Canvas>("Canvas");
    GuiContext::m_renderer = std::make_unique<threepp::GLRenderer>(GuiContext::m_canvas->size());
    GuiContext::m_renderer->autoClear = false;

    GuiContext::m_scenes["default"] = threepp::Scene::create();
    auto scene = GuiContext::m_scenes["default"];
    scene->background = threepp::Color::aliceblue;
    
    auto camera = threepp::PerspectiveCamera::create(75, m_canvas->aspect(), 0.1f, 1000);
    camera->position.z = 5;
    GuiContext::m_cameras[scene] = camera;

    std::cout << "Amount of cameras: " << GuiContext::m_cameras.size() << std::endl; 

    GuiContext::m_canvas->onWindowResize([&](threepp::WindowSize size) {
        for (auto &[scene, camera] : GuiContext::m_cameras) {
            if(typeid(*camera) == typeid(threepp::PerspectiveCamera))
                dynamic_cast<std::shared_ptr<threepp::PerspectiveCamera>&>(*camera)->aspect = size.aspect();
            camera->updateProjectionMatrix();
        }
        GuiContext::m_renderer->setSize(size);
    });
}

void GuiContext::initImgui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // Enable Multi-Viewport / Platform Windows

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

    ImGui_ImplGlfw_InitForOpenGL((GLFWwindow *)m_canvas->windowPtr(), true);
#if EMSCRIPTEN
    ImGui_ImplOpenGL3_Init("#version 300 es");
#else
    ImGui_ImplOpenGL3_Init("#version 330 core");
#endif
}