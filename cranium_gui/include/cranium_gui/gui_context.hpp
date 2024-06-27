#ifndef GUI_CONTEXT_HPP
#define GUI_CONTEXT_HPP

#include <stdexcept>
#include <functional>
#include <utility>
#include <iostream>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "threepp/threepp.hpp"

/*!
 * @brief GUI context class which creates a new ImGui context and handles rendering
*/
class GuiContext {
public:
    explicit GuiContext() {
        initCanvas();
        //initImgui();
    }

    GuiContext(GuiContext&&) = delete;
    GuiContext(const GuiContext&) = delete;
    GuiContext& operator=(const GuiContext&) = delete;

    std::shared_ptr<threepp::Canvas>& getCanvas() {
        return GuiContext::m_canvas;
    }

    void render();

    std::shared_ptr<threepp::Scene> addScene(const std::string& name) {
        GuiContext::m_scenes[name] = threepp::Scene::create();
        return GuiContext::m_scenes[name];
    }

    void addCamera(const std::string& sceneName, const std::shared_ptr<threepp::Camera>& camera) {
        if(GuiContext::m_scenes.find(sceneName) == GuiContext::m_scenes.end()) throw std::runtime_error("Scene not found");
        GuiContext::m_cameras[GuiContext::m_scenes[sceneName]] = camera;
    }

    ~GuiContext() {
        // ImGui_ImplOpenGL3_Shutdown();
        // ImGui_ImplGlfw_Shutdown();
        // ImGui::DestroyContext();
    }

protected:
    virtual void onRender() = 0;

    inline static std::shared_ptr<threepp::Canvas> m_canvas;
    inline static std::shared_ptr<threepp::GLRenderer> m_renderer;
    inline static std::unordered_map<std::string, std::shared_ptr<threepp::Scene>> m_scenes;
    inline static std::unordered_map<std::shared_ptr<threepp::Scene>, std::shared_ptr<threepp::Camera>> m_cameras;

    void initCanvas();
    void initImgui();
};

class GuiFunctionalContext: public GuiContext {

public:
    explicit GuiFunctionalContext(std::function<void()> f)
        : GuiContext(),
          f_(std::move(f)) {}


protected:
    void onRender() override {
        f_();
    }

private:
    std::function<void()> f_;
};

#endif // GUI_CONTEXT_HPP
