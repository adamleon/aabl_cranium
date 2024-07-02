#ifndef GUI_CONTEXT_HPP
#define GUI_CONTEXT_HPP

#include <stdexcept>
#include <functional>
#include <utility>
#include <iostream>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include "threepp/threepp.hpp"

/*!
 * @brief GUI context class which creates a new ImGui context and handles rendering
 */
class GuiManager
{
public:
    GuiManager()
    {
        s_instances++;
        if (!s_initialized)
        {
            initialize();
            s_initialized = true;
        }
    }

    bool addOnRenderCallback(std::string name, std::function<void()> callback)
    {
        if (callback == nullptr)
            throw std::runtime_error("Callback is null");
        if (GuiManager::s_onRenderCallbacks.find(name) != GuiManager::s_onRenderCallbacks.end())
            return false;
        GuiManager::s_onRenderCallbacks[name] = callback;
        std::cout << s_onRenderCallbacks.size() << std::endl;
        return true;
    }

    bool removeOnRenderCallback(std::string name)
    {
        if (GuiManager::s_onRenderCallbacks.find(name) == GuiManager::s_onRenderCallbacks.end())
            return false;
        GuiManager::s_onRenderCallbacks.erase(name);
        return true;
    }

    std::shared_ptr<threepp::Scene> addScene(const std::string &name)
    {
        GuiManager::s_scenes[name] = threepp::Scene::create();
        return GuiManager::s_scenes[name];
    }

    void addCamera(const std::string &sceneName, const std::shared_ptr<threepp::Camera> &camera)
    {
        if (GuiManager::s_scenes.find(sceneName) == GuiManager::s_scenes.end())
            throw std::runtime_error("Scene not found");
        GuiManager::s_cameras[GuiManager::s_scenes[sceneName]] = camera;
    }

    void render();

    ~GuiManager()
    {
        s_instances--;
        if(s_instances == 0) {
            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
            s_initialized = false;
        }
    }

private:
    inline static std::shared_ptr<threepp::Canvas> s_canvas;
    inline static std::shared_ptr<threepp::GLRenderer> s_renderer;
    inline static std::unordered_map<std::string, std::shared_ptr<threepp::Scene>> s_scenes;
    inline static std::unordered_map<std::shared_ptr<threepp::Scene>, std::shared_ptr<threepp::Camera>> s_cameras;
    inline static std::unordered_map<std::string, std::function<void()>> s_onRenderCallbacks = {};
    inline static bool s_initialized = false;
    inline static int s_instances = 0;

    void initialize();
};

#endif // GUI_CONTEXT_HPP