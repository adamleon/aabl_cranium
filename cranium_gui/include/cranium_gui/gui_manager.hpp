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
    // @brief Constructor for the GUI manager
    GuiManager()
    {
        s_instances++; // Keep track of the number of instances of the GUI manager
        // If this is the first instance of the GUI manager, initialize the GUI
        if (!s_initialized)
        {
            initialize();
            s_initialized = true;
        }
    }

    // @brief Add a callback to be called on render
    // @param name The name of the callback. Has to be unique
    // @param callback The callback to be called on render
    // @return True if the callback was added, false if the name is already in use
    // @throw std::runtime_error if the callback is null
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

    // @brief Remove a callback from the render loop
    // @param name The name of the callback to remove
    // @return True if the callback was removed, false if the callback was not found
    bool removeOnRenderCallback(std::string name)
    {
        if (GuiManager::s_onRenderCallbacks.find(name) == GuiManager::s_onRenderCallbacks.end())
            return false;
        GuiManager::s_onRenderCallbacks.erase(name);
        return true;
    }

    // @brief Add a scene to the GUI
    // @param name The name of the scene. Name has to be unique
    // @return The scene that was added. Nullptr if the scene already exists
    std::shared_ptr<threepp::Scene> addScene(const std::string &name)
    {
        if (GuiManager::s_scenes.find(name) != GuiManager::s_scenes.end())
            return nullptr;
        GuiManager::s_scenes[name] = threepp::Scene::create();
        return GuiManager::s_scenes[name];
    }

    // @brief Remove a scene from the GUI
    // @param name The name of the scene to remove
    // @return True if the scene was removed, false if the scene was not found
    void addCamera(
        const std::string &sceneName,
        const std::shared_ptr<threepp::Camera> &camera)
    {
        if (GuiManager::s_scenes.find(sceneName) == GuiManager::s_scenes.end())
            throw std::runtime_error("Scene not found");
        GuiManager::s_cameras[GuiManager::s_scenes[sceneName]] = camera;
    }

    // @brief Invoke a task on the GUI thread after a delay
    // @param task The task to invoke
    // @param delay The delay before the task is invoked
    void invokeLater(std::function<void()> task, float delay = 0.0f)
    {
        s_taskManager.invokeLater(task, delay);
    }

    // @brief Render all of the GUI elements
    // This fuction only needs to be called once. It will render all of the GUI elements
    void render();

    // @brief Destructor for the GUI manager
    ~GuiManager()
    {
        // Decrement the number of instances of the GUI manager
        s_instances--;
        // If there are no more instances of the GUI manager, shut down the GUI
        if (s_instances == 0)
        {
            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
            s_initialized = false;
        }
    }

private:
    // The canvas for the GUI
    inline static std::shared_ptr<threepp::Canvas> s_canvas;
    // The renderer for the GUI
    inline static std::shared_ptr<threepp::GLRenderer> s_renderer;
    // The scenes for the GUI. The key is the name of the scene
    inline static std::unordered_map<
        std::string,
        std::shared_ptr<threepp::Scene>>
        s_scenes;
    // The cameras for the GUI. The key is the scene that the camera is in
    inline static std::unordered_map<
        std::shared_ptr<threepp::Scene>,
        std::shared_ptr<threepp::Camera>>
        s_cameras;
    // The callbacks to be called on render. The key is the name of the callback
    inline static std::unordered_map<
        std::string,
        std::function<void()>>
        s_onRenderCallbacks = {};

    // The task manager for the GUI
    // Can be used to invoke changes to the GUI from other threads
    inline static threepp::TaskManager s_taskManager;

    inline static bool s_initialized = false; // Whether the GUI has been initialized
    inline static int s_instances = 0;        // The number of instances of the GUI manager

    // @brief Initialize the GUI for threepp and ImGui
    void initialize();
};

#endif // GUI_CONTEXT_HPP