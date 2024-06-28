#include "cranium_gui/gui_manager.hpp"

class Menu  {
public:
    Menu(const std::shared_ptr<GuiManager> manager) {
        if(manager->addOnRenderCallback("menu", std::bind(&Menu::onRender, this)))
            std::cout << "Callback added" << std::endl;
        else
            std::cout << "Callback not added" << std::endl;
    }

private:
    void onRender() {
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
    }

    float f1 = 2.0f;
    float f2 = 2.0f;
};