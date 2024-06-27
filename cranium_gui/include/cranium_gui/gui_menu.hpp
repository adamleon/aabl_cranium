#include "threepp/extras/imgui/ImguiContext.hpp"
#include "threepp/threepp.hpp"

class Menu : public ImguiContext {
public:
    explicit Menu(const threepp::Canvas& canvas): ImguiContext(canvas.windowPtr()) {
    }

    void onRender() override {
        ImGui::SetNextWindowPos({0, 0}, 0, {0, 0});
        ImGui::SetNextWindowSize({230, 0}, 0);

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

        ImGui::Begin("Test Window");

        ImGui::SliderFloat("float", &f, 1.0f, 5.9f);

        ImGui::End();

        
        ImGui::Begin("Test Window 2");

        ImGui::SliderFloat("float", &f, 1.0f, 5.9f);

        ImGui::End();
    }

private:
    float f = 2.0f;
};