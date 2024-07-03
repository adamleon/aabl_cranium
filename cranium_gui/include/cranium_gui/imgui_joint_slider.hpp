#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_internal.h>
#include <string>

namespace ImGui
{
    int JointSlider(const char *label, int *slider_angle, int *set_angle)
    {
        const ImGuiStyle &Style = GetStyle();

        ImDrawList *DrawList = GetWindowDrawList();
        ImGuiWindow *Window = GetCurrentWindow();
        if (Window->SkipItems)
            return false;

        int changed_angle = *slider_angle;

        BeginGroup();
        PushStyleVar(ImGuiStyleVar_GrabMinSize, 40);
        PushStyleVar(ImGuiStyleVar_FramePadding, 20);

        // header and spacing
        int changed = SliderInt("##slider", &changed_angle, -180, 180, "%d°");
        int hovered = IsItemActive() || IsItemHovered(); // IsItemDragged() ?

        if (changed)
        {
            *slider_angle = changed_angle;
        }

        if (BeginPopupContextItem())
        {
            if (Button("Reset"))
            {
                CloseCurrentPopup();
                *slider_angle = *set_angle;
            }
            EndPopup();
        }

        ImVec2 Canvas(GetItemRectSize().x, 20);

        ImRect bb(Window->DC.CursorPos, Window->DC.CursorPos + Canvas);
        ItemSize(bb);
        if (!ItemAdd(bb, NULL))
            return true;

        const ImGuiID id = Window->GetID(label);
        ItemHoverable(ImRect(bb.Min, bb.Min + Canvas), id, 0);

        RenderFrame(bb.Min, bb.Max, GetColorU32(ImGuiCol_WindowBg, 1), true, Style.FrameRounding);
        float position = ((((float)*set_angle + 180.0f) / 360.0f) * (Canvas.x - 40)) + bb.Min.x + 20;
        DrawList->AddTriangleFilled(ImVec2(position, bb.Min.y), ImVec2(position - 5, bb.Min.y + 5), ImVec2(position + 5, bb.Min.y + 5), GetColorU32(ImGuiCol_Text));
        DrawList->AddRectFilled(ImVec2(position - 20, bb.Min.y + 5), ImVec2(position + 20, bb.Max.y), GetColorU32(ImGuiCol_Text));

        std::string angleString = std::to_string(*set_angle) + "°";
        float stringWidth = CalcTextSize(angleString.c_str()).x;
        DrawList->AddText(ImVec2(position - stringWidth / 2, bb.Min.y + 6), GetColorU32(ImGuiCol_FrameBg), angleString.c_str());

        EndGroup();
        SameLine();
        BeginGroup();
        if (Button("-", ImVec2(20, 20)))
        {
            *slider_angle -= 10;
        }
        SameLine();
        if (Button("+", ImVec2(20, 20)))
        {
            *slider_angle += 10;
        }

        if (Button("Set", ImVec2(48, 20)))
        {
            *set_angle = *slider_angle;
        }
        EndGroup();

        return changed;
    }

    int FancySlider(const char *label, float *angle, float clamp_min, float clamp_max, ImGuiSliderFlags flags = 0)
    {
        enum
        {
            SLIDER_HEIGHT = 20
        };
        ImGuiContext &g = *GImGui;

        ImDrawList *DrawList = GetWindowDrawList();
        ImGuiWindow *Window = GetCurrentWindow();
        if (Window->SkipItems)
            return false;

        const ImGuiID id = Window->GetID(label);
        ImVec2 content_size(GetContentRegionAvail().x, SLIDER_HEIGHT);

        ImRect bounding_box(Window->DC.CursorPos, Window->DC.CursorPos + content_size);
        ImVec2 min_clamp_area = ImVec2(
            ((clamp_min + 180.0f) / 360.0f) * content_size.x,
            0);
        ImVec2 max_clamp_area = ImVec2(
            ((clamp_max + 180.0f) / 360.0f) * content_size.x,
            0);
        ImRect active_area = ImRect(
            bounding_box.Min + min_clamp_area, 
            bounding_box.Max - max_clamp_area);

        ItemSize(bounding_box);
        if (!ItemAdd(bounding_box, id, &bounding_box))
            return false;

        RenderNavHighlight(bounding_box, id);
        RenderFrame(bounding_box.Min, bounding_box.Max, GetColorU32(ImGuiCol_WindowBg, 1), false);
        float half_slider_height = (float)SLIDER_HEIGHT * 0.5f;
        DrawList->AddRectFilled(
            bounding_box.Min, bounding_box.Max,
            GetColorU32(ImGuiCol_Border), half_slider_height);
        DrawList->AddRectFilled(
            active_area.Min, active_area.Max,
            GetColorU32(ImGuiCol_FrameBg), half_slider_height);

        const bool hovered = ItemHoverable(active_area, id, g.LastItemData.InFlags);
        const bool clicked = hovered && IsMouseClicked(0, ImGuiInputFlags_None, id);

        const bool make_active = (clicked || g.NavActivateId == id);
        if (make_active && clicked)
            SetKeyOwner(ImGuiKey_MouseLeft, id);
        if (make_active)
        {
            SetActiveID(id, Window);
            SetFocusID(id, Window);
            FocusWindow(Window);
            g.ActiveIdUsingNavDirMask |= (1 << ImGuiDir_Left) | (1 << ImGuiDir_Right);
        }

        ImRect grab_bb;
        PushStyleVar(ImGuiStyleVar_GrabMinSize, SLIDER_HEIGHT);
        const bool value_changed = SliderBehavior(active_area, id, ImGuiDataType_Float, angle, &clamp_min, &clamp_max, "", flags, &grab_bb);
        if (value_changed)
        {
            MarkItemEdited(id);
        }
        PopStyleVar();

        // Render grab
        if (grab_bb.Max.x > grab_bb.Min.x)
            Window->DrawList->AddCircleFilled(grab_bb.GetCenter(), half_slider_height - 1.0f, GetColorU32(GetActiveID() == id ? ImGuiCol_SliderGrabActive : ImGuiCol_SliderGrab));

        return value_changed;
    }
}