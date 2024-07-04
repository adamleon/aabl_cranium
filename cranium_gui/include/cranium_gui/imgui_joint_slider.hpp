#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_internal.h>
#include <string>
#include <sstream>
#include <iomanip>

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

    int FancySlider(const char *label, float *angle, float *setpoint_angle, float clamp_min, float clamp_max, ImGuiSliderFlags flags = 0)
    {
        struct Config
        {
            const float slider_height = 30;
            const ImVec2 setpoint_box_size = CalcTextSize("-000.0°");
            const float info_height = setpoint_box_size.y * 1.2;
            const float gap_height = 10;
            const float half_slider_height = slider_height * 0.5f;
            const float content_width = GetContentRegionAvail().x;
            float anglePosition(float angle) const
            {
                return ((angle + 180.0f) / 360.0f) * content_width;
            }
        };
        enum
        {
            SLIDER_HEIGHT = 30
        };
        ImGuiContext &g = *GImGui;

        ImDrawList *DrawList = GetWindowDrawList();
        ImGuiWindow *Window = GetCurrentWindow();
        if (Window->SkipItems)
            return false;

        const ImGuiID id = Window->GetID(label);
        ImVec2 slider_content_size(GetContentRegionAvail().x, SLIDER_HEIGHT);
        ImRect bounding_box_slider(Window->DC.CursorPos, Window->DC.CursorPos + slider_content_size);
        ImRect bounding_box_info(bounding_box_slider.Min + ImVec2(0, SLIDER_HEIGHT), bounding_box_slider.Max + ImVec2(0, SLIDER_HEIGHT));
        ImRect bounding_box_total(bounding_box_slider.Min, bounding_box_info.Max);

        ImVec2 min_clamp_area = ImVec2(
            ((clamp_min + 180.0f) / 360.0f) * slider_content_size.x,
            0);
        ImVec2 max_clamp_area = ImVec2(
            ((180.0f - clamp_max) / 360.0f) * slider_content_size.x,
            0);
        ImRect active_area = ImRect(
            bounding_box_slider.Min + min_clamp_area, 
            bounding_box_slider.Max - max_clamp_area);

        ItemSize(bounding_box_total);
        if (!ItemAdd(bounding_box_total, id, &bounding_box_total))
            return false;

        RenderNavHighlight(bounding_box_total, id);
        RenderFrame(bounding_box_slider.Min, bounding_box_slider.Max, GetColorU32(ImGuiCol_WindowBg, 1), false);
        float half_slider_height = (float)SLIDER_HEIGHT * 0.5f;

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

        auto clamp_min_position = ImVec2(
            ((clamp_min + 180.0f) / 360.0f) * slider_content_size.x, 
            0);
        auto clamp_max_position = ImVec2(
            ((clamp_max + 180.0f) / 360.0f) * slider_content_size.x, 
            0);

        std::stringstream ss_clamp_min;
        ss_clamp_min << std::fixed << std::setprecision(1) << clamp_min << "°";
        ImVec2 clamp_min_size = CalcTextSize(ss_clamp_min.str().c_str());
        ImVec2 clamp_min_text_position = 
            bounding_box_info.Min + 
            clamp_min_position + 
            ImVec2(-clamp_min_size.x * 0.5f + half_slider_height, 0);

        std::stringstream ss_clamp_max;
        ss_clamp_max << std::fixed << std::setprecision(1) << clamp_max << "°";
        ImVec2 clamp_max_size = CalcTextSize(ss_clamp_max.str().c_str());
        ImVec2 clamp_max_text_position = 
            bounding_box_info.Min + 
            clamp_max_position + 
            ImVec2(-clamp_max_size.x * 0.5f - half_slider_height, 0);
        
        ImVec2 clamp_offset = ImVec2(0, 1.8*half_slider_height - clamp_min_size.y);

        auto setpoint_position = ImVec2(
            ((*setpoint_angle + 180.0f) / 360.0f) * slider_content_size.x, 
            0) + bounding_box_info.Min;
        std::stringstream ss_setpoint;
        ss_setpoint << std::fixed << std::setprecision(1) << *setpoint_angle << "°";
        ImVec2 setpoint_size = CalcTextSize(ss_setpoint.str().c_str());
        std::string tab_width = "-000.0°";
        float setpoint_tab_width = CalcTextSize(tab_width.c_str()).x;

        ImFont* font = GImGui->Font;
        font->Scale = half_slider_height / font->FontSize;
        PushFont(font);

        DrawList->AddLine(
            clamp_min_text_position + ImVec2(clamp_min_size.x*0.5f,0), 
            clamp_min_text_position + ImVec2(clamp_min_size.x*0.5f,0) + clamp_offset, 
            GetColorU32(ImGuiCol_Text),
            0.2f*half_slider_height);
        DrawList->AddText(clamp_min_text_position + clamp_offset,
             GetColorU32(ImGuiCol_Text), ss_clamp_min.str().c_str());
             
        DrawList->AddLine(
            clamp_max_text_position + ImVec2(clamp_max_size.x*0.5f,0), 
            clamp_max_text_position + ImVec2(clamp_max_size.x*0.5f,0) + clamp_offset, 
            GetColorU32(ImGuiCol_Text),
            0.2f*half_slider_height);
        DrawList->AddText(clamp_max_text_position + clamp_offset,
             GetColorU32(ImGuiCol_Text), ss_clamp_max.str().c_str());

        DrawList->AddRectFilled(
            setpoint_position - ImVec2(setpoint_tab_width*0.55f, setpoint_size.y - 1.8f*half_slider_height),
            setpoint_position + ImVec2(setpoint_tab_width*0.55f, SLIDER_HEIGHT),
            GetColorU32(ImGuiCol_Text), 0.25f * half_slider_height);
        DrawList->AddTriangleFilled(
            setpoint_position + ImVec2(0, 0.4f) * half_slider_height,
            setpoint_position + ImVec2(-0.66f, 1.0f) * half_slider_height,
            setpoint_position + ImVec2(0.66f, 1.0f) * half_slider_height,
            GetColorU32(ImGuiCol_Text));
        DrawList->AddText(
            setpoint_position - ImVec2(setpoint_size.x * 0.5f, 0) + clamp_offset,
            GetColorU32(ImGuiCol_FrameBg), ss_setpoint.str().c_str());

        // Render slider
        DrawList->AddRectFilled(
            bounding_box_slider.Min, bounding_box_slider.Max,
            GetColorU32(ImGuiCol_Button), half_slider_height);
        DrawList->AddRectFilled(
            active_area.Min, active_area.Max,
            GetColorU32(ImGuiCol_FrameBg), half_slider_height);

        // Render grab
        if (grab_bb.Max.x > grab_bb.Min.x)
            DrawList->AddCircleFilled(grab_bb.GetCenter(),0.9f*half_slider_height, GetColorU32(GetActiveID() == id ? ImGuiCol_SliderGrabActive : ImGuiCol_SliderGrab));
        PopFont();
        return value_changed;
    }
}