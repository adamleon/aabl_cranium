#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_internal.h>
#include "imgui_theme.hpp"
#include <string>
#include <sstream>
#include <iomanip>

namespace ImGui
{
    int JointSlider(const char *label, float *angle, float *current_angle, float *commanded_angle, float clamp_min, float clamp_max, ImGuiSliderFlags flags = 0)
    {
        struct Config
        {
            const float info_height = 20;
            const float gap_height = 5;
            const float slider_height = 30;
            const float half_slider_height = slider_height * 0.5f;
            const float line_thickness = gap_height * 0.4f;
            const std::string current_angle_max_text = "-000.0°";
            const float content_width = GetContentRegionAvail().x;
            float anglePosition(float angle) const
            {
                return ((angle + 180.0f) / 360.0f) * (content_width-slider_height) + half_slider_height;
            }
            std::string angleText(float angle) const
            {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(1) << angle << "°";
                return ss.str();
            }
            ImVec2 angleTextSize(float angle) {
                return angleTextSize(angleText(angle));
            }
            ImVec2 angleTextSize(const std::string &angle) {
                return CalcTextSize(angle.c_str());
            }
        };
        enum
        {
            SLIDER_HEIGHT = 30
        };

        // Configure the slider
        Config config;
        ImGuiContext &g = *GImGui;

        ImDrawList *DrawList = GetWindowDrawList();
        ImGuiWindow *Window = GetCurrentWindow();
        if (Window->SkipItems)
            return false;

        const ImGuiID id = Window->GetID(label);

        // Calculate the bounding boxes
        // The slider is the main part of the widget
        ImRect bounding_box_slider(
            Window->DC.CursorPos, 
            Window->DC.CursorPos + ImVec2(config.content_width, config.slider_height));
        // The info is the text that shows the current value
        // It is placed below the slider, with a gap in between
        ImRect bounding_box_gap(
            bounding_box_slider.GetBL(), bounding_box_slider.GetBR() + ImVec2(0, config.gap_height));
        ImRect bounding_box_info(
            bounding_box_gap.GetBL(), bounding_box_gap.GetBR() + ImVec2(0, config.info_height));
        // The total bounding box is the union of the slider and the info
        ImRect bounding_box_total(bounding_box_slider.Min, bounding_box_info.Max);

        // Set the size of the widget
        ItemSize(bounding_box_total);
        if (!ItemAdd(bounding_box_total, id, &bounding_box_total))
            return false;
        RenderNavHighlight(bounding_box_total, id);

        // Start slider logic
        // The slider is the active area
        ImVec2 min_clamp_position = ImVec2(config.anglePosition(clamp_min), 0);
        ImVec2 max_clamp_position = ImVec2(config.anglePosition(clamp_max), 0);
        ImRect active_slider_area = ImRect(
            bounding_box_slider.Min + min_clamp_position - ImVec2(config.half_slider_height, 0),
            bounding_box_slider.Min + max_clamp_position + ImVec2(config.half_slider_height, config.slider_height));

        // Update the slider
        const bool hovered = ItemHoverable(active_slider_area, id, g.LastItemData.InFlags);
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

        // Perform the slider behavior
        ImRect grab_bb;
        PushStyleVar(ImGuiStyleVar_GrabMinSize, config.slider_height);
        auto slider_behavior_area = ImRect(
            active_slider_area.Min - ImVec2(2, 0),
            active_slider_area.Max + ImVec2(2, 0));
        const bool value_changed = SliderBehavior(slider_behavior_area, id, ImGuiDataType_Float, angle, &clamp_min, &clamp_max, "", flags, &grab_bb);
        if (value_changed)
        {
            MarkItemEdited(id);
        }
        PopStyleVar();


        // Render the content
        RenderFrame(bounding_box_slider.Min, bounding_box_slider.Max, GetColorU32(ImGuiCol_WindowBg, 1), false);
        
        ImFont* font = GImGui->Font;
        font->Scale = 0.8f * config.info_height / font->FontSize;
        PushFont(font);

        ImVec2 min_clamp_size = config.angleTextSize(clamp_min);
        ImVec2 min_clamp_text_position = 
            bounding_box_info.Min + 
            min_clamp_position + 
            ImVec2(-min_clamp_size.x * 0.5f, 0);

        ImVec2 max_clamp_size = config.angleTextSize(clamp_max);
        ImVec2 max_clamp_text_position = 
            bounding_box_info.Min + 
            max_clamp_position + 
            ImVec2(-max_clamp_size.x * 0.5f, 0);
        
        auto current_angle_position = bounding_box_info.Min + ImVec2(config.anglePosition(*current_angle), 0);
        ImVec2 current_angle_size = config.angleTextSize(*current_angle);
        float current_angle_tab_width = config.angleTextSize(config.current_angle_max_text).x;
        ImVec2 gap_offset = ImVec2(0, config.gap_height);

        auto command_angle_position = bounding_box_slider.Min + ImVec2(config.anglePosition(*commanded_angle), config.half_slider_height);
        ImVec2 command_angle_size = config.angleTextSize(*commanded_angle);

        // Render Min and Max Clamps
        DrawList->AddLine(
            bounding_box_info.Min + min_clamp_position - gap_offset - ImVec2(0.5f, 0),
            bounding_box_info.Min + min_clamp_position - ImVec2(0.5f, 0), 
            colorText,
            config.line_thickness);
        DrawList->AddText(min_clamp_text_position,
            colorText, config.angleText(clamp_min).c_str());
             
        DrawList->AddLine(
            bounding_box_info.Min + max_clamp_position - gap_offset - ImVec2(0.5f, 0),
            bounding_box_info.Min + max_clamp_position - ImVec2(0.5f, 0),
            colorText,
            config.line_thickness);
        DrawList->AddText(max_clamp_text_position,
            colorText, config.angleText(clamp_max).c_str());

        // Render Current Angle Tab
        DrawList->AddRectFilled(
            current_angle_position - ImVec2(current_angle_tab_width*0.55f, 0),
            current_angle_position + ImVec2(current_angle_tab_width*0.55f, config.info_height),
            colorElementA, 0.25f * config.info_height);
        DrawList->AddTriangleFilled(
            current_angle_position - ImVec2(0, 0.8f) * config.gap_height,
            current_angle_position + ImVec2(-0.66f, 0.0f) * config.gap_height,
            current_angle_position + ImVec2(0.66f, 0.0f) * config.gap_height,
            colorElementA);
        DrawList->AddText(
            current_angle_position - ImVec2(current_angle_size.x * 0.5f, 0),
            colorTextAlt, config.angleText(*current_angle).c_str());

        // Render slider
        DrawList->AddRectFilled(
            bounding_box_slider.Min + ImVec2(config.half_slider_height, 0),
            bounding_box_slider.Max - ImVec2(config.half_slider_height, 0),
            colorDisabledFrame);
        DrawList->AddCircleFilled(
            bounding_box_slider.GetTL() + ImVec2(config.half_slider_height, config.half_slider_height),
            config.half_slider_height,
            colorDisabledFrame);
        DrawList->AddCircleFilled(
            bounding_box_slider.GetTR() + ImVec2(-config.half_slider_height, config.half_slider_height),
            config.half_slider_height,
            colorDisabledFrame);
        DrawList->AddRectFilled(
            active_slider_area.Min + ImVec2(config.half_slider_height, 0),
            active_slider_area.Max - ImVec2(config.half_slider_height, 0),
           colorActiveFrame);
                DrawList->AddCircleFilled(
            active_slider_area.GetTL() + ImVec2(config.half_slider_height, config.half_slider_height),
            config.half_slider_height,
            colorActiveFrame);
        DrawList->AddCircleFilled(
            active_slider_area.GetTR() + ImVec2(-config.half_slider_height, config.half_slider_height),
            config.half_slider_height,
            colorActiveFrame);

        PopFont();
        font = GImGui->Font;
        font->Scale = 0.5f * (current_angle_tab_width / config.slider_height);
        PushFont(font);
        // Render Commanded Angle Tab
        DrawList->AddCircleFilled(
            command_angle_position, config.half_slider_height,
            colorElementALight);
        DrawList->AddText(
            command_angle_position - ImVec2(command_angle_size.x * 0.5f, command_angle_size.y * 0.5f),
            GetColorU32(ImGuiCol_Text), config.angleText(*commanded_angle).c_str());

        // Render grab
        if (grab_bb.Max.x > grab_bb.Min.x)
            DrawList->AddCircleFilled(grab_bb.GetCenter(),0.8f*config.half_slider_height, GetColorU32(GetActiveID() == id ? colorButtonActive : colorButtonHovered));
        PopFont();
        return value_changed;
    }
}