#pragma once
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgreOverlayContainer.h>
#include <Overlay/OgreTextAreaOverlayElement.h>
#include <Overlay/OgreBorderPanelOverlayElement.h>
#include <Overlay/OgreFontManager.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include "arms_rviz_control_plugin/teleop_hud_metrics.hpp"
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace arms_rviz_control_plugin {
class EventBusTeleopDashboard {
public:
    enum class Hit : uint8_t { NoHit, Teleop, Mirror, LeftArm, RightArm };

    EventBusTeleopDashboard() {
        static std::atomic<unsigned> next_id{0};
        name_ = "EventBusTeleopHUD/" + std::to_string(next_id++);
        font_ = Ogre::FontManager::getSingleton().create(
            name_ + "/font", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        font_->setType(Ogre::FT_TRUETYPE);
        font_->setSource("LiberationSans-Bold.ttf");
        font_->setTrueTypeSize(80);
        font_->setTrueTypeResolution(192);
        font_->load();
        createMaterial("border", Ogre::ColourValue(0.43, 0.49, 0.57, 1));
        auto& manager = Ogre::OverlayManager::getSingleton();
        overlay_ = manager.create(name_);
        overlay_->setZOrder(500);
        panel_ = static_cast<Ogre::OverlayContainer*>(
            manager.createOverlayElement("Panel", name_ + "/panel"));
        panel_->setMetricsMode(Ogre::GMM_PIXELS);
        panel_->setHorizontalAlignment(Ogre::GHA_LEFT);
        overlay_->add2D(panel_);

        card_ = static_cast<Ogre::BorderPanelOverlayElement*>(
            manager.createOverlayElement("BorderPanel", name_ + "/card"));
        card_->setMetricsMode(Ogre::GMM_PIXELS);
        card_->setBorderMaterialName(name_ + "/border");
        panel_->addChild(card_);
        mode_split_ = makeDivider("mode_split");
        arm_split_ = makeDivider("arm_split");
        text_parent_ = card_;
        teleop_label_ = addText("TELEOP", 12, 10, 18, label());
        teleop_value_ = addText("OFF", 80, 10, 18, muted());
        mirror_label_ = addText("MIRROR", 140, 10, 18, label());
        mirror_value_ = addText("OFF", 210, 10, 18, muted());
        left_arm_label_ = addText("L", 12, 38, 22, label());
        left_arm_value_ = addText("RUN", 30, 38, 22, muted());
        right_arm_label_ = addText("R", 140, 38, 22, label());
        right_arm_value_ = addText("RUN", 158, 38, 22, muted());
        left_stick_label_ = addText("STICK", 12, 64, 15, label());
        left_stick_value_ = addText("XY", 60, 64, 15, muted());
        right_stick_label_ = addText("STICK", 140, 64, 15, label());
        right_stick_value_ = addText("XY", 188, 64, 15, muted());
        left_grip_label_ = addText("GRIP", 12, 84, 15, label());
        left_grip_value_ = addText("%", 56, 84, 15, muted());
        right_grip_label_ = addText("GRIP", 140, 84, 15, label());
        right_grip_value_ = addText("%", 184, 84, 15, muted());
        applyLayout();
    }
    ~EventBusTeleopDashboard() {
        auto& manager = Ogre::OverlayManager::getSingleton();
        overlay_->hide();
        for (auto* text : texts_) {
            text->getParent()->removeChild(text->getName());
            manager.destroyOverlayElement(text);
        }
        for (auto* split : {mode_split_, arm_split_}) {
            card_->removeChild(split->getName());
            manager.destroyOverlayElement(split);
        }
        panel_->removeChild(card_->getName());
        manager.destroyOverlayElement(card_);
        overlay_->remove2D(panel_);
        manager.destroyOverlayElement(panel_);
        manager.destroy(overlay_);
        for (const auto& material : materials_) {
            Ogre::MaterialManager::getSingleton().remove(material->getName());
        }
        Ogre::FontManager::getSingleton().remove(font_->getName());
    }
    EventBusTeleopDashboard(const EventBusTeleopDashboard&) = delete;
    EventBusTeleopDashboard& operator=(const EventBusTeleopDashboard&) = delete;

    void setScale(float scale) {
        scale = std::clamp(scale, 0.5f, 2.0f);
        if (scale == scale_) return;
        scale_ = scale;
        applyLayout();
    }
    void show() { overlay_->show(); }
    void hide() { overlay_->hide(); }
    void setOrigin(float x, float y) {
        origin_x_ = x;
        origin_y_ = y;
        if (panel_) panel_->setPosition(origin_x_, origin_y_);
    }
    float height() const { return card_h_; }
    float scale() const { return scale_; }
    void refreshLayout() {
        if (!layout_pending_) return;
        for (auto* text : texts_) text->_positionsOutOfDate();
        layout_pending_ = false;
    }
    void setMode(bool update_mode, bool chassis_mode, bool body_tracking, bool mirror,
                 bool left_paused, bool right_paused,
                 bool left_grip_pct, bool right_grip_pct,
                 uint8_t left_stick_plane, uint8_t right_stick_plane,
                 uint8_t body_stick_plane, uint8_t body_right_stick_plane,
                 bool dex_hand_session) {
        const bool hide_stick = dex_hand_session && !chassis_mode;
        const bool hide_grip = dex_hand_session;
        if (hide_stick != stick_hidden_ || hide_grip != grip_hidden_) {
            stick_hidden_ = hide_stick;
            grip_hidden_ = hide_grip;
            applyLayout();
        }

        std::string left_stick_label = "STICK";
        std::string right_stick_label = "STICK";
        std::string left_stick_value = "XY";
        std::string right_stick_value = "XY";
        bool left_alt = false;
        bool right_alt = false;
        if (chassis_mode) {
            left_stick_label = "CHASSIS";
            right_stick_label = "CHASSIS";
            left_stick_value = "XY";
            right_stick_value = "YAW";
            right_alt = true;
        } else if (body_tracking) {
            left_stick_label = "WAIST";
            right_stick_label = "WAIST";
            left_alt = body_stick_plane != 0;
            right_alt = body_right_stick_plane != 0;
            left_stick_value = left_alt ? "ROLL" : "XY";
            right_stick_value = right_alt ? "PYAW" : "ZYAW";
        } else {
            left_alt = left_stick_plane != 0;
            right_alt = right_stick_plane != 0;
            left_stick_value = left_alt ? "ZYAW" : "XY";
            right_stick_value = right_alt ? "ZYAW" : "XY";
        }

        const auto on_color = Ogre::ColourValue(0.22, 0.72, 0.46);
        const auto off_color = Ogre::ColourValue(0.30, 0.55, 0.90);
        const auto pause_color = Ogre::ColourValue(0.90, 0.34, 0.40);
        const auto amber = Ogre::ColourValue(0.86, 0.63, 0.22);
        const auto label_color = label();
        const bool right_active_alt = right_alt || (body_tracking && !chassis_mode);
        const auto left_stick_color = chassis_mode
            ? amber
            : (!update_mode ? muted() : (left_alt ? amber : off_color));
        const auto right_stick_color = chassis_mode
            ? amber
            : (!update_mode ? muted() : (right_active_alt ? amber : off_color));

        setPair(teleop_label_, teleop_value_, "TELEOP", update_mode ? "ON" : "OFF",
                label_color, update_mode ? on_color : off_color);
        setPair(mirror_label_, mirror_value_, "MIRROR", mirror ? "ON" : "OFF",
                label_color, mirror ? amber : muted());
        setPair(left_arm_label_, left_arm_value_, "L", left_paused ? "PAUSE" : "RUN",
                label_color, left_paused ? pause_color : on_color);
        setPair(right_arm_label_, right_arm_value_, "R", right_paused ? "PAUSE" : "RUN",
                label_color, right_paused ? pause_color : on_color);
        setPair(left_stick_label_, left_stick_value_, left_stick_label, left_stick_value,
                label_color, left_stick_color);
        setPair(right_stick_label_, right_stick_value_, right_stick_label, right_stick_value,
                label_color, right_stick_color);
        setPair(left_grip_label_, left_grip_value_, "GRIP", left_grip_pct ? "%" : "SW",
                label_color, left_grip_pct ? off_color : amber);
        setPair(right_grip_label_, right_grip_value_, "GRIP", right_grip_pct ? "%" : "SW",
                label_color, right_grip_pct ? off_color : amber);
    }

    void cardRect(float& x, float& y, float& w, float& h) const {
        x = origin_x_;
        y = origin_y_;
        w = card_w_;
        h = card_h_;
    }
    Hit hitTest(float x, float y) const {
        return hitTestLocal(x - origin_x_, y - origin_y_);
    }
    Hit hitTestLocal(float lx, float ly) const {
        auto inside = [](float px, float py, float x0, float y0, float w, float h) {
            return px >= x0 && py >= y0 && px < x0 + w && py < y0 + h;
        };
        if (!inside(lx, ly, 0.f, 0.f, card_w_, card_h_)) return Hit::NoHit;
        if (inside(lx, ly, hit_teleop_x_, hit_mode_y_, hit_col_w_, hit_mode_h_)) return Hit::Teleop;
        if (inside(lx, ly, hit_mirror_x_, hit_mode_y_, hit_col_w_, hit_mode_h_)) return Hit::Mirror;
        if (inside(lx, ly, hit_teleop_x_, hit_arm_y_, hit_col_w_, hit_arm_h_)) return Hit::LeftArm;
        if (inside(lx, ly, hit_mirror_x_, hit_arm_y_, hit_col_w_, hit_arm_h_)) return Hit::RightArm;
        return Hit::NoHit;
    }

private:
    static void setPair(Ogre::TextAreaOverlayElement* label_el,
                        Ogre::TextAreaOverlayElement* value_el,
                        const std::string& label_text, const std::string& value_text,
                        const Ogre::ColourValue& label_color,
                        const Ogre::ColourValue& value_color) {
        if (label_el->getCaption() != label_text) label_el->setCaption(label_text);
        if (label_el->getColour() != label_color) label_el->setColour(label_color);
        if (value_el->getCaption() != value_text) value_el->setCaption(value_text);
        if (value_el->getColour() != value_color) value_el->setColour(value_color);
    }
    void createMaterial(const std::string& suffix, const Ogre::ColourValue& color) {
        auto material = Ogre::MaterialManager::getSingleton().create(
            name_ + "/" + suffix, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        auto* pass = material->getTechnique(0)->getPass(0);
        pass->setLightingEnabled(false);
        pass->setDepthCheckEnabled(false);
        pass->setDepthWriteEnabled(false);
        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        auto* unit = pass->createTextureUnitState();
        unit->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, color);
        unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, color.a);
        materials_.push_back(material);
    }
    static Ogre::ColourValue label() { return Ogre::ColourValue(0.f, 0.f, 0.f, 1); }
    static Ogre::ColourValue muted() { return Ogre::ColourValue(0.72f, 0.76f, 0.81f, 1); }
    Ogre::BorderPanelOverlayElement* makeDivider(const std::string& suffix) {
        auto* split = static_cast<Ogre::BorderPanelOverlayElement*>(
            Ogre::OverlayManager::getSingleton().createOverlayElement(
                "BorderPanel", name_ + "/" + suffix));
        split->setMetricsMode(Ogre::GMM_PIXELS);
        split->setMaterialName(name_ + "/border");
        split->setBorderSize(0);
        card_->addChild(split);
        return split;
    }
    Ogre::TextAreaOverlayElement* addText(const std::string& caption, float x, float y,
                                          float height, const Ogre::ColourValue& color) {
        auto* text = static_cast<Ogre::TextAreaOverlayElement*>(
            Ogre::OverlayManager::getSingleton().createOverlayElement(
                "TextArea", name_ + "/text" + std::to_string(texts_.size())));
        texts_.push_back(text);
        text->initialise();
        text->setMetricsMode(Ogre::GMM_PIXELS);
        text->setPosition(x, y);
        text->setDimensions(170, height + 2);
        text->setFontName(font_->getName());
        text->setCharHeight(height);
        text->setCaption(caption);
        text->setColour(color);
        text_parent_->addChild(text);
        return text;
    }
    void placePair(Ogre::TextAreaOverlayElement* label_el,
                   Ogre::TextAreaOverlayElement* value_el,
                   float x, float y, float col_w, float h,
                   float label_w, float value_w, bool value_right_align = false) {
        const float pair_gap = 2.f * scale_;
        label_el->setPosition(x, y);
        label_el->setCharHeight(h);
        label_el->setDimensions(label_w, h + 2);
        label_el->show();
        const float value_box_w = std::min(value_w, col_w - label_w - pair_gap);
        const float value_x = value_right_align
            ? (x + col_w - value_box_w)
            : (x + label_w + pair_gap);
        value_el->setPosition(value_x, y);
        value_el->setCharHeight(h);
        value_el->setDimensions(value_box_w, h + 2);
        value_el->show();
    }
    void applyLayout() {
        const float s = scale_;
        const float card_w = teleopHudCardWidth(s);
        const float pad = 10.f * s;
        const float bottom_pad = 8.f * s;
        const float gutter = pad;
        const float split_w = std::max(1.f, s);
        const float col_w = (card_w - 2.f * pad - split_w - 2.f * gutter) * 0.5f;
        const float mid_x = pad + col_w + gutter;
        const float right_x = mid_x + split_w + gutter;
        const float arm_y = 38.f * s;
        const float arm_h = 22.f * s;
        float content_bottom = arm_y + arm_h;
        if (!stick_hidden_) content_bottom = std::max(content_bottom, 79.f * s);
        if (!grip_hidden_) {
            const float grip_y = stick_hidden_ ? 64.f * s : 84.f * s;
            content_bottom = std::max(content_bottom, grip_y + 15.f * s);
        }
        const float card_h = content_bottom + bottom_pad;
        card_w_ = card_w;
        card_h_ = card_h;
        hit_col_w_ = col_w;
        hit_teleop_x_ = pad;
        hit_mirror_x_ = right_x;
        hit_mode_y_ = 8.f * s;
        hit_mode_h_ = 24.f * s;
        hit_arm_y_ = 36.f * s;
        hit_arm_h_ = arm_h + 4.f * s;

        card_->setDimensions(card_w, card_h);
        card_->setBorderSize(std::max(1.f, s));
        card_->setPosition(0, 0);
        card_->show();

        placePair(teleop_label_, teleop_value_, pad, 10.f * s, col_w, 18.f * s, 70.f * s, 36.f * s);
        placePair(mirror_label_, mirror_value_, right_x, 10.f * s, col_w, 18.f * s, 70.f * s, 36.f * s);
        mode_split_->setPosition(mid_x, 8.f * s);
        mode_split_->setDimensions(split_w, 24.f * s);
        mode_split_->show();

        placePair(left_arm_label_, left_arm_value_, pad, arm_y, col_w, arm_h, 18.f * s, 64.f * s,
                  true);
        placePair(right_arm_label_, right_arm_value_, right_x, arm_y, col_w, arm_h, 18.f * s, 64.f * s,
                  true);

        if (!stick_hidden_) {
            placePair(left_stick_label_, left_stick_value_, pad, 64.f * s, col_w, 15.f * s,
                      62.f * s, 48.f * s, true);
            placePair(right_stick_label_, right_stick_value_, right_x, 64.f * s, col_w, 15.f * s,
                      62.f * s, 48.f * s, true);
        } else {
            left_stick_label_->hide();
            left_stick_value_->hide();
            right_stick_label_->hide();
            right_stick_value_->hide();
        }
        if (!grip_hidden_) {
            const float grip_y = stick_hidden_ ? 64.f * s : 84.f * s;
            placePair(left_grip_label_, left_grip_value_, pad, grip_y, col_w, 15.f * s,
                      44.f * s, 32.f * s, true);
            placePair(right_grip_label_, right_grip_value_, right_x, grip_y, col_w, 15.f * s,
                      44.f * s, 32.f * s, true);
        } else {
            left_grip_label_->hide();
            left_grip_value_->hide();
            right_grip_label_->hide();
            right_grip_value_->hide();
        }
        const float arm_split_top = 36.f * s;
        const float arm_split_h = std::max(split_w, card_h - arm_split_top - bottom_pad);
        arm_split_->setPosition(mid_x, arm_split_top);
        arm_split_->setDimensions(split_w, arm_split_h);
        arm_split_->show();
        panel_->setPosition(origin_x_, origin_y_);
        panel_->setDimensions(card_w, card_h);
        layout_pending_ = true;
    }

    float scale_{1};
    bool layout_pending_{true};
    bool stick_hidden_{false};
    bool grip_hidden_{false};
    float origin_x_{16};
    float origin_y_{16};
    float card_w_{240};
    float card_h_{110};
    float hit_teleop_x_{12};
    float hit_mirror_x_{180};
    float hit_col_w_{150};
    float hit_mode_y_{8};
    float hit_mode_h_{24};
    float hit_arm_y_{36};
    float hit_arm_h_{26};
    std::string name_;
    Ogre::Overlay* overlay_{};
    Ogre::OverlayContainer* panel_{};
    Ogre::FontPtr font_;
    std::vector<Ogre::MaterialPtr> materials_;
    Ogre::BorderPanelOverlayElement* card_{};
    Ogre::BorderPanelOverlayElement* mode_split_{};
    Ogre::BorderPanelOverlayElement* arm_split_{};
    Ogre::OverlayContainer* text_parent_{};
    std::vector<Ogre::TextAreaOverlayElement*> texts_;
    Ogre::TextAreaOverlayElement* teleop_label_{};
    Ogre::TextAreaOverlayElement* teleop_value_{};
    Ogre::TextAreaOverlayElement* mirror_label_{};
    Ogre::TextAreaOverlayElement* mirror_value_{};
    Ogre::TextAreaOverlayElement* left_arm_label_{};
    Ogre::TextAreaOverlayElement* left_arm_value_{};
    Ogre::TextAreaOverlayElement* right_arm_label_{};
    Ogre::TextAreaOverlayElement* right_arm_value_{};
    Ogre::TextAreaOverlayElement* left_stick_label_{};
    Ogre::TextAreaOverlayElement* left_stick_value_{};
    Ogre::TextAreaOverlayElement* right_stick_label_{};
    Ogre::TextAreaOverlayElement* right_stick_value_{};
    Ogre::TextAreaOverlayElement* left_grip_label_{};
    Ogre::TextAreaOverlayElement* left_grip_value_{};
    Ogre::TextAreaOverlayElement* right_grip_label_{};
    Ogre::TextAreaOverlayElement* right_grip_value_{};
};
}
