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
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace arms_rviz_control_plugin {
class TeleopModeDashboard {
public:
    enum class Hit : uint8_t { NoHit, Teleop, Mirror, LeftArm, RightArm };

    TeleopModeDashboard() {
        static std::atomic<unsigned> next_id{0};
        name_ = "TeleopModeHUD/" + std::to_string(next_id++);
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
        teleop_ = addText("TELEOP OFF", 12, 10, 18, muted());
        mirror_ = addText("MIRROR OFF", 180, 10, 18, muted());
        left_arm_ = addText("L RUN", 12, 38, 22, muted());
        right_arm_ = addText("R RUN", 180, 38, 22, muted());
        left_stick_ = addText("STICK XY", 12, 64, 15, muted());
        right_stick_ = addText("STICK XY", 180, 64, 15, muted());
        left_grip_ = addText("GRIP %", 12, 84, 15, muted());
        right_grip_ = addText("GRIP %", 180, 84, 15, muted());
        applyLayout();
    }
    ~TeleopModeDashboard() {
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
    TeleopModeDashboard(const TeleopModeDashboard&) = delete;
    TeleopModeDashboard& operator=(const TeleopModeDashboard&) = delete;

    void setScale(float scale) {
        scale = std::clamp(scale, 0.5f, 2.0f);
        if (scale == scale_) return;
        scale_ = scale;
        applyLayout();
    }
    void show() { overlay_->show(); }
    void hide() { overlay_->hide(); }
    void refreshLayout() {
        if (!layout_pending_) return;
        for (auto* text : texts_) text->_positionsOutOfDate();
        layout_pending_ = false;
    }
    // stick_plane: arm PLANE_XY / PLANE_Z_YAW.
    // body_*_plane: left 0=XY / 1=ROLL；right 0=ZYAW / 1=PYAW。CHASSIS 优先于腰部。
    // dex_hand_session: optical_r0_caps≠0（vr-xrt wrist）；隐藏 STICK/GRIP，CHASSIS 行保留。
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

        const std::string teleop = update_mode ? "TELEOP ON" : "TELEOP OFF";
        const std::string mirror_caption = mirror ? "MIRROR ON" : "MIRROR OFF";
        const std::string left = left_paused ? "L PAUSE" : "L RUN";
        const std::string right = right_paused ? "R PAUSE" : "R RUN";
        std::string left_stick;
        std::string right_stick;
        bool left_alt = false;
        bool right_alt = false;
        if (chassis_mode) {
            left_stick = "CHASSIS XY";
            right_stick = "CHASSIS YAW";
            right_alt = true;
        } else if (body_tracking) {
            left_alt = body_stick_plane != 0;
            right_alt = body_right_stick_plane != 0;
            left_stick = left_alt ? "WAIST ROLL" : "WAIST XY";
            right_stick = right_alt ? "WAIST PYAW" : "WAIST ZYAW";
        } else {
            left_alt = left_stick_plane != 0;
            right_alt = right_stick_plane != 0;
            left_stick = left_alt ? "STICK ZYAW" : "STICK XY";
            right_stick = right_alt ? "STICK ZYAW" : "STICK XY";
        }
        const std::string left_grip = left_grip_pct ? "GRIP %" : "GRIP SW";
        const std::string right_grip = right_grip_pct ? "GRIP %" : "GRIP SW";
        const auto on_color = Ogre::ColourValue(0.22, 0.72, 0.46);
        const auto off_color = Ogre::ColourValue(0.30, 0.55, 0.90);
        const auto pause_color = Ogre::ColourValue(0.90, 0.34, 0.40);
        const auto amber = Ogre::ColourValue(0.86, 0.63, 0.22);
        const auto teleop_color = update_mode ? on_color : off_color;
        const auto mirror_color = mirror ? amber : muted();
        const auto left_color = left_paused ? pause_color : on_color;
        const auto right_color = right_paused ? pause_color : on_color;
        const bool right_active_alt = right_alt || (body_tracking && !chassis_mode);
        const auto left_stick_color = chassis_mode
            ? amber
            : (!update_mode ? muted() : (left_alt ? amber : off_color));
        const auto right_stick_color = chassis_mode
            ? amber
            : (!update_mode ? muted() : (right_active_alt ? amber : off_color));
        const auto left_grip_color = left_grip_pct ? off_color : amber;
        const auto right_grip_color = right_grip_pct ? off_color : amber;
        if (teleop_->getCaption() != teleop) teleop_->setCaption(teleop);
        if (mirror_->getCaption() != mirror_caption) mirror_->setCaption(mirror_caption);
        if (left_arm_->getCaption() != left) left_arm_->setCaption(left);
        if (right_arm_->getCaption() != right) right_arm_->setCaption(right);
        if (left_stick_->getCaption() != left_stick) left_stick_->setCaption(left_stick);
        if (right_stick_->getCaption() != right_stick) right_stick_->setCaption(right_stick);
        if (left_grip_->getCaption() != left_grip) left_grip_->setCaption(left_grip);
        if (right_grip_->getCaption() != right_grip) right_grip_->setCaption(right_grip);
        if (teleop_->getColour() != teleop_color) teleop_->setColour(teleop_color);
        if (mirror_->getColour() != mirror_color) mirror_->setColour(mirror_color);
        if (left_arm_->getColour() != left_color) left_arm_->setColour(left_color);
        if (right_arm_->getColour() != right_color) right_arm_->setColour(right_color);
        if (left_stick_->getColour() != left_stick_color) left_stick_->setColour(left_stick_color);
        if (right_stick_->getColour() != right_stick_color) right_stick_->setColour(right_stick_color);
        if (left_grip_->getColour() != left_grip_color) left_grip_->setColour(left_grip_color);
        if (right_grip_->getColour() != right_grip_color) right_grip_->setColour(right_grip_color);
    }

    void cardRect(float& x, float& y, float& w, float& h) const {
        x = origin_x_;
        y = origin_y_;
        w = card_w_;
        h = card_h_;
    }
    // Viewport absolute coords (panel origin included).
    Hit hitTest(float x, float y) const {
        return hitTestLocal(x - origin_x_, y - origin_y_);
    }
    // Card-local coords (for the transparent click layer).
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
    void applyLayout() {
        const float s = scale_;
        const float card_w = 360.f * s;
        const float pad = 12.f * s;
        const float bottom_pad = 8.f * s;
        const float split_w = std::max(1.f, s);
        const float col_w = (card_w - 2.f * pad - split_w - 16.f * s) * 0.5f;
        const float mid_x = pad + col_w + 8.f * s;
        const float right_x = mid_x + split_w + 8.f * s;
        const float origin = 16.f * s;
        const float arm_y = 38.f * s;
        const float arm_h = 22.f * s;
        // Content bottom drives card height so dividers never poke past the border.
        float content_bottom = arm_y + arm_h;
        if (!stick_hidden_) content_bottom = std::max(content_bottom, 79.f * s);
        if (!grip_hidden_) {
            const float grip_y = stick_hidden_ ? 64.f * s : 84.f * s;
            content_bottom = std::max(content_bottom, grip_y + 15.f * s);
        }
        const float card_h = content_bottom + bottom_pad;
        card_w_ = card_w;
        card_h_ = card_h;
        origin_x_ = origin;
        origin_y_ = origin;
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
        teleop_->setPosition(pad, 10.f * s);
        teleop_->setCharHeight(18.f * s);
        teleop_->setDimensions(col_w, 18.f * s + 2);
        teleop_->show();
        mirror_->setPosition(right_x, 10.f * s);
        mirror_->setCharHeight(18.f * s);
        mirror_->setDimensions(col_w, 18.f * s + 2);
        mirror_->show();
        mode_split_->setPosition(mid_x, 8.f * s);
        mode_split_->setDimensions(split_w, 24.f * s);
        mode_split_->show();
        left_arm_->setPosition(pad, arm_y);
        left_arm_->setCharHeight(arm_h);
        left_arm_->setDimensions(col_w, arm_h + 2);
        left_arm_->show();
        right_arm_->setPosition(right_x, arm_y);
        right_arm_->setCharHeight(arm_h);
        right_arm_->setDimensions(col_w, arm_h + 2);
        right_arm_->show();

        if (!stick_hidden_) {
            left_stick_->setPosition(pad, 64.f * s);
            left_stick_->setCharHeight(15.f * s);
            left_stick_->setDimensions(col_w, 15.f * s + 2);
            left_stick_->show();
            right_stick_->setPosition(right_x, 64.f * s);
            right_stick_->setCharHeight(15.f * s);
            right_stick_->setDimensions(col_w, 15.f * s + 2);
            right_stick_->show();
        } else {
            left_stick_->hide();
            right_stick_->hide();
        }
        if (!grip_hidden_) {
            const float grip_y = stick_hidden_ ? 64.f * s : 84.f * s;
            left_grip_->setPosition(pad, grip_y);
            left_grip_->setCharHeight(15.f * s);
            left_grip_->setDimensions(col_w, 15.f * s + 2);
            left_grip_->show();
            right_grip_->setPosition(right_x, grip_y);
            right_grip_->setCharHeight(15.f * s);
            right_grip_->setDimensions(col_w, 15.f * s + 2);
            right_grip_->show();
        } else {
            left_grip_->hide();
            right_grip_->hide();
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
    float card_w_{360};
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
    Ogre::TextAreaOverlayElement* teleop_{};
    Ogre::TextAreaOverlayElement* mirror_{};
    Ogre::TextAreaOverlayElement* left_arm_{};
    Ogre::TextAreaOverlayElement* right_arm_{};
    Ogre::TextAreaOverlayElement* left_stick_{};
    Ogre::TextAreaOverlayElement* right_stick_{};
    Ogre::TextAreaOverlayElement* left_grip_{};
    Ogre::TextAreaOverlayElement* right_grip_{};
};
}
