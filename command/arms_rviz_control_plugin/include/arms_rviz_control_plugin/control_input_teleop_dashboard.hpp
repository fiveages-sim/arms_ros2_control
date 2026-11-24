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
#include <arms_ros2_control_msgs/msg/teleop_mode.hpp>
#include "arms_rviz_control_plugin/teleop_hud_metrics.hpp"
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace arms_rviz_control_plugin {
class ControlInputTeleopDashboard {
public:
    ControlInputTeleopDashboard() {
        static std::atomic<unsigned> next_id{0};
        name_ = "ControlInputTeleopHUD/" + std::to_string(next_id++);
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
        footer_split_ = makeDivider("footer_split");
        text_parent_ = card_;
        target_label_ = addText("ARM", 12, 10, 18, label());
        target_value_ = addText("L", 60, 10, 18, muted());
        mirror_label_ = addText("MIRROR", 12, 36, 15, label());
        mirror_value_ = addText("OFF", 90, 36, 15, muted());
        speed_label_ = addText("SPD", 150, 36, 15, label());
        speed_value_ = addText("LOW", 190, 36, 15, muted());
        applyLayout();
    }
    ~ControlInputTeleopDashboard() {
        auto& manager = Ogre::OverlayManager::getSingleton();
        overlay_->hide();
        for (auto* text : texts_) {
            text->getParent()->removeChild(text->getName());
            manager.destroyOverlayElement(text);
        }
        card_->removeChild(footer_split_->getName());
        manager.destroyOverlayElement(footer_split_);
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
    ControlInputTeleopDashboard(const ControlInputTeleopDashboard&) = delete;
    ControlInputTeleopDashboard& operator=(const ControlInputTeleopDashboard&) = delete;

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
    void setMode(uint8_t control_mode, bool mirror, int32_t arm_target, int32_t speed_level) {
        using JoyMode = arms_ros2_control_msgs::msg::TeleopMode;
        const bool chassis = control_mode == JoyMode::CONTROL_CHASSIS;
        const auto off_color = Ogre::ColourValue(0.30, 0.55, 0.90);
        const auto amber = Ogre::ColourValue(0.86, 0.63, 0.22);
        const auto label_color = label();

        const std::string target_label = chassis ? "" : "ARM";
        const std::string target_value = chassis ? "CHASSIS" : (arm_target == 2 ? "R" : "L");
        const auto target_color = chassis ? amber : (arm_target == 2 ? amber : off_color);

        const std::string mirror_value = mirror ? "ON" : "OFF";
        const auto mirror_color = mirror ? amber : muted();

        const std::string speed_value = speed_level != 0 ? "HIGH" : "LOW";
        const auto speed_color = speed_level != 0 ? amber : off_color;

        if (target_label_->getCaption() != target_label) target_label_->setCaption(target_label);
        if (target_label_->getColour() != label_color) target_label_->setColour(label_color);
        if (target_value_->getCaption() != target_value) target_value_->setCaption(target_value);
        if (target_value_->getColour() != target_color) target_value_->setColour(target_color);
        if (chassis) target_label_->hide();
        else target_label_->show();

        if (mirror_label_->getCaption() != "MIRROR") mirror_label_->setCaption("MIRROR");
        if (mirror_label_->getColour() != label_color) mirror_label_->setColour(label_color);
        if (mirror_value_->getCaption() != mirror_value) mirror_value_->setCaption(mirror_value);
        if (mirror_value_->getColour() != mirror_color) mirror_value_->setColour(mirror_color);

        if (speed_label_->getCaption() != "SPD") speed_label_->setCaption("SPD");
        if (speed_label_->getColour() != label_color) speed_label_->setColour(label_color);
        if (speed_value_->getCaption() != speed_value) speed_value_->setCaption(speed_value);
        if (speed_value_->getColour() != speed_color) speed_value_->setColour(speed_color);

        if (chassis != chassis_layout_) {
            chassis_layout_ = chassis;
            applyLayout();
        }
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
    void applyLayout() {
        const float s = scale_;
        const float pad = 12.f * s;
        const float bottom_pad = 8.f * s;
        const float split_w = std::max(1.f, s);
        const float pair_gap = 4.f * s;

        const float row1_y = 10.f * s;
        const float row1_h = 18.f * s;
        const float row2_y = 36.f * s;
        const float row2_h = 15.f * s;

        const float target_label_w = chassis_layout_ ? 0.f : 44.f * s;
        const float target_value_w = chassis_layout_ ? 84.f * s : 24.f * s;
        const float mirror_label_w = 72.f * s;
        const float mirror_value_w = 36.f * s;
        const float speed_label_w = 40.f * s;
        const float speed_value_w = 48.f * s;

        const float card_w = teleopHudCardWidth(s);
        const float card_h = row2_y + row2_h + bottom_pad;
        card_h_ = card_h;

        card_->setDimensions(card_w, card_h);
        card_->setBorderSize(std::max(1.f, s));
        card_->setPosition(0, 0);
        card_->show();

        if (!chassis_layout_) {
            target_label_->setPosition(pad, row1_y);
            target_label_->setCharHeight(row1_h);
            target_label_->setDimensions(target_label_w, row1_h + 2);
            target_label_->show();
            target_value_->setPosition(pad + target_label_w + pair_gap, row1_y);
        } else {
            target_label_->hide();
            target_value_->setPosition(pad, row1_y);
        }
        target_value_->setCharHeight(row1_h);
        target_value_->setDimensions(target_value_w, row1_h + 2);
        target_value_->show();

        const float mid_x = 0.5f * (card_w - split_w);
        const float left_col_x = pad;
        const float right_col_x = mid_x + split_w + pad;

        mirror_label_->setPosition(left_col_x, row2_y);
        mirror_label_->setCharHeight(row2_h);
        mirror_label_->setDimensions(mirror_label_w, row2_h + 2);
        mirror_label_->show();
        mirror_value_->setPosition(mid_x - mirror_value_w, row2_y);
        mirror_value_->setCharHeight(row2_h);
        mirror_value_->setDimensions(mirror_value_w, row2_h + 2);
        mirror_value_->show();

        footer_split_->setPosition(mid_x, row2_y);
        footer_split_->setDimensions(split_w, row2_h);
        footer_split_->show();

        speed_label_->setPosition(right_col_x, row2_y);
        speed_label_->setCharHeight(row2_h);
        speed_label_->setDimensions(speed_label_w, row2_h + 2);
        speed_label_->show();
        speed_value_->setPosition(card_w - pad - speed_value_w, row2_y);
        speed_value_->setCharHeight(row2_h);
        speed_value_->setDimensions(speed_value_w, row2_h + 2);
        speed_value_->show();

        panel_->setPosition(origin_x_, origin_y_);
        panel_->setDimensions(card_w, card_h);
        layout_pending_ = true;
    }

    float scale_{1};
    bool layout_pending_{true};
    bool chassis_layout_{false};
    float origin_x_{16};
    float origin_y_{16};
    float card_h_{70};
    std::string name_;
    Ogre::Overlay* overlay_{};
    Ogre::OverlayContainer* panel_{};
    Ogre::FontPtr font_;
    std::vector<Ogre::MaterialPtr> materials_;
    Ogre::BorderPanelOverlayElement* card_{};
    Ogre::BorderPanelOverlayElement* footer_split_{};
    Ogre::OverlayContainer* text_parent_{};
    std::vector<Ogre::TextAreaOverlayElement*> texts_;
    Ogre::TextAreaOverlayElement* target_label_{};
    Ogre::TextAreaOverlayElement* target_value_{};
    Ogre::TextAreaOverlayElement* mirror_label_{};
    Ogre::TextAreaOverlayElement* mirror_value_{};
    Ogre::TextAreaOverlayElement* speed_label_{};
    Ogre::TextAreaOverlayElement* speed_value_{};
};
}
