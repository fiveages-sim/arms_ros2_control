#pragma once
#include "arms_rviz_control_plugin/error_sparkline.hpp"
#include <memory>
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
#include <QString>
#include <array>
#include <atomic>
#include <cmath>
#include <vector>

namespace arms_rviz_control_plugin {
// Screen-space text rendered by RViz's Ogre overlay system; no QWidget; only the small trend textures update at 5 Hz.
class ErrorDashboard {
public:
    ErrorDashboard() {
        static std::atomic<unsigned> next_id{0};
        name_ = "TrackingErrorHUD/" + std::to_string(next_id++);
        font_ = Ogre::FontManager::getSingleton().create(name_+"/font", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        font_->setType(Ogre::FT_TRUETYPE);
        font_->setSource("LiberationSans-Bold.ttf");
        font_->setTrueTypeSize(40);
        font_->setTrueTypeResolution(192);
        font_->load();
        createMaterial("border", Ogre::ColourValue(0.43, 0.49, 0.57, 1));
        auto& manager = Ogre::OverlayManager::getSingleton();
        overlay_ = manager.create(name_);
        overlay_->setZOrder(500);
        panel_ = static_cast<Ogre::OverlayContainer*>(manager.createOverlayElement("Panel", name_+"/panel"));
        panel_->setMetricsMode(Ogre::GMM_PIXELS);
        panel_->setHorizontalAlignment(Ogre::GHA_RIGHT);
        panel_->setPosition(-366, 16);
        panel_->setDimensions(350, 574);
        overlay_->add2D(panel_);
        const char* names[] = {"LEFT ARM", "RIGHT ARM", "BODY", "HEAD"};
        for (size_t row = 0; row < 4; ++row) {
            const float top = row * 146;
            auto* card = static_cast<Ogre::BorderPanelOverlayElement*>(manager.createOverlayElement(
                "BorderPanel", name_+"/card"+std::to_string(row)));
            card->setMetricsMode(Ogre::GMM_PIXELS);
            card->setPosition(0, top);
            card->setDimensions(350, 136);
            card->setBorderSize(1);
            card->setBorderMaterialName(name_+"/border");
            panel_->addChild(card);
            cards_[row] = card;
            card->hide();
            text_parent_ = card;
            addText(names[row], 12, 8, 24, Ogre::ColourValue::Black);
            for (size_t col = 0; col < 2; ++col) {
                charts_[row][col] = std::make_unique<ErrorSparkline>(name_+"/chart"+std::to_string(row*2+col), card, 12+col*174);
                values_[row][col] = addText("--", 12+col*174, 38, 46, muted());
                addText(col == 0 ? "POS (mm)" : "ANG (deg)", 12+col*174, 86, 20, Ogre::ColourValue::Black);
            }
            status_[row] = addText("Waiting for poses", 12, 112, 16, Ogre::ColourValue::Black);
        }
    }
    ~ErrorDashboard() {
        auto& manager = Ogre::OverlayManager::getSingleton();
        overlay_->hide();
        for (auto& row : charts_) for (auto& chart : row) chart.reset();
        for (auto* text : texts_) {
            text->getParent()->removeChild(text->getName());
            manager.destroyOverlayElement(text);
        }
        for (auto* card : cards_) {
            panel_->removeChild(card->getName());
            manager.destroyOverlayElement(card);
        }
        overlay_->remove2D(panel_);
        manager.destroyOverlayElement(panel_);
        manager.destroy(overlay_);
        for (const auto& material : materials_) Ogre::MaterialManager::getSingleton().remove(material->getName());
        Ogre::FontManager::getSingleton().remove(font_->getName());
    }
    ErrorDashboard(const ErrorDashboard&) = delete;
    ErrorDashboard& operator=(const ErrorDashboard&) = delete;
    void setActive(const std::array<bool, 4>& active) {
        if (active == active_) return;
        size_t visible = 0;
        for (size_t row = 0; row < cards_.size(); ++row) {
            if (active[row] != active_[row]) {
                for (auto& chart : charts_[row]) chart->clear();
            }
            if (active[row]) {
                cards_[row]->setPosition(0, 146*visible++);
                cards_[row]->show();
            } else {
                cards_[row]->hide();
            }
        }
        active_ = active;
        layout_pending_ = true;
    }
    void clearHistory() { for (auto& row : charts_) for (auto& chart : row) chart->clear(); }
    void show() { overlay_->show(); }
    void hide() { overlay_->hide(); }
    void refreshLayout() {
        // TextArea adjusts its width on the first rendered frame. Refresh once afterwards
        // so short static labels use their final clipping rectangle.
        if (!layout_pending_) return;
        for (auto* text : texts_) text->_positionsOutOfDate();
        layout_pending_ = false;
    }
    void setRow(size_t row, double mm, double degrees, bool valid,
                const QString& status, const std::array<double, 4>& limits) {
        charts_[row][0]->update(mm, valid, limits[0], limits[1]);
        charts_[row][1]->update(degrees, valid, limits[2], limits[3]);
        setReading(values_[row][0], mm, valid, limits[0], limits[1]);
        setReading(values_[row][1], degrees, valid, limits[2], limits[3]);
        // The font shipped with RViz supports Latin labels; retain all invalid-state reasons.
        const std::string message = valid ? "" :
            status == "当前位姿已超时" ? "State timed out" :
            status == "参考坐标系不一致" ? "Frame mismatch" :
            status == "位姿数据无效" ? "Invalid pose" : "Waiting for poses";
        if (status_[row]->getCaption() != message) status_[row]->setCaption(message);
    }
private:
    void createMaterial(const std::string& suffix, const Ogre::ColourValue& color) {
        auto material = Ogre::MaterialManager::getSingleton().create(
            name_+"/"+suffix, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
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
    Ogre::TextAreaOverlayElement* addText(const std::string& caption, float x, float y,
                                         float height, const Ogre::ColourValue& color) {
        auto* text = static_cast<Ogre::TextAreaOverlayElement*>(
            Ogre::OverlayManager::getSingleton().createOverlayElement(
                "TextArea", name_+"/text"+std::to_string(texts_.size())));
        texts_.push_back(text);
        text->initialise();
        text->setMetricsMode(Ogre::GMM_PIXELS);
        text->setPosition(x, y);
        text->setDimensions(170, height+2);
        text->setFontName(font_->getName());
        text->setCharHeight(height);
        text->setCaption(caption);
        text->setColour(color);
        text_parent_->addChild(text);
        return text;
    }
    static void setReading(Ogre::TextAreaOverlayElement* text, double value, bool valid,
                           double green, double red) {
        valid = valid && std::isfinite(value) && value >= 0 &&
                std::isfinite(green) && std::isfinite(red) && green >= 0 && red > green;
        const auto caption = valid ? QString::number(value, 'f', 2).toStdString() : "--";
        const auto color = !valid ? muted() : value <= green ? Ogre::ColourValue(0.22, 0.72, 0.46) :
            value >= red ? Ogre::ColourValue(0.90, 0.34, 0.40) : Ogre::ColourValue(0.86, 0.63, 0.22);
        if (text->getCaption() != caption) text->setCaption(caption);
        if (text->getColour() != color) text->setColour(color);
        // Keep large readings inside their card without clipping the actual value.
        const float height = caption.size() > 7 ? 32 : 46;
        if (text->getCharHeight() != height) text->setCharHeight(height);
    }
    std::array<std::array<std::unique_ptr<ErrorSparkline>, 2>, 4> charts_{};
    std::array<bool, 4> active_{};
    bool layout_pending_{true};
    std::string name_;
    Ogre::Overlay* overlay_{};
    Ogre::OverlayContainer* panel_{};
    Ogre::FontPtr font_;
    std::vector<Ogre::MaterialPtr> materials_;
    std::array<Ogre::OverlayContainer*, 4> cards_{};
    Ogre::OverlayContainer* text_parent_{};
    std::vector<Ogre::TextAreaOverlayElement*> texts_;
    std::array<std::array<Ogre::TextAreaOverlayElement*, 2>, 4> values_{};
    std::array<Ogre::TextAreaOverlayElement*, 4> status_{};
};
}
