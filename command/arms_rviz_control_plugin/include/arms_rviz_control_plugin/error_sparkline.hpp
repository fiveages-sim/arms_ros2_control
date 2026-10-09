#pragma once
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgreOverlayContainer.h>
#include <OgreTextureManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <QImage>
#include <QPainter>
#include <chrono>
#include <deque>
#include <cmath>
#include <algorithm>

namespace arms_rviz_control_plugin {
// Small transparent texture on a native Ogre overlay, sampled at most 5 Hz.
class ErrorSparkline {
public:
    ErrorSparkline(const std::string& name, Ogre::OverlayContainer* parent, float x)
        : name_(name), parent_(parent) {
        texture_ = Ogre::TextureManager::getSingleton().createManual(name_,
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
            width, height, 0, Ogre::PF_BYTE_RGBA, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
        material_ = Ogre::MaterialManager::getSingleton().create(name_,
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        auto* pass = material_->getTechnique(0)->getPass(0);
        pass->setLightingEnabled(false);
        pass->setDepthCheckEnabled(false);
        pass->setDepthWriteEnabled(false);
        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        pass->createTextureUnitState(name_)->setTextureFiltering(Ogre::TFO_NONE);
        panel_ = static_cast<Ogre::OverlayContainer*>(
            Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", name_));
        panel_->setMetricsMode(Ogre::GMM_PIXELS);
        panel_->setPosition(x, 38);
        panel_->setDimensions(width, height);
        panel_->setMaterialName(name_);
        parent_->addChild(panel_);
        QImage blank(width, height, QImage::Format_RGBA8888);
        blank.fill(Qt::transparent);
        upload(blank);
    }
    ~ErrorSparkline() {
        parent_->removeChild(panel_->getName());
        Ogre::OverlayManager::getSingleton().destroyOverlayElement(panel_);
        Ogre::MaterialManager::getSingleton().remove(name_);
        material_.reset();
        Ogre::TextureManager::getSingleton().remove(name_);
    }
    void clear() {
        samples_.clear();
        last_ = {};
        QImage blank(width, height, QImage::Format_RGBA8888);
        blank.fill(Qt::transparent);
        upload(blank);
    }
    void update(double value, bool valid, double green, double red) {
        const auto now = std::chrono::steady_clock::now();
        if (last_.time_since_epoch().count() && now-last_ < std::chrono::milliseconds(200)) return;
        last_ = now;
        const double t = std::chrono::duration<double>(now.time_since_epoch()).count();
        valid = valid && std::isfinite(value) && value >= 0;
        samples_.push_back({t, value, valid});
        while (!samples_.empty() && samples_.front().time < t-window_seconds) samples_.pop_front();
        const double ceiling = 2*red;
        if (!std::isfinite(ceiling) || ceiling <= 0) return;
        QImage image(width, height, QImage::Format_RGBA8888);
        image.fill(Qt::transparent);
        QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setPen(QPen(QColor(110, 125, 145, 65), 1));
        painter.drawLine(1, 44, width-1, 44);
        painter.setPen(QPen(QColor(200, 80, 90, 65), 1, Qt::DashLine));
        painter.drawLine(1, 23, width-1, 23); // Red threshold; fixed upper bound is twice red.
        QPointF previous;
        bool connected = false;
        double previous_time = 0;
        for (const auto& sample : samples_) {
            if (!sample.valid) { connected = false; continue; }
            const QPointF point(1+(sample.time-(t-window_seconds))/window_seconds*(width-2),
                                44-std::clamp(sample.value/ceiling, 0.0, 1.0)*42);
            const QColor color = sample.value <= green ? QColor(56,184,117,110) :
                sample.value >= red ? QColor(230,87,102,110) : QColor(219,161,56,110);
            painter.setPen(QPen(color, 2));
            if (connected && sample.time-previous_time < 0.6) painter.drawLine(previous, point);
            else painter.drawPoint(point);
            if (sample.value > ceiling) painter.drawLine(QPointF(point.x(), 1), QPointF(point.x(), 5));
            previous = point;
            previous_time = sample.time;
            connected = true;
        }
        painter.end();
        upload(image);
    }
private:
    void upload(QImage& image) {
        Ogre::PixelBox pixels(width, height, 1, Ogre::PF_BYTE_RGBA, image.bits());
        texture_->getBuffer()->blitFromMemory(pixels);
    }
    static constexpr double window_seconds = 5.0;
    static constexpr int width = 150, height = 46;
    struct Sample { double time, value; bool valid; };
    std::string name_;
    Ogre::OverlayContainer* parent_;
    Ogre::OverlayContainer* panel_{};
    Ogre::TexturePtr texture_;
    Ogre::MaterialPtr material_;
    std::deque<Sample> samples_;
    std::chrono::steady_clock::time_point last_{};
};
}
