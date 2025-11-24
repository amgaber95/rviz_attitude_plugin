#include "rviz_attitude_plugin/overlay_system.hpp"
#include "rviz_attitude_plugin/attitude_widget.hpp"

#include <atomic>
#include <OgreHardwarePixelBuffer.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_system.hpp>

#include <QImage>
#include <QPainter>

#include <algorithm>
#include <cstring>

namespace rviz_attitude_plugin
{

void OverlayGeometryManager::setGeometry(
  int width, int height,
  int offset_x, int offset_y,
  Anchor anchor)
{
  geometry_.width = width;
  geometry_.height = height;
  geometry_.offset_x = offset_x;
  geometry_.offset_y = offset_y;
  geometry_.anchor = anchor;
}

const OverlayGeometryManager::Geometry & OverlayGeometryManager::getGeometry() const
{
  return geometry_;
}

std::pair<int, int> OverlayGeometryManager::calculateClampedOffsets(const QSize & panel_size) const
{
  const int max_x_offset = std::max(0, panel_size.width() - geometry_.width);
  const int max_y_offset = std::max(0, panel_size.height() - geometry_.height);

  const int clamped_x = std::clamp(geometry_.offset_x, 0, max_x_offset);
  const int clamped_y = std::clamp(geometry_.offset_y, 0, max_y_offset);

  return {clamped_x, clamped_y};
}

std::pair<int, int> OverlayGeometryManager::calculateAbsolutePosition(const QSize & panel_size) const
{
  auto [clamped_x, clamped_y] = calculateClampedOffsets(panel_size);

  int absolute_x = clamped_x;
  int absolute_y = clamped_y;

  // Apply anchor based on corner position
  switch (geometry_.anchor) {
    case Anchor::TopRight:
      // Anchor to top-right: offset from right edge, top stays as-is
      absolute_x = panel_size.width() - geometry_.width - clamped_x;
      break;
    case Anchor::TopLeft:
      // Anchor to top-left: offset from left edge, top stays as-is
      break;
    case Anchor::BottomRight:
      // Anchor to bottom-right: offset from right and bottom edges
      absolute_x = panel_size.width() - geometry_.width - clamped_x;
      absolute_y = panel_size.height() - geometry_.height - clamped_y;
      break;
    case Anchor::BottomLeft:
      // Anchor to bottom-left: offset from left edge and bottom edge
      absolute_y = panel_size.height() - geometry_.height - clamped_y;
      break;
  }

  return {absolute_x, absolute_y};
}

bool OverlayGeometryManager::fitsWithinPanel(const QSize & panel_size) const
{
  return geometry_.width <= panel_size.width() &&
         geometry_.height <= panel_size.height();
}

std::pair<int, int> OverlayGeometryManager::getDimensions() const
{
  return {geometry_.width, geometry_.height};
}

std::pair<int, int> OverlayGeometryManager::getOffsets() const
{
  return {geometry_.offset_x, geometry_.offset_y};
}

OverlayGeometryManager::Anchor OverlayGeometryManager::getAnchor() const
{
  return geometry_.anchor;
}

ScopedPixelBuffer::ScopedPixelBuffer(const Ogre::HardwarePixelBufferSharedPtr & buffer)
: buffer_(buffer)
{
  if (buffer_) {
    buffer_->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  }
}

ScopedPixelBuffer::ScopedPixelBuffer(ScopedPixelBuffer && other) noexcept
: buffer_(std::move(other.buffer_))
{
}

ScopedPixelBuffer & ScopedPixelBuffer::operator=(ScopedPixelBuffer && other) noexcept
{
  if (this != &other) {
    if (buffer_) {
      buffer_->unlock();
    }
    buffer_ = std::move(other.buffer_);
  }
  return *this;
}

ScopedPixelBuffer::~ScopedPixelBuffer()
{
  if (buffer_) {
    buffer_->unlock();
  }
}

bool ScopedPixelBuffer::valid() const
{
  return static_cast<bool>(buffer_);
}

QImage ScopedPixelBuffer::getQImage(unsigned int width, unsigned int height)
{
  if (!buffer_) {
    return QImage();
  }

  const Ogre::PixelBox & pixel_box = buffer_->getCurrentLock();
  auto * dest = static_cast<Ogre::uint8 *>(pixel_box.data);
  if (!dest) {
    return QImage();
  }
  // Clear to transparent
  std::memset(dest, 0, width * height * 4);
  return QImage(dest, width, height, QImage::Format_ARGB32);
}


OverlayPanel::OverlayPanel(const std::string & name)
: name_(name)
{
  auto * overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  if (!overlay_mgr) {
    RVIZ_COMMON_LOG_ERROR_STREAM("Ogre OverlayManager not available for Attitude HUD");
    overlay_ = nullptr;
    panel_ = nullptr;
    return;
  }

  const std::string overlay_name = name_ + "Overlay";
  const std::string panel_name = name_ + "Panel";
  const std::string material_name = name_ + "Material";

  overlay_ = overlay_mgr->create(overlay_name);
  panel_ = static_cast<Ogre::PanelOverlayElement *>(
    overlay_mgr->createOverlayElement("Panel", panel_name));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);
  panel_->setHorizontalAlignment(Ogre::GHA_LEFT);
  panel_->setVerticalAlignment(Ogre::GVA_TOP);

  material_ = Ogre::MaterialManager::getSingleton().create(
    material_name,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  panel_->setMaterialName(material_->getName());
  overlay_->add2D(panel_);
  overlay_->hide();
}

OverlayPanel::~OverlayPanel()
{
  if (overlay_) {
    auto * overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
    if (overlay_mgr) {
      overlay_mgr->destroyOverlayElement(panel_);
      overlay_mgr->destroy(overlay_);
    }
  }

  if (material_) {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
  }

  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_->getName());
  }
}

void OverlayPanel::show()
{
  if (overlay_) {
    overlay_->show();
  }
}

void OverlayPanel::hide()
{
  if (overlay_) {
    overlay_->hide();
  }
}

bool OverlayPanel::isVisible() const
{
  return overlay_ && overlay_->isVisible();
}

void OverlayPanel::setPosition(int left, int top)
{
  if (panel_) {
    panel_->setPosition(static_cast<Ogre::Real>(left), static_cast<Ogre::Real>(top));
  }
}

void OverlayPanel::setDimensions(unsigned int width, unsigned int height)
{
  if (panel_) {
    panel_->setDimensions(static_cast<Ogre::Real>(width), static_cast<Ogre::Real>(height));
  }
}

void OverlayPanel::updateTextureSize(unsigned int width, unsigned int height)
{
  if (!panel_) {
    return;
  }

  if (width == 0) {
    width = 1;
  }
  if (height == 0) {
    height = 1;
  }

  const std::string texture_name = name_ + "Texture";

  if (!texture_ || texture_->getWidth() != width || texture_->getHeight() != height) {
    if (texture_) {
      Ogre::TextureManager::getSingleton().remove(texture_->getName());
      material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    }

    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      width,
      height,
      0,
      Ogre::PF_A8R8G8B8,
      Ogre::TU_DEFAULT);

    material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_->getName());
    material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  }
}

ScopedPixelBuffer OverlayPanel::getPixelBuffer()
{
  if (!texture_) {
    return ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr());
  }
  return ScopedPixelBuffer(texture_->getBuffer());
}

unsigned int OverlayPanel::textureWidth() const
{
  return texture_ ? texture_->getWidth() : 0;
}

unsigned int OverlayPanel::textureHeight() const
{
  return texture_ ? texture_->getHeight() : 0;
}

// ============================================================================
// OverlayManager - Implementation
// ============================================================================

OverlayManager::OverlayManager()
: render_panel_(nullptr)
{
}

OverlayManager::~OverlayManager() = default;

void OverlayManager::attach(rviz_common::DisplayContext * context)
{
  if (!overlay_panel_) {
    static std::atomic<int> overlay_count{0};
    rviz_rendering::RenderSystem::get()->prepareOverlays(context->getSceneManager());
    overlay_panel_ = std::make_unique<OverlayPanel>("AttitudeDisplayHUD" + std::to_string(overlay_count++));
  }
  if (!render_panel_) {
    auto * view_manager = context->getViewManager();
    if (view_manager) {
      render_panel_ = view_manager->getRenderPanel();
    }
  }
}

void OverlayManager::setGeometry(int width,
                             int height,
                             int offset_x,
                             int offset_y,
                             OverlayGeometryManager::Anchor anchor)
{
  if (!overlay_panel_ || !render_panel_) return;

  const auto panel_size = render_panel_->size();
  const int max_x_offset = std::max(0, panel_size.width() - width);
  const int max_y_offset = std::max(0, panel_size.height() - height);

  offset_x = std::clamp(offset_x, 0, max_x_offset);
  offset_y = std::clamp(offset_y, 0, max_y_offset);

  int x = offset_x;
  int y = offset_y;

  // Calculate position based on anchor corner
  switch (anchor) {
    case OverlayGeometryManager::Anchor::TopRight:
      x = panel_size.width() - width - offset_x;
      break;
    case OverlayGeometryManager::Anchor::TopLeft:
      // x and y already set correctly
      break;
    case OverlayGeometryManager::Anchor::BottomRight:
      x = panel_size.width() - width - offset_x;
      y = panel_size.height() - height - offset_y;
      break;
    case OverlayGeometryManager::Anchor::BottomLeft:
      y = panel_size.height() - height - offset_y;
      break;
  }

  overlay_panel_->updateTextureSize(static_cast<unsigned int>(width), static_cast<unsigned int>(height));
  overlay_panel_->setDimensions(static_cast<unsigned int>(width), static_cast<unsigned int>(height));
  overlay_panel_->setPosition(x, y);
}

void OverlayManager::setVisible(bool visible)
{
  if (!overlay_panel_) return;
  if (visible) overlay_panel_->show(); else overlay_panel_->hide();
}

void OverlayManager::render(AttitudeWidget & widget)
{
  if (!overlay_panel_) return;
  const auto width = overlay_panel_->textureWidth();
  const auto height = overlay_panel_->textureHeight();
  if (width == 0 || height == 0) return;

  overlay_panel_->updateTextureSize(width, height);
  overlay_panel_->setDimensions(width, height);

  // Ensure widget matches overlay dimensions for correct rendering
  // TODO: Consider having widget manage its own preferred size
  widget.resize(static_cast<int>(width), static_cast<int>(height));

  ScopedPixelBuffer buffer = overlay_panel_->getPixelBuffer();
  if (!buffer.valid()) return;

  QImage image = buffer.getQImage(width, height);
  if (image.isNull()) return;
  image.fill(Qt::transparent);

  QPainter painter(&image);
  widget.render(&painter);
  painter.end();
}

}  // namespace rviz_attitude_plugin
