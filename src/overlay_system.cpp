/*
 * RViz Attitude Display Plugin - Overlay System Implementation
 * 
 * Implements the OverlayPanel and OverlayManager classes for managing
 * Ogre overlay resources and rendering Qt widgets to the RViz viewport.
 */

#include "rviz_attitude_plugin/overlay_system.hpp"

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>

#include <QImage>
#include <QPainter>
#include <QWidget>

#include <algorithm>
#include <cstring>

namespace rviz_attitude_plugin
{

// ============================================================================
// OverlayPanel Implementation
// ============================================================================

OverlayPanel::OverlayPanel(const std::string & name)
: name_(name),
  overlay_(nullptr),
  panel_(nullptr)
{
  auto * overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  if (!overlay_mgr) {
    RCLCPP_ERROR(rclcpp::get_logger("rviz_attitude_plugin"), "Ogre OverlayManager not available");
    return;
  }

  // Create resource names
  const std::string overlay_name = name_ + "Overlay";
  const std::string panel_name = name_ + "Panel";
  const std::string material_name = name_ + "Material";

  // Create overlay container
  overlay_ = overlay_mgr->create(overlay_name);

  // Create panel element
  panel_ = static_cast<Ogre::PanelOverlayElement *>(
    overlay_mgr->createOverlayElement("Panel", panel_name));
  
  panel_->setMetricsMode(Ogre::GMM_PIXELS);
  panel_->setHorizontalAlignment(Ogre::GHA_LEFT);
  panel_->setVerticalAlignment(Ogre::GVA_TOP);

  // Create material
  material_ = Ogre::MaterialManager::getSingleton().create(
    material_name,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  panel_->setMaterialName(material_->getName());
  overlay_->add2D(panel_);
  
  // Start hidden
  overlay_->hide();
}

OverlayPanel::~OverlayPanel()
{
  // Clean up Ogre resources in reverse order of creation
  if (overlay_) {
    auto * overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
    if (overlay_mgr) {
      if (panel_) {
        overlay_mgr->destroyOverlayElement(panel_);
        panel_ = nullptr;
      }
      overlay_mgr->destroy(overlay_);
      overlay_ = nullptr;
    }
  }

  // Remove material
  if (material_) {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
    material_.reset();
  }

  // Remove texture
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_->getName());
    texture_.reset();
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
  if (!panel_ || !material_) {
    return;
  }

  // Ensure minimum size
  if (width == 0) {
    width = 1;
  }
  if (height == 0) {
    height = 1;
  }

  const std::string texture_name = name_ + "Texture";

  // Check if we need to recreate the texture
  bool needs_recreation = !texture_ || 
                         texture_->getWidth() != width || 
                         texture_->getHeight() != height;

  if (needs_recreation) {
    // Remove old texture if it exists
    if (texture_) {
      Ogre::TextureManager::getSingleton().remove(texture_->getName());
      material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
      texture_.reset();
    }

    // Create new texture
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      width,
      height,
      0,
      Ogre::PF_A8R8G8B8,  // ARGB format for Qt compatibility
      Ogre::TU_DEFAULT);

    // Attach texture to material
    auto * pass = material_->getTechnique(0)->getPass(0);
    pass->createTextureUnitState(texture_->getName());
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
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
// OverlayManager Implementation
// ============================================================================

OverlayManager::OverlayManager()
: context_(nullptr),
  widget_(nullptr),
  initialized_(false),
  visible_(false)
{
}

OverlayManager::~OverlayManager()
{
  shutdown();
}

void OverlayManager::initialize(
  rviz_common::DisplayContext * context,
  QWidget * widget,
  const std::string & overlay_name)
{
  if (initialized_) {
    shutdown();
  }

  context_ = context;
  widget_ = widget;

  if (!context_ || !widget_) {
    RCLCPP_ERROR(rclcpp::get_logger("rviz_attitude_plugin"), "OverlayManager: Invalid context or widget");
    return;
  }

  try {
    panel_ = std::make_unique<OverlayPanel>(overlay_name);
    initialized_ = true;
    RCLCPP_DEBUG(rclcpp::get_logger("rviz_attitude_plugin"), "OverlayManager initialized: %s", overlay_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rviz_attitude_plugin"), "Failed to create overlay panel: %s", e.what());
    panel_.reset();
    initialized_ = false;
  }
}

void OverlayManager::shutdown()
{
  if (panel_) {
    panel_->hide();
    panel_.reset();
  }

  initialized_ = false;
  visible_ = false;
  context_ = nullptr;
  widget_ = nullptr;
}

void OverlayManager::render()
{
  if (!initialized_ || !visible_ || !panel_ || !widget_) {
    return;
  }

  // Get geometry
  const auto & geom = geometry_manager_.getGeometry();
  const unsigned int width = static_cast<unsigned int>(geom.width);
  const unsigned int height = static_cast<unsigned int>(geom.height);

  if (width == 0 || height == 0) {
    return;
  }

  // Update texture size if needed
  panel_->updateTextureSize(width, height);

  // Get pixel buffer for writing
  auto pixel_buffer = panel_->getPixelBuffer();
  if (!pixel_buffer.valid()) {
    return;
  }

  // Get QImage that writes directly to the pixel buffer
  QImage image = pixel_buffer.getQImage(width, height);
  if (image.isNull()) {
    return;
  }

  // Render widget to the image
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);

  // Resize widget if needed
  if (widget_->size() != QSize(static_cast<int>(width), static_cast<int>(height))) {
    widget_->resize(static_cast<int>(width), static_cast<int>(height));
  }

  // Render the widget
  widget_->render(&painter);

  // QPainter destructor flushes to image, pixel_buffer destructor unlocks the buffer
  painter.end();

  // Update position
  updatePosition();
}

void OverlayManager::updateProperties(
  int width,
  int height,
  int offset_x,
  int offset_y,
  OverlayGeometryManager::Anchor anchor,
  bool visible)
{
  if (!initialized_ || !panel_) {
    return;
  }

  // Update geometry
  geometry_manager_.setGeometry(width, height, offset_x, offset_y, anchor);

  // Update dimensions
  panel_->setDimensions(static_cast<unsigned int>(width), static_cast<unsigned int>(height));

  // Update visibility
  visible_ = visible;
  if (visible_) {
    panel_->show();
    updatePosition();
  } else {
    panel_->hide();
  }
}

bool OverlayManager::isVisible() const
{
  return visible_ && initialized_ && panel_ && panel_->isVisible();
}

int OverlayManager::getWidth() const
{
  return geometry_manager_.getGeometry().width;
}

int OverlayManager::getHeight() const
{
  return geometry_manager_.getGeometry().height;
}

void OverlayManager::updatePosition()
{
  if (!initialized_ || !panel_) {
    return;
  }

  auto * render_panel = getRenderPanel();
  if (!render_panel) {
    return;
  }

  // Get panel size
  const QSize panel_size = render_panel->size();
  
  // Calculate position based on anchor
  auto [x, y] = geometry_manager_.calculatePosition(panel_size);

  // Update panel position
  panel_->setPosition(x, y);
}

rviz_common::RenderPanel * OverlayManager::getRenderPanel()
{
  if (!context_) {
    return nullptr;
  }

  // Get the first (main) render panel from the context
  auto render_panel = context_->getViewManager()->getRenderPanel();
  return render_panel;
}

}  // namespace rviz_attitude_plugin
