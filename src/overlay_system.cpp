/*
 * RViz Attitude Display Plugin - Overlay System Implementation
 * 
 * Implements the OverlayPanel class for managing Ogre overlay resources
 * and rendering Qt widgets to the RViz viewport.
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

#include <rviz_common/logging.hpp>

#include <algorithm>
#include <cstring>

namespace rviz_attitude_plugin
{

// ============================================================================
// ScopedPixelBuffer Implementation
// ============================================================================

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

  // Clear buffer to transparent
  std::memset(dest, 0, width * height * 4);

  // Return QImage that writes directly to the buffer memory
  return QImage(dest, static_cast<int>(width), static_cast<int>(height), QImage::Format_ARGB32);
}

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
    RVIZ_COMMON_LOG_ERROR_STREAM("Ogre OverlayManager not available");
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

}  // namespace rviz_attitude_plugin
