/*
 * RViz Attitude Display Plugin - Overlay utilities
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__OVERLAY_SYSTEM_HPP_
#define RVIZ_ATTITUDE_PLUGIN__OVERLAY_SYSTEM_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include <QImage>
#include <QSize>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>

namespace rviz_attitude_plugin
{

/**
 * @brief Computes overlay geometry relative to an RViz render panel.
 *
 * Stores basic width/height/offset data and resolves the absolute position
 * using a configurable anchor. Methods are inline so the helper can be used
 * from multiple translation units without requiring a dedicated source file.
 */
class OverlayGeometryManager
{
public:
  enum class Anchor
  {
    TopRight = 0,
    TopLeft = 1,
    BottomRight = 2,
    BottomLeft = 3
  };

  struct Geometry
  {
    int width{320};
    int height{240};
    int offset_x{16};
    int offset_y{16};
    Anchor anchor{Anchor::BottomRight};
  };

  OverlayGeometryManager() = default;
  ~OverlayGeometryManager() = default;

  inline void setGeometry(int width,
                          int height,
                          int offset_x,
                          int offset_y,
                          Anchor anchor)
  {
    geometry_.width = std::max(1, width);
    geometry_.height = std::max(1, height);
    geometry_.offset_x = std::max(0, offset_x);
    geometry_.offset_y = std::max(0, offset_y);
    geometry_.anchor = anchor;
  }

  inline const Geometry & getGeometry() const
  {
    return geometry_;
  }

  inline std::pair<int, int> clampOffsets(const QSize & panel_size) const
  {
    const int max_x = std::max(0, panel_size.width() - geometry_.width);
    const int max_y = std::max(0, panel_size.height() - geometry_.height);

    const int clamped_x = std::clamp(geometry_.offset_x, 0, max_x);
    const int clamped_y = std::clamp(geometry_.offset_y, 0, max_y);
    return {clamped_x, clamped_y};
  }

  inline std::pair<int, int> calculatePosition(const QSize & panel_size) const
  {
    auto [offset_x, offset_y] = clampOffsets(panel_size);
    int x = offset_x;
    int y = offset_y;

    switch (geometry_.anchor) {
      case Anchor::TopRight:
        x = panel_size.width() - geometry_.width - offset_x;
        break;
      case Anchor::TopLeft:
        break;
      case Anchor::BottomRight:
        x = panel_size.width() - geometry_.width - offset_x;
        y = panel_size.height() - geometry_.height - offset_y;
        break;
      case Anchor::BottomLeft:
        y = panel_size.height() - geometry_.height - offset_y;
        break;
    }

    return {x, y};
  }

private:
  Geometry geometry_;
};

/**
 * @brief RAII wrapper that locks an Ogre pixel buffer for the lifetime of the object.
 *
 * Handles lock/unlock automatically and exposes a convenience helper for
 * creating a QImage that writes directly into the buffer memory.
 */
class ScopedPixelBuffer
{
public:
  explicit ScopedPixelBuffer(const Ogre::HardwarePixelBufferSharedPtr & buffer)
  : buffer_(buffer)
  {
    if (buffer_) {
      buffer_->lock(Ogre::HardwareBuffer::HBL_NORMAL);
    }
  }

  ScopedPixelBuffer(const ScopedPixelBuffer &) = delete;
  ScopedPixelBuffer & operator=(const ScopedPixelBuffer &) = delete;

  ScopedPixelBuffer(ScopedPixelBuffer && other) noexcept
  : buffer_(std::move(other.buffer_))
  {
  }

  ScopedPixelBuffer & operator=(ScopedPixelBuffer && other) noexcept
  {
    if (this != &other) {
      release();
      buffer_ = std::move(other.buffer_);
    }
    return *this;
  }

  ~ScopedPixelBuffer()
  {
    release();
  }

  inline bool valid() const
  {
    return static_cast<bool>(buffer_);
  }

  inline QImage getQImage(unsigned int width, unsigned int height)
  {
    if (!buffer_) {
      return QImage();
    }

    const Ogre::PixelBox & pixel_box = buffer_->getCurrentLock();
    auto * dest = static_cast<Ogre::uint8 *>(pixel_box.data);
    if (!dest) {
      return QImage();
    }

    return QImage(dest, static_cast<int>(width), static_cast<int>(height), QImage::Format_ARGB32);
  }

private:
  inline void release()
  {
    if (buffer_) {
      buffer_->unlock();
      buffer_.reset();
    }
  }

  Ogre::HardwarePixelBufferSharedPtr buffer_;
};

/**
 * @brief Manages a single Ogre overlay panel for rendering.
 *
 * Handles creation, destruction, and updates of Ogre overlay resources including:
 * - Overlay container
 * - Panel element  
 * - Material with texture
 * - Texture for pixel buffer access
 *
 * Provides methods for positioning, resizing, and updating the overlay texture
 * from a QImage via the pixel buffer.
 */
class OverlayPanel
{
public:
  /**
   * @brief Construct an overlay panel with the given name.
   * 
   * Creates Ogre overlay, panel element, material, and texture resources.
   * The overlay is initially hidden.
   * 
   * @param name Base name for the overlay resources (will be suffixed with "Overlay", "Panel", etc.)
   */
  explicit OverlayPanel(const std::string & name);
  
  /**
   * @brief Destructor - cleans up all Ogre resources.
   */
  ~OverlayPanel();

  // Disable copying
  OverlayPanel(const OverlayPanel &) = delete;
  OverlayPanel & operator=(const OverlayPanel &) = delete;

  /**
   * @brief Show the overlay.
   */
  void show();

  /**
   * @brief Hide the overlay.
   */
  void hide();

  /**
   * @brief Check if the overlay is currently visible.
   * @return true if visible, false otherwise
   */
  bool isVisible() const;

  /**
   * @brief Set the overlay position in pixels.
   * @param left Left coordinate in pixels from the left edge
   * @param top Top coordinate in pixels from the top edge
   */
  void setPosition(int left, int top);

  /**
   * @brief Set the overlay dimensions in pixels.
   * @param width Width in pixels
   * @param height Height in pixels
   */
  void setDimensions(unsigned int width, unsigned int height);

  /**
   * @brief Update the texture size, recreating the texture if necessary.
   * 
   * If the texture size changes, the old texture is destroyed and a new one
   * is created with the specified dimensions.
   * 
   * @param width New texture width (minimum 1)
   * @param height New texture height (minimum 1)
   */
  void updateTextureSize(unsigned int width, unsigned int height);

  /**
   * @brief Get a locked pixel buffer for writing texture data.
   * 
   * Returns a RAII-wrapped pixel buffer that is automatically unlocked
   * when the ScopedPixelBuffer object is destroyed.
   * 
   * @return ScopedPixelBuffer that provides access to the texture pixels
   */
  ScopedPixelBuffer getPixelBuffer();

  /**
   * @brief Get the current texture width.
   * @return Texture width in pixels, or 0 if no texture exists
   */
  unsigned int textureWidth() const;

  /**
   * @brief Get the current texture height.
   * @return Texture height in pixels, or 0 if no texture exists
   */
  unsigned int textureHeight() const;

private:
  std::string name_;
  Ogre::Overlay * overlay_;
  Ogre::PanelOverlayElement * panel_;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
};

}  // namespace rviz_attitude_plugin

// Forward declarations
namespace rviz_common {
class DisplayContext;
class RenderPanel;
}

class QWidget;

namespace rviz_attitude_plugin
{

// ============================================================================
// OverlayManager - High-level overlay coordinator and rendering pipeline
// ============================================================================

/**
 * @brief High-level manager coordinating the complete overlay rendering pipeline.
 * 
 * Responsibilities:
 * - Initialize and manage OverlayPanel lifecycle
 * - Coordinate OverlayGeometryManager for positioning
 * - Render QWidget (AttitudeWidget) to QImage
 * - Transfer QImage to Ogre texture via pixel buffer
 * - Manage overlay visibility and Z-order
 * - Handle property updates (size, position, anchor, visibility)
 * 
 * This class provides the complete integration between Qt widgets and
 * RViz's Ogre-based rendering system.
 */
class OverlayManager
{
public:
  /**
   * @brief Construct an overlay manager.
   * 
   * Does not create any resources until initialize() is called.
   */
  OverlayManager();

  /**
   * @brief Destructor - ensures proper cleanup.
   */
  ~OverlayManager();

  // Disable copying
  OverlayManager(const OverlayManager &) = delete;
  OverlayManager & operator=(const OverlayManager &) = delete;

  /**
   * @brief Initialize the overlay system.
   * 
   * Creates the OverlayPanel and sets up the rendering pipeline.
   * Must be called before render() can be used.
   * 
   * @param context RViz display context for accessing render panel
   * @param widget The AttitudeWidget to render
   * @param overlay_name Unique name for the overlay resources
   */
  void initialize(
    rviz_common::DisplayContext * context,
    QWidget * widget,
    const std::string & overlay_name);

  /**
   * @brief Shutdown and clean up overlay resources.
   * 
   * Destroys the OverlayPanel and releases all resources.
   * Safe to call multiple times.
   */
  void shutdown();

  /**
   * @brief Render the widget to the overlay.
   * 
   * This is the main rendering method that:
   * 1. Renders the widget to a QImage
   * 2. Copies the QImage to the Ogre texture
   * 3. Updates overlay position based on current geometry
   * 
   * Should be called from the display's update() method.
   * Does nothing if not initialized or if invisible.
   */
  void render();

  /**
   * @brief Update overlay properties (geometry, visibility).
   * 
   * Updates the overlay's size, position, anchor point, and visibility.
   * Call this when RViz properties change.
   * 
   * @param width Overlay width in pixels
   * @param height Overlay height in pixels
   * @param offset_x Offset from anchor point (X)
   * @param offset_y Offset from anchor point (Y)
   * @param anchor Anchor position (TopRight, TopLeft, BottomRight, BottomLeft)
   * @param visible Whether the overlay should be visible
   */
  void updateProperties(
    int width,
    int height,
    int offset_x,
    int offset_y,
    OverlayGeometryManager::Anchor anchor,
    bool visible);

  /**
   * @brief Check if the overlay is currently visible.
   * @return true if visible, false otherwise
   */
  bool isVisible() const;

  /**
   * @brief Get the current overlay width.
   * @return Width in pixels
   */
  int getWidth() const;

  /**
   * @brief Get the current overlay height.
   * @return Height in pixels
   */
  int getHeight() const;

private:
  /**
   * @brief Update overlay position based on current geometry and render panel size.
   */
  void updatePosition();

  /**
   * @brief Get the current RViz render panel.
   * @return Pointer to render panel, or nullptr if not available
   */
  rviz_common::RenderPanel * getRenderPanel();

  rviz_common::DisplayContext * context_;
  QWidget * widget_;
  std::unique_ptr<OverlayPanel> panel_;
  OverlayGeometryManager geometry_manager_;
  bool initialized_;
  bool visible_;
};

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__OVERLAY_SYSTEM_HPP_

