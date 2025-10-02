/*
 * The overlay system provides a complete solution for rendering Qt widgets
 * as Ogre overlays on the RViz viewport.
 */
#ifndef RVIZ_ATTITUDE_PLUGIN__OVERLAY_SYSTEM_HPP_
#define RVIZ_ATTITUDE_PLUGIN__OVERLAY_SYSTEM_HPP_

#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include <QImage>
#include <QSize>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

namespace rviz_common { class DisplayContext; class RenderPanel; }
class QWidget;

namespace rviz_attitude_plugin
{

class AttitudeWidget;

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

  /**
   * @brief Geometry parameters for overlay positioning.
   */
  struct Geometry
  {
    int width{320};
    int height{240};
    int offset_x{16};
    int offset_y{16};
    Anchor anchor{Anchor::BottomRight};
  };

  void setGeometry(int width, int height,
                   int offset_x, int offset_y,
                   Anchor anchor);

  const Geometry & getGeometry() const;

  std::pair<int, int> calculateClampedOffsets(const QSize & panel_size) const;
  std::pair<int, int> calculateAbsolutePosition(const QSize & panel_size) const;
  bool fitsWithinPanel(const QSize & panel_size) const;
  std::pair<int, int> getDimensions() const;
  std::pair<int, int> getOffsets() const;
  Anchor getAnchor() const;

private:
  Geometry geometry_;
};

/**
 * @brief RAII wrapper for Ogre pixel buffer access.
 * 
 * Automatically locks buffer on construction and unlocks on destruction.
 * Provides safe QImage access to the buffer contents.
 */
class ScopedPixelBuffer
{
public:
  explicit ScopedPixelBuffer(const Ogre::HardwarePixelBufferSharedPtr & buffer);
  ScopedPixelBuffer(const ScopedPixelBuffer &) = delete;
  ScopedPixelBuffer & operator=(const ScopedPixelBuffer &) = delete;
  ScopedPixelBuffer(ScopedPixelBuffer && other) noexcept;
  ScopedPixelBuffer & operator=(ScopedPixelBuffer && other) noexcept;
  ~ScopedPixelBuffer();

  bool valid() const;
  QImage getQImage(unsigned int width, unsigned int height);

private:
  Ogre::HardwarePixelBufferSharedPtr buffer_;
};

// ============================================================================
// OverlayPanel - Low-level Ogre overlay panel management
// ============================================================================

/**
 * @brief Ogre overlay panel for rendering Qt widgets.
 * 
 * Manages an Ogre overlay panel that can be positioned and sized on screen.
 * Provides Qt-compatible pixel buffer access for custom rendering.
 * Handles Ogre resources (overlay, panel, material, texture).
 */
class OverlayPanel
{
public:
  explicit OverlayPanel(const std::string & name);
  ~OverlayPanel();

  void show();
  void hide();
  bool isVisible() const;

  void setPosition(int left, int top);
  void setDimensions(unsigned int width, unsigned int height);
  void updateTextureSize(unsigned int width, unsigned int height);

  ScopedPixelBuffer getPixelBuffer();
  unsigned int textureWidth() const;
  unsigned int textureHeight() const;

private:
  std::string name_;
  Ogre::Overlay * overlay_;
  Ogre::PanelOverlayElement * panel_;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
};

class OverlayManager
{
public:
  OverlayManager();
  ~OverlayManager();

  void attach(rviz_common::DisplayContext * context);

  void setGeometry(int width,
                   int height,
                   int offset_x,
                   int offset_y,
                   OverlayGeometryManager::Anchor anchor);

  void setVisible(bool visible);
  void render(AttitudeWidget & widget);

  rviz_common::RenderPanel * getRenderPanel() const { return render_panel_; }

private:
  rviz_common::RenderPanel * render_panel_;
  std::unique_ptr<OverlayPanel> overlay_panel_;
};

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__OVERLAY_SYSTEM_HPP_
