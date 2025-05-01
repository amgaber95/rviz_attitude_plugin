/*
 * RViz Attitude Display Plugin - Overlay geometry helpers (header-only)
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__OVERLAY_SYSTEM_HPP_
#define RVIZ_ATTITUDE_PLUGIN__OVERLAY_SYSTEM_HPP_

#include <algorithm>
#include <utility>

#include <QSize>

namespace rviz_attitude_plugin
{

/**
 * @brief Calculates overlay positioning relative to an RViz render panel.
 *
 * Stores simple width/height/offset information and resolves the absolute
 * position based on an anchor selection. All methods are inline to avoid
 * translation unit coupling when included broadly.
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

  /**
   * @brief Clamp the configured offsets so the overlay stays inside panel bounds.
   */
  inline std::pair<int, int> clampOffsets(const QSize & panel_size) const
  {
    const int max_x = std::max(0, panel_size.width() - geometry_.width);
    const int max_y = std::max(0, panel_size.height() - geometry_.height);

    const int clamped_x = std::clamp(geometry_.offset_x, 0, max_x);
    const int clamped_y = std::clamp(geometry_.offset_y, 0, max_y);
    return {clamped_x, clamped_y};
  }

  /**
   * @brief Calculate absolute overlay position relative to the selected anchor.
   */
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
        // already relative to top-left
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

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__OVERLAY_SYSTEM_HPP_

