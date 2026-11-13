// stereo_synth: right image of the ZED 2i stereo pair, synthesized from the left colour image and its
// pixel-aligned depth (depth-image-based rendering), so that the simulation renders 2 images instead of 3.
//
// The left and right cameras of a rectified pair have the same intrinsics and differ only by the
// baseline b along the optical x axis (right camera at +b): a point at depth Z seen at column u in the
// left image is at column u - fx * b / Z in the right one (same row). Every left pixel is moved by its
// disparity; where two pixels land on the same spot the nearer one wins (z-buffer). Neighbouring
// pixels of the same surface fill the columns between them (no cracks when a surface is stretched).
// Columns that no left pixel reaches (background hidden behind an edge in the left view, and the
// right border, outside the left field of view) are filled with the farther of the two neighbours,
// i.e. with the background.
//
// Inputs: color_in (ideal left colour, rgb8/bgr8/mono8), depth_in (ideal depth, 32FC1, metres, same
// stamp as the colour image), camera_info_in (left camera_info).
// Outputs: right_out (ideal right image, same stamp, frame right_frame_id; camera_noise adds the
// noise afterwards, independent of the left image) and right_info_out (same intrinsics, P[3] = -fx * b:
// the baseline as stereo VIO reads it).
// Limits (the simulation is a bit optimistic): same exposure and lighting as the left image,
// view-dependent effects copied from the left view, filled strips are an approximation.

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

namespace
{
constexpr float FAR = std::numeric_limits<float>::infinity();

int64_t stamp_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}
}  // namespace

class StereoSynth : public rclcpp::Node
{
public:
  StereoSynth()
  : Node("stereo_synth")
  {
    baseline_ = declare_parameter<double>("baseline", 0.12);              // m, ZED 2i datasheet
    right_frame_ = declare_parameter<std::string>("right_frame_id", "camera_right_optical_frame");
    // two neighbouring pixels belong to the same surface if their depths differ less than this (relative)
    same_surface_ = declare_parameter<double>("same_surface", 0.05);
    // colour and depth come from two gz sensors with the same stamps (checked on a recorded flight):
    // they are paired by stamp, with this tolerance
    max_dt_ns_ = static_cast<int64_t>(declare_parameter<double>("max_dt", 0.005) * 1e9);

    // reliable, like the gz bridge and camera_noise
    right_pub_ = create_publisher<Image>("right_out", 5);
    info_pub_ = create_publisher<CameraInfo>("right_info_out", 5);
    info_sub_ = create_subscription<CameraInfo>(
      "camera_info_in", 5, [this](CameraInfo::ConstSharedPtr msg) {info_ = *msg;});
    color_sub_ = create_subscription<Image>(
      "color_in", 5, [this](Image::ConstSharedPtr msg) {push(colors_, msg); pair();});
    depth_sub_ = create_subscription<Image>(
      "depth_in", 5, [this](Image::ConstSharedPtr msg) {push(depths_, msg); pair();});
  }

private:
  static void push(std::deque<Image::ConstSharedPtr> & q, const Image::ConstSharedPtr & msg)
  {
    q.push_back(msg);
    if (q.size() > 5) {q.pop_front();}
  }

  // oldest colour image with a depth image of the same stamp; older unpaired images are dropped
  void pair()
  {
    while (!colors_.empty() && !depths_.empty()) {
      const int64_t tc = stamp_ns(colors_.front()->header.stamp);
      const int64_t td = stamp_ns(depths_.front()->header.stamp);
      if (std::llabs(tc - td) <= max_dt_ns_) {
        synthesize(*colors_.front(), *depths_.front());
        colors_.pop_front();
        depths_.pop_front();
      } else if (tc < td) {
        colors_.pop_front();
      } else {
        depths_.pop_front();
      }
    }
  }

  void synthesize(const Image & color, const Image & depth)
  {
    if (!info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for camera_info_in");
      return;
    }
    const int w = static_cast<int>(color.width), h = static_cast<int>(color.height);
    if (depth.encoding != "32FC1" || static_cast<int>(depth.width) != w || static_cast<int>(depth.height) != h ||
      depth.is_bigendian || color.width == 0 || color.step % color.width != 0)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "unsupported images: colour %s %ux%u, depth %s %ux%u", color.encoding.c_str(), color.width,
        color.height, depth.encoding.c_str(), depth.width, depth.height);
      return;
    }
    const size_t bpp = color.step / color.width;                     // bytes per pixel
    const double fb = info_->k[0] * baseline_;                        // fx * b: disparity = fb / Z

    auto out = std::make_unique<Image>();
    out->header.stamp = color.header.stamp;
    out->header.frame_id = right_frame_;
    out->height = color.height;
    out->width = color.width;
    out->encoding = color.encoding;
    out->is_bigendian = color.is_bigendian;
    out->step = color.step;
    out->data.assign(color.data.size(), 0);

    std::vector<float> zbuf(w), xr(w), z(w);
    std::vector<int> src(w);                                          // left column copied in each right column
    for (int v = 0; v < h; ++v) {
      const auto * drow = reinterpret_cast<const float *>(&depth.data[v * depth.step]);
      for (int u = 0; u < w; ++u) {
        const float d = drow[u];
        z[u] = (std::isfinite(d) && d > 0.0f) ? d : FAR;             // beyond the far clip: no disparity
        xr[u] = static_cast<float>(u - (std::isinf(z[u]) ? 0.0 : fb / z[u]));
      }
      std::fill(zbuf.begin(), zbuf.end(), FAR);
      std::fill(src.begin(), src.end(), -1);
      auto splat = [&](int col, int u) {
          if (col >= 0 && col < w && (src[col] < 0 || z[u] < zbuf[col])) {
            zbuf[col] = z[u];
            src[col] = u;
          }
        };
      for (int u = 0; u < w; ++u) {
        splat(static_cast<int>(std::lround(xr[u])), u);
        if (u + 1 < w && same_surface(z[u], z[u + 1])) {
          // same surface: every column between the two projections, from the nearer source pixel
          const float a = std::min(xr[u], xr[u + 1]), b = std::max(xr[u], xr[u + 1]);
          for (int col = static_cast<int>(std::ceil(a)); col <= static_cast<int>(std::floor(b)); ++col) {
            splat(col, std::abs(col - xr[u]) <= std::abs(col - xr[u + 1]) ? u : u + 1);
          }
        }
      }
      // holes: background of the two neighbours of each run of empty columns
      for (int col = 0; col < w; ) {
        if (src[col] >= 0) {++col; continue;}
        int end = col;
        while (end < w && src[end] < 0) {++end;}
        const int left = col > 0 ? src[col - 1] : -1, right = end < w ? src[end] : -1;
        int fill = left;
        if (fill < 0 || (right >= 0 && z[right] > z[left])) {fill = right;}
        for (int c = col; c < end; ++c) {src[c] = fill;}
        col = end;
      }
      const uint8_t * in = &color.data[v * color.step];
      uint8_t * o = &out->data[v * out->step];
      for (int col = 0; col < w; ++col) {
        if (src[col] >= 0) {std::memcpy(o + col * bpp, in + src[col] * bpp, bpp);}
      }
    }

    CameraInfo info = *info_;
    info.header.stamp = color.header.stamp;
    info.header.frame_id = right_frame_;
    info.p[3] = -fb;                                                  // Tx = -fx * baseline (rectified pair)
    right_pub_->publish(std::move(out));
    info_pub_->publish(info);
  }

  bool same_surface(float a, float b) const
  {
    if (std::isinf(a) || std::isinf(b)) {return std::isinf(a) && std::isinf(b);}
    return std::abs(a - b) <= same_surface_ * std::min(a, b);
  }

  double baseline_ = 0.12, same_surface_ = 0.05;
  int64_t max_dt_ns_ = 5000000;
  std::string right_frame_;
  std::optional<CameraInfo> info_;
  std::deque<Image::ConstSharedPtr> colors_, depths_;
  rclcpp::Publisher<Image>::SharedPtr right_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr info_pub_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_, depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoSynth>());
  rclcpp::shutdown();
  return 0;
}
