// camera_noise: ZED 2i-like noise on the ideal Gazebo images (color_in/depth_in -> color_out/depth_out).
//
// Depth defaults are for VGA (672x376), from:
//   [A] Abdelsalam et al. 2024, "Depth accuracy analysis of the ZED 2i Stereo Camera in an indoor
//       Environment", RAS 179:104753 (ZED 2i, ULTRA mode)
//   [O] Ortiz et al. 2018, "Depth Data Error Modeling of the ZED 3D Vision Sensor from Stereolabs",
//       ELCVIA 17(1) (original ZED)
// HD720 (1280x720) values, for reference: max_range 14 [A], spatial_a 0.0184, spatial_b 0.2106 [O]
// ([A]'s HD720 fit is poorly constrained, var(b) = 1.46 > b), jitter_start 8, jitter_gain 0.5,
// spatial_cell 46, blob_cell 23, edge_width 2 (pixel sizes scaled by 1280/672).
// Headers (stamp, frame_id) are kept, so camera_info and RTAB-Map sync are unaffected.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using sensor_msgs::msg::Image;

namespace
{

constexpr int kDofLevels = 8;
// per-frame noise is a random slice of precomputed pools: Student-t and Gaussian draws for ~1M
// pixels per frame would take most of the frame budget; a shifted slice is still white noise
constexpr size_t kPoolSize = size_t{1} << 20;   // >= pixels of one HD720 frame (921600)
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// linear interpolation from a coarse grid (one node every `cell` pixels) to n pixels: node i0 and
// i0 + 1 with weights w0, w1, normalised to unit norm so unit-variance nodes give unit variance
struct Interp
{
  std::vector<int> i0;
  std::vector<float> w0, w1;
  int nodes;

  Interp(int n, int cell)
  : i0(n), w0(n), w1(n), nodes((n - 1) / cell + 2)
  {
    for (int i = 0; i < n; ++i) {
      const double pos = static_cast<double>(i) / cell;
      const int k = static_cast<int>(std::floor(pos));
      const double f = pos - k, g = 1 - f, norm = std::hypot(g, f);
      i0[i] = k;
      w0[i] = static_cast<float>(g / norm);
      w1[i] = static_cast<float>(f / norm);
    }
  }
};

// unit-variance Gaussian random field, correlated over ~cell pixels; the coarse grid evolves as an
// AR(1) process with time constant tau [s], tau <= 0 keeps it fixed
class SmoothField
{
public:
  SmoothField(std::mt19937 & rng, int h, int w, int cell, double tau)
  : rng_(rng), h_(h), w_(w), tau_(tau), y_(h, cell), x_(w, cell),
    grid_(static_cast<size_t>(y_.nodes) * x_.nodes), rows_(static_cast<size_t>(h) * x_.nodes),
    out_(static_cast<size_t>(h) * w)
  {
    for (auto & g : grid_) {
      g = normal_(rng_);
    }
  }

  bool matches(int h, int w) const {return h == h_ && w == w_;}

  const std::vector<float> & step(double dt)
  {
    if (tau_ > 0 && dt > 0) {
      const float a = static_cast<float>(std::exp(-dt / tau_)), b = std::sqrt(1 - a * a);
      for (auto & g : grid_) {
        g = a * g + b * normal_(rng_);
      }
    }
    // separable interpolation: rows first (h x grid width), then columns
    const int gw = x_.nodes;
    for (int r = 0; r < h_; ++r) {
      const float * g0 = &grid_[static_cast<size_t>(y_.i0[r]) * gw];
      const float * g1 = g0 + gw;
      float * row = &rows_[static_cast<size_t>(r) * gw];
      for (int c = 0; c < gw; ++c) {
        row[c] = g0[c] * y_.w0[r] + g1[c] * y_.w1[r];
      }
    }
    for (int r = 0; r < h_; ++r) {
      const float * row = &rows_[static_cast<size_t>(r) * gw];
      float * o = &out_[static_cast<size_t>(r) * w_];
      for (int c = 0; c < w_; ++c) {
        o[c] = row[x_.i0[c]] * x_.w0[c] + row[x_.i0[c] + 1] * x_.w1[c];
      }
    }
    return out_;
  }

private:
  std::mt19937 & rng_;
  std::normal_distribution<float> normal_;
  int h_, w_;
  double tau_;
  Interp y_, x_;
  std::vector<float> grid_, rows_, out_;
};

// depth discontinuities (|dz| > threshold * z between neighbours), grown by `width` pixels
void depth_edges(
  const std::vector<float> & z, int h, int w, float threshold, int width, std::vector<uint8_t> & m)
{
  m.assign(z.size(), 0);
  auto check = [&](size_t a, size_t b) {
      const float za = z[a], zb = z[b];
      if (std::isfinite(za) && std::isfinite(zb) &&
        std::fabs(za - zb) > threshold * std::min(za, zb))
      {
        m[a] = m[b] = 1;
      }
    };
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      const size_t i = static_cast<size_t>(r) * w + c;
      if (c + 1 < w) {check(i, i + 1);}
      if (r + 1 < h) {check(i, i + w);}
    }
  }
  std::vector<uint8_t> g;
  for (int k = 0; k < width; ++k) {
    g = m;
    for (int r = 0; r < h; ++r) {
      for (int c = 0; c < w; ++c) {
        const size_t i = static_cast<size_t>(r) * w + c;
        if (!m[i]) {continue;}
        if (r > 0) {g[i - w] = 1;}
        if (r + 1 < h) {g[i + w] = 1;}
        if (c > 0) {g[i - 1] = 1;}
        if (c + 1 < w) {g[i + 1] = 1;}
      }
    }
    m.swap(g);
  }
}

int color_channels(const std::string & encoding)
{
  if (encoding == "mono8") {return 1;}
  if (encoding == "rgb8" || encoding == "bgr8") {return 3;}
  if (encoding == "rgba8" || encoding == "bgra8") {return 4;}
  return 0;
}

}  // namespace

class CameraNoise : public rclcpp::Node
{
public:
  explicit CameraNoise(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("camera_noise", options)
  {
    // RGB: read noise + shot noise, sigma^2 = read_sigma^2 + shot_gain * I (in 8-bit levels);
    // not covered by the papers, plausible values for a small CMOS sensor
    color_enabled_ = declare_parameter<bool>("color.enabled", true);
    const double read_sigma = declare_parameter<double>("color.read_sigma", 2.0);
    const double shot_gain = declare_parameter<double>("color.shot_gain", 0.1);

    depth_enabled_ = declare_parameter<bool>("depth.enabled", true);
    // valid range: 0.3 m datasheet minimum, 7 m max at VGA [A]
    min_range_ = declare_parameter<double>("depth.min_range", 0.3);
    max_range_ = declare_parameter<double>("depth.max_range", 7.0);
    // spatial noise, sigma_s(Z) = a * exp(b * Z) [m]: VGA RMSE fit of [A] (a = 1.7841 cm)
    spatial_a_ = declare_parameter<double>("depth.spatial_a", 0.017841);
    spatial_b_ = declare_parameter<double>("depth.spatial_b", 0.25436);
    // applied as a smooth field: most of the error is static, not per-frame [O]; a slow drift
    // (time constant spatial_tau [s], <= 0 = fixed) keeps the frame-to-frame std near [O]'s ~8 mm at 4 m
    spatial_cell_ = declare_parameter<int>("depth.spatial_cell", 24);
    spatial_tau_ = declare_parameter<double>("depth.spatial_tau", 30.0);
    // per-frame noise, ~8 mm at 4 m [O], growing with Z^2 as in stereo triangulation, plus
    // extra jitter beyond 4 m at VGA [A]
    temporal_sigma_4m_ = declare_parameter<double>("depth.temporal_sigma_4m", 0.008);
    jitter_start_ = declare_parameter<double>("depth.jitter_start", 4.0);
    jitter_gain_ = declare_parameter<double>("depth.jitter_gain", 1.0);
    // Student-t tails, light near and heavy far [A]; degrees of freedom interpolated over Z
    const double dof_near = declare_parameter<double>("depth.dof_near", 10.0);
    const double dof_far = declare_parameter<double>("depth.dof_far", 3.0);
    // systematic error bias_quadratic * Z^2; sign unknown from [A] (absolute errors only), off
    bias_quadratic_ = declare_parameter<double>("depth.bias_quadratic", 0.0);
    // holes: random pixels + clustered blobs (~2% invalid on an easy scene [O]) + depth edges
    const double dropout = declare_parameter<double>("depth.dropout", 0.005);
    const double blob_fraction = declare_parameter<double>("depth.blob_fraction", 0.02);
    blob_cell_ = declare_parameter<int>("depth.blob_cell", 12);
    edge_threshold_ = declare_parameter<double>("depth.edge_threshold", 0.05);
    edge_width_ = declare_parameter<int>("depth.edge_width", 1);

    // 'color', 'depth' or 'both': the launch runs one process per stream
    const auto stream = declare_parameter<std::string>("stream", "both");
    const int seed = declare_parameter<int>("seed", -1);
    rng_.seed(seed < 0 ? std::random_device{}() : static_cast<uint32_t>(seed));

    // Gaussian pool (colour noise, random dropout) and its quantiles for the hole thresholds
    std::normal_distribution<float> normal;
    normal_pool_.resize(4 * kPoolSize);
    for (auto & v : normal_pool_) {
      v = normal(rng_);
    }
    auto quantile = [this](double frac) {
        if (frac <= 0) {return std::numeric_limits<float>::infinity();}
        std::vector<float> s(normal_pool_.begin(), normal_pool_.begin() + 200000);
        const auto k = static_cast<size_t>((1 - frac) * (s.size() - 1));
        std::nth_element(s.begin(), s.begin() + k, s.end());
        return s[k];
      };
    dropout_threshold_ = quantile(dropout);
    blob_threshold_ = quantile(blob_fraction);

    if (stream != "color") {
      // unit-variance Student-t pools at kDofLevels degrees of freedom between dof_far and
      // dof_near (t variance needs dof > 2), level 0 = dof_far
      t_pool_.resize(kDofLevels * kPoolSize);
      for (int l = 0; l < kDofLevels; ++l) {
        const double d = std::max(dof_far, 2.1) +
          (std::max(dof_near, 2.1) - std::max(dof_far, 2.1)) * l / (kDofLevels - 1);
        std::student_t_distribution<double> t(d);
        const double scale = std::sqrt((d - 2) / d);
        for (size_t i = 0; i < kPoolSize; ++i) {
          t_pool_[l * kPoolSize + i] = static_cast<float>(t(rng_) * scale);
        }
      }
    }
    for (int i = 0; i < 256; ++i) {
      color_sigma_[i] = static_cast<float>(std::sqrt(read_sigma * read_sigma + shot_gain * i));
    }

    // reliable, like the gz bridge: best effort drops most ~1-4 MB images (fragmented in DDS)
    if (stream != "depth") {
      color_pub_ = create_publisher<Image>("color_out", 5);
      color_sub_ = create_subscription<Image>(
        "color_in", 5, [this](Image::ConstSharedPtr msg) {on_color(*msg);});
    }
    if (stream != "color") {
      depth_pub_ = create_publisher<Image>("depth_out", 5);
      depth_sub_ = create_subscription<Image>(
        "depth_in", 5, [this](Image::ConstSharedPtr msg) {on_depth(*msg);});
    }
  }

private:
  size_t pool_start(size_t pool_size, size_t n)
  {
    return std::uniform_int_distribution<size_t>(0, pool_size - n)(rng_);
  }

  bool passthrough(const Image & msg, bool enabled, bool supported)
  {
    if (supported && !msg.is_bigendian) {
      return !enabled;
    }
    if (warned_.insert(msg.encoding).second) {
      RCLCPP_WARN(
        get_logger(), "unsupported encoding %s (big endian: %d), image passed through unchanged",
        msg.encoding.c_str(), msg.is_bigendian);
    }
    return true;
  }

  void on_color(const Image & msg)
  {
    const int ch = color_channels(msg.encoding);
    if (passthrough(msg, color_enabled_, ch > 0)) {
      color_pub_->publish(msg);
      return;
    }
    const size_t row_len = static_cast<size_t>(msg.width) * ch;
    const size_t n = row_len * msg.height;
    auto out = std::make_unique<Image>();
    out->header = msg.header;
    out->encoding = msg.encoding;
    out->height = msg.height;
    out->width = msg.width;
    out->step = static_cast<uint32_t>(row_len);
    out->data.resize(n);

    const float * noise = n <= normal_pool_.size() ?
      &normal_pool_[pool_start(normal_pool_.size(), n)] : nullptr;
    std::normal_distribution<float> normal;
    for (uint32_t r = 0; r < msg.height; ++r) {
      const uint8_t * in = &msg.data[static_cast<size_t>(r) * msg.step];
      uint8_t * o = &out->data[static_cast<size_t>(r) * row_len];
      for (size_t c = 0; c < row_len; ++c) {
        if (ch == 4 && c % 4 == 3) {   // leave alpha untouched
          o[c] = in[c];
          continue;
        }
        const float e = noise ? noise[r * row_len + c] : normal(rng_);
        const float v = std::nearbyint(in[c] + color_sigma_[in[c]] * e);
        o[c] = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
      }
    }
    color_pub_->publish(std::move(out));
  }

  void on_depth(const Image & msg)
  {
    const bool mm = msg.encoding == "16UC1";
    if (passthrough(msg, depth_enabled_, mm || msg.encoding == "32FC1")) {
      depth_pub_->publish(msg);
      return;
    }
    const int h = static_cast<int>(msg.height), w = static_cast<int>(msg.width);
    const size_t n = static_cast<size_t>(h) * w;
    // depth in metres, NaN outside the valid range (gz gives +-inf outside its clip range)
    z_.resize(n);
    for (int r = 0; r < h; ++r) {
      const uint8_t * row = &msg.data[static_cast<size_t>(r) * msg.step];
      for (int c = 0; c < w; ++c) {
        float z;
        if (mm) {
          uint16_t v;
          std::memcpy(&v, row + 2 * c, 2);
          z = v > 0 ? v * 0.001f : kNaN;
        } else {
          std::memcpy(&z, row + 4 * c, 4);
        }
        z_[static_cast<size_t>(r) * w + c] = (std::isfinite(z) && z >= min_range_ &&
          z <= max_range_) ? z : kNaN;
      }
    }

    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const double dt = last_stamp_ < 0 ? 0.0 : stamp - last_stamp_;
    last_stamp_ = stamp;
    if (!spatial_ || !spatial_->matches(h, w)) {
      spatial_ = std::make_unique<SmoothField>(rng_, h, w, spatial_cell_, spatial_tau_);
      blobs_ = std::make_unique<SmoothField>(rng_, h, w, blob_cell_, spatial_tau_);
    }
    const auto & spatial = spatial_->step(dt);
    const auto & blobs = blobs_->step(dt);
    if (edge_threshold_ > 0) {
      depth_edges(z_, h, w, static_cast<float>(edge_threshold_), edge_width_, edges_);
    } else {
      edges_.assign(n, 0);
    }

    // pools: Student-t (wraps if the image is larger than the pool) and Gaussian for the dropout
    const size_t t_start = std::uniform_int_distribution<size_t>(0, kPoolSize - 1)(rng_);
    const float * drop = n <= normal_pool_.size() ?
      &normal_pool_[pool_start(normal_pool_.size(), n)] : nullptr;
    const float range = static_cast<float>(std::max(max_range_ - 1.0, 1e-6));
    std::vector<float> noisy(n);
    for (size_t i = 0; i < n; ++i) {
      const float z = z_[i];
      if (!std::isfinite(z) || edges_[i] || blobs[i] > blob_threshold_ ||
        (drop && drop[i] > dropout_threshold_))
      {
        noisy[i] = kNaN;
        continue;
      }
      const float sigma_s = static_cast<float>(spatial_a_ * std::exp(spatial_b_ * z));
      const float q = z / 4;
      const float sigma_t = static_cast<float>(temporal_sigma_4m_ * q * q *
        (1 + jitter_gain_ * std::max(z - jitter_start_, 0.0)));
      // dof falls linearly from dof_near at 1 m to dof_far at max_range: nearest pool level
      const float frac = std::clamp((z - 1) / range, 0.0f, 1.0f);
      const int level = static_cast<int>(std::lround((1 - frac) * (kDofLevels - 1)));
      size_t k = t_start + i % kPoolSize;
      if (k >= kPoolSize) {k -= kPoolSize;}
      const float t = t_pool_[level * kPoolSize + k];
      const float v = z + static_cast<float>(bias_quadratic_) * z * z + sigma_s * spatial[i] +
        sigma_t * t;
      noisy[i] = v < min_range_ ? kNaN : v;
    }

    auto out = std::make_unique<Image>();
    out->header = msg.header;
    out->encoding = msg.encoding;
    out->height = msg.height;
    out->width = msg.width;
    if (mm) {
      out->step = 2 * msg.width;
      out->data.resize(2 * n);
      for (size_t i = 0; i < n; ++i) {
        const uint16_t v = std::isfinite(noisy[i]) ?
          static_cast<uint16_t>(std::clamp(std::nearbyint(noisy[i] * 1000.0f), 0.0f, 65535.0f)) : 0;
        std::memcpy(&out->data[2 * i], &v, 2);
      }
    } else {
      out->step = 4 * msg.width;
      out->data.resize(4 * n);
      std::memcpy(out->data.data(), noisy.data(), 4 * n);
    }
    depth_pub_->publish(std::move(out));
  }

  bool color_enabled_, depth_enabled_;
  double min_range_, max_range_, spatial_a_, spatial_b_, spatial_tau_, temporal_sigma_4m_;
  double jitter_start_, jitter_gain_, bias_quadratic_, edge_threshold_;
  int spatial_cell_, blob_cell_, edge_width_;
  float dropout_threshold_, blob_threshold_;
  float color_sigma_[256];

  std::mt19937 rng_;
  std::vector<float> normal_pool_, t_pool_;
  std::unique_ptr<SmoothField> spatial_, blobs_;
  std::vector<float> z_;
  std::vector<uint8_t> edges_;
  double last_stamp_ = -1;
  std::set<std::string> warned_;

  rclcpp::Publisher<Image>::SharedPtr color_pub_, depth_pub_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_, depth_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNoise>());
  rclcpp::shutdown();
  return 0;
}
