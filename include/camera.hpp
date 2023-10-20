#pragma once

#include <Eigen/Dense>


class Camera {
  Eigen::Matrix4f c2w;
  double focal_length;
  double sensor_width;
  double sensor_height;
  int image_width;
  int image_height;

public:
  explicit Camera(Eigen::Matrix4f c2w, double focal_length, double sensor_width, double sensor_height, int image_width, int image_height) :
    c2w{c2w}, focal_length{focal_length}, sensor_width{sensor_width}, sensor_height{sensor_height}, image_width{image_width}, image_height{image_height} {}

  explicit Camera(Camera&& other) noexcept :
    c2w{std::move(other.c2w)}, focal_length{other.focal_length}, sensor_width{other.sensor_width}, sensor_height{other.sensor_height}, image_width{other.image_width}, image_height{other.image_height} {}

  Camera& operator=(Camera&& other) noexcept {
    c2w = std::move(other.c2w);
    focal_length = other.focal_length;
    sensor_width = other.sensor_width;
    sensor_height = other.sensor_height;
    image_width = other.image_width;
    image_height = other.image_height;
    return *this;
  }

  inline Eigen::Matrix4f get_c2w() const { return c2w; }
  inline Eigen::Matrix4f get_w2c() const { return c2w.inverse(); }
  inline double get_focal_length() const { return focal_length; }
  inline double get_sensor_width() const { return sensor_width; }
  inline double get_sensor_height() const { return sensor_height; }
  inline int get_image_width() const { return image_width; }
  inline int get_image_height() const { return image_height; }
  inline double min_x() const { return -sensor_width / 2; }
  inline double max_x() const { return sensor_width / 2; }
  inline double min_y() const { return -sensor_height / 2; }
  inline double max_y() const { return sensor_height / 2; }

  Eigen::Vector2i image_to_pixel(const Eigen::Vector2f& image_point) const {
    return Eigen::Vector2i{
      (int) ((image_point[0] - min_x()) * image_width / sensor_width),
      (int) ((image_point[1] - min_y()) * image_height / sensor_height)
    };
  }

  Eigen::Vector2f pixel_to_image(const Eigen::Vector2i& pixel) const {
    return Eigen::Vector2f{
      pixel[0] * sensor_width / image_width + min_x(),
      pixel[1] * sensor_height / image_height + min_y()
    };
  }
};