#pragma once
#include <vector>
#include <string>
#include "colourT.hpp"

struct image {
private:
  int index(int x, int y);
public:
  std::vector<colourT> data;
  std::vector<double> z_buffer;
  int width;
  int height;

  image(int width, int height): data(width*height, colourT()), z_buffer(width*height, std::numeric_limits<double>::max()), width{width}, height{height} {};

  void set(int x, int y, colourT colour);
  void set(int x, int y, colourT colour, double z_value);
  void write(std::string name);
  int max_x() { return width / 2 - 1; }
  int max_y() { return height / 2 - 1; }
  int min_x() { return - width / 2; }
  int min_y() { return - height / 2; }
};
