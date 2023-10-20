#include "image.hpp"
#include <cassert>
#include <fstream>
#include <iostream>

int image::index(int x, int y) {
  assert (x <= max_x());
  assert (x >= min_x());
  assert (y <= max_y());
  assert (y >= min_y());

  return (y + height/2)*width + x + width / 2;
}

void image::set(int x, int y, colourT colour) {
  data[index(x, y)] = colour;
}

void image::set(int x, int y, colourT colour, double z_value) {
  int i = index(x, y);
  if (z_value < z_buffer[i]) {
    set(x, y, colour);
    z_buffer[i] = z_value;
  }
}

void image::write(std::string name) {
  assert(name.substr(name.size() - 4) == ".ppm");

  std::ofstream file_stream;
  file_stream.open(name, std::ofstream::binary);

  file_stream << "P3" << std::endl;
  file_stream << height << " " << width << std::endl;
  file_stream << 255 << std::endl;

  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      colourT pixel = data[(height - i - 1)*width + j];
      file_stream << int(pixel.red) << " ";
      file_stream << int(pixel.green) << " ";
      file_stream << int(pixel.blue) << " ";
    }
    file_stream << std::endl;
  }

  file_stream.close();
}
