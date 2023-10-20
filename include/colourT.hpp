#pragma once

#include <opencv2/core.hpp>

struct colourT {
  uint8_t red;
  uint8_t green;
  uint8_t blue;

  colourT(uint8_t red, uint8_t green, uint8_t blue): red {red}, green {green}, blue {blue} {};
  colourT(): red {0}, green {0}, blue {0} {};

  cv::Vec3b cv_colour() {
    return cv::Vec3b{blue, green, red};
  }
};

namespace colour {
    const colourT RED = colourT(255, 0, 0);;
    const colourT GREEN = colourT(0, 255, 0);;
    const colourT BLUE = colourT(0, 0, 255);;
};

colourT weighted_average(std::vector<colourT> colours, std::vector<double> weights);