#include <vector>
#include "colourT.hpp"

colourT weighted_average(std::vector<colourT> colours, std::vector<double> weights) {
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;

  for (int i = 0; i < colours.size(); i++) {
    red += colours[i].red * weights[i];
    green += colours[i].green * weights[i];
    blue += colours[i].blue * weights[i];
  }

  return colourT{red, green, blue};
}