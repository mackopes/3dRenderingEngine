#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include <chrono>

#include "mesh_data.hpp"
#include "render.hpp"
#include "camera.hpp"
#include "image.hpp"
#include "math/geometry.hpp"
#include "SDL.h"


using Eigen::Vector2d;
using Eigen::Matrix3d;


Mesh parse_object_file(const std::string& name) {
  std::ifstream file;
  file.open(name);

  if(!file.is_open()) {
    perror("Error open");
    exit(EXIT_FAILURE);
  }

  Mesh mesh_data;
  std::string line;
  int line_counter = 0;
  while(getline(file, line)) {
    if (!mesh_data.parse_line(line)) {
      std::cout << "Cannot parse the .obj file" << std::endl;
      std::cout << "Line("  "): " << line << std::endl;
      exit(0);
    }
    line_counter++;
  }

  return mesh_data;
}

void fillTexture(SDL_Texture * texture, cv::Mat const &mat) {
    unsigned char * texture_data = NULL;
    int texture_pitch = 0;

    SDL_LockTexture(texture, 0, (void **)&texture_data, &texture_pitch);
    memcpy(texture_data, (void *)mat.data, mat.cols * mat.rows * 3);
    SDL_UnlockTexture(texture);
}


void loop(SDL_Renderer * renderer, Mesh &mesh_data, int render_width, int render_height) {
  int time_to_360 = 10; // seconds
  double angular_velocity = 2 * M_PI / time_to_360;
  // cv::namedWindow("Render", cv::WINDOW_AUTOSIZE);

  Eigen::Matrix4f start_camera_matrix = Eigen::Matrix4f::Identity();
  start_camera_matrix = math::translate(Eigen::Vector3f{0, 0, 10}) * start_camera_matrix;
  auto start = std::chrono::high_resolution_clock::now();

  SDL_Texture * texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_BGR24,
        SDL_TEXTUREACCESS_STREAMING,
        render_width, render_height
    );

  bool quit = false;
  SDL_Event e;

  while(!quit) {
    while (SDL_PollEvent(&e)) {
      switch (e.type) {
        case SDL_QUIT:
          quit = true;
          break;

        case SDL_KEYDOWN:
          switch (e.key.keysym.sym) {
            case SDLK_ESCAPE:
              quit = true;
              break;
          break;
        }
      }
    }

    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start);

    double rotation_angle = angular_velocity * elapsed.count() / 1000;
    rotation_angle = std::fmod(rotation_angle, 2 * M_PI);

    auto rendering_start = std::chrono::high_resolution_clock::now();

    Eigen::Matrix4f camera_matrix = math::rot_y(rotation_angle) * start_camera_matrix;
    Camera camera {
      camera_matrix,
      0.050,
      0.036,
      0.024,
      render_width,
      render_height
    };

    cv::Mat img = render_mesh(camera, mesh_data);

    auto rendering_end = std::chrono::high_resolution_clock::now();
    auto rendering_time = std::chrono::duration_cast<std::chrono::milliseconds>(rendering_end - rendering_start);

    double fps = static_cast<double>(1000 / rendering_time.count());

    // write fps to image
    std::stringstream ss;
    ss << fps;
    std::string fps_string = ss.str();
    cv::putText(img, fps_string, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    fillTexture(texture, img);
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);

    // SDL_Delay(10);

    // imshow("Render", img);
    // cv::waitKey(1); // Wait for a keystroke in the window
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "please supply the object file" << std::endl;
    exit(0);
  }

  srand (time (NULL));

  std::string object_file = argv[1];
  Mesh mesh_data = parse_object_file(object_file);

  if (argc >= 3) {
    std::string diffuse_texture = argv[2];
    mesh_data.attach_texture(diffuse_texture);
  }

  if (SDL_Init(SDL_INIT_EVERYTHING) != 0){
      std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
      return 1;
  }

  SDL_Window * window = nullptr;
  SDL_Renderer * renderer = nullptr;

  int render_width = 1500;
  int render_height = 1000;

  if (SDL_CreateWindowAndRenderer(render_width, render_height, 0, &window, &renderer) < 0) {
      std::cerr << "Error creating window or renderer: " << SDL_GetError() << std::endl;
      SDL_Quit();
      return 1;
  }



  loop(renderer, mesh_data, render_width, render_height);
}