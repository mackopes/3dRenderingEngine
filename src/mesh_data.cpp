#include "mesh_data.hpp"

colourT get_colour_from_texture(const cv::Mat texture, int x, int y){
    colourT colour;
    int flipped_y = texture.rows - y;
    cv::Vec3b intensity = texture.at<cv::Vec3b>(flipped_y, x);
    colour.red = intensity.val[2];
    colour.green = intensity.val[1];
    colour.blue = intensity.val[0];
    return colour;
}

bool Mesh::parse_line(const std::string& line) {
  if (line.size() == 0) {
    // empty line
    return true;
  }

  if (line[0] == '/' && line[1] == '/') {
    // just a comment
    return true;
  }

  if (line[0] == '#') {
    // just a comment
    return true;
  }

  std::stringstream line_stream(line);
  std::string element_type;
  line_stream >> element_type;

  // TODO: consider moving streams
  if (element_type == "v") {
    return parse_vertex(line_stream);
  } else if (element_type == "f") {
    return parse_face(line_stream);
  } else if (element_type == "vt") {
    return parse_vertex_texture(line_stream);
  } else if (element_type == "vn") {
    return parse_vertex_normal(line_stream);
  }

  return false;
}

bool Mesh::parse_vertex(std::stringstream& line_stream){
  double x, y, z;
  line_stream >> x >> y >> z;
  vertices.push_back(Vector3f(x, y , z));
  return true;
}

bool Mesh::parse_vertex_texture(std::stringstream& line_stream) {
  double u, v;
  line_stream >> u;

  if (! (line_stream >> v)) {
    u = 0;
  }

  vertices_texture.push_back(Vector2f(u, v));
  return true;
}

bool Mesh::parse_vertex_normal(std::stringstream& line_stream) {
  double x, y, z;
  line_stream >> x >> y >> z;

  vertices_normal.push_back(Vector3f(x, y, z));
  return true;
}

bool Mesh::parse_face(std::stringstream& line_stream){
  FaceVectorData current_vertex;
  std::vector<FaceVectorData> current_face;

  while (line_stream >> current_vertex.vertex_index) {

    if (line_stream.peek() == '/') { // there's texture or normal data present
      line_stream.get(); // throw away /
      if (line_stream.peek() != '/') { // texture data present
        line_stream >> current_vertex.vertex_texture_index;
      } else {
        current_vertex.vertex_texture_index = 0;
      }

      if (line_stream.peek() == '/') {
        line_stream.get();
        line_stream >> current_vertex.vertex_normal_index;
      } else {
        current_vertex.vertex_normal_index = 0;
      }
    } else {
      current_vertex.vertex_texture_index = 0;
      current_vertex.vertex_normal_index = 0;
    }

    if (current_vertex.vertex_index > vertices.size()) {
      // TODO: raise error instead
      return false;
    }

    if (current_vertex.vertex_normal_index > vertices_normal.size()) {
      return false;
    }

    if (current_vertex.vertex_texture_index > vertices_texture.size()) {
      return false;
    }

    current_vertex.vertex_index--;
    current_vertex.vertex_texture_index--;
    current_vertex.vertex_normal_index--;

    current_face.push_back(current_vertex);
  }

  if (current_face.size() != 3) {
    throw std::runtime_error("face has to be a triangle");
  }

  faces.push_back(current_face);
  return true;
}

std::vector<Vector3f> Mesh::triangle(int idx) {
  std::vector<Vector3f> t;

  for (auto f: faces[idx]) {
    t.push_back(vertices[f.vertex_index]);
  }

  return t;
}

PolygonData Mesh::polygon_data(int idx) {
  PolygonData p;
  for (auto f: faces[idx]) {
    p.vertices.push_back(vertices[f.vertex_index]);
  }

  auto a = p.vertices[1] - p.vertices[0];
  auto b = p.vertices[2] - p.vertices[0];
  auto backup_normal = a.cross(b).normalized();

  for (auto f: faces[idx]) {
    if (texture.empty() || f.vertex_texture_index == -1) {
      p.diffuse_colours.push_back(colour::RED);
    } else {
      Vector2f texture_coord = vertices_texture[f.vertex_texture_index];
      int x = int(texture_coord[0] * texture.rows);
      int y = int(texture_coord[1] * texture.cols);
      p.diffuse_colours.push_back(get_colour_from_texture(texture, x,y));
    }

    if (f.vertex_normal_index == -1) {
      p.normals.push_back(backup_normal);
    } else {
      p.normals.push_back(vertices_normal[f.vertex_normal_index]);
    }
  }

  return p;
}

bool Mesh::attach_texture(std::string texture_path) {
  cv::Mat texture = cv::imread(texture_path, cv::IMREAD_COLOR);
  if (texture.empty()) {
    return false;
  }

  this->texture = texture;

  return true;
}