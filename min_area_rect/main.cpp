#include<iostream>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp> // image load, visualize, and save

#include "graham_scan.h"

// x, y
int directions[8][2] = {
  {-1,  1},
  { 0,  1},
  { 1,  1},
  { 1,  0},
  { 1, -1},
  { 0, -1},
  {-1, -1},
  {-1,  0}
};

bool in_image(const Point p, const int width, const int height) {
  if (p.x >= 0 && p.y >= 0 && p.x < width && p.y < height) return true;
  return false;
}

bool is_start(const Point & curr, const Point & start) {
  if (curr.x == start.x && curr.y == start.y) return true;
  return false;
}

bool is_border(const std::vector<std::vector<int>> & image, int x, int y) {
    int height = image.size();
    int width = image[0].size();
    
    const int dx[4] = {-1, 1,  0, 0};
    const int dy[4] = { 0, 0, -1, 1};

    for (int i = 0; i < 4; i++) {
      int nx = x + dx[i];
      int ny = y + dy[i];
      if (ny < 0 || nx >= width || ny < 0 || ny >= height) return true;
      if (image[ny][nx] == 0) return true;
    }
    return false;
}

Point raster_scan(const std::vector<std::vector<int>> & image, std::vector<std::vector<char>> & visited, const Point & curr) {
  int height = image.size();
  int width = image[0].size();
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (is_border(image, x, y) && image[y][x] == 1 && visited[y][x] != 1) {
        return { x, y };
      }
    }
  }
  return { -1, -1 };
}

bool is_invalid_point(const Point & p) {
  if (p.x == -1 && p.y == -1) return true;
  return false;
}

std::vector<std::vector<Point>> contour_tracing(const std::vector<std::vector<int>> & image, std::vector<std::vector<char>> & visisted) {
  // raster scan
  const int width = image[0].size();
  const int height = image.size();

  std::vector<std::vector<Point>> contours;
  Point curr = { 0, 0 };

  while (true) {
      curr = raster_scan(image, visisted, curr);
      bool not_found = is_invalid_point(curr);
      // no pixel found
      if (not_found) break;
    
      // get contour
      std::vector<Point> contour;
      int dir = 7;
      Point start = curr;
      contour.push_back(start);

      do {
        visisted[curr.y][curr.x] = 1;
        Point next;
        bool has_route = false;
        for (int i = 0; i < 8; i++) {
          int d = (dir + i + 6) % 8;
          next.x = curr.x + directions[d][0];
          next.y = curr.y + directions[d][1];
          if (in_image(next, width, height) && image[next.y][next.x] == 1 && visisted[next.y][next.x] != 1) {
            contour.push_back(next);
            dir = d;
            curr = next;
            has_route = true;
            break;
          }
        }

        // foreground but inside the outer contour.
        if (!has_route) break;

      } while (!is_start(curr, start));

    if (contour.size() > 1) contours.push_back(contour);
  }
  
  return contours;
}

std::vector<std::vector<int>> load_image_as_binary(const char * filename) {
  cv::Mat img = cv::imread(filename);
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  std::vector<std::vector<int>> image (gray.rows, std::vector<int>(gray.cols, 0));
  for (int y = 0; y < gray.rows; y++) {
    for (int x = 0; x < gray.cols; x++) {
      int intensity = gray.at<unsigned char>(y, x) > 0 ? 1 : 0;
      image[y][x] = intensity;
    }
  }
  return image;
}

void visualize(std::vector<std::vector<int>> & image, std::vector<FloatPoint> & box) {
  int height = image.size();
  int width = image[0].size();

  cv::Mat vis(height, width, CV_8UC1);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      vis.at<uchar>(y, x) = image[y][x] ? 255 : 0;
    }
  }

  cv::Mat color;
  cv::cvtColor(vis, color, cv::COLOR_GRAY2BGR);

  /* visualize contour */
  auto c = cv::Scalar(0, 255, 0);
  cv::line(color, cv::Point((int)box[0].x, (int)box[0].y), cv::Point((int)box[1].x, (int)box[1].y), c, 2);
  cv::line(color, cv::Point((int)box[0].x, (int)box[0].y), cv::Point((int)box[3].x, (int)box[3].y), c, 2);
  cv::line(color, cv::Point((int)box[1].x, (int)box[1].y), cv::Point((int)box[2].x, (int)box[2].y), c, 2);
  cv::line(color, cv::Point((int)box[2].x, (int)box[2].y), cv::Point((int)box[3].x, (int)box[3].y), c, 2);
  cv::imwrite("./images/rect.png", color);
}

std::vector<FloatPoint> rotate_angle(std::vector<Point> points, float angle) {
  /**
  C++ version of rotating vectors
  rotation = np.array([
          [math.cos(angle), -math.sin(angle)],
          [math.sin(angle),  math.cos(angle)]
  ])
  rotated = np.dot(points, rotation.T)
  **/
  std::vector<FloatPoint> rotated(points.size());
  for (int i = 0; i < points.size(); i++) {
    rotated[i].x = std::cos(angle) * points[i].x - std::sin(angle) * points[i].y; 
    rotated[i].y = std::sin(angle) * points[i].x + std::cos(angle) * points[i].y; 
  }
  return rotated;
}

std::vector<FloatPoint> rotate_angle_reverse(std::vector<FloatPoint> points, float angle) {
  /**
  C++ version of rotating vectors
  rotation = np.array([
          [math.cos(angle), -math.sin(angle)],
          [math.sin(angle),  math.cos(angle)]
  ])
  rotated = np.dot(points, rotation.T)
  **/
  std::vector<FloatPoint> rotated(points.size());
  for (int i = 0; i < points.size(); i++) {
    rotated[i].x = std::cos(angle) * points[i].x + std::sin(angle) * points[i].y; 
    rotated[i].y = - std::sin(angle) * points[i].x + std::cos(angle) * points[i].y; 
  }
  return rotated;
}

std::tuple<float, float> minmax_x(std::vector<FloatPoint> rotated) {
  float min_val = std::numeric_limits<float>::max();
  float max_val = std::numeric_limits<float>::min();
  
  for (auto & p : rotated) {
    if (p.x < min_val) min_val = p.x;
    if (p.x > max_val) max_val = p.x;
  }

  return { min_val, max_val };
}

std::tuple<float, float> minmax_y(std::vector<FloatPoint> rotated) {
  float min_val = std::numeric_limits<float>::max();
  float max_val = std::numeric_limits<float>::min();
  
  for (auto & p : rotated) {
    if (p.y < min_val) min_val = p.y;
    if (p.y > max_val) max_val = p.y;
  }

  return { min_val, max_val };
}

std::vector<FloatPoint> min_area_rect(std::vector<Point> & hull) {
  float min_area = std::numeric_limits<float>::max();
  std::vector<FloatPoint> best_rect;

  for (int i = 0; i < hull.size(); i++) {
    Point p1 = hull[i];
    Point p2 = hull[(i + 1) % hull.size()];

    float angle = - std::atan2(p2.y - p1.y, p2.x - p1.x);
    std::vector<FloatPoint> rotated = rotate_angle(hull, angle);

    // todo: obtain min_x, min_y, max_x, max_y
    auto [min_x, max_x] = minmax_x(rotated);
    auto [min_y, max_y] = minmax_y(rotated);
    float area = (max_x - min_x) * (max_y - min_y);
    
    if (area < min_area) {
      min_area = area;
      std::vector<FloatPoint> rect = {
        { min_x, min_y },
        { max_x, min_y },
        { max_x, max_y },
        { min_x, max_y },
      };
      best_rect = rotate_angle_reverse(rect, angle);
    }
  }
  return best_rect;
}


int main()
{
  std::vector<std::vector<int>> image = load_image_as_binary("images/test2.png");
  std::vector<std::vector<char>> visited(image.size(), std::vector<char>(image[0].size(), 0));
  std::vector<std::vector<Point>> contours = contour_tracing(image, visited);

  std::vector<Point> hull = graham_scan(contours[0]);
  std::vector<FloatPoint> bounding_box = min_area_rect(hull);
  visualize(image, bounding_box);

  return 0;
}
