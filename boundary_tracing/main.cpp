#include<iostream>
#include <vector>

#include <opencv2/opencv.hpp> // image load, visualize, and save

struct Point {
  int x;
  int y;
};

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

bool is_connceted_start(const Point & next, const Point & start) {
  return next.x == start.x && next.y == start.y;
}

std::vector<std::vector<Point>> contour_tracing(const std::vector<std::vector<int>> & image, std::vector<std::vector<char>> & visisted) {
  // raster scan
  const int width = image[0].size();
  const int height = image.size();

  std::vector<std::vector<Point>> contours;
  Point curr = { 0, 0 };

  bool has_route = false;

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
        //visisted[curr.y][curr.x] = 1;
        Point next;
        has_route = false;
        for (int i = 0; i < 8; i++) {
          int d = (dir + i + 6) % 8;
          next.x = curr.x + directions[d][0];
          next.y = curr.y + directions[d][1];
          if (in_image(next, width, height) && image[next.y][next.x] == 1 && visisted[next.y][next.x] != 1 && is_border(image, next.x, next.y)) {
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

    for (const auto & c : contour) visisted[c.y][c.x] = 1;
    if (contour.size() > 1 && has_route) contours.push_back(contour);

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

void visualize(std::vector<std::vector<int>> & image, std::vector<std::vector<Point>> & contours) {
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
  std::vector<cv::Scalar> color_list = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255) };

  for (int i = 0; i < contours.size(); i++) {
    auto c = color_list[i % 6];
    for (int j = 0; j < contours[i].size() - 1; j++) {
      cv::line(color, cv::Point(contours[i][j].x, contours[i][j].y), cv::Point(contours[i][j+1].x, contours[i][j+1].y), c, 1);
    }
  }

  cv::imwrite("./images/output11.png", color);
}


int main()
{
  /**
  std::vector<std::vector<int>> image = {
      // 0 - 4
      {1,1,0,0,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {1,1,1,0,1, 1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {1,1,0,0,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      // 5 - 9
      {0,0,0,0,0, 0,0,0,0,0, 0,1,1,1,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,0,0,0,0, 0,1,1,1,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,0,0,0,0, 0,1,1,1,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,0,0,0,0, 0,1,1,1,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,0,0,0,0, 0,1,1,1,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      // 10 - 14
      {0,0,0,0,0, 0,0,0,0,0, 0,1,1,1,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,1,1,1,1, 1,1,1,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,1,0,0,0, 0,0,1,1,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      // 15 - 19
      {0,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,1,1,1,1, 1,1,1,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      // 20 - 24
      {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,1,1, 1,0,0,0,0},
      {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,1,1, 1,0,0,0,0},
      {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,1,1, 1,0,0,0,0},
      {0,0,0,0,0, 0,0,0,0,0, 1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      // 25 - 29
      {0,0,0,0,0, 0,0,0,0,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,0,0,1,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,0,1,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 0,1,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
      {0,0,0,0,0, 1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
  };
  **/
  std::vector<std::vector<int>> image = load_image_as_binary("images/test1.png");

  std::vector<std::vector<char>> visited(image.size(), std::vector<char>(image[0].size(), 0));

  std::vector<std::vector<Point>> result = contour_tracing(image, visited);
  for (auto points : result) {
    for (auto p : points) {
      std::cout << "(" << p.y << ", " << p.x << ") ";
    }
    std::cout << std::endl;
  }
  visualize(image, result);
  return 0;
}
