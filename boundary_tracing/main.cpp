#include<iostream>
#include <vector>

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

bool is_isolated(const std::vector<std::vector<int>> & image, Point p) {
    int height = image.size();
    int width = image[0].size();
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dy == 0 && dx == 0) continue;
            int ny = p.y + dy;
            int nx = p.x + dx;
            if (ny >= 0 && ny < height && nx >= 0 && nx < width) {
                if (image[ny][nx] == 1) return false; // not isolated because other foreground pixels found
            }
        }
    }
    return true;
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
    
      // skip 1-pixel forground
      if (is_isolated(image, curr)) {
        visisted[curr.y][curr.x] = 1;
        continue;
      }

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
        if (!has_route) break; // foreground but inside the outer contour.

      } while (!is_start(curr, start));

    if (contour.size() > 1) contours.push_back(contour);
  }
  
  return contours;
}

int main()
{
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

  std::vector<std::vector<char>> visited(image.size(), std::vector<char>(image[0].size(), 0));

  std::vector<std::vector<Point>> result = contour_tracing(image, visited);
  for (auto points : result) {
    for (auto p : points) {
      std::cout << "(" << p.y << ", " << p.x << ") ";
    }
    std::cout << std::endl;
  }
  return 0;
}
