#include <Pathfinding.h>
#include <vector>
#include <queue>

Pathfinding::Pathfinding() {}

void Pathfinding::addWood(short a, short b, short c, short d) {
  wood[a * M + b][c * M + d] = true;
  wood[c * M + d][a * M + b] = true;
}

void Pathfinding::addBlocks(std::vector<std::string> vblocks, std::vector<std::string> hblocks) {
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      if(vblocks[i][j] == '1') {
        addWood(j, 4 - i - 1, j + 1, 4 - i - 1);
      }
    }
  }
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 5; j++) {
      if(hblocks[i][j] == '1') {
        addWood(j, 3 - i - 1, j, 3 - i);
      }
    }
  }
}

void Pathfinding::addGate(short a, short b) {
  ++n_bonus;
  bonus[a * M + b] = n_bonus;
}

void Pathfinding::addGate(std::string s) {
  ++n_bonus;
  bonus[(s[0] - '0') * M + (s[2] - '0')] = n_bonus;
}

bool Pathfinding::checkBlock(int a, int b, int dir) {
  if(dir == 0 && b < M - 1 && wood[a * M + b][a * M + b + 1]) {
    return true;
  }
  else if(dir == 2 && b > 0 && wood[a * M + b - 1][a * M + b]) {
    return true;
  }
  else if(dir == 1 && a < N - 1 && wood[(a + 1) * M + b][a * M + b]) {
    return true;
  }
  else if(dir == 3 && a > 0 && wood[(a - 1) * M + b][a * M + b]) {
    return true;
  }
  return false;
}

void Pathfinding::findPath() {
  path.clear();
  std::pair<short, short> vis[N * M][1 << n_bonus];
  bool flag = false;
  for(short i = 0; i < N*M; i++) {
    for(short j = 0; j < (1 << n_bonus); j++) {
      vis[i][j] = {-1, -1};
    }
  }
  std::queue<T> q;
  T t; t.x = (start.second == 'h' ? start.first : 0); t.y = (start.second == 'h' ? 0 : start.first); t.z = 0; t.p = -1, t.pz = 0;
  if(bonus[t.x * M + t.y]) {
    t.z |= (1 << (bonus[t.x * M + t.y] - 1));
  }
  T nt;
  vis[t.x * M + t.y][t.z] = {-1, 0};
  q.push(t);
  while(q.size()) {
      t = q.front();
      q.pop();
      nt.p = t.x * M + t.y;
      nt.pz = t.z;
      for(short i = 0; i < 4; i++) {
        nt.x = dx[i] + t.x;
        nt.y = dy[i] + t.y;
        nt.z = t.z; 
        if(nt.x >= 0 && nt.x < N && nt.y >= 0 && nt.y < M && bonus[nt.x * M + nt.y]) {
          nt.z |= (1 << (bonus[nt.x * M + nt.y] - 1));
        }
        if(nt.x >= 0 && nt.x < N && nt.y >= 0 && nt.y < M && !wood[nt.p][nt.x * M + nt.y] && vis[nt.x * M + nt.y][nt.z].second == -1) {
          vis[nt.x * M + nt.y][nt.z] = {nt.p, nt.pz};
          if(nt.x == target.first && nt.y == target.second && nt.z == (1 << n_bonus) - 1) {
            flag = true;
            break;
          }
          q.push(nt);
        }
    }
    if(flag) {
      break;
    }
  }
  // final is nt
  std::vector<short> coords;
  std::pair<short, short> tt = {nt.x * M + nt.y, nt.z};
  while(tt.first != -1) {
    coords.push_back(tt.first);
    tt = vis[tt.first][tt.second];
  }
  short a, b, c, d;
  short dist = 37;
  short dir = (start.second == 'h' ? 0 : 1), ndir = (start.second == 'h' ? 0 : 1); // 0 up 1 right 2 down 3 left
  for(short i = coords.size() - 2; i >= 0; i--) {
    a = coords[i + 1] / M;
    b = coords[i + 1] % M;
    c = coords[i] / M;
    d = coords[i] % M;
    if(d - b > 0) {
      ndir = 0;
    }
    else if(c - a > 0) {
      ndir = 1;
    }
    else if(d - b < 0) {
      ndir = 2;
    }
    else {
      ndir = 3;
    } 
    if(dir != ndir) {
      if(dist % 10 == 7) {
        path.push_back("t" + std::to_string(37));
        dist -= 37;
      }
      while(dist >= 50) {
        path.push_back("t" + std::to_string(50.5));
        dist -= 50;
      }
      dist = 50;
      // get distance
      bool extra_dist = checkBlock(a, b, dir);
      if(extra_dist) {
        path.push_back("c");
      }
      if(ndir - dir == 1 || ndir - dir == -3) {
        path.push_back("r");
      }
      else if(ndir - dir == -1 || ndir - dir == 3) {
        path.push_back("l");
      }
      else {
        if(checkBlock(a, b, (dir + 3) % 4)) {
          path.push_back("l");
          path.push_back("c");
          path.push_back("l");
        }
        else if(checkBlock(a, b, (dir + 1) % 4)) {
          path.push_back("r");
          path.push_back("c");
          path.push_back("r");
        }
        else {
          path.push_back("r");
          path.push_back("r");
        }
        // path.push_back("a");
        // dist = -dist;
      }
      dir = ndir;
    }
    else {
      dist += 50;
    }
  }
  if(dist % 10 == 7) {
    path.push_back("t" + std::to_string(37));
    dist -= 37;
  }
  while(dist >= 50) {
    path.push_back("t" + std::to_string(50.5));
    dist -= 50;
  }
}

std::vector<std::string> Pathfinding::getPath() {
  return path;
}

void Pathfinding::setStart(std::pair<short, char> s) {
  start = s;
}

void Pathfinding::setTarget(std::pair<short, short> t) {
  target = t;
}