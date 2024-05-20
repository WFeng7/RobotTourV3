#include <Pathfinding.h>
#include <vector>
#include <queue>

Pathfinding::Pathfinding() {}

void Pathfinding::addWood(short a, short b, short c, short d) {
  wood[a * N + b][c * N + d] = true;
  wood[c * N + d][a * N + b] = true;
}

void Pathfinding::addGate(short a, short b) {
  ++n_bonus;
  bonus[a * N + b] = n_bonus;
}

void Pathfinding::findPath() {
  std::pair<short, short> vis[N * N][1 << n_bonus];
  bool flag = false;
  for(short i = 0; i < N*N; i++) {
    for(short j = 0; j < (1 << n_bonus); j++) {
      vis[i][j] = {-1, -1};
    }
  }
  std::queue<T> q;
  T t; t.x = start; t.y = 0; t.z = 0; t.p = -1, t.pz = 0;
  T nt;
  vis[t.x * N][0] = {-1, 0};
  q.push(t);
  while(q.size()) {
      t = q.front();
      q.pop();
      nt.p = t.x * N + t.y;
      nt.pz = t.z;
      for(short i = 0; i < 4; i++) {
        nt.x = dx[i] + t.x;
        nt.y = dy[i] + t.y;
        nt.z = t.z;
        if(bonus[nt.x * N + nt.y]) {
          nt.z |= (1 << (bonus[nt.x * N + nt.y] - 1));
        }
        if(nt.x >= 0 && nt.x < N && nt.y >= 0 && nt.y < N && !wood[nt.p][nt.x * N + nt.y] && vis[nt.x * N + nt.y][nt.z].second == -1) {
          vis[nt.x * N + nt.y][nt.z] = {nt.p, nt.pz};
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
  std::pair<short, short> tt = {nt.x * N + nt.y, nt.z};
  while(tt.first != -1) {
    coords.push_back(tt.first);
    tt = vis[tt.first][tt.second];
  }
  short a, b, c, d;
  short dist = 34;
  short dir = 0, ndir = 0; // 0 up 1 right 2 down 3 left
  for(short i = coords.size() - 2; i >= 0; i--) {
    a = coords[i + 1] / N;
    b = coords[i + 1] % N;
    c = coords[i] / N;
    d = coords[i] % N;
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
      path.push_back("t" + std::to_string(dist));
      dist = 50;
      if(ndir - dir == 1 || ndir - dir == -3) {
        path.push_back("r");
      }
      else if(ndir - dir == -1 || ndir - dir == 3) {
        path.push_back("l");
      }
      else {
        // path.push_back("r");
        // path.push_back("r");
        path.push_back("a");
        // dist = -dist;
      }
      dir = ndir;
    }
    else {
      dist += 50;
    }
  }
  if(dist != 0) {
    path.push_back("t" + std::to_string(dist));
  }
}

std::vector<std::string> Pathfinding::getPath() {
  return path;
}

void Pathfinding::setStart(short s) {
  start = s;
}

void Pathfinding::setTarget(std::pair<short, short> t) {
  target = t;
}