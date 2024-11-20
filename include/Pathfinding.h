#ifndef Pathfinding_h
#define Pathfinding_h

#include <Arduino.h>
#include <vector>

struct T {
    short x, y, z, p, pz;
};

class Pathfinding {
    private:
        static const short N = 5, M = 4;
        const short dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
        short n_bonus = 0;
        std::vector<std::string> path;
        short bonus[N * M] = { 0 };
        std::pair<short, char> start;
        std::pair<short, short> target;
        std::pair<short, short> last_gate = {-1, -1};
        short last_dir;
        bool wood[N*M][N*M] = { false };
    public:
        Pathfinding();
        void addWood(short a, short b, short c, short d);
        void addBlocks(std::vector<std::string> vblocks, std::vector<std::string> hblocks);
        void addGate(short a, short b);
        void addGate(std::string s);
        bool checkBlock(int a, int b, int dir);
        void findPath();
        void gridPath();
        std::vector<std::string> getPath();
        void setStart(std::pair<short, char> s);
        void setTarget(std::pair<short, short> t);
        void setLastGate(std::pair<short, short> l);
};

#endif