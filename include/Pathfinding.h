#ifndef Pathfinding_h
#define Pathfinding_h

#include <Arduino.h>
#include <vector>


struct T {
    short x, y, z, p, pz;
};

class Pathfinding {
    private:
        static const short N = 4;
        const short dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
        short n_bonus;
        std::vector<std::string> path;
        short bonus[N * N] = { 0 };
        short start;
        std::pair<short, short> target;
        bool wood[N*N][N*N] = { false };
    public:
        Pathfinding();
        void addWood(short a, short b, short c, short d);
        void addBlocks(std::vector<std::string> vblocks, std::vector<std::string> hblocks);
        void addGate(short a, short b);
        void addGate(std::string s);
        void findPath();
        std::vector<std::string> getPath();

        void setStart(short s);
        void setTarget(std::pair<short, short> t);
};

#endif