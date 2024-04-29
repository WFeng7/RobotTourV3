#ifndef Pathfind_h
#define Pathfind_h

#include <string>
#include <iostream>

class Pathfind {
    private:
        const short N = 4; // start robot in the forward dir by half a step
        const short dx[4] = {0, 0, 1, -1}, dy[4] = {1, -1, 0, 0};
    public:
        Pathfind();
        std::string find_path();
};      


#endif