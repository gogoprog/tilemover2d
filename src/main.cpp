#include "tilemover2d.h"
#include <iostream>
using namespace std;
using namespace tilemover2d;

int main()
{
    cout << "tilemover2d test application" << endl;

    World world;
    Path path;
    Agent *agents[3];

    cout << "Initializing world" << endl;

    world.init(256, 256, 32, 32);

    cout << "Finding a path" << endl;

    world.findPath(path, Point(1,1), Point(4, 4));

    path.debugPrint();

    cout << "Creating agent" << endl;

    agents[0] = & world.createAgent(Position(0, 0));

    return 0;
}