#include "tilemover2d.h"
#include <iostream>
using namespace std;
using namespace tilemover2d;

int main()
{
    cout << "tilemover2d test application" << endl;

    World world;

    world.init(25600, 25600);

    world.solve(Point(1,1), Point(4, 20000));


    return 0;
}