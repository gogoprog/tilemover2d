#include "tilemover2d.h"

#include <cstdio>

namespace tilemover2d
{

void Path::debugPrint() const
{
    for(int i=0; i<points.size(); i++)
    {
        const Point & p = points[i];

        printf("(%d, %d)\n", p.x, p.y);
    }
}

World::World()
    :
    pather(this)
{

}

void World::init(const int _width, const int _height)
{
    width = _width;
    height = _height;

    tiles = new Tile[width * height];
    memset(tiles, 0, sizeof(Tile) * width * height);
}

const Tile & World::getTile(const int x, const int y) const
{
    return tiles[y*width + x];
}

void World::solve(const Point & from, const Point & to)
{
    float cost = 0;
    Path path;

    int result = pather.Solve(pointToNode(from), pointToNode(to), &lastComputedPath, &cost);

    path.points.resize(lastComputedPath.size());
    for(int i=0; i<lastComputedPath.size(); i++)
    {
        nodeToPoint(lastComputedPath[i], path.points[i]);
    }

    path.cost = cost;

    path.debugPrint();
}

float World::LeastCostEstimate(void* stateStart, void* stateEnd)
{
    Point start, end;
    nodeToPoint(stateStart, start);
    nodeToPoint(stateEnd, end);
    int dx = start.x - end.x;
    int dy = start.y - end.y;
    return float(dx*dx + dy*dy);
}

void World::AdjacentCost(void* state, MP_VECTOR< micropather::StateCost > *adjacent)
{
    static const int dx[8] = { 1, 0, -1, 0 };
    static const int dy[8] = { 0, 1, 0, -1 };

    Point p;

    nodeToPoint(state, p);

    for( int i=0; i<8; ++i )
    {
        int nx = p.x + dx[i];
        int ny = p.y + dy[i];

        if(nx < 0 || ny < 0 || nx >= width || ny >= height)
        {
            continue;
        }

        const Tile & tile = getTile(nx, ny);

        if(!tile.blocking)
        {
            micropather::StateCost node_cost = { pointToNode(Point(nx, ny)), 1.0f };
            adjacent->push_back(node_cost);
        }
    }
}

void World::nodeToPoint(void* node, Point & p) 
{
    intptr_t index = (intptr_t)node;
    p.y = index / width;
    p.x = index - p.y * width;
}

void *World::pointToNode(const Point & p)
{
    return (void*) (p.y*width + p.x);
}

}
