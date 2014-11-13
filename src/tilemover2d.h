#pragma once

#include "micropather.h"

typedef unsigned int uint;

namespace tilemover2d
{
    struct Tile
    {
        bool
            blocking;
    };

    struct Point
    {
        Point() {}
        Point(const uint _x, const uint _y) : x(_x), y(_y) {}

        uint
            x,
            y;
    };

    struct Path
    {
        MP_VECTOR<Point>
            points;
        float
            cost;

        void debugPrint() const;
    };

    class World : public micropather::Graph
    {
    public:
        World();
        void init(const int width, const int height);
        const Tile & getTile(const int x, const int y) const;

        void solve(const Point & from, const Point & to);

    private:
        virtual float LeastCostEstimate(void* stateStart, void* stateEnd);
        virtual void AdjacentCost(void* state, MP_VECTOR< micropather::StateCost > *adjacent);
        virtual void PrintStateInfo( void* state ) {}

        void nodeToPoint(void* node, Point & p);
        void *pointToNode(const Point & p);

        micropather::MicroPather
            pather;
        uint
            width,
            height;
        Tile
            * tiles;
        MP_VECTOR< void* >
            lastComputedPath;
    };
}
