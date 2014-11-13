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

    struct Position
    {
        Position() {}
        Position(const float _x, const float _y) : x(_x), y(_y) {}

        float
            x,
            y;
    };

    struct Path
    {
        MP_VECTOR<Position>
            positions;
        float
            cost;

        void debugPrint() const;
    };

    class World;
    class Agent
    {
        friend class World;

    public:

        enum class State
        {
            IDLE,
            ACTIVE
        };

        Agent();
        State getState() const { return state; };
        const Path & getPath() const { return path; };
        void moveTo(const Position & position);

        float
            speed;

    private:
        Position
            position;
        State
            state;
        Path
            path;
        World
            * world;
    };

    class World : public micropather::Graph
    {
    public:
        World();
        void init(const int width, const int height, const float tile_width, const float tile_height);
        void setTileBlocking(const int x, const int y, const bool blocking);
        const Tile & getTile(const int x, const int y) const;
        Tile & getTile(const int x, const int y);
        bool findPath(Path & path, const Position & from, const Position & to);
        bool findPath(Path & path, const Point & from, const Point & to);
        Agent & createAgent(const Position & position);
        void update(const float dt);

    private:
        virtual float LeastCostEstimate(void* stateStart, void* stateEnd);
        virtual void AdjacentCost(void* state, MP_VECTOR< micropather::StateCost > *adjacent);
        virtual void PrintStateInfo( void* state ) {}
        void nodeToPoint(void* node, Point & p);
        void *pointToNode(const Point & p);
        void pointToPosition(Position & position, const Point & point);
        void positionToPoint(Point & point, const Position & position);

        micropather::MicroPather
            pather;
        uint
            width,
            height;
        float
            tileWidth,
            tileHeight;
        Tile
            * tiles;
        MP_VECTOR< void* >
            lastComputedPath;
        MP_VECTOR< Agent *>
            agents;
    };
}
