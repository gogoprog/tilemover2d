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
        void debugPrint() const;

        MP_VECTOR<Position>
            positions;
        float
            cost;
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
        const Position & getPosition() const { return position; };
        void moveTo(const Position & position);

        float
            speed,
            radius;
    private:
        void update(const float dt);

        Position
            position;
        State
            state;
        Path
            path;
        World
            * world;
        float
            currentTime,
            currentDuration;
        uint
            currentTargetIndex;
    };

    class World : public micropather::Graph
    {
    public:
        World();
        void init(const int width, const int height, const float tile_width, const float tile_height);
        void setTileBlocking(const int x, const int y, const bool blocking);
        const Tile & getTile(const int x, const int y) const;
        bool findPath(Path & path, const Position & from, const Position & to);
        bool findPath(Path & path, const Point & from, const Point & to);
        Agent & createAgent(const Position & position);
        void update(const float dt);

    private:
        virtual float LeastCostEstimate(void* stateStart, void* stateEnd) override;
        virtual void AdjacentCost(void* state, MP_VECTOR< micropather::StateCost > *adjacent) override;
        virtual void PrintStateInfo( void* state ) override {}

        Tile & getTile(const int x, const int y);
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
        bool
            mustReset;
    };
}
