#pragma once

#include "micropather.h"

typedef unsigned int uint;

namespace tilemover2d
{
    struct Tile
    {
        bool isBlocking() const { return blocking; }

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

    struct Vector2
    {
        Vector2() {}
        Vector2(const float _x, const float _y) : x(_x), y(_y) {}

        Vector2 & operator+=(const Vector2 & other);
        Vector2 & operator-=(const Vector2 & other);
        Vector2 & operator*=(const float value);

        float
            x,
            y;
    };

    Vector2 operator+(const Vector2 & a, const Vector2 & b);
    Vector2 operator-(const Vector2 & a, const Vector2 & b);
    Vector2 operator*(const Vector2 & a, const float value);

    typedef Vector2 Position;

    struct Path
    {
        void debugPrint() const;

        MP_VECTOR<Vector2>
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
        const Vector2 & getPosition() const { return position; };
        const Vector2 & getVelocity() const { return velocity; };
        void moveTo(const Vector2 & position);

        float
            speed,
            radius;
    private:
        void update(const float dt);
        void postUpdate();
        void recomputePath();
        void prepareForTargetIndex(const uint index);

        Vector2
            position,
            velocity,
            desiredDisplacement;
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
        bool findPath(Path & path, const Vector2 & from, const Vector2 & to);
        bool findPath(Path & path, const Point & from, const Point & to);
        bool findPoint(Point & point, const Vector2 & position) const;
        Agent & createAgent(const Vector2 & position);
        const MP_VECTOR< Agent *> & getAgents() const { return agents; }
        void update(const float dt);

    private:
        virtual float LeastCostEstimate(void* stateStart, void* stateEnd) override;
        virtual void AdjacentCost(void* state, MP_VECTOR< micropather::StateCost > *adjacent) override;
        virtual void PrintStateInfo( void* state ) override {}

        Tile & internalGetTile(const int x, const int y);
        void getPointFromNode(Point & p, void* node) const;
        void *getNodeFromPoint(const Point & p) const;
        void getPositionFromPoint(Vector2 & position, const Point & point) const;
        void getPointFromPosition(Point & point, const Vector2 & position) const;

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
