// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ASTARHIGHLEVEL_HPP_
#define WESTBOT_ROBOTROCK_ASTARHIGHLEVEL_HPP_

#include <memory>

#include <QList>
#include <QObject>
#include <QPair>

#include <WestBot/AStar/AStar.hpp>
#include <WestBot/AStar/MapNode.hpp>
#include <WestBot/AStar/NodeState.hpp>
#include <WestBot/AStar/Utils.hpp>

#include "Action.hpp"
#include "TrajectoryManager.hpp"

namespace WestBot {
namespace RobotRock {

class AStarHighLevel : public QObject
{
    Q_OBJECT

public:
    using Ptr = std::shared_ptr< AStarHighLevel >;

    struct MapNode
    {
        uint type;
        uint expandCost;
        char c;
    };

    AStarHighLevel(
        const TrajectoryManager::Ptr& trajectoryManager,
        float inv,
        uint mapWidth,
        uint mapHeight );

    ~AStarHighLevel() override = default;

    void setMap( uint mapWidth, uint mapHeight );
    void destroyMap();

    void setCurrentPos( uint x, uint y );
    void setTarget( uint x, uint y );

    void setObstacle( uint xStart, uint yStart, uint xEnd, uint yEnd );
    void setDirt( uint xStart, uint yStart, uint xEnd, uint yEnd );

    void processCurrentRoute( bool saveChanges );

    void dumpMap();

signals:
    void newRoute( QList< QPair< uint, uint > > path ); // This can emit a new QList of actions

private:
    TrajectoryManager::Ptr _trajectoryManager;
    float _inv;
    uint _mapWidth;
    uint _mapHeight;
    WestBot::AStar::AStar _astar;
    QList< QPair< uint, uint > > _processedPath;

    QList< Action::Ptr > _actions;

    MapNode** _map;
};

}
}

#endif // WESTBOT_ROBOTROCK_INPUT_HPP_
