// Copyright (c) 2019 All Rights Reserved WestBot

#include <iostream>

#include <WestBot/RobotRock/AStarHighLevel.hpp>
#include <WestBot/RobotRock/MoveAction.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    // TEST FOR A*
    MoveAction::Ptr moveGenericAction(
        const TrajectoryManager::Ptr& trajectoryManager,
        int x,
        int y,
        float inv )
    {
        return std::make_shared< MoveAction >(
            trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_XY_ABS,
            0.0,
            0.0,
            x,
            inv * y,
            true );
    }
}

AStarHighLevel::AStarHighLevel(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv,
    uint mapWidth,
    uint mapHeight )
    : _trajectoryManager( trajectoryManager )
    , _inv( inv )
    , _mapWidth( mapWidth )
    , _mapHeight( mapHeight )
    , _map( nullptr )
{
    _astar.setHeuristics( WestBot::AStar::AStarHeuristics::euclidean );

    setMap( _mapWidth, _mapHeight );
}

void AStarHighLevel::setMap( uint mapWidth, uint mapHeight )
{
    destroyMap();

    _mapWidth = mapWidth;
    _mapHeight = mapHeight;

    if( mapWidth == 0 || mapHeight == 0 )
    {
        return;
    }

    _map = new MapNode*[ _mapWidth ];

    for( uint i = 0; i < _mapWidth; i++ )
    {
        _map[ i ] = new MapNode[ _mapHeight ];
    }

    for( uint i = 0; i < _mapWidth; i++ )
    {
        for( uint j = 0; j < _mapHeight; j++ )
        {
            _map[i][j].c = '0';
            _map[i][j].expandCost = 1;
        }
    }

    _astar.setMatrix( _mapWidth, _mapHeight );
}

void AStarHighLevel::destroyMap()
{
    if( _map != NULL )
    {
        for (uint i = 0; i < _mapWidth; i++)
            delete [] _map[i];
        delete [] _map;

        _map = NULL;
        _astar.destroyMatrix();
        _processedPath.clear();
    }
}

void AStarHighLevel::setCurrentPos( uint x, uint y )
{
    _astar.setStart( x, y );

    QPair< uint, uint > start;

    start = _astar.getStart();

    _map[ start.first ][ start.second ].c = 'S';
}

void AStarHighLevel::setTarget( uint x, uint y )
{
    _astar.setEnd( x, y );
    QPair< uint, uint > end;

    end = _astar.getEnd();

    _map[ end.first ][ end.second ].c = 'E';
}

void AStarHighLevel::setObstacle( uint xStart, uint yStart, uint xEnd, uint yEnd )
{
    for( uint i = xStart; i <= xEnd; i++ )
    {
        for( uint j = yStart; j <= yEnd; j++ )
        {
            _astar.setWall( i, j );
            _map[ i ][ j ].c = 'X';
        }
    }
}

void AStarHighLevel::setDirt( uint xStart, uint yStart, uint xEnd, uint yEnd )
{
    for( uint i = xStart; i <= xEnd; i++ )
    {
        for( uint j = yStart; j <= yEnd; j++ )
        {
            _astar.setWay( i, j );
            _astar.setExpandCost( 2, i, j );
            _map[ i ][ j ].c = 'D';
        }
    }
}

void AStarHighLevel::processCurrentRoute( bool saveChanges )
{
    _processedPath = _astar.getPath( saveChanges );

    // TODO: handle save changes properly
    if( 0 == _processedPath.size() )
    {
        return;
    }

    for( auto pathIt = _processedPath.begin(); pathIt != _processedPath.end(); ++pathIt )
    {
        _map[ pathIt->first ][ pathIt->second ].c = '*';
        _actions.push_back(
            moveGenericAction( _trajectoryManager,
            pathIt->first,
            pathIt->second,
            _inv ) );
    }

    emit newRoute( _actions );
}

void AStarHighLevel::dumpMap()
{
    for( uint i = 0; i < _mapWidth; i++ )
    {
        for( uint j = 0; j < _mapHeight; j++ )
        {
            std::cout << _map[ i ][ j ].c;
        }

        std::cout << std::endl;
    }
}
