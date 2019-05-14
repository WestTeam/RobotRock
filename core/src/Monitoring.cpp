// Copyright (c) 2019 All Rights Reserved WestBot

#include <WestBot/RobotRock/Monitoring.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

Monitoring::Monitoring( const Hal::Ptr& hal, const Odometry::Ptr& odo )
    : _hal( hal )
    , _odo( odo )
    , _screen( "/dev/ttyAL0" )
    , _delayMs( 1000 )
    , _color( Color::Unknown )
{
    // Set name once at boot time
    _screen.send( "tTeam", "WestTeam" );
}

void Monitoring::setRefreshRate( int delayMs )
{
    _delayMs = delayMs;
}

void Monitoring::updateColor( Color color )
{
    _color = color;
}

//
// Private methods
//
void Monitoring::run()
{
    while( 1 )
    {
        dump();
        QThread::msleep( _delayMs );
    }
}

// Code couleur
// jaune 65057
// violet 34833
// rouge 63488
// vert 2016
//
void Monitoring::dump()
{
    if( _color == Color::Blue )
    {
        _screen.setColor( "tTeam", "34833" );
    }
    else
    {
        _screen.setColor( "tTeam", "65057" );
    }

    // For debugging purpose ONLY!!!
    static int score = 0;
    _screen.send( "tScore", QString::number( ++score ).toLatin1() );

    RobotPos currPos = _odo->getPosition();

    QString odoStr = QString( "x:%1 y:%2@%3°" )
        .arg( currPos.x, 4 )
        .arg( currPos.y, 4 )
        .arg( DEG( currPos.theta ), 4, 'f', 2 );

    _screen.send( "tPosOdo", odoStr.toLatin1() );
    _screen.send( "tPosLidar", "" );

    uint32_t v0 = _hal->_voltage12V.read< uint32_t >();
    uint32_t v1 = _hal->_voltage24V.read< uint32_t >();
    uint32_t v2 = _hal->_voltageA5V.read< uint32_t >();
    uint32_t v3 = _hal->_voltageA12V.read< uint32_t >();

    _screen.send( "tVoltage0", QString::number( v0 ).toLatin1() );
    _screen.send( "tVoltage1", QString::number( v1 ).toLatin1() );
    _screen.send( "tVoltage2", QString::number( v2 ).toLatin1() );
    _screen.send( "tVoltage3", QString::number( v3 ).toLatin1() );

    _screen.send( "tStatus0", "OFF" );
    _screen.send( "tStatus1", "OFF" );
    _screen.send( "tStatus2", "OFF" );
    _screen.send( "tStatus", "OFF" );
    _screen.send( "tStatus4", "OFF" );
    _screen.send( "tStatus5", "OFF" );
    _screen.send( "tStatus6", "OFF" );
    _screen.send( "tStatus7", "OFF" );
}
