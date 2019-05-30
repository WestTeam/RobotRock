// Copyright (c) 2019 All Rights Reserved WestBot

#include <WestBot/HumanAfterAll/Category.hpp>


#include <WestBot/RobotRock/Monitoring.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Monitoring" )
}


Monitoring::Monitoring(
    const Hal::Ptr& hal,
    const Odometry::Ptr& odo,
    const ArmsManager::Ptr& armsManager )
    : _hal( hal )
    , _odo( odo )
    , _armsManager( armsManager )
    , _screen( "/dev/ttyAL0" )
    , _delayMs( 1000 )
    , _color( Color::Unknown )
{
    // Set name once at boot time
    _screen.send( "tTeam", "WestTeam" );

    _screen.setColor( "tVoltage2", "2016" );

    _screen.send( "tScore", QString::number( 40+18 ).toLatin1() );


    _stop = false;
    //start();
}

Monitoring::~Monitoring()
{
    _stop = true;
    QThread::msleep( 1000 );
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
    while( !_stop )
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
    _screen.send( "tScore", QString::number( _armsManager->getScore() ).toLatin1() );

    RobotPos currPos = _odo->getPosition();

    QString odoStr = QString( "x:%1 y:%2@%3°" )
        .arg( currPos.x, 4 )
        .arg( currPos.y, 4 )
        .arg( DEG( currPos.theta ), 4, 'f', 2 );

    _screen.send( "tPosOdo", odoStr.toLatin1() );
    _screen.send( "tPosLidar", "" );

    int32_t v0 = _hal->_voltage12V.read< int32_t >();
    int32_t v1 = _hal->_voltage24V.read< int32_t >();
    int32_t v2 = _hal->_voltageA5V.read< int32_t >();
    int32_t v3 = _hal->_voltageA12V.read< int32_t >();




/*    tDebug(LOG) << "v0" << v0 << 12.0 + ((double)v0-580000.0)/800000.0;
    tDebug(LOG) << "v1" << v1 << 24.0 + ((double)v1-(-1113000.0))/280000.0;
    tDebug(LOG) << "v2" << v2 << 12.0 + ((double)v2-580000.0)/800000.0;
    tDebug(LOG) << "v3" << v3 << 12.0 + ((double)v3-580000.0)/800000.0;
*/

    double dv0 = 12.0 + ((double)v0-580000.0)/800000.0;
    double dv1 = 24.0 + ((double)v1-(-1113000.0))/280000.0;
    double dv2 = 12.0 + ((double)v2-580000.0)/800000.0;
    double dv3 = 12.0 + ((double)v3-580000.0)/800000.0;

    if (dv0 > 11.0)
    {
        _screen.setColor( "tVoltage0", "2016" );
    } else if (dv0 > 9.0)
    {
        _screen.setColor( "tVoltage0", "65057" );

    } else {
        _screen.setColor( "tVoltage0", "63488" );

    }

    if (dv1 > 11.0*2.0)
    {
        _screen.setColor( "tVoltage1", "2016" );
    } else if (dv1 > 9.0*2.0)
    {
        _screen.setColor( "tVoltage1", "65057" );

    } else {
        _screen.setColor( "tVoltage1", "63488" );
    }


    if (dv3 > 11.0)
    {
        _screen.setColor( "tVoltage3", "2016" );
    } else if (dv3 > 9.0)
    {
        _screen.setColor( "tVoltage3", "65057" );

    } else {
        _screen.setColor( "tVoltage3", "63488" );

    }



    _screen.send( "tVoltage0", QString::number( dv0 ).toLatin1() );
    _screen.send( "tVoltage1", QString::number( dv1 ).toLatin1() );
    _screen.send( "tVoltage2", QString::number( dv2 ).toLatin1() );
    _screen.send( "tVoltage3", QString::number( dv3 ).toLatin1() );

    _screen.send( "tStatus0", "OFF" );
    _screen.send( "tStatus1", "OFF" );
    _screen.send( "tStatus2", "OFF" );
    _screen.send( "tStatus", "OFF" );
    _screen.send( "tStatus4", "OFF" );
    _screen.send( "tStatus5", "OFF" );
    _screen.send( "tStatus6", "OFF" );
    _screen.send( "tStatus7", "OFF" );
}
