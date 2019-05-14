// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <math.h>

#include <WestBot/RobotRock/Odometry.hpp>

bool WestBot::RobotRock::operator==( RobotPos& lhs, RobotPos& rhs )
{
    if( ( fabs( lhs.x - rhs.x ) <= 1.0 ) &&
        ( fabs( lhs.y - rhs.y ) <= 1.0) &&
        ( DEG( fabs( lhs.theta - rhs.theta ) ) <= 0.1 ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}
