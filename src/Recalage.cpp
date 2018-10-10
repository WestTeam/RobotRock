// Copyright (c) 2018 All Rights Reserved WestBot

#include <iostream>
#include <math.h>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/Recalage.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Recalage" )

    const double TELEM_THETA0 = -0.118;
    const double TELEM_X = 206.08;
    const double TELEM_Y = 101.1;
    const double TELEM_THETA = atan2( TELEM_Y, TELEM_X );
    const double TELEM_R = sqrt( TELEM_X * TELEM_X + TELEM_Y * TELEM_Y );

    const double ALGO_TELEM_DROP_MESURE = 100;
    const double ALGO_BORDER_ASSOCIATE_DROP = 150;
    const double ALGO_BORDER_FILTER_DROP = 10;
}

Recalage::Recalage()
    : _odoThetaReg( nullptr )
    , _odoXReg( nullptr )
    , _odoYReg( nullptr )
    , _attached( false )
{
}

bool Recalage::init( Hal& hal )
{
    if( ! _attached )
    {
        _odoThetaReg = std::make_shared< ItemRegister >( hal._odometryTheta );
        _odoXReg = std::make_shared< ItemRegister >( hal._odometryX );
        _odoYReg = std::make_shared< ItemRegister >( hal._odometryY );
    }

    _attached = true;

    tInfo( LOG ) << "Recalage initialized";

    return true;
}

RobotPos Recalage::getPos()
{
    RobotPos pos;
	double odoTheta = _odoThetaReg->read< int16_t >();
	double odoX = _odoThetaReg->read< int16_t >();
	double odoY = _odoThetaReg->read< int16_t >();

    pos.x = odoX * cos( error.theta ) + odoY * sin( error.theta ) + error.x;
    pos.y = -odoX * sin( error.theta ) + odoY * cos( error.theta ) + error.y;
    pos.theta = odoTheta - error.theta;
	return pos;
}

RobotPos Recalage::sendPos( const RobotPos& robotPos )
{
	RobotPos pos;
    pos.x = ( robotPos.x - error.x ) * cos( error.theta ) -
            ( robotPos.y - error.y ) * sin( error.theta );
    pos.y = ( robotPos.x - error.x ) * sin( error.theta ) +
            ( robotPos.y - error.y ) * cos( error.theta );
    pos.theta = robotPos.theta + error.theta;
	return pos;
}

void Recalage::errorInit( double errX, double errY, double errTheta )
{
	error.x = errX;
	error.y = errY;
	error.theta = errTheta;
}

void Recalage::errorModify( double errX, double errY, double errTheta )
{
    error.x = error.x * cos( errTheta ) +
              error.y * sin( errTheta ) +
              errX;
    error.y = -error.x * sin( errTheta ) +
              error.y * cos( errTheta ) +
              errY;
	error.theta += errTheta;
}

bool Recalage::calibrate(
    int len,					// nb de mesures télémètre
    const double* mesR,			// mesure télémètre
    const double* mesTheta )    // angle correspondant à la masure
{
    /*double errX;
    double errY;
    double errTheta;

    RobotPos robotPos = getPos();

	{
		//position du télémètre
		double telemTheta = robotPos.theta + TELEM_THETA0;
		double telemX = robotPos.x + TELEM_R*cos(robotPos.theta+TELEM_THETA);
		double telemY = robotPos.y + TELEM_R*sin(robotPos.theta+TELEM_THETA);

		//initialisation tableaux de point associés à une bordure
		typedef struct {
			MatrixXd dot;
			int len;
		}Mesure;
		Mesure mesure[tableBorderNb];
		for(int k=0; k<tableBorderNb; k++) {
			mesure[k].dot.resize(len,2);
			mesure[k].len = 0;
		}

		//association point bordure
		for(int j =0; j<len; j++) {
			//drop false points
			if(mesR[j]<ALGO_TELEM_DROP_MESURE)
				continue;

			//position du point mesuré
			double theta = telemTheta + mesTheta[j];
			double x = telemX + mesR[j]*cos(theta);
			double y = telemY + mesR[j]*sin(theta);

			//associate dot to border
			double d = INFINITY;
			int K = 0;
			for(int k=0; k<tableBorderNb; k++) {
				double dist;
				//AB.AP<0 => distance = AP
				if( (tableBorder[k].bx-tableBorder[k].ax)*(x-tableBorder[k].ax) + (tableBorder[k].by-tableBorder[k].ay)*(y-tableBorder[k].ay) < 0 )
					dist = sqrt( (x-tableBorder[k].ax)*(x-tableBorder[k].ax) + (y-tableBorder[k].ay)*(y-tableBorder[k].ay) );
				//BA.BP<0 => distance = BP
				else if( (tableBorder[k].ax-tableBorder[k].bx)*(x-tableBorder[k].bx) + (tableBorder[k].ay-tableBorder[k].by)*(y-tableBorder[k].by) < 0 )
					dist = sqrt( (x-tableBorder[k].bx)*(x-tableBorder[k].bx) + (y-tableBorder[k].by)*(y-tableBorder[k].by) );
				//sinon => distance = norm(AB^AP)/norm(AB)
				else
					dist = abs( (tableBorder[k].bx-tableBorder[k].ax)*(y-tableBorder[k].ay) + (tableBorder[k].by-tableBorder[k].ay)*(x-tableBorder[k].ax) )/( sqrt((tableBorder[k].ax-tableBorder[k].bx)*(tableBorder[k].ax-tableBorder[k].bx)+(tableBorder[k].ay-tableBorder[k].by)*(tableBorder[k].ay-tableBorder[k].by)) );

				if(dist<d) {
					d = dist;
					K = k;
				}
			}
			if(d<ALGO_BORDER_ASSOCIATE_DROP) {
				mesure[K].dot(mesure[K].len,0) = x;
				mesure[K].dot(mesure[K].len,1) = y;
				mesure[K].len++;
			}
		}

		//suppression points
		{
			int lenUpdate = 0;
			for(int k=0; k<tableBorderNb; k++) {

				//qDebug() << "border" << k << mesure[k].len << "dir" << tableBorder[k].dir;

				if(mesure[k].len<5){
					mesure[k].len = 0;
					continue;
				}

				int X,Y;
				if(tableBorder[k].dir) {
					X = 0;
					Y = 1;
				}
				else {
					X = 1;
					Y = 0;
				}


				MatrixXd in(mesure[k].len,2);
				VectorXd out(mesure[k].len);
				for(int j =0; j<mesure[k].len; j++) {
					in(j,0) = mesure[k].dot(j,X);
					in(j,1) = 1;
					out(j) = mesure[k].dot(j,Y);
				}

				MatrixXd droite = in.jacobiSvd(ComputeThinU|ComputeThinV).solve(out);
				VectorXf u(2);
				u(0) = -droite(0,0);
				u(1) = 1;
				u = u/u.norm();

				int len = 0;
				for(int j =0; j<mesure[k].len; j++) {
					double d = abs( (mesure[k].dot(j,X))*(u[0]) + (mesure[k].dot(j,Y)-droite(1,0))*(u[1]) );
					if(d<ALGO_BORDER_FILTER_DROP) {
						mesure[k].dot(len,0) = mesure[k].dot(j,0);
						mesure[k].dot(len,1) = mesure[k].dot(j,1);
						len++;
					}
				}
				mesure[k].len = len;
				lenUpdate += len;

				//qDebug() << "border end" << k << mesure[k].len << "dir" << tableBorder[k].dir;
			}
			len = lenUpdate;
		}

		//calcul erreur odometrie
		{
			MatrixXd in(len,3);
			VectorXd out(len);
			int i = 0;
			for(int k=0; k<tableBorderNb; k++) {
				if(mesure[k].len==0)
					continue;

				for(int j =0; j<mesure[k].len; j++) {
					if(tableBorder[k].dir) {
						in(i,0) = -mesure[k].dot(j,0);
						in(i,1) = 0;
						in(i,2) = 1;
						out(i) = tableBorder[k].ay - mesure[k].dot(j,1);
					}
					else {
						in(i,0) = mesure[k].dot(j,1);
						in(i,1) = 1;
						in(i,2) = 0;
						out(i) = tableBorder[k].ax - mesure[k].dot(j,0);
					}
					i++;
				}
			}

			MatrixXd A = in.jacobiSvd(ComputeThinU|ComputeThinV).solve(out);
			errX = A(1,0);
			errY = A(2,0);
			errTheta = asin(A(0,0));
		}

	}

	errorModify(errX,errY,errTheta);

    std::cout
        << "Calibration done with error: "
        << errX << " "
        << errY << " "
        << errTheta << std::endl;
    */
	return true;
}
