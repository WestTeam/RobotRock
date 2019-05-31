// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <iostream>
#include <math.h>

#include <../3rdparty/Eigen/Dense>

#include <QMutexLocker>
#include <QDate>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/Recalage.hpp>

using namespace Eigen;
using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{

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
    : _odometry( nullptr )
    , _lidar( nullptr )
{
    tDebug( LOG ) << "Recalage constructor engaged";

    _initDone = false;
    _continuousMode = false;
    _trigger = false;
    _finishing = false;

    _speedTargetHz = 1.0;
    _currentSpeedHz = 0.0;
}


Recalage::~Recalage()
{
    tDebug( LOG ) << "Recalage destructor called...";

    if( _attached )
    {
        _finishing = true;

        this->QThread::wait(2000);

        _lidar->stopScan();
        _lidar->stopMotor();
    }
    tDebug( LOG ) << "Recalage thread stopped";

}

bool Recalage::init( const Hal::Ptr& hal, const Odometry::Ptr& odometry, const LidarBase::Ptr& lidar )
{
    if( ! _attached )
    {
        _hal = hal;
        _odometry = odometry;
        _lidar = lidar;

        // launch data thread
        this->start();
    }

    _attached = true;

    tInfo( LOG ) << "Recalage initialized";

    return true;
}

bool Recalage::isInitDone()
{
    return _initDone;
}

void Recalage::borderListClear()
{
    _tableBorderNb = 0;
}

bool Recalage::borderListAdd( uint8_t dir,double ax, double ay, double bx, double by )
{
    if (_tableBorderNb < BORDER_COUNT_MAX)
    {
        _tableBorder[_tableBorderNb].dir = dir;
        _tableBorder[_tableBorderNb].ax = ax;
        _tableBorder[_tableBorderNb].ay = ay;
        _tableBorder[_tableBorderNb].bx = bx;
        _tableBorder[_tableBorderNb].by = by;

        _tableBorderNb++;
    } else {
        tWarning(LOG) << "Recalage: Cannot add new Border, current list is full";
        return false;
    }

    return true;
}


void Recalage::setLidarPosition(double x, double y, double theta0)
{
    _lidarPosX = x;
    _lidarPosY = y;
    _lidarTheta0 = theta0;

    _lidarTheta = atan2( _lidarPosY, _lidarPosX );
    _lidarR = sqrt( _lidarPosX * _lidarPosX + _lidarPosY * _lidarPosY );

}

void Recalage::setTargetSpeedHz(double hz)
{
    _speedTargetHz = hz;
    _initDone = false;
}

bool Recalage::getSpeedHz(double &hz)
{
    if (_initDone == true && _currentSpeedHz != 0.0)
    {
        hz = _currentSpeedHz;
    } else {
        hz = 0;
        return false;
    }
    return true;
}

void Recalage::setCalibrationMode(bool continiuous)
{
    _continuousMode = continiuous;
}

bool Recalage::triggerCalibration(bool blocking)
{
    if (_initDone)
    {
        _trigger = true;

        if (blocking)
        {
            while (_trigger == true)
            {
                this->QThread::msleep(10);
            }
        }
    } else {
        return false;
    }

    return true;
}

void Recalage::getAccumulatedError(RobotPos &pos)
{
    QMutexLocker locker( &_lock );
    pos = _error;
}

void Recalage::getLatestCalibratedPos(RobotPos &pos)
{
    QMutexLocker locker( &_lock );
    pos = _latestPos;
}

bool Recalage::calibrate(
    LidarData (&data)[LIDAR_MAX_SCAN_POINTS],
    uint32_t dataCount,
    RobotPos &currentError,
    RobotPos &absPos,
    double thetaBias)
{
    //tDebug( LOG ) << "Calibrating...";

    double errX = 0;
    double errY = 0;
    double errTheta = 0;



#ifdef DEBUG
    for( int pos = 0; pos < dataCount; ++pos )
    {
        tDebug( LOG ) << "Dist:" << mesR[ pos ] << "Theta:" << mesTheta[ pos ];
    }
#endif

    //QMutexLocker locker( &_lock );

    //initialisation tableaux de point associés à une bordure
    typedef struct {
        MatrixXd dot;
        int len;
    }Mesure;
    Mesure mesure[_tableBorderNb];
    for(int k=0; k<_tableBorderNb; k++) {
        mesure[k].dot.resize(dataCount,2);
        mesure[k].len = 0;
    }

    //association point bordure
    for(unsigned int j =0; j<dataCount; j++) {
        //drop false points
        if(data[j].r<ALGO_TELEM_DROP_MESURE)
            continue;

        RobotPos robotPos = data[j].pos;
        robotPos.theta += thetaBias;

        //position du télémètre
        double telemTheta = robotPos.theta + _lidarTheta0;
        double telemX = robotPos.x + _lidarR*cos(robotPos.theta+_lidarTheta);
        double telemY = robotPos.y + _lidarR*sin(robotPos.theta+_lidarTheta);


        //position du point mesuré
        double theta = telemTheta + data[j].theta;
        double x = telemX + data[j].r*cos(theta);
        double y = telemY + data[j].r*sin(theta);

        //associate dot to border
        double d = INFINITY;
        int K = 0;
        for(int k=0; k<_tableBorderNb; k++) {
            double dist;
            //AB.AP<0 => distance = AP
            if( (_tableBorder[k].bx-_tableBorder[k].ax)*(x-_tableBorder[k].ax) + (_tableBorder[k].by-_tableBorder[k].ay)*(y-_tableBorder[k].ay) < 0 )
                dist = sqrt( (x-_tableBorder[k].ax)*(x-_tableBorder[k].ax) + (y-_tableBorder[k].ay)*(y-_tableBorder[k].ay) );
            //BA.BP<0 => distance = BP
            else if( (_tableBorder[k].ax-_tableBorder[k].bx)*(x-_tableBorder[k].bx) + (_tableBorder[k].ay-_tableBorder[k].by)*(y-_tableBorder[k].by) < 0 )
                dist = sqrt( (x-_tableBorder[k].bx)*(x-_tableBorder[k].bx) + (y-_tableBorder[k].by)*(y-_tableBorder[k].by) );
            //sinon => distance = norm(AB^AP)/norm(AB)
            else
                dist = abs( (_tableBorder[k].bx-_tableBorder[k].ax)*(y-_tableBorder[k].ay) + (_tableBorder[k].by-_tableBorder[k].ay)*(x-_tableBorder[k].ax) )/( sqrt((_tableBorder[k].ax-_tableBorder[k].bx)*(_tableBorder[k].ax-_tableBorder[k].bx)+(_tableBorder[k].ay-_tableBorder[k].by)*(_tableBorder[k].ay-_tableBorder[k].by)) );

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
    int mesureLen = 0;
    {
        for(int k=0; k<_tableBorderNb; k++) {

            //qDebug() << "border" << k << mesure[k].len << "dir" << _tableBorder[k].dir;

            if(mesure[k].len<5){
                mesure[k].len = 0;
                continue;
            }

            int X,Y;
            if(_tableBorder[k].dir) {
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
            mesureLen += len;

            //qDebug() << "border end" << k << mesure[k].len << "dir" << _tableBorder[k].dir;
        }
    }

    if( mesureLen < 50 )
    {
        //tDebug( LOG ) << "No object found";
        return false;
    }

    //calcul erreur odometrie
    int lenX = 0;
    int lenY = 0;
    {
        MatrixXd in(mesureLen,3);
        VectorXd out(mesureLen);
        int i = 0;
        for(int k=0; k<_tableBorderNb; k++) {
            if(mesure[k].len==0)
                continue;

            for(int j =0; j<mesure[k].len; j++) {
                if(_tableBorder[k].dir) {
                    in(i,0) = -mesure[k].dot(j,0);
                    in(i,1) = 0;
                    in(i,2) = 1;
                    out(i) = _tableBorder[k].ay - mesure[k].dot(j,1);
                    lenY ++;
                }
                else {
                    in(i,0) = mesure[k].dot(j,1);
                    in(i,1) = 1;
                    in(i,2) = 0;
                    out(i) = _tableBorder[k].ax - mesure[k].dot(j,0);
                    lenX ++;
                }
                i++;
            }
        }

        MatrixXd A = in.jacobiSvd(ComputeThinU|ComputeThinV).solve(out);
        errX = A(1,0);
        errY = A(2,0);
        errTheta = asin(A(0,0));
    }

    // invert theta err (why?)
    errTheta *= -1.0;

    double qualityX = (double)lenX / 400.0;
    double qualityY = (double)lenY / 400.0;
    double qualityTheta;
    if(qualityX>1)
        qualityX = 1;
    if(qualityY>1)
        qualityY = 1;
    if(qualityX>qualityY)
        qualityTheta = qualityX;
    else
        qualityTheta = qualityY;

    //tDebug( LOG ) << "Calibrate quality" << qualityX << qualityY << qualityTheta;

    currentError.x = errX*qualityX;
    currentError.y = errY*qualityY;
    currentError.theta = errTheta*qualityTheta+thetaBias; // we add the input bias as output to make sure data is coherent

    //errorModify(errX*qualityX,errY*qualityY,errTheta*qualityTheta);


    //tDebug( LOG ) << "Recalage: Calibrate error no quality " << errX << errY << errTheta << DEG(errTheta);
    //tDebug( LOG ) << "Recalage: Calibrate error wt quality" << currentError.x << currentError.y << currentError.theta << DEG(currentError.theta);

    // get latest available position
    absPos = data[dataCount-1].pos;

    absPos.x        += currentError.x;
    absPos.y        += currentError.y;
    absPos.theta    += currentError.theta;

    //tDebug( LOG ) << "Recalage: Computed position " << absPos.x << absPos.y << absPos.theta << DEG(absPos.theta);


    //sendPos( robotPos ); // Apply new robot position based on previous computed error

    return true;
}



QList<LidarCircle::Obstacle> Recalage::locate(
    LidarData (&data)[LIDAR_MAX_SCAN_POINTS],
    uint32_t dataCount)
{
    QVector<float> d;
    QVector<float> a;
    QVector<float> q;

    for (int i = 0; i < dataCount; i++)
    {
        d << data[i].r;
        a << data[i].theta;
        q << data[i].quality;
    }

    LidarCircle _lidarCircle(100.0,_lidarPosX,_lidarPosY,-1.0);
    QList<LidarCircle::Obstacle> ret;


    QList<LidarCircle::Obstacle> list = _lidarCircle.compute(a,d);

    QList<LidarCircle::Obstacle> list_rel = list;

    QList<LidarCircle::Obstacle> list_abs;

    RobotPos robotPos = data[0].pos;


    _lidarCircle.transformToRobot(list_rel);
    list_abs = list_rel;
    _lidarCircle.transformToAbs(list_abs,robotPos.x,robotPos.y,robotPos.theta);
    _lidarCircle.associate(list_abs);
    //tInfo( LOG ) << "lidarcicle count" << list.count();

    //



    for (int i = 0; i < list.size(); i++)
    {//( QList<LidarCircle::Obstacle>::iterator it = list.begin(); it != list.end(); ++it){

        //pos.Q = it->Q;
        //pos.X = robotPos.x+(_lidarPosX+it->X)*cos(robotPos.theta)+(_lidarPosY+it->Y)*sin(robotPos.theta);
        //pos.Y = robotPos.y+(_lidarPosY+it->Y)*cos(robotPos.theta)-(_lidarPosX+it->X)*sin(robotPos.theta);

        float dist = hypot(list_rel[i].X,list_rel[i].Y);

        if (list_rel[i].Q >= 0.3 && dist <= 1000.0 && list_abs[i].type == LidarCircle::Obstacle::Type::ROBOT)
        {

            ret << list_abs[i];

            tInfo( LOG ) << "Robot Detection:Abs:" << dist << list_abs[i].X << list_abs[i].Y << list_abs[i].Q;
            //tInfo( LOG ) << "Robot Detection:Rel:" << dist << list_rel[i].X << list_rel[i].Y << list_rel[i].Q;
        }
    }

    return ret;
}


void Recalage::run()
{
    tDebug( LOG ) << "Recalage: Run";

#define PERCENTAGE_START 25.0
#define PERCENTAGE_STEP 2.5
#define RETRY_COUNT 5
#define STABLE_COUNT ((uint32_t)(1.0/(1.0/_speedTargetHz)))

    _lidar->setMinimumQuality(0);
    float mPercentage = PERCENTAGE_START;
    _lidar->startMotor(mPercentage);
    this->QThread::msleep(2000);
    _lidar->startScan();

    uint32_t dataCount;
    RobotPos currentError;
    RobotPos absPos;

    uint64_t last_scan_ok = 0;
    uint8_t retry_count = 0;
    uint8_t consecutive_speed_ok = 0;
    uint8_t consecutive_speed_ko = 0;
    bool ok;
    unsigned int dropScanCount = 0;

    while (!_finishing)
    {
        if( ! _initDone )
        {
                ok = _lidar->get360ScanData(data,dataCount);
                if (ok && dataCount != 0)
                {
                    uint64_t now = QDateTime::currentMSecsSinceEpoch();

                    if (last_scan_ok != 0)
                    {
                        uint64_t period = now-last_scan_ok;

                        last_scan_ok = now;

                        double freq = 1.0/((double)(period)/1000.0);
                        _currentSpeedHz = freq;
                        tDebug( LOG ) << "Recalage: freq" << freq << period << consecutive_speed_ko;

                        if (freq <= _speedTargetHz)
                        {
                            consecutive_speed_ok = 0;
                            consecutive_speed_ko++;
                            if (consecutive_speed_ko==STABLE_COUNT)
                            {
                                mPercentage+=PERCENTAGE_STEP;
                                if (mPercentage >= 100.0)
                                    mPercentage = 100.0;

                                _lidar->startMotor(mPercentage);
                                //this->QThread::msleep(2000);
                                last_scan_ok = 0;
                            }
                        } else {
                            consecutive_speed_ok++;
                            if (consecutive_speed_ok==STABLE_COUNT)
                            {
                                _initDone = true;
                            }
                        }
                    } else {
                        consecutive_speed_ok = 0;
                        consecutive_speed_ko = 0;
                        last_scan_ok = now;
                    }

                    retry_count = 0;

                } else {
                    retry_count++;
                    if (retry_count == RETRY_COUNT)
                    {
                        consecutive_speed_ko = 0;
                        consecutive_speed_ok = 0;
                        retry_count = 0;

                        mPercentage+=PERCENTAGE_STEP;
                        if (mPercentage >= 100.0)
                            mPercentage = 100.0;

                        _lidar->startMotor(mPercentage);
                        //this->QThread::msleep(1000);
                        _lidar->stopScan();
                        _lidar->startScan();

                        last_scan_ok = 0;
                    }
                }
            }
        else
        {
                ok = _lidar->get360ScanData(data,dataCount);

                if (ok && dataCount != 0)
                {
                    uint64_t now = QDateTime::currentMSecsSinceEpoch();

                    uint64_t period = now-last_scan_ok;
                    last_scan_ok = now;

                    if (last_scan_ok != 0)
                    {
                        double freq = 1.0/((double)(period)/1000.0);
                        _currentSpeedHz = freq;
                    }

                    // if != 0 we continue (ie we do not process the data)
                    if (dropScanCount)
                    {
                        dropScanCount--;
                        continue;
                    }

                    bool ok;
                    // we check if the first pos is the same as last pos
                    ok = (data[0].pos == data[dataCount-1].pos);

                    // first calibration in order to correct theta first
                    if (ok)
                    {
                        //ok = calibrate(data,dataCount,currentError,absPos,0.0);

                        QList<LidarCircle::Obstacle> list = locate(data,dataCount);

                        uint8_t speed_reductor = 0;

                        for (int i = 0; i < list.length(); i++)
                        {
                            float dist = hypot(list[i].X-data[0].pos.x, list[i].Y-data[0].pos.y);

                            if (dist < 1000)
                                speed_reductor = 100-50;
                            if (dist < 800)
                                speed_reductor = 100-30;
                            if (dist < 650)
                                speed_reductor = 100-0;
                            if (dist < 400)
                                speed_reductor = 100-0;
                        }

                        _hal->_pidDistanceSpeedReductor.write(speed_reductor);
                        _hal->_pidAngleSpeedReductor.write(speed_reductor);
                        if (speed_reductor != 0)
                            tDebug( LOG ) << "Recalage: Speed Reductor to " << speed_reductor << "%";
                    }

                    if (false)//(ok)
                    {
                        double thetaBias = currentError.theta;

                        // second one to have correction on X/Y
                        ok = calibrate(data,dataCount,currentError,absPos,thetaBias);

                        if (ok)
                        {
                            tDebug( LOG ) << "Recalage: Corrected Pos X:" << absPos.x << "Y:" << absPos.y << "Theta:" << absPos.theta;
                            tDebug( LOG ) << "Recalage: Detected Error X:" << currentError.x << "Y:" << currentError.y << "Theta:" << currentError.theta << thetaBias;

                            // we lock the update of internal data that gets outputed outside
                            QMutexLocker locker( &_lock );

                            // update latest pos
                            _latestPos = absPos;

                            if (_continuousMode || _trigger)
                            {
                                _trigger = false;

                                // the bias is included in the Error output, so we can use directly the currentError output
                                // we update odometry with the error we deducted
                                // we explicitely do not update the absolute position because this one may have changed if
                                // the robot has been moving during the scan & processing
                                _odometry->addError(currentError);

                                // we update accumulated error
                                _error.x += currentError.x;
                                _error.y += currentError.y;
                                _error.theta += currentError.theta;

                                // we set this in order to make sure we will process next data with updated position
                                // in case buffering with previous position occured
                                dropScanCount = 2;
                            }
                        }
                    }
                }
            }
        }
/*
        bool ok;
        ok = _lidar->get360ScanData(data,dataCount);
        if (ok && dataCount != 0)
            calibrate(data,dataCount,currentError,absPos);

        this->QThread::msleep(2000);
*/
    }



/*

    void borderListClear();
    bool borderListAdd( uint8_t dir,double ax, double ay, double bx, double by );

    void setLidarPosition(double x, double y);
    void setLidarThetaOffset(double theta0);

    void setTargetSpeedHz(double hz);
    bool getSpeedHz(double &hz);

    void setCalibrationMode(bool continiuous = false);

    bool triggerCalibration(bool blocking);

    void getAccumulatedError(RobotPos &pos);
    void getLatestCalibratedPos(RobotPos &pos);

    //void errorInit( double errX, double errY, double errTheta );

    //void errorModify( double errX, double errY, double errTheta );

    bool calibrate(
        LidarData (&data)[LIDAR_MAX_SCAN_POINTS],
        uin32_t data_count,
        RobotPos &currentError,
        RobotPos &absPos);
*/


/*
RobotPos Recalage::getPos()
{
    QMutexLocker locker( _lock );
    RobotPos pos;

    pos.theta = _odoThetaReg->read< int16_t >();
    pos.x = _odoXReg->read< int16_t >();
    pos.y = _odoYReg->read< int16_t >();

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

    //_odoThetaReg->write( static_cast< int >( pos.theta ) );
    //_odoXReg->write( static_cast< int >( pos.x ) );
    //_odoYReg->write( static_cast< int >( pos.y ) );

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
    int mesLen,					// nb de mesures télémètre
    const double* mesR,			// mesure télémètre
    const double* mesTheta )    // angle correspondant à la masure
{
    tInfo( LOG ) << "Calibrating...";

    double errX = 0;
    double errY = 0;
    double errTheta = 0;

    RobotPos robotPos = getPos(); // Position du robot

#ifdef DEBUG
    for( int pos = 0; pos < mesLen; ++pos )
    {
        tDebug( LOG ) << "Dist:" << mesR[ pos ] << "Theta:" << mesTheta[ pos ];
    }
#endif

    QMutexLocker locker( _lock );

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
        mesure[k].dot.resize(mesLen,2);
        mesure[k].len = 0;
    }

    //association point bordure
    for(unsigned int j =0; j<mesLen; j++) {
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
    int mesureLen = 0;
    {
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
            mesureLen += len;

            //qDebug() << "border end" << k << mesure[k].len << "dir" << tableBorder[k].dir;
        }
    }

    if( mesureLen < 50 )
    {
        tDebug( LOG ) << "No object found";
        return false;
    }

    //calcul erreur odometrie
    int lenX = 0;
    int lenY = 0;
    {
        MatrixXd in(mesureLen,3);
        VectorXd out(mesureLen);
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
                    lenY ++;
                }
                else {
                    in(i,0) = mesure[k].dot(j,1);
                    in(i,1) = 1;
                    in(i,2) = 0;
                    out(i) = tableBorder[k].ax - mesure[k].dot(j,0);
                    lenX ++;
                }
                i++;
            }
        }

        MatrixXd A = in.jacobiSvd(ComputeThinU|ComputeThinV).solve(out);
        errX = A(1,0);
        errY = A(2,0);
        errTheta = asin(A(0,0));
    }

    double qualityX = (double)lenX / 400.0;
    double qualityY = (double)lenY / 400.0;
    double qualityTheta;
    if(qualityX>1)
        qualityX = 1;
    if(qualityY>1)
        qualityY = 1;
    if(qualityX>qualityY)
        qualityTheta = qualityX;
    else
        qualityTheta = qualityY;

    tDebug( LOG ) << "Calibrate quality" << qualityX << qualityY << qualityTheta;

    errorModify(errX*qualityX,errY*qualityY,errTheta*qualityTheta);

    tDebug( LOG ) << "Calibrate error" << errX << errY << errTheta;

    sendPos( robotPos ); // Apply new robot position based on previous computed error

    return true;
}*/
