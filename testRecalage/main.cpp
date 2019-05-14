#include <QCoreApplication>
#include <QThread>
#include <QDate>
#include <random>


#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/OdometrySimu.hpp>
#include <WestBot/RobotRock/Lidar.hpp>
#include <WestBot/RobotRock/Recalage.hpp>


#define DEBUG
#define SIMU

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.testRecalage" )
}


typedef struct
{
    double x; // mm
    double y; // mm
    double r;
    double theta;
} LidarPoint;

bool lidarPointCompare (const LidarPoint& first, const LidarPoint& second)
{
/*  double first_deg = DEG(first.theta);
  if (first_deg < 0.0)
      first_deg += 180.0;
  double second_deg = DEG(second.theta);
  if (second_deg < 0.0)
      second_deg += 180.0;
*/
  return (first.theta < second.theta);
}


class LidarTest: public LidarBase
{
public:
    using Ptr = std::shared_ptr< LidarTest >;

    LidarTest(const Odometry::Ptr odometry)
    {
        _odometry = odometry;
        tInfo(LOG) << "LidarTest: Constructor";
    }

    ~LidarTest()
    {
        tInfo(LOG) << "LidarTest: Destructor";

    }

    void startMotor(float percentage)
    {
        tInfo(LOG) << "LidarTest: startMotor" << percentage;
        // 25% == 1hz
        // 50 = 2hz
        // 100 = 4hz
        _periodScan = 1.0/(15.0*percentage/100.0)*1000.0;
    }
    void stopMotor()
    {
        tInfo(LOG) << "LidarTest: stopMotor";
        _periodScan == 0;
    }

    void startScan()
    {
        _scanning = true;
        _dateLastScan = QDateTime::currentMSecsSinceEpoch();
        tInfo(LOG) << "LidarTest: startScan";

    }
    void stopScan()
    {
        _scanning = false;
        tInfo(LOG) << "LidarTest: stopScan";
    }

    void setMinimumQuality(uint8_t minQuality)
    {
        tInfo(LOG) << "LidarTest: setMinimumQuality" << minQuality;
    }

    bool get360ScanData(LidarData (&data)[LIDAR_MAX_SCAN_POINTS], uint32_t &count)
    {
        //tInfo(LOG) << "LidarTest: get360ScanData" << _periodScan;

        count = 0;

        if (_scanning && _periodScan <= 2000)
        {
            uint64_t date = QDateTime::currentMSecsSinceEpoch();
            uint64_t delta = date-_dateLastScan;

            if (_dateLastScan != 0 && delta <= _periodScan)
            {
                QThread::msleep(_periodScan-delta);
                //tInfo(LOG) << "LidarTest: get360ScanData : sleep" << _periodScan-delta;

            }
            _dateLastScan = QDateTime::currentMSecsSinceEpoch();

            // generate
            std::list<LidarPoint>::iterator it;
            count = 0;

            RobotPos pos_odo = _odometry->getPosition();
            //tInfo(LOG) << "LidarTest: get360ScanData : position used" << pos_odo.x << pos_odo.y << DEG(pos_odo.theta);


/*
            pos_with_error.x += _posErrX;
            pos_with_error.y += _posErrY;
            pos_with_error.theta += _posErrTheta;
*/

            std::uniform_real_distribution<double> unif_error(-_measureMaxError,+_measureMaxError);

            std::uniform_real_distribution<double> unif_missing(0.0,100.0);

            for (it=_listPoint.begin(); it!=_listPoint.end(); ++it)
            {
                if (_missingRate <= unif_missing(re_missing))
                {
                    uint32_t index = count++;
                    data[index].pos = pos_odo;
                    double r_error = unif_error(re_error);
                    data[index].r = it->r+r_error;
                    data[index].theta = it->theta;
                 }


                //tInfo(LOG) << "LidarTest: generate " << it->r << it->theta;
            }

            return true;
        } else {
            return false;
        }

    }

    void setRobotPosition(RobotPos pos)
    {
        _pos = pos;
    }

    void setLidarPosition(RobotPos pos)
    {
        _lidarPos = pos;
    }

    void setMeasureMaxError(double max)
    {
        _measureMaxError = max;
    }

    void setPointMissingRate(double missingRate)
    {
        _missingRate = missingRate;
    }


    void borderListClear()
    {
        _tableBorderNb = 0;
    }

    bool borderListAdd( uint8_t dir,double ax, double ay, double bx, double by )
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
            tWarning(LOG) << "LidarTest: Cannot add new Border, current list is full";
            return false;
        }

        return true;
    }


    void generate()
    {
        _listPoint.clear();

        RobotPos lidarPosAbs;

        double angleCenterLidar = atan2(_lidarPos.y-0,_lidarPos.x-0);
//        tInfo(LOG) << "LidarTest: angleCenterLidar " << angleCenterLidar;
        double distanceCenterLidar = sqrt(_lidarPos.x*_lidarPos.x+_lidarPos.y*_lidarPos.y);

        // we recompute the real absolute position of the lidar center
        lidarPosAbs.x = _pos.x + distanceCenterLidar * cos(_pos.theta+angleCenterLidar);
        lidarPosAbs.y = _pos.y + distanceCenterLidar * sin(_pos.theta+angleCenterLidar);
        // and we compute also the real angle of the measure relatively to the current robot angle and also the angle offset of the lidar
        lidarPosAbs.theta = _pos.theta+_lidarPos.theta;

        //tInfo(LOG) << "LidarTest: lidarPosAbs " << lidarPosAbs.x << lidarPosAbs.y << lidarPosAbs.theta;


        uint32_t count = LIDAR_MAX_SCAN_POINTS;
        uint32_t count_per_bordure = LIDAR_MAX_SCAN_POINTS/_tableBorderNb;
        for (uint8_t i = 0; i < _tableBorderNb; i++)
        {
            double dx = (_tableBorder[i].bx-_tableBorder[i].ax)/(double)count_per_bordure;
            double dy = (_tableBorder[i].by-_tableBorder[i].ay)/(double)count_per_bordure;
            //tInfo(LOG) << "LidarTest: generate dx/dy " << dx << dy << count_per_bordure;

            for (uint32_t p=0;p < count_per_bordure; p++)
            {
                LidarPoint pt;
                pt.x = _tableBorder[i].ax + dx*p;
                pt.y = _tableBorder[i].ay + dy*p;
                _listPoint.push_back(pt);
                //tInfo(LOG) << "LidarTest: generate " << pt.x << pt.y;
            }
        }

        // now compute R/THETA for each depending on current position
        std::list<LidarPoint>::iterator it;
        for (it=_listPoint.begin(); it!=_listPoint.end(); ++it)
        {
            it->r = sqrt((it->x-lidarPosAbs.x)*(it->x-lidarPosAbs.x) + (it->y-lidarPosAbs.y)*(it->y-lidarPosAbs.y));
            it->theta = atan2((it->y-lidarPosAbs.y),(it->x-lidarPosAbs.x));
            it->theta -= lidarPosAbs.theta;
            if (it->theta < 0.0)
                it->theta += M_PI;
            if (it->theta >= 2*M_PI)
                it->theta -= 2*M_PI;

            //tInfo(LOG) << "LidarTest: generate " << it->r << it->theta;
        }
        tInfo(LOG) << "LidarTest: generate point count " << _listPoint.size();

        // now we sort the points depending on theta
        //tInfo(LOG) << "LidarTest: first and last theta " << _listPoint.front().theta << _listPoint.back().theta;
        _listPoint.sort(lidarPointCompare);
        //tInfo(LOG) << "LidarTest: first and last theta " << _listPoint.front().theta << _listPoint.back().theta;

    }


private:
    uint64_t _dateLastScan = 0;
    uint64_t _periodScan = 0;
    bool _scanning = false;
    RobotPos _pos = {.x = 0.0, .y = 0.0, .theta = 0.0};
    /*double _posErrX = 0;
    double _posErrY = 0;
    double _posErrTheta = 0;*/
    Odometry::Ptr _odometry;
    double _measureMaxError = 0.0;
    double _missingRate = 0.0;
    RobotPos _lidarPos = {.x = 0.0, .y = 0.0, .theta = 0.0};

    std::default_random_engine re_error;
    std::default_random_engine re_missing;

    struct {
        bool dir;
        double	ax;
        double	ay;
        double	bx;
        double	by;
    } _tableBorder[BORDER_COUNT_MAX];

    int _tableBorderNb = 0;

    std::list<LidarPoint> _listPoint;

};

void log_management(int argc, char *argv[]){
    QCoreApplication app(argc,argv);

    Handler handler( app.instance() );
    ConsoleAppender consoleAppender;
    handler.addAppender( & consoleAppender );

#ifdef DEBUG
    handler.setEnableDebugLevel( true );
#endif

    app.exec();
}

int main( int argc, char *argv[] )
{
    std::thread thread_logs(log_management,argc,argv);
/*
    QCoreApplication app( argc, argv );

    Handler handler( app.instance() );
    ConsoleAppender consoleAppender;
    handler.addAppender( & consoleAppender );

#ifdef DEBUG
    handler.setEnableDebugLevel( true );
#endif
*/

    Hal::Ptr hal = std::make_shared< Hal >();

    hal->_motor5Override.write(1);
    hal->_motor5Value.write(0);

    ///// GENERAL CONFIG /////

    RobotPos rpos; // real position
    RobotPos rpos_with_error; // position currently in odo (with error)
    RobotPos lidarPos = {.x = 200, .y = 100, .theta = -0.418};// lidar position from robot center

    Odometry::Ptr odometryPtr = std::make_shared< OdometrySimu >();
    LidarTest::Ptr lidarTestPtr = std::make_shared< LidarTest >(odometryPtr);

    lidarTestPtr->borderListAdd(0,0   ,-1500,0   ,1500);
    lidarTestPtr->borderListAdd(1,0   ,1500 ,2000,1500);
    lidarTestPtr->borderListAdd(0,2000,1500 ,2000,-1500);
    lidarTestPtr->borderListAdd(1,2000,-1500,0   ,-1500);

    unsigned int tId;
    unsigned int i;
    double targetHz;

    // TEST 1: No error odo, No error measure
    {
        tId = 1;

        rpos = {.x = 400, .y = 200, .theta = RAD(45)};
        rpos_with_error = {.x = rpos.x-0, .y = rpos.y+0, .theta = rpos.theta+RAD(0)};

        // SETUP ODO
        odometryPtr->setPosition(rpos_with_error);

        // SETUP LIDAR
        lidarTestPtr->setRobotPosition(rpos);
        lidarTestPtr->setLidarPosition(lidarPos);

        lidarTestPtr->setMeasureMaxError(0.0);
        lidarTestPtr->setPointMissingRate(0.0);

        // generate internal points from config
        lidarTestPtr->generate();


        Recalage recalage;

        recalage.setCalibrationMode(false);
        recalage.setLidarPosition(lidarPos.x,lidarPos.y,lidarPos.theta);
        targetHz = 2.0;
        recalage.setTargetSpeedHz(targetHz);

        recalage.init(odometryPtr,lidarTestPtr);

        // Bordure setup
        recalage.borderListAdd(0,0   ,-1500,0   ,1500);
        recalage.borderListAdd(1,0   ,1500 ,2000,1500);
        recalage.borderListAdd(0,2000,1500 ,2000,-1500);
        recalage.borderListAdd(1,2000,-1500,0   ,-1500);

        i = 0;

        // check first if init ok & target speed ok
        while (recalage.isRunning())
        {
            i++;
            if (i == 200)
            {
                tFatal(LOG) << "testRecalage: Test " << tId << ": KO";
            }
            double hz;
            recalage.getSpeedHz(hz);

            if (recalage.isInitDone() && hz >= targetHz)
            {
                tInfo(LOG) << "testRecalage: Test " << tId << ": Init OK";

                bool ok = recalage.triggerCalibration(true);
                if (ok)
                {
                    RobotPos pos;
                    recalage.getLatestCalibratedPos(pos);

                    if (fabs(pos.x-rpos.x) > 1.0 || fabs(pos.y-rpos.y) > 1.0 || DEG(fabs(pos.theta-rpos.theta)) > 0.1)
                        tFatal(LOG) << "testRecalage: Test " << tId << ": Calibration KO" << pos.x << pos.y << pos.theta;
                    else
                    {
                        tInfo(LOG) << "testRecalage: Test " << tId << ": triggerCalibration OK";
                        break;
                    }
                }
                else {
                    tFatal(LOG) << "testRecalage: Test " << tId << ": triggerCalibration KO";
                }

                break;
            }
            QThread::msleep(500);
        }

    } // END TEST 1

    // TEST 2: Increment error odo, No error measure
    {
        tId = 2;

        for (unsigned int e = 0; e < 4; e++)
        {
            double error_x = ((double)e+1)*7.0;
            double error_y = ((double)e+1)*6.0;
            double error_theta = ((double)e+1)*1.5;

            tDebug(LOG) << "testRecalage: Test " << tId << ": Error insertion: X/Y/Theta" << error_x << error_y << error_theta;

            rpos = {.x = 400, .y = 200, .theta = RAD(45)};
            rpos_with_error = {.x = rpos.x-error_x, .y = rpos.y+error_y, .theta = rpos.theta+RAD(error_theta)};

            // SETUP ODO
            odometryPtr->setPosition(rpos_with_error);

            // SETUP LIDAR
            lidarTestPtr->setRobotPosition(rpos);
            lidarTestPtr->setLidarPosition(lidarPos);

            lidarTestPtr->setMeasureMaxError(0.0);
            lidarTestPtr->setPointMissingRate(0.0);

            // generate internal points from config
            lidarTestPtr->generate();


            Recalage recalage;

            recalage.setCalibrationMode(true);
            recalage.setLidarPosition(lidarPos.x,lidarPos.y,lidarPos.theta);
            targetHz = 2.0;
            recalage.setTargetSpeedHz(targetHz);

            recalage.init(odometryPtr,lidarTestPtr);

            // Bordure setup
            recalage.borderListAdd(0,0   ,-1500,0   ,1500);
            recalage.borderListAdd(1,0   ,1500 ,2000,1500);
            recalage.borderListAdd(0,2000,1500 ,2000,-1500);
            recalage.borderListAdd(1,2000,-1500,0   ,-1500);

            i = 0;

            while (recalage.isInitDone());


            // check first if init ok & target speed ok
            while (recalage.isRunning())
            {
                i++;
                if (i == 200)
                {
                    tFatal(LOG) << "testRecalage: Test " << tId << ": KO";
                }

                if (recalage.isInitDone())
                {
                    RobotPos pos;
                    recalage.getLatestCalibratedPos(pos);

                    if (fabs(pos.x-rpos.x) > 1.0 || fabs(pos.y-rpos.y) > 1.0 || DEG(fabs(pos.theta-rpos.theta)) > 0.1)
                        tDebug(LOG) << "testRecalage: Test " << tId << ": Waiting Calibration process" << pos.x << pos.y << pos.theta;
                    else
                    {
                        tInfo(LOG) << "testRecalage: Test " << tId << ": Calibration OK" << pos.x << pos.y << pos.theta;
                        break;
                    }

                }
                QThread::msleep(500);
            }
        } // for
    } // END TEST 2

    // TEST 3: No Error Odo, Increment measure error
    {
        tId = 3;

        for (unsigned int e = 0; e < 4; e++)
        {
            double error_measure = 20.0*(double)e;

            double error_x = 0;
            double error_y = 0;
            double error_theta = 0;

            tDebug(LOG) << "testRecalage: Test " << tId << ": Error insertion: Measure/X/Y/Theta" << error_measure << error_x << error_y << error_theta;

            rpos = {.x = 400, .y = 200, .theta = RAD(45)};
            rpos_with_error = {.x = rpos.x-error_x, .y = rpos.y+error_y, .theta = rpos.theta+RAD(error_theta)};

            // SETUP ODO
            odometryPtr->setPosition(rpos_with_error);

            // SETUP LIDAR
            lidarTestPtr->setRobotPosition(rpos);
            lidarTestPtr->setLidarPosition(lidarPos);

            lidarTestPtr->setMeasureMaxError(error_measure);
            lidarTestPtr->setPointMissingRate(0.0);

            // generate internal points from config
            lidarTestPtr->generate();


            Recalage recalage;

            recalage.setCalibrationMode(true);
            recalage.setLidarPosition(lidarPos.x,lidarPos.y,lidarPos.theta);
            targetHz = 2.0;
            recalage.setTargetSpeedHz(targetHz);

            recalage.init(odometryPtr,lidarTestPtr);

            // Bordure setup
            recalage.borderListAdd(0,0   ,-1500,0   ,1500);
            recalage.borderListAdd(1,0   ,1500 ,2000,1500);
            recalage.borderListAdd(0,2000,1500 ,2000,-1500);
            recalage.borderListAdd(1,2000,-1500,0   ,-1500);

            i = 0;

            while (recalage.isInitDone());


            // check first if init ok & target speed ok
            while (recalage.isRunning())
            {
                i++;
                if (i == 200)
                {
                    tFatal(LOG) << "testRecalage: Test " << tId << ": KO";
                }

                if (recalage.isInitDone())
                {
                    RobotPos pos;
                    recalage.getLatestCalibratedPos(pos);

                    if (fabs(pos.x-rpos.x) > 1.0 || fabs(pos.y-rpos.y) > 1.0 || DEG(fabs(pos.theta-rpos.theta)) > 0.1)
                        tDebug(LOG) << "testRecalage: Test " << tId << ": Waiting Calibration process" << pos.x << pos.y << pos.theta;
                    else
                    {
                        tInfo(LOG) << "testRecalage: Test " << tId << ": Calibration OK" << pos.x << pos.y << pos.theta;
                        break;
                    }

                }
                QThread::msleep(500);
            }
        } // for
    } // END TEST 3


    // TEST 4: Increment Error Odo, Increment measure error
    {
        tId = 4;

        for (unsigned int e = 0; e < 4; e++)
        {
            double error_measure = 20.0*(double)e;

            double error_x = ((double)e+1)*7.0;
            double error_y = ((double)e+1)*6.0;
            double error_theta = ((double)e+1)*1.5;

            tDebug(LOG) << "testRecalage: Test " << tId << ": Error insertion: Measure/X/Y/Theta" << error_measure << error_x << error_y << error_theta;

            rpos = {.x = 400, .y = 200, .theta = RAD(45)};
            rpos_with_error = {.x = rpos.x-error_x, .y = rpos.y+error_y, .theta = rpos.theta+RAD(error_theta)};

            // SETUP ODO
            odometryPtr->setPosition(rpos_with_error);

            // SETUP LIDAR
            lidarTestPtr->setRobotPosition(rpos);
            lidarTestPtr->setLidarPosition(lidarPos);

            lidarTestPtr->setMeasureMaxError(error_measure);
            lidarTestPtr->setPointMissingRate(0.0);

            // generate internal points from config
            lidarTestPtr->generate();


            Recalage recalage;

            recalage.setCalibrationMode(true);
            recalage.setLidarPosition(lidarPos.x,lidarPos.y,lidarPos.theta);
            targetHz = 2.0;
            recalage.setTargetSpeedHz(targetHz);

            recalage.init(odometryPtr,lidarTestPtr);

            // Bordure setup
            recalage.borderListAdd(0,0   ,-1500,0   ,1500);
            recalage.borderListAdd(1,0   ,1500 ,2000,1500);
            recalage.borderListAdd(0,2000,1500 ,2000,-1500);
            recalage.borderListAdd(1,2000,-1500,0   ,-1500);

            i = 0;

            while (recalage.isInitDone());


            // check first if init ok & target speed ok
            while (recalage.isRunning())
            {
                i++;
                if (i == 200)
                {
                    tFatal(LOG) << "testRecalage: Test " << tId << ": KO";
                }

                if (recalage.isInitDone())
                {
                    RobotPos pos;
                    recalage.getLatestCalibratedPos(pos);

                    if (fabs(pos.x-rpos.x) > 1.0 || fabs(pos.y-rpos.y) > 1.0 || DEG(fabs(pos.theta-rpos.theta)) > 0.1)
                        tDebug(LOG) << "testRecalage: Test " << tId << ": Waiting Calibration process" << pos.x << pos.y << pos.theta;
                    else
                    {
                        tInfo(LOG) << "testRecalage: Test " << tId << ": Calibration OK" << pos.x << pos.y << pos.theta;
                        break;
                    }

                }
                QThread::msleep(500);
            }
        } // for
    } // END TEST 4


    // TEST 5: Test InitDone with RPLIDAR
    {
        tId = 5;

        rpos = {.x = 400, .y = 200, .theta = RAD(45)};
        rpos_with_error = {.x = rpos.x-0, .y = rpos.y+0, .theta = rpos.theta+RAD(0)};

        lidarTestPtr.reset();

        LidarRPLidarA2::Ptr lidarTestPtr = std::make_shared<LidarRPLidarA2>( QString("/dev/ttyAL6"), 256000, std::make_shared< ItemRegister >( hal->_motor5Value ));
        lidarTestPtr->init();

        // SETUP ODO
        odometryPtr->setPosition(rpos_with_error);

        Recalage recalage;

        recalage.setCalibrationMode(false);
        recalage.setLidarPosition(lidarPos.x,lidarPos.y,lidarPos.theta);
        targetHz = 3.0;
        recalage.setTargetSpeedHz(targetHz);

        recalage.init(odometryPtr,lidarTestPtr);

        // Bordure setup
        recalage.borderListAdd(0,0   ,-1500,0   ,1500);
        recalage.borderListAdd(1,0   ,1500 ,2000,1500);
        recalage.borderListAdd(0,2000,1500 ,2000,-1500);
        recalage.borderListAdd(1,2000,-1500,0   ,-1500);

        i = 0;

        // check first if init ok & target speed ok
        while (recalage.isRunning())
        {
            i++;
            if (i == 200)
            {
                tFatal(LOG) << "testRecalage: Test " << tId << ": KO";
            }
            double hz;
            recalage.getSpeedHz(hz);

            if (recalage.isInitDone() && hz >= targetHz)
            {
                tInfo(LOG) << "testRecalage: Test " << tId << ": Init OK with freq" << hz;

                break;
            }
            QThread::msleep(500);
        }

        lidarTestPtr.reset();

    } // END TEST 1


    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";

    QCoreApplication::quit();
    thread_logs.join();
}
