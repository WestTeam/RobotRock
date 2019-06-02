#ifndef WESTBOT_ROBOTROCK_LIDARCIRCLE_HPP_
#define WESTBOT_ROBOTROCK_LIDARCIRCLE_HPP_

#include <QCoreApplication>
#include <QString>
#include <QFile>
#include <QVector>
#include <QDebug>
#include <math.h>


class LidarCircle {
public:
    struct Obstacle {
        QVector<float>	x;
        QVector<float>	y;
        float			X = 0;
        float			Y = 0;
        float			Q = 0;
        enum Type {
            BALISE0,
            BALISE1,
            BALISE2,
            BALISE3,
            BALISE4,
            BALISE5,
            ROBOT,
            UNKNOWN,
        }type = UNKNOWN;
    };

    LidarCircle(float diam,float lidarX,float lidarY,float lidarSign) :DIAM(diam),DMAX(diam*1.1),LIDAR_X(lidarX),LIDAR_Y(lidarY),LIDAR_SIGN(lidarSign) {}

    QList<Obstacle> compute( const QVector<float>& a, const QVector<float>& d) {
        QVector<float> x;
        QVector<float> y;
        for(int i=0; i<a.length(); i++) {
            if(d[i]==0)
                continue;
            float x1 = d[i]*cosf(LIDAR_SIGN*a[i]);
            float y1 = d[i]*sinf(LIDAR_SIGN*a[i]);
            if( x1>ROBOT_X0 && x1<ROBOT_X1 && y1>ROBOT_Y0 && y1<ROBOT_Y1)
                continue;
            x << x1;
            y << y1;
        }

        QList<Obstacle> obs;
        {
            int i = 0;
            do {
                Obstacle o;
                do {
                    o.x << x[i];
                    o.y << y[i];
                    i++;
                }while( i<x.length() && (o.x.last()-x[i])*(o.x.last()-x[i])+(o.y.last()-y[i])*(o.y.last()-y[i]) < DMIN*DMIN );
                obs << o;
            }while(i<x.length());
        }

        if((obs.first().x.first()-obs.last().x.last())*(obs.first().x.first()-obs.last().x.last())+(obs.first().y.first()-obs.last().y.last())*(obs.first().y.first()-obs.last().y.last())<DMIN*DMIN) {
            obs.last().x << obs.first().x;
            obs.last().y << obs.first().y;
            obs.removeFirst();
        }

        for(int o=0; o<obs.length(); o++) {
            #define dtmp sqrtf( (obs[o].x.first()-obs[o].x.last())*(obs[o].x.first()-obs[o].x.last())+(obs[o].y.first()-obs[o].y.last())*(obs[o].y.first()-obs[o].y.last()) )
            while( dtmp > DMAX && dtmp < DMAX*3.0 ) {
                float D = 0;
                int I = 0;
                for(int i=1; i<obs[o].x.length(); i++) {
                    float d = (obs[o].x[i]-obs[o].x[i-1])*(obs[o].x[i]-obs[o].x[i-1])+(obs[o].y[i]-obs[o].y[i-1])*(obs[o].y[i]-obs[o].y[i-1]);
                    if(d>D) {
                        D = d;
                        I = i;
                    }
                }
                Obstacle tmp = obs[o];
                for(int i=I; i<tmp.x.length(); i++) {
                    obs[o].x.removeLast();
                    obs[o].y.removeLast();
                }
                for(int i=0; i<I; i++) {
                    tmp.x.removeFirst();
                    tmp.y.removeFirst();
                }
                obs << tmp;
            }
        }

        for(int o=0; o<obs.length(); o++) {
            if( dtmp>DMAX*3.0 || obs[o].x.length()<4 )
                continue;
            float N = 0;
            for(int i=0; i<obs[o].x.length(); i++) {
                for(int j=i+1; j<obs[o].x.length(); j++) {
                    float xi = obs[o].x[i];
                    float yi = obs[o].y[i];
                    float xj = obs[o].x[j];
                    float yj = obs[o].y[j];
                    float a = (-yi+yj) / (xi-xj);
                    float b = (xi*xi-xj*xj+yi*yi-yj*yj) /(xi-xj) /2.0;
                    float a2 = a*a+1;
                    float b2 = 2.0*a*(b-xi)-2.0*yi;
                    float c2 = (b-xi)*(b-xi) +yi*yi -(DIAM/2.0)*(DIAM/2.0);
                    float d2 = b2*b2-4*a2*c2;
                    if(d2<=0)
                        continue;
                    float y1 = -(b2-sqrtf(d2))/2.0/a2;
                    float y2 = -(b2+sqrtf(d2))/2.0/a2;
                    float x1 = a*y1+b;
                    float x2 = a*y2+b;
                    if(x1*x1+y1*y1 > x2*x2+y2*y2) {
                        obs[o].X += x1;
                        obs[o].Y += y1;
                    }
                    else {
                        obs[o].X += x2;
                        obs[o].Y += y2;
                    }
                    N += 1;
                }
            }
            obs[o].X /= float(N);
            obs[o].Y /= float(N);
            for(int i=0; i<obs[o].x.length(); i++) {
                float q = sqrtf( (obs[o].x[i]-obs[o].X)*(obs[o].x[i]-obs[o].X) + (obs[o].y[i]-obs[o].Y)*(obs[o].y[i]-obs[o].Y) )
                        - (DIAM/2.0);
                obs[o].Q += q*q;
            }
            obs[o].Q = float(obs[o].x.length())*float(obs[o].x.length())/obs[o].Q *2.0;
            //qDebug() << o << obs[o].x.length() << obs[o].X << obs[o].Y << obs[o].Q;
        }

        return obs;
    }

    void transformToRobot( QList<Obstacle>& obs) {
        for(int o=0; o<obs.length(); o++) {
            obs[o].X += LIDAR_X;
            obs[o].Y += LIDAR_Y;
        }
    }

    void transformToAbs( QList<Obstacle>& obs, float x, float y, float theta) {
        for(int o=0; o<obs.length(); o++) {
            float X = obs[o].X*cosf(theta)-obs[o].Y*sinf(theta)+x;
            float Y = obs[o].X*sinf(theta)+obs[o].Y*cosf(theta)+y;
            obs[o].X = X;
            obs[o].Y = Y;
        }
    }

    void associate( QList<Obstacle>& obs) {
        for(int o=0; o<obs.length(); o++) {
            float D = 10000;
            int I = 0;
            for(int i=0; i<6; i++) {
                if(sqrtf((obs[o].X-BALISE_X[i])*(obs[o].X-BALISE_X[i])+(obs[o].Y-BALISE_Y[i])*(obs[o].Y-BALISE_Y[i]))<D) {
                    D = sqrtf((obs[o].X-BALISE_X[i])*(obs[o].X-BALISE_X[i])+(obs[o].Y-BALISE_Y[i])*(obs[o].Y-BALISE_Y[i]));
                    I = i;
                }
            }
            if(D<BALISE_DMAX) {
                obs[o].type = Obstacle::Type(I);
            }
        }
        for(int i=0; i<6; i++) {
            float D = 10000;
            int O = 0;
            for(int o=0; o<obs.length(); o++) {
                if(obs[o].type!=i)
                    continue;
                if(sqrtf((obs[o].X-BALISE_X[i])*(obs[o].X-BALISE_X[i])+(obs[o].Y-BALISE_Y[i])*(obs[o].Y-BALISE_Y[i]))<D) {
                    D = sqrtf((obs[o].X-BALISE_X[i])*(obs[o].X-BALISE_X[i])+(obs[o].Y-BALISE_Y[i])*(obs[o].Y-BALISE_Y[i]));
                    O = o;
                }
            }
            for(int o=0; o<obs.length(); o++) {
                if(obs[o].type==i&&o!=O)
                    obs[o].type = Obstacle::UNKNOWN;
            }
        }
        for(int o=0; o<obs.length(); o++) {
            if(obs[o].type != Obstacle::UNKNOWN)
                continue;
            if(obs[o].X>TABLE_X0&&obs[o].X<TABLE_X1&&obs[o].Y>TABLE_Y0&&obs[o].Y<TABLE_Y1)
                obs[o].type = Obstacle::ROBOT;
        }
    }


    const float BALISE_X[6] = {50,1000,1950,1950,1000,50};
    const float BALISE_Y[6] = {-1594,-1594,-1594,1594,1594,1594};
private:
    const float DIAM;
    const float DMAX;
    const float DMIN = 30;
    const float ROBOT_X0 = -300;
    const float ROBOT_Y0 = -160;
    const float ROBOT_X1 = 20;
    const float ROBOT_Y1 = 160;
    const float TABLE_X0 = 0+100;
    const float TABLE_Y0 = -1500+100;
    const float TABLE_X1 = 1578-100;
    const float TABLE_Y1 = 1500-100;

    const float LIDAR_X;
    const float LIDAR_Y;
    const float LIDAR_SIGN = 1;

    const float BALISE_DMAX = 300;
};
/*
int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    QString file = "setup0-lidardata0.txt";
    //QString file = "setup3-lidardata0.txt";
    QFile f("../"+file);
    f.open(QIODevice::ReadOnly);
    QVector<float> d;
    QVector<float> a;
    QVector<float> q;
    while(!f.atEnd()) {
        QString data = f.readLine();
        QStringList dataList = data.split(';');
        d << dataList[0].toDouble();
        a << dataList[1].toDouble();
        q << dataList[2].toDouble();
    }
    f.close();

    LidarCircle circle(75,100,0,-1);
    QList<LidarCircle::Obstacle> obs = circle.compute(a,d);
    circle.transformToRobot(obs);
    circle.transformToAbs(obs,675,-1500+450-52-172.8,M_PI/2);

    /*for(int i=0; i<2; i++) {
        LidarCircle::Obstacle o;
        o.X = circle.BALISE_X[i];
        o.Y = circle.BALISE_Y[i];
        obs << o;
    }
    circle.transformToAbs(obs,50,-40,0);
    circle.transformToAbs(obs,0,0,.05);

    circle.associate(obs);


    for(int o=0; o<obs.length(); o++)
        if(obs[o].Q!=0)
            qDebug() << o << obs[o].x.length() << obs[o].X << obs[o].Y << obs[o].Q << obs[o].type;


    return app.exec();
}
}*/


#endif
