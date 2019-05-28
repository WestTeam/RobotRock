// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <iostream>
#include <math.h>

#include <QString>
#include <QVector>
#include <WestBot/HumanAfterAll/Category.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;


class LidarCircle {
public:
    struct Obstacle {
        QVector<float>	x;
        QVector<float>	y;
        float			X = 0;
        float			Y = 0;
        float			Q = 0;
    };

    LidarCircle(float diam) :DIAM(diam),DMAX(diam*1.1) {}

    QList<Obstacle> compute( const QVector<float>& a, const QVector<float>& d) {
        QVector<float> x;
        QVector<float> y;
        for(int i=a.length()/2; i<a.length(); i++) {
            if(d[i]==0)
                continue;
            float x1 = d[i]*cosf(a[i]);
            float y1 = d[i]*sinf(a[i]);
            if( x1>ROBOT_X0 && x1<ROBOT_X1 && y1>ROBOT_Y0 && y1<ROBOT_Y1)
                continue;
            x << x1;
            y << y1;
        }
        for(int i=0; i<a.length()/2; i++) {
            if(d[i]==0)
                continue;
            float x1 = d[i]*cosf(a[i]);
            float y1 = d[i]*sinf(a[i]);
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

private:
    const float DIAM;
    const float DMAX;
    const float DMIN = 30;
    const float ROBOT_X0 = -1000;
    const float ROBOT_Y0 = -1000;
    const float ROBOT_X1 = 20;
    const float ROBOT_Y1 = 1000;
};
