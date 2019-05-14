// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_SIMTCPSERVER_HPP_
#define WESTBOT_ROBOTROCK_SIMTCPSERVER_HPP_

#include <memory>

#include <QHash>
#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

#include "Odometry.hpp"

namespace WestBot {
namespace RobotRock {

typedef struct
{
    uint8_t objectId;
    uint8_t objectType;
    uint8_t objectColor;
    RobotPos objectPos; // robotpos = x,y,teta en double
    double objectSize; // utile uniquement pour definir la taille du bras
    uint8_t objectMode; // utile seulement pour les palets
} __attribute__( ( packed ) ) SimData;

bool operator==( SimData& lhs, SimData& rhs );

class SimTcpServer : public  QTcpServer
{
    Q_OBJECT

public:
    using SocketPtr = std::shared_ptr< QTcpSocket >;

    SimTcpServer( QObject* parent = nullptr );
    ~SimTcpServer() override = default;

    void disconnectClient( const SocketPtr& socket );

    void showConnectionInformation() const;

    void updateClients(SimData &data);


signals:
    void error( QTcpSocket::SocketError socketError );

protected:
    void incomingConnection( qintptr socketDescriptor ) override;

private:
    void sendSimData( const SocketPtr& socket, SimData &data); // TODO: XXX

private:
    QHash< QObject*, SocketPtr > _clients;
    QTimer _pushTimer;
    SimData _data;
};

}
}

#endif // WESTBOT_ROBOTROCK_SIMTCPSERVER_HPP_

