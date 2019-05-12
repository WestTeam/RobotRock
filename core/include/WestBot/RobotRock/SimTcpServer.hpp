// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_SIMTCPSERVER_HPP_
#define WESTBOT_ROBOTROCK_SIMTCPSERVER_HPP_

#include <memory>

#include <QHash>
#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

namespace WestBot {
namespace RobotRock {

class SimTcpServer : public  QTcpServer
{
    Q_OBJECT

public:
    using SocketPtr = std::shared_ptr< QTcpSocket >;

    SimTcpServer( QObject* parent = nullptr );
    ~SimTcpServer() override = default;

    void disconnectClient( const SocketPtr& socket );

    void showConnectionInformation() const;

signals:
    void error( QTcpSocket::SocketError socketError );

protected:
    void incomingConnection( qintptr socketDescriptor ) override;

private:
    void sendSimData( const SocketPtr& socket ); // TODO: XXX

private:
    QHash< QObject*, SocketPtr > _clients;
    QTimer _pushTimer;
};

}
}

#endif // WESTBOT_ROBOTROCK_SIMTCPSERVER_HPP_

