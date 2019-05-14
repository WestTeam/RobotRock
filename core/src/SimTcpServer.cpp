// Copyright (c) 2019 All Rights Reserved WestBot

#include <QNetworkInterface>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/SimTcpServer.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.SimTcpServer" )
}

SimTcpServer::SimTcpServer( QObject* parent )
    : QTcpServer( parent )
{
    _pushTimer.setInterval( 100 );

    connect(
        & _pushTimer,
        & QTimer::timeout,
        this,
        [ this ]()
        {
            // Push data to all clients
            for( auto& client : _clients )
            {
                this->sendSimData( client );
            }
        } );

    _pushTimer.start();
}

void SimTcpServer::disconnectClient( const SocketPtr& socket )
{
    _clients.remove( socket.get() );
}

void SimTcpServer::showConnectionInformation() const
{
    if( isListening() )
    {
        QString ipAddress;
        QList< QHostAddress > ipAddressesList = QNetworkInterface::allAddresses();

        // use the first non-localhost IPv4 address
        for( int i = 0; i < ipAddressesList.size(); ++i )
        {
            if( ipAddressesList.at( i ) != QHostAddress::LocalHost &&
                    ipAddressesList.at( i ).toIPv4Address() )
            {
                ipAddress = ipAddressesList.at( i ).toString();
                break;
            }
        }

        // if we did not find one, use IPv4 localhost
        if( ipAddress.isEmpty() )
            ipAddress = QHostAddress( QHostAddress::LocalHost).toString();
        {
            tDebug( LOG )
                << "The server is running on\n\nIP:" << ipAddress << "\nport:"
                << serverPort();
        }
    }
    else
    {
        tDebug( LOG ) << "No client bound on server.";
    }
}

//
// Protected methods
//
void SimTcpServer::incomingConnection( qintptr socketDescriptor )
{
    SocketPtr tcpSocket =
        std::make_shared< QTcpSocket >( nextPendingConnection() );

    qDebug() << "New incoming connection...";

    if( ! tcpSocket->setSocketDescriptor( socketDescriptor ) )
    {
        emit error( tcpSocket->error() );
        return;
    }

    _clients.insert( tcpSocket.get(), tcpSocket );

    connect(
        tcpSocket.get(),
        & QTcpSocket::connected,
        this,
        [ this, tcpSocket ]()
        {
            sendSimData( tcpSocket );
        } );

    connect(
        tcpSocket.get(),
        & QTcpSocket::disconnected,
        this,
        [ this, tcpSocket ]()
        {
            disconnectClient( tcpSocket );
        } );

    connect(
        tcpSocket.get(),
        & QTcpSocket::readyRead,
        this,
        [ this, tcpSocket ]()
        {
            const QByteArray data = tcpSocket->readAll();
            tDebug( LOG ) << "Client send a data:" << data;
        } );

}

void SimTcpServer::sendSimData( const SocketPtr& socket )
{
    /*
    uint8_t objectId;
    uint8_t objectType;
    uint8_t objectColor;
    RobotPos objectPos; // robotpos = x,y,teta en double
    double objectSize; // utile uniquement pour definir la taille du bras
    uint8_t objectMode; // utile seulement pour les palets
    */

    // For test purpose;
    _data.objectId = 0; // premier truc
    _data.objectType = 0; // un palet
    _data.objectColor = 0; // yellow side
    _data.objectPos.x = 450;
    _data.objectPos.y = 500;
    _data.objectPos.theta = 0;
    _data.objectSize = 0; // dont care
    _data.objectMode = 1; // horizontale

    socket->write( ( char *) & _data, sizeof( SimData ) );
    socket->flush();
}

