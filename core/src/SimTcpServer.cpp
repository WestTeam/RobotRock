// Copyright (c) 2019 All Rights Reserved WestBot

#include <QDebug>
#include <QThread>

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
            if( data == "start" )
            {
                emit startStrat();
            }
            else if( data == "stop" )
            {
                emit stop();

            }
            else if( data == "color0" )
            {
                emit changeColor( DigitalValue::ON );
            }
            else if( data == "color1" )
            {
                emit changeColor( DigitalValue::OFF );
            }
            else
            {
                tDebug( LOG ) << "Client send a data:" << data;
            }
        } );

    emit onClientConnected( tcpSocket );
}

void SimTcpServer::updateClients(SimData &data)
{
    // Push data to all clients
    for( auto& client : _clients )
    {
        this->sendSimData( client, data );
    }
}

void SimTcpServer::sendSimData( const SocketPtr& socket, SimData &data )
{
    socket->write( ( char *) & data, sizeof( SimData ) );
}


void SimTcpServer::sendSimData( const SocketPtr& socket, QList< SimData > datas )
{


    for( auto data : datas )
    {
        /*SimData _data;
        _data.objectId = id;
        //_data.objectPos.x = rand() % 3000 + 1;
        //_data.objectPos.y = rand() % 2000 + 1;
        _data.objectPos.x = data.objectPos.x;
        _data.objectPos.y = data.objectPos.y;
        _data.objectPos.theta = 0;
        _data.objectType = 1;
        _data.objectColor = rand() % 3;
        _data.objectSize = 100.0;
        _data.objectMode = 0;
        id+=1;*/

        socket->write( ( char *) & data, sizeof( SimData ) );
        socket->waitForBytesWritten( 1000 );
        QThread::msleep( 10 );
    }
}

