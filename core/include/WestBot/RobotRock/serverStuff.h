#ifndef SERVERSTUFF_H
#define SERVERSTUFF_H

#include <QTcpServer>
#include <QTcpSocket>
#include <QDataStream>
#include <QList>
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

class ServerStuff : public QObject
{
    Q_OBJECT

public:
    ServerStuff(QObject *pwgt = nullptr );
    QTcpServer *tcpServer;
    QList<QTcpSocket *> getClients();

    void updateClients( SimData& data );
    void send();

public slots:
    virtual void newConnection();
    void readClient();
    void gotDisconnection();
    qint64 sendToClient(QTcpSocket *socket, const QString &str);

signals:
    void gotNewMesssage(QString msg);
    void smbDisconnected();

private:
    void sendSimData( QTcpSocket* socket, SimData &data); // TODO: XXX

private:
    quint16 m_nNextBlockSize;
    QList<QTcpSocket*> clients;
    QTimer _timer;
};

}
}

#endif // SERVERSTUFF_H
