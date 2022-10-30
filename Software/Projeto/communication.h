#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <QThread>
#include <QObject>
#include <QTimer>
#include <QSerialPort>
#include <QMutex>
//#include "types.h"
//#include "crc16.h"

class Communication : public QObject
{
    Q_OBJECT
public:
    Communication();
    ~Communication();
    QSerialPort::SerialPortError ConfigPort(QByteArray Str);
    void ClosePort();

    int Send(QString *data);
    QSerialPort *serial;

    QByteArray buf;

public slots:
    void readData();

private:
    QMetaObject::Connection connectSerialRXIface;
signals:
    void Rx_Finish();
};




#endif // COMMUNICATION_H
