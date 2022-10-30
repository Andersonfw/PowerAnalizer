#include "communication.h"

/*! *********************************************************************************
/// @brief		Construtor
/// @fn			Communication::Communication()
*************************************************************************************/
Communication::Communication()
{
    serial = new QSerialPort();
    connectSerialRXIface = connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
}

/*! *********************************************************************************
/// @fn 		int Communication::ConfigPort(QByteArray Str,long par)
/// @brief		Configura a porta Serial
/// @param[in]  str => QString com o nome da porta serial
*************************************************************************************/
QSerialPort::SerialPortError Communication::ConfigPort(QByteArray Str)
{
    serial->setPortName(Str);
    serial->clearError();
    serial->open(QIODevice::ReadWrite);
    if(!serial->isOpen())
        return serial->error();

    return QSerialPort::NoError;;
}

/*! *********************************************************************************
/// @fn     void Communication::Send(QString *data)
/// @brief  Escreve na porta serial
/// @param  QString *data => dado a ser enviado
*************************************************************************************/
int Communication::Send(QString *data)
{
    qint64 bytesWritten;
    QSerialPort::SerialPortError result;
    QByteArray dados;
    dados = data->toLocal8Bit();

    serial->clearError();
    serial->clear(QSerialPort::AllDirections);
    bytesWritten = serial->write(dados);
    result = serial->error();

    if(bytesWritten < dados.length())
        return -1;

    return result;
}

/*! *********************************************************************************
/// @fn     void Communication::ClosePort()
/// @brief  Fecha a porta serial
*************************************************************************************/
void Communication::ClosePort()
{
    if(serial->isOpen())
        serial->close();
}


/*! *********************************************************************************
/// @fn     void Communication::readData()
/// @brief Slot para recepção de dados pela porta serial
*************************************************************************************/
void Communication::readData()
{
    buf.append(serial->readAll());
    if(buf.contains("\r"))
        emit Rx_Finish();
}


/*! *********************************************************************************
/// @brief		Destrutor da classe Communication
*************************************************************************************/
Communication::~Communication()
{
    disconnect(connectSerialRXIface);
    delete serial;
    serial = nullptr;
}

