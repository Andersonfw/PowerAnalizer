#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <unistd.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowTitle("PowerAnalyzer 1.00");

    Action.setWindowTitle("Instrução");
    Action.addButton(QMessageBox::Ok)->setText("Ok");
    Action.setIcon(QMessageBox::Warning);
    Action.setMinimumHeight(120);
    Action.setMinimumWidth(500);
    Action.setMaximumHeight(120);
    Action.setMaximumWidth(500);
    Action.setGeometry(QStyle::alignedRect(Qt::LeftToRight,Qt::AlignCenter,Action.size(),geometry()));

    ERRO.setWindowTitle("ERRO!!!");
    ERRO.setStandardButtons(QMessageBox::Ok);
    ERRO.setIcon(QMessageBox::Critical);
    ERRO.setGeometry(QStyle::alignedRect(Qt::LeftToRight,Qt::AlignCenter,ERRO.size(),geometry()));

    LED_Status.setWindowTitle("Teste do LED");
    LED_Status.addButton(QMessageBox::Ok)->setText("Sim");
    LED_Status.addButton(QMessageBox::No)->setText("Não");
    LED_Status.setDefaultButton(QMessageBox::Ok);

    stopLog = false;

    //Configura o Timer de Resposta
    Timer_Resposta = new QTimer();
    Timer_Resposta->setSingleShot(true);
    Timer_Resposta->setInterval(1500);
    connect(Timer_Resposta, SIGNAL(timeout()), this, SLOT(timeout_serial()));

    //Configura o Timer de Refresh
    timer = new QTimer();
    timer->setSingleShot(false);
    timer->setInterval(500);
    connect(timer, SIGNAL(timeout()), this, SLOT(Task()));

    //Adiciona todas as portas COM existentes no computador à lista de portas
    loadSerialList();
    comport = new Communication(); //Instancia comport (objeto da classe comunication),
    //que será a porta serial usada

    connect(comport, SIGNAL(Rx_Finish()),this,SLOT(dataUpDatecomport()));

}

MainWindow::~MainWindow()
{
    delete ui;
}
/*! **********************************************************************************
 * @fn    MainWindow::BoxError( QString text)
 * @brief Abre uma janela com mensagem de erro
 *************************************************************************************/
void MainWindow::BoxError( QString text)
{
    ERRO.setIcon(QMessageBox::Critical);
    ERRO.setText(text);
    ERRO.setGeometry(QStyle::alignedRect(
                         Qt::LeftToRight,
                         Qt::AlignCenter,
                         ERRO.size(),
                         geometry()));
    ERRO.exec();
}
/*! **********************************************************************************
 * @fn    MainWindow::BoxWarning( QString text)
 * @brief Abre uma janela com mensagem de warning
 *************************************************************************************/
void MainWindow::BoxWarning( QString text)
{
    Action.setWindowTitle("Instrução");
    Action.setIcon(QMessageBox::Warning);
    Action.setText(text);
    Action.setGeometry(QStyle::alignedRect(
                           Qt::LeftToRight,
                           Qt::AlignCenter,
                           Action.size(),
                           geometry()));
    Action.exec();
}
/*! **********************************************************************************
 * @fn    MainWindow::BoxtInfo( QString text)
 * @brief Abre uma janela com mensagem de informação
 *************************************************************************************/
void MainWindow::BoxtInfo( QString text)
{
    Action.setWindowTitle("Informação");
    Action.setIcon(QMessageBox::Information);
    Action.setText(text);
    Action.setGeometry(QStyle::alignedRect(
                           Qt::LeftToRight,
                           Qt::AlignCenter,
                           Action.size(),
                           geometry()));
    Action.exec();
}
/*! **********************************************************************************
 * @fn    MainWindow::loadSerialList()
 * @briefLista as portas seriais Disponiveis
 *************************************************************************************/
void MainWindow::loadSerialList()
{
    ui->COM_name->clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        ui->COM_name->addItem(info.portName());
}
/*! **********************************************************************************
 * @fn    void on_Conect_Button_clicked()
 * @brief Conecta ou Desconecta uma porta Serial
 *************************************************************************************/
void MainWindow::on_Conect_Button_clicked()
{
    int conection_status;

    if((ui->Conect_Button->text()) == "Conectar")
    {

        //Pega o nome da COM que será conectada
        QByteArray portname= (ui->COM_name->currentText()).toLocal8Bit();

        //Tenta conectar-se a uma porta COM
        conection_status = comport->ConfigPort(portname);


        switch (conection_status) //Pega o status da conexão
        {
        case QSerialPort::NoError:
            //comport->StartRX();
            ui->Conect_Button->setText("Desconectar");
            break;

        case QSerialPort::PermissionError:
            BoxError("A porta está sendo usada por outra aplicação.");
            break;


        case QSerialPort::DeviceNotFoundError:
            BoxError("A porta não existe.");
            break;

        default:
            break;
        }
    }
    else
    {
        ui->Conect_Button->setText("Conectar");
        comport->ClosePort();
    }
}

/*! **********************************************************************************
 * @fn    void on_Refresh_Button_clicked()
 * @brief Atualiza lista de Portas Seriais
 *************************************************************************************/
void MainWindow::on_Refresh_Button_clicked()
{
    loadSerialList();
}

/*! **********************************************************************************
 * @fn    on_Action_Button_clicked()
 * @brief Executa as tarefas do teste
 *************************************************************************************/
void MainWindow::on_Action_Button_clicked()
{
    if (comport->serial->isOpen())
    {
        if((ui->Action_Button->text()) == "Iniciar" )
        {
            int timeSleep = ui->timedelay->value();

            filename = ui->lineEdit->text();
            ui->lineEdit->setEnabled(FALSE);
            ui->timerun->setEnabled(FALSE);
            ui->timedelay->setEnabled(FALSE);
            Run = TRUE;

            if(!filename.endsWith(".csv",Qt::CaseInsensitive))
                filename.append(".csv");

            saveNewFile();
            if(file.isOpen())
            {
                QString str = QString("%1").arg("Tempo(s);Tensão(mV);I_Shunt(mA);I_Hall(mA);P_Shunt(W);P_Shunt(W);\r\n");
                str.replace(".",",");
                file.write(str.toLocal8Bit(),str.toLocal8Bit().length());
            }
            ui->Action_Button->setText("Parar");
            ui->RX_TextEdit->setTextColor(QColor("black"));
            ui->RX_TextEdit->setFontPointSize(18);
            ui->RX_TextEdit->setText("Iniciando LOG!!!\n");
            timeRun = ui->timerun->value();
            Time_old = QDateTime::currentDateTime();
            if(timeRun <= 0)
                infinite = true;
            SendMsg = "";
            SendMsg.append("#\r\n");
            Send_Message(SendMsg);
            timer->start(timeSleep * 1000);

        }

        else
        {
            ui->Action_Button->setText("Iniciar");
            ui->lineEdit->setEnabled(TRUE);
            ui->timerun->setEnabled(TRUE);
            ui->timedelay->setEnabled(TRUE);
            file.close();
            timer->stop();
            int_Erro = EROOR_NONE;
            Reset_Test();
            Run = FALSE;
        }
    }
    else
    {
        ui->Action_Button->setText("Iniciar");
        int_Erro = ERROR_COM_PORT;
        Reset_Test();
    }


}
/*! **********************************************************************************
 * @fn    on_Clear_Button_clicked()
 * @brief Limpa a tela principal
 *************************************************************************************/
void MainWindow::on_Clear_Button_clicked()
{
    ui->RX_TextEdit->clear();
    ui->RX_TextEdit->setText("");
}
/*! *********************************************************************************
* @fn     void MainWindow::Send_Message(QString info_to_send)
* @brief  Efetua o Cálculo do CRC e envia a mensagem pela porta serial
* @param[in]  QString info_to_send => mensagem a ser enviada.
*************************************************************************************/
void MainWindow::Send_Message(QString info_to_send)
{

    //Envia a mensagem pela porta serial
    timerON = true;
    if(Timer_Resposta->isActive())
        Timer_Resposta->stop();

    Timer_Resposta->start();
    if(comport->Send(&info_to_send) != QSerialPort::NoError)
        return;
}
///
/// \brief DataSource::saveNewFile
/// \return
///
bool MainWindow::saveNewFile()
{
    QString name = filename;
    filename =  QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) +"//"+ QDateTime::currentDateTime().toString("dd-MM-yy-hh-mm-ss")+ " " + name;
    if(!filename.isNull())
    {
        file.setFileName(filename);
        if (file.open(QIODevice::WriteOnly))
            return true;
    }
    return false;
}
/*! *********************************************************************************
* @fn     void MainWindow::Reset_Test()
* @brief  Verifica se ocooreu algum erro e reinia o teste
*************************************************************************************/
void MainWindow::Reset_Test()
{
    QString Teste_Status;

    if(int_Erro == EROOR_NONE)
        BoxtInfo("Log Finalizado!!!\r\n");


    else
    {
        if (int_Erro == ERROR_COM_PORT)
        {
            Teste_Status = "Conecte-se a uma porta COM antes de prosseguir.\r\n\r\n";
        }

        else Teste_Status = "Ocorreu um erro na execução do log."
                            "\r\n\r\n" + Instrucao_erro[int_Erro] + "\r\n\r\n";

        BoxError(Teste_Status);
    }


    int_Erro = EROOR_NONE;
    Run = FALSE;
    stopLog = false;
    ui->Action_Button->setText("Iniciar");
    ui->lineEdit->setEnabled(TRUE);
    ui->timerun->setEnabled(TRUE);
    ui->timedelay->setEnabled(TRUE);
    file.close();
}
/*! *********************************************************************************
* @fn     void MainWindow::Task()
* @brief  tarefa inicial do teste
*************************************************************************************/
void MainWindow::Task()
{
    if (comport->serial->isOpen() && Run)
    {


        if(!infinite)
        {
            timeRun -= ui->timedelay->value();
            if(timeRun <= 0)
            {
                timer->stop();
                stopLog = true;
                //                int_Erro = EROOR_NONE;
                //                Reset_Test();
                //return;
            }
        }
        SendMsg = "";
        SendMsg.append("#\r\n");
        Send_Message(SendMsg);
    }
    else
    {
        ui->Action_Button->setText("Iniciar");
        int_Erro = ERROR_COM_PORT;
        timer->stop();
        Reset_Test();
        return;
    }
    return;
}

/*! **********************************************************************************
 * @fn    void timeout_serial()
 * @brief Se estourar o timer e a flag ainda estiver setada
 * @brief é pq não foi recebido resposta na serial.
 *************************************************************************************/
void MainWindow::timeout_serial()
{
    if(timerON)
    {
        timer->stop();
        int_Erro = TIMEOUT_ERROR;
        Reset_Test();
    }
}

/*! **********************************************************************************
 * @fn    void dataUpDatecomport()
 * @brief Funcao executada apos a sinalizacao RX_Finish (/dev/ttyS2)()
 * @brief le os dados da serial comport e solicita a atualizacao
 * @brief da janela de recepcao
 *************************************************************************************/
void MainWindow::dataUpDatecomport()
{
    QStringList Data_List;
    QString SendMsg;
    int size;
    QString voltage, I_Shunt, I_Hall, Power_Shunt, Power_Hall, Vout;
    QDateTime  Time_Now;
    float Time = 0;

    timerON = false;

    Data_List = QString(comport->buf).replace("\n", "").split(";");
    comport->buf.clear();
    size = Data_List.size();

    for(int j = 0; j<(size); j++)
    {
        str_rx = Data_List[j];
        if (str_rx.size() == 0) continue;

        if   (str_rx.startsWith("V="))
        {
            voltage = str_rx.split("=")[1];
        }

        if   (str_rx.startsWith("I_S="))
        {
            I_Shunt = str_rx.split("=")[1];
        }

        if   (str_rx.startsWith("I_H="))
        {
            I_Hall = str_rx.split("=")[1];
        }

        if   (str_rx.startsWith("P_S="))
        {
            Power_Shunt = str_rx.split("=")[1];
        }

        if   (str_rx.startsWith("P_H="))
        {
            Power_Hall = str_rx.split("=")[1];
        }

        if   (str_rx.startsWith("V_OUT="))
        {
            Vout = str_rx.split("=")[1].replace("\r", "");
        }
    }

    Time_Now = QDateTime::currentDateTime();
    Time = Time_old.msecsTo(Time_Now) /1000;

    ui->RX_TextEdit->setTextColor(QColor("black"));
    ui->RX_TextEdit->setFontPointSize(10);
    ui->RX_TextEdit->append("Tempo: " +  QString::number(Time) +" s");
    ui->RX_TextEdit->append("Tensão na carga = " + voltage + " mV");
    ui->RX_TextEdit->append("Corrente HALL = "+ I_Hall + " mA" );
    ui->RX_TextEdit->append("Corrente SHUNT = " + I_Shunt + " mA");
    ui->RX_TextEdit->append("Potência HALL = " + Power_Hall + " W");
    ui->RX_TextEdit->append("Potência SHUNT = " + Power_Shunt + " W");
    ui->RX_TextEdit->append("Tensão de sáida = " + Vout + " mV\r\n");

    if(file.isOpen())
    {
        QString str = (QString::number(Time) + ";" + voltage + ";" + I_Shunt + ";" +I_Hall + ";" + Power_Shunt + ";" + Power_Hall + "\r\n");
        str.replace(".",",");
        file.write(str.toLocal8Bit(),str.toLocal8Bit().length());
    }

    if(stopLog)
    {
        int_Erro = EROOR_NONE;
        Reset_Test();
    }
}
