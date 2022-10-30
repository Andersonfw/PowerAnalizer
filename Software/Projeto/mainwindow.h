#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "communication.h"
#include <QtSerialPort/QSerialPortInfo>
#include <QtSerialPort/QSerialPort>
#include <QMessageBox>
#include <QDateTime>
#include <QFile>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <windows.h>
#include <QByteArray>
#include <QMessageBox>
#include <QMutex>
#include <QTimer>
#include <QFileDialog>
#include <QStandardPaths>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QMessageBox ERRO;
    QMessageBox Action;
    QMessageBox LED_Status;

private slots:

    void Reset_Test();

    void Task();

    void on_Conect_Button_clicked();

    void on_Refresh_Button_clicked();

    void on_Action_Button_clicked();

    void on_Clear_Button_clicked();

    void loadSerialList();

    void dataUpDatecomport(); //Recepção da Porta Serial

    void timeout_serial(); //Timeout da serial

private:
    Communication *comport;
    Ui::MainWindow *ui;

    enum
    {
        EROOR_NONE,
        ERROR_COMMUNICATION,
        ERROR_HARDWARE,
        ERROR_ID,
        TIMEOUT_ERROR,
        ERROR_COM_PORT

    };

    void Send_Message(QString info_to_send);
    void BoxError( QString text);
    void BoxWarning( QString text);
    void BoxtInfo( QString text);
    bool saveNewFile();


    QString Msg_5Hz = "Conecte todos os cabos do gerador de sinais de teste aos cabos do módulo a ser testado, atentando para as cores correspondentes."
                          "\r\n MV      ->    JIGA TESTE \r\n VM      ->    VM \r\n PR       ->    PR \r\n BR & AZ ->    Azul(5Hz)"
                          "\r\n\r\nEm seguida clique no botão 'Avançar'.\r\n\n"
                           "ATENÇÂO!!! Verifique se ambos os Leds estão piscando.";

    QString Msg_5HzHT = "Conecte todos os cabos do gerador de sinais de teste aos cabos do módulo a ser testado, atentando para as cores correspondentes."
                      "\r\n MV      ->    JIGA TESTE \r\n BR & AZ ->    Azul(5Hz)"
                      "\r\n\r\nEm seguida clique no botão 'Avançar'.\r\n\n"
                      "ATENÇÂO!!! Verifique se ambos os Leds estão piscando.";

    QString Msg_500Hz = "TESTE 5Hz OK!!! \r\n\n"
                        "Conecte agora o cabo Branco(480Hz) da JIGA nos cabos BR e AZ do MV."
                        "\r\n\r\nEm seguida clique no botão 'Avançar'.\r\n\n"
                         "ATENÇÂO!!! Verifique se ambos os Leds estão piscando.\r\n";

    QString Msg_ACC = "TESTE 480Hz OK!!!  \r\n\nTESTE do Acelerômetro!!! \r\n\n"
                      "\r\nDesconecte os cabos Azul e Branco e desligue a alimentação do módulo.\r\n\""
                      "Em seguida volte a energizá-lo e agite o módulo para validar o acelerômetro\r\n\n"
                      "Verifique se o Led LD1 irá piscar.\r\n\n"
                      "ATENÇÂO!!! Não encerre o teste nesta etapa.";

    QString Instrucao_erro[5] = {"",
                                     "Verificar se a porta COM selecionada é a correta ou se o Gateway não foi desconectado do computador.",
                                     "Verifique se as conexões estão corretas ou se a PCI possui problemas de Hardware.",
                                     "Verifique se o ID inserido é o correto.",
                                     "TIMEOUT!!!. Verifique se o equipamento está ligado corretamente."
                                };


    QString str_rx;

    QString id;

    QTimer *Timer_Resposta;

     QTimer *timer;

    bool Run = false;

    int int_Erro = 0;
    int State;
    QString SendMsg;
    QString filename;
    QFile file;
    QDateTime  Time_old;
    int timeRun = 0;
    bool infinite = false;

    int OldOdom;
    int NewOdom;
    int OldACC;
    int NewACC;

    bool OdomTest;
    bool ACCTest;
    bool Test500Hz;
    bool timerON;
    bool stopLog;

signals:
    void Test_Init();

};

#endif // MAINWINDOW_H
