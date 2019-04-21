#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include<stdlib.h>



//#pragma execution_character_set("utf-8")

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    setWindowIcon(QIcon(":/images/seu.png"));
    ui->setupUi(this);

    this->setMaximumSize(380,315);
    this->setMinimumSize(380,315);


}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_start_clicked()
{
    system("gnome-terminal -e ./start.sh");
}

void MainWindow::on_pushButton_stop_clicked()
{
    system("gnome-terminal -e ./stop.sh");
}

void MainWindow::on_pushButton_rtk_clicked()
{
    system("gnome-terminal -x ~/Desktop/rtcm3.2/main /dev/ttyUSB0");
}
