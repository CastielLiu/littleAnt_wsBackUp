#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QStringListModel>
#include <QStackedWidget>
#include <unistd.h>

#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>

#include <string>
#include <cstring>
#include <sstream>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


private slots:





    void on_pushButton_start_clicked();

    void on_pushButton_stop_clicked();

    void on_pushButton_rtk_clicked();

private:
    Ui::MainWindow *ui;





};

#endif // MAINWINDOW_H
