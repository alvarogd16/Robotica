#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include <QTimer>
#include "ui_counterDlg.h"
#include <iostream>

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();
    QTimer *timer;

    public slots:
        void doButton();
        void incLCD();
        void printHello(int cont);

    signals:
        void itsTen(int cont);

    private:
        int cont = 0;
        bool activado = true;
};

#endif // ejemplo1_H
