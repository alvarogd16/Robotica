#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );

    timer = new QTimer();
    connect(timer, SIGNAL( timeout()), this, SLOT(incLCD()));
    timer->start(100);

    connect(this, SIGNAL(itsTen(int)), this, SLOT(printHello(int)));
}

void ejemplo1::doButton()
{
	activado = !activado;
}

void ejemplo1::incLCD() {
    if(activado){
        lcdNumber->display(cont);
        cont++;

        if(cont == 10)
            emit itsTen(cont);
    }
}

void ejemplo1::printHello(int cont) {
    std::cout << "the number" << cont << std::endl;
}




