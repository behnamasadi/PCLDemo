#include "registration.h"

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Registration w;
    w.show();
    return a.exec();
}
