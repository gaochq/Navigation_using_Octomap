#include <QtGui/QApplication>
#include "orb_slam2_dense.h"


int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    orb_slam2_dense foo;
    foo.show();
    return app.exec();
}
