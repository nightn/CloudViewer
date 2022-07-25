#include "CloudViewer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  CloudViewer w;
  w.show();
  return a.exec();
}

