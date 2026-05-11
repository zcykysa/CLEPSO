#include "MainWindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);//创建一个应用程序对象 有且只有一个
    MainWindow w;//创建一个窗口对象
    w.show();//显示窗口
    return a.exec();//让应用程序对象进入消息循环
}
