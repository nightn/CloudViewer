#ifndef ABOUTWIN_H
#define ABOUTWIN_H

#include <QWidget>
#include <QDialog>
#include "ui_AboutWin.h"
#include "GBK.h"

class AboutWin : public QDialog
{
	Q_OBJECT

public:
	AboutWin(QWidget *parent = 0);
	~AboutWin();

private:
	Ui::AboutWin ui;
};

#endif // ABOUTWIN_H
