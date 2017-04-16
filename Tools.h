#pragma once
#include <string>
#include <QTime>
#include <QString>

class Tools
{
public:
	Tools();
	~Tools();



};

std::string getFileName(std::string file_name);
void timeStart();
QString timeOff();

static QString windows_qss = "QWidget{ 	 	background-color: rgb(240, 240, 240); }  QDockWidget{ 	color: rgb(0, 0, 0);	 	background-color: rgb(240, 240, 240);  	border-color: rgb(63, 63, 70); 	border-top-color: rgb(0, 0, 0);	 	font: 10pt \"Microsoft YaHei UI\"; }  QTableWidget{	 	background-color: rgb(255, 255 ,255);	 	border-color: rgb(130, 135, 140); 	color: rgb(0, 0, 0); 	alternate-background-color: rgb(218, 218, 218); 	font: 9pt \"Microsoft YaHei UI\"; }  QTreeWidget{ 	background-color: rgb(255, 255 ,255);	 	border-color: rgb(130, 135, 140); 	color: rgb(0, 0, 0); 	alternate-background-color: rgb(218, 218, 218); 	font: 9pt \"Microsoft YaHei UI\"; }   QToolBar{ 	background-color: rgb(240, 240, 240); 	border-bottom: 1px solid #828790; }  QStatusBar{ 	color: rgb(0, 0, 0); 	font: 9pt \"Microsoft YaHei UI\"; }  QMenuBar{ 	background-color: rgb(240, 240, 240); 	color: rgb(0, 0, 0); 	font: 9pt \"Microsoft YaHei UI\"; 	border-bottom: 1px solid #828790; }  QMenuBar::item:selected{ 	background-color: rgb(205, 233, 255); 	 }  QMenu{ 	font: 9pt \"Microsoft YaHei UI\";	 	color: rgb(0, 0, 0); 	background-color: rgb(241, 241, 241); }  QMenu::item:selected{ 	background-color: rgb(145, 201, 247); }  QLabel{ 	color: rgb(0, 0, 0); 	font: 10pt \"Microsoft YaHei UI\"; }  QCheckBox{ 	color: rgb(0, 0, 0); 	font: 10pt \"Microsoft YaHei UI\"; }  QLCDNumber{ 	color: rgb(0, 0, 0); 	font: 10pt \"Microsoft YaHei UI\"; }  QPushButton{ 	color: rgb(0, 0, 0); 	 	background-color: rgb(226, 230, 234); } ";
static QString darcula_qss = "QWidget{ 	 	background-color: rgb(60, 63, 65); }  QDockWidget{ 	color: rgb(208, 208, 208);	 	background-color: rgb(60, 63, 65);  	border-color: rgb(63, 63, 70); 	border-top-color: rgb(255, 255, 255);	 	font: 10pt \"Microsoft YaHei UI\"; }  QTableWidget{	 	background-color: rgb(43, 43, 43);	 	border-color: rgb(63, 63, 70); 	color: rgb(241, 241, 241); 	alternate-background-color: rgb(85, 85, 85); 	font: 9pt \"Microsoft YaHei UI\"; }  QTreeWidget{ 	background-color: rgb(43, 43, 43);	 	border-color: rgb(63, 63, 70); 	color: rgb(241, 241, 241); 	alternate-background-color: rgb(85, 85, 85); 	font: 9pt \"Microsoft YaHei UI\"; }   QHeaderView::section{ 	background-color: rgb(53, 53, 53); 	color: rgb(241, 241, 241); 	border:0px solid #E0DDDC; 	border-bottom:1px solid #262626; 	height: 30px; }   QToolBar{ 	background-color: rgb(60, 63, 65); 	border-bottom: 1px solid #262626; }  QStatusBar{ 	color: rgb(241, 241, 241); 	font: 9pt \"Microsoft YaHei UI\"; }  QMenuBar{ 	background-color: rgb(60, 63, 65); 	color: rgb(241, 241, 241); 	font: 9pt \"Microsoft YaHei UI\"; 	border-bottom: 1px solid #262626; }  QMenuBar::item:selected{ 	background-color: rgb(75, 110, 175); }  QMenu{ 	font: 9pt \"Microsoft YaHei UI\";	 	color: rgb(241, 241, 241); 	background-color: rgb(60, 63, 65); }  QMenu::item:selected{ 	background-color: rgb(75, 110, 175); }   QLabel{ 	color: rgb(241, 241, 241); 	font: 10pt \"Microsoft YaHei UI\"; }  QCheckBox{ 	color: rgb(241, 241, 241); 	font: 10pt \"Microsoft YaHei UI\"; }  QLCDNumber{ 	color: rgb(241, 241, 241); 	font: 10pt \"Microsoft YaHei UI\"; }  QPushButton{ 	color: rgb(241, 241, 241); 	 	background-color: rgb(73, 78, 80); } ";


