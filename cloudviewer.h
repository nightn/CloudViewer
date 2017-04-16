#ifndef CLOUDVIEWER_H
#define CLOUDVIEWER_H

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include "MyCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>

#include <QtWidgets/QMainWindow>
#include "GBK.h"
#include "ui_cloudviewer.h"
#include "AboutWin.h"
#include "Tools.h"

#include <vector>
#include <QtWidgets/QMainWindow>
#include <QString>
#include <QDebug>
#include <QLabel>
#include <QMessageBox>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QColorDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "QVTKWidget.h"
#include <vtkRenderWindow.h>
#include <QTextEdit>
#include <QTime>
#include <QMouseEvent> 
#include <QDesktopServices> 
#include <QUrl>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class CloudViewer : public QMainWindow
{
	Q_OBJECT

public:
	CloudViewer(QWidget *parent = 0);
	~CloudViewer();

private:
	Ui::CloudViewerClass ui;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
	MyCloud mycloud;
	std::vector<MyCloud> mycloud_vec;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	QString save_filename;
	long total_points = 0; //Total amount of points in the viewer

	unsigned int red = 255;
	unsigned int green = 255;
	unsigned int blue = 255;
	unsigned int p = 2;
	std::vector<int> pointcolor;
	std::vector<int> bgcolor;

	QVBoxLayout *layout;

	int theme_id = 1; // 0: Windows theme, 1: Darcula theme
	bool enable_console = true;

	/***** Slots of QMenuBar and QToolBar *****/
	// File menu slots
	void open();
	void add();
	void clear();
	void save();
	void savemulti();
	void change();
	void exit();
	// Display menu slots
	void pointcolorChanged();
	void bgcolorChanged();
	void mainview();
	void leftview();
	void topview();
	// View menu slots
	void data();
	void properties();
	void console();
	void rgbDock();
	// Generate menu slots
	void cube();
	void createSphere();
	void createCylinder();
	// Process menu slots 
	int convertSurface();  //法线估计、曲面重建、网格面片显示
	int convertWireframe(); //法线估计、曲面重建、网格线框显示
	// Option menu slots
	void windowsTheme();
	void darculaTheme();
	void langEnglish();
	void langChinese();
	// About menu slots
	void about();
	void help();

	/***** Utils Methods ***/
	void initial();
	void showPointcloud();  //显示点云
	void showPointcloudAdd();  //添加给viewer，显示点云

	void setCloudColor(unsigned int r, unsigned int g, unsigned int b);
	void setA(unsigned int a);

	void setPropertyTable();
	void setConsoleTable();

	void consoleLog(QString operation, QString subname, QString filename, QString note);

	public slots:
	/***** Slots of RGB widget *****/
	// Change color or size of cloud when slider is released or colorBtn is pressed
	void colorBtnPressed();
	void RGBsliderReleased();
	void psliderReleased();
	void pSliderChanged(int value);
	void rSliderChanged(int value);
	void gSliderChanged(int value);
	void bSliderChanged(int value);
	// Slots of checkBox
	void cooCbxChecked(int value);
	void bgcCbxChecked(int value);

	/***** Slots of dataTree(QTreeWidget) widget *****/
	// Item in dataTree is left-clicked
	void itemSelected(QTreeWidgetItem*, int);
	// Item in dataTree is right-clicked
	void popMenu(const QPoint&);
	void hideItem();
	void showItem();
	void deleteItem();

	void popMenuInConsole(const QPoint&);
	void clearConsole();
	void enableConsole();
	void disableConsole();
};

#endif // CLOUDVIEWER_H