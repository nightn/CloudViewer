# 在 Windows 上编译 CloudViewer

该文档介绍在 Windows 系统上编译运行 CloudViewer，首先介绍了开发环境的搭建，然后使用 CMake 编译 CloudViewer。所需软件及类库的版本如下（其他版本的环境搭建及配置基本相似）：

- Visual Studio Community 2017
- Qt5.10.1-msvc2017_x64
- PCL1.9.1-x64
- vtk8.1.0
- cmake3.14

## 目录

- [环境搭建](#环境搭建)
  - [资源下载](#资源下载)
  - [VS安装](#VS安装)
  - [PCL安装](#PCL安装)
  - [Qt安装](Qt安装)
  - [VTK编译](#VTK编译)
  - [环境变量配置](#环境变量配置)
- [CloudViewer编译运行](#CloudViewer编译运行)
- CloudViewer打包（TODO）

## 环境搭建

### 资源下载

- **Visual Studio**
  - [Visual Studio Community 2017](https://www.visualstudio.com/thank-you-downloading-visual-studio/?sku=Community&rel=15) 
- **Qt**
  - [Qt5.10.1](http://download.qt.io/archive/qt/5.10/5.10.1/)
  - [qt-vsaddin-msvc2017](https://download.qt.io/development_releases/vsaddin/)
- **PCL**
  - [PCL-1.9.1-AllInOne-msvc2017-win64.exe](https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.9.1/PCL-1.9.1-AllInOne-msvc2017-win64.exe)
  - [PCL-1.9.1-pdb-msvc2017-win64.zip](https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.9.1/pcl-1.9.1-pdb-msvc2017-win64.zip)

- **VTK**
  - [vtk8.1.0 source code](https://github.com/Kitware/VTK/archive/v8.1.0.zip)
  - [cmake](<https://cmake.org/download/>) 

### VS安装

作为演示，为简单起见，安装 Visual Studio 2017 社区版，在安装时，仅勾选了必需的 `Desktop development with C++`。如个人有其他开发需要（如 .NET 开发），可以自行勾选。

<img src="http://nightn.github.io/cloudviewer/img2/vs2017-community-install.png" width="600"/>

VS2017 与之前版本不同，是在线安装，即边下载边安装，耗时与网速和电脑配置相关。耐心等待安装完成。

<img src="http://nightn.github.io/cloudviewer/img2/vs2017-community-installing.png" width="400"/>

### PCL安装

PCL 是点云开源库，可以在其 Github realese 页面获取 windows 安装程序和 pdb 文件。这里选择目前最新版 PCL1.9.1 进行安装，将所有组件都勾选上。

<img src="http://nightn.github.io/cloudviewer/img2/pcl1.9.1-install.png" width="500"/>

安装过程中，会跳出 `OpenNI2` 的单独安装对话框，将安装路径修改为 `3rdParty/OpenNI2`，即和其他 PCL 依赖的第三方库放在一起，然后继续，完成 PCL 的安装。

<img src="http://nightn.github.io/cloudviewer/img2/openni2-path.png" width="500"/>

### Qt安装

安装 Qt5.10.1，在安装组件选择时，根据 Visual Studio 的版本选择对应的 Qt，在此选择 `MSVC 2017 64-bit`，其他组件可以按需选择。

<img src="http://nightn.github.io/cloudviewer/img2/qt-setup.png" width="400"/>

耐心等待 Qt 安装完成即可。

<img src="http://nightn.github.io/cloudviewer/img2/qt-install-finish.png" width="400"/>

然后为 Visual Studio 安装 qt addin 插件，在前面的下载链接中下载插件文件，后缀名为 `.vsix` ，双击完成安装。重新打开 Visual Studio，可以在菜单栏上看到 `QT VS Tools` 菜单。

<img src="http://nightn.github.io/cloudviewer/img2/qt-vsaddin.png" width="400"/>

安装完成后，打开 Visual Studio 进行 Qt 配置。打开 `Qt VS Tools` - `Qt Options`，设置对应 Qt 版本及路径。

<img src="http://nightn.github.io/cloudviewer/img2/qt-options.png" width="500"/>

### VTK编译

PCL 依赖 VTK，在其 `3rdParty` 目录下可以看到 VTK。那为什么还要自己手动编译 VTK 呢？这是因为，PCL 安装目录下的 VTK 并不完整，其中就少了与 Qt 相关的模块，而这些模块是 CloudViewer 编译运行所需要的（如 `QVTKWidget`）。所以我们需要手动编译 VTK，并替换 PCL `3rdParty` 下的 VTK。

可以发现，PCL1.9.1 依赖 VTK8.1，所以我们编译这个版本的 VTK（如果你使用的是其他版本的 PCL，请编译对应版本的 VTK）。可以在 VTK Github 下的 release 页面获取对应版本的 VTK 源码。在此之前，假设你已经安装好了 cmake 和 Visual Studio，前者用于生成平台相关的解决方案，后者用于编译。

首先将 VTK 源码解压，打开 cmake-gui，设置源码目录（此处即为 VTK 源码目录）和生成目录。并使用 `Add Entry` 按钮添加缓存变量 **CMAKE_DEBUG_POSTFIX**，类型为 **STRING**，值设置为 **-gd**。这是为了将最后编译的 debug 文件与 release 文件区分开来。

<img src="http://nightn.github.io/cloudviewer/img2/vtk-config.png" width="500"/>

单击 `Configure` 进行配置，根据安装的 Visual Studio 的版本选择对应的生成器，选择 x64 平台。然后开始配置。

<img src="http://nightn.github.io/cloudviewer/img2/cmake-generator-config.png" width="500"/>

配置过程中，cmake 会检测当前环境，编译器等，并生成缓存变量。第一次配置完成后，需要对一些缓存变量进行修改，如 **CMAKE_INSTALL_PREFIX**，它规定了最后构建的 VTK 安装在什么地方，这个目录可以随意选择，到时候拷贝到 PCL `3rdParty` 目录下即可。此外，可以选择需要构建的内容：**BUILD_DOCUMENTATION**, **BUILD_EXAMPLES**, **BUILD_SHARED_LIBS**, **BUILD_TESTING**。考虑到项目并没有直接使用 VTK，而只是用于替换 PCL 中的 VTK，所以只勾选了 **BUILD_SHARED_LIBS**，不对文档、实例和测试进行生成，这样可以节省生成和构建的时间。另外一个比较重要的是，要勾选 **VTK_Group_Qt**。具体如下图所示，单击 `Configure`，进行配置。

<img src="http://nightn.github.io/cloudviewer/img2/cmake-vtk-configuring-2.png" width="500"/>

然后可能会提示 **Qt5_DIR** NOT FOUND，所以，将该缓存变量设置为 Qt cmake 目录，在我的机器上该目录为 `C:\Qt\Qt5.10.1\5.10.1\msvc2017_64\lib\cmake\Qt5`（具体根据 Qt 的版本和安装目录进行合适的设置）。将 **VTK_QT_VERSION** 设置为 5。

<img src="http://nightn.github.io/cloudviewer/img2/cmake-vtk-configuring-qt.png" width="500"/>

然后 `Configure`，配置完成后，开始 `Generate`，生成完毕后，可以在生成目录下发现 Visual Studio 解决方案。打开解决方案，其中包含了上百个项目，默认是 Debug x64 模式，右击 **ALL_BUILD** 项目，选择生成，生成完成后，右击 **INSTALL** 项目，选择生成，即开始安装，将生成 debug 库文件。待构建完成后，切换为 Release x64 模式，按同样的操作，生成 release 库文件（构建过程比较耗时，大概需要 1 个小时左右）。最终完成 VTK 的构建，可以在先前配置的安装目录下找到构建好的文件。

最后，将构建好的整个目录拷贝至 PCL `3rdParty` 目录下，更名为 `VTK`，替换掉原来的 VTK。此外，为了能够在 Qt Designer 中使用 QVTKWidget 控件，将 `PCL_ROOT/3rdParty/VTK/plugins/designer/QVTKWidget.dll` 拷贝至 `QTDIR/Qt5.10.1/5.10.1/msvc2017_64/plugins/designer` 目录下（PCL_ROOT 和 QTDIR 分别是 PCL 和 Qt 的安装根目录，具体目录设置根据软件版本和安装路径而定）。

### 环境变量配置

添加以下环境变量（如已添加则忽略）。

- **PCL_ROOT**

  指向 PCL 安装根目录（示例：`C:\Program Files\PCL 1.9.1`）。

- **QTDIR**

  指向 QT 安装根目录（示例：`D:\Program\Qt\Qt5.10.1`）。

在 **PATH** 环境变量添加以下值：

- **%PCL_ROOT%\bin**
- **%PCL_ROOT%\3rdParty\FLANN\bin**
- **%PCL_ROOT%\3rdParty\VTK\bin**
- **%PCL_ROOT%\3rdParty\Qhull\bin**
- **%PCL_ROOT%\3rdParty\OpenNI2\Tools**

## CloudViewer编译运行

克隆 CloudViewer。

```shell
# 克隆 CloudViewer
git clone https://github.com/nightn/CloudViewer.git
```

使用 cmake 配置和生成（也可以选择使用 cmake-gui）。

```shell
cd CloudViewer
# configure & generate 使用 -G 指定 generator
cmake -H. -Bbuild -G "Visual Studio 15 2017 Win64"
```

cmake 运行完成后，将在 `build` 目录生成 Visual Studio 解决方案（假设使用 Visual Studio 作为 generator），可以使用 Visual Studio IDE 编译运行，也可以使用 cmake。

```shell
# 编译
cmake --build build
# 运行（以 Debug 为例）
./build/src/Debug/CloudViewer.exe
```

运行结果如下图所示。

<img src="http://nightn.github.io/cloudviewer/img2/windows-run-result.png" width="600"/>
