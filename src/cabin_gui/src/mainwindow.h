#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QKeyEvent>
#include <QTcpSocket>
#include <QTcpServer>
#include <QHostAddress>
#include <QImage>
#include <QLabel>
#include <QMap>                             
#include <QSet>  
#include <QTimer>    
#include <QtEndian>
#include <QMutex>
#include <QVector>
#include <QPainter>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QListWidget>
#include <QDialog>
#include <QMessageBox>
#include <QDebug>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "cabin_interface/msg/control_move.hpp"
#include "cabin_interface/msg/control_cmd.hpp"
#include <ping360_sonar_msgs/msg/sonar_echo.hpp>

#include <rclcpp/rclcpp.hpp>


class SonarWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SonarWidget(QWidget *parent = nullptr);

    void setRange(float new_rangeMin, float new_rangeMax);      //设置声纳最小最大探测距离
    void setPixelsPerMeter(float ppm);                  //设置映射比例
    void setColorTable(const QVector<QColor>& colors);  //设置伪彩色表
    // 更新声呐数据
    void updateSonarData(float new_angle, uint16_t num_samples,
                         float new_rangeMin, float new_rangeMax,
                         const std::vector<uint8_t>& intensities);
signals:
    void scaleChanged(float newScale);                  //缩放倍数变化的信号
    void popOutRequested();
protected:
    void paintEvent(QPaintEvent *event) override;       //重写成绘制基础图像
    void resizeEvent(QResizeEvent *event) override;     //重写成调整缩放按钮的位置
    void mouseDoubleClickEvent(QMouseEvent *event) override;  //重写鼠标双击
private slots:
    void onZoomIn();               //增大显示缩放的槽函数
    void onZoomOut();              //缩小显示缩放的槽函数
private:
    void initImage();              //初始化底图
    void updateSingleRay(float new_angle, const std::vector<uint8_t>& intensities,
                         float new_rangeMin, float new_rangeMax);   //更新单个线条
    void drawGrid(QPainter &painter);                       //绘制极坐标网络
    void drawCenterMark(QPainter &painter);                 //绘制中心点
    void drawScanLine(QPainter &painter);                   //绘制扫描线
    void updateButtonPositions();                           //更新缩放按钮位置（在resizeEvent中记得调用）

    QImage sonarImage;    //目前底图
    QMutex mutex;         //互斥锁
    float rangeMin = 0.0f; 
    float rangeMax = 10.0f;
    float pixelsPerMeter = 25.0f;  //映射比例
    int imageSize = 400;           //底图大小（像素）
    float displayScale = 1.5f;     //显示缩放比例
    static constexpr float MIN_SCALE = 0.2f;  //最小缩放限制
    static constexpr float MAX_SCALE = 5.0f;  //最大缩放限制

    float currentAngle = 0.0f;      //最近更新的角度
    bool hasNewData = false;        //是否收到数据
    QVector<QColor> colorTable;     //伪RGB查找表

    QPushButton *zoomInBtn;         //放大
    QPushButton *zoomOutBtn;        //缩小

};
QVector<QColor> generateJetColorTable();

class MapWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MapWidget(QWidget *parent = nullptr);
    void updateRobotPosition(double x, double y, double z);   //更新机器人当前位置
    void setSavedPoints(const QVector<QPair<int, QPointF>>& points);//批量设置已保存点
    void addOrReplaceSavedPoint(int id, double x, double y);  //添加/覆盖保存点
    void removeSavedPoint(int id);                            //移除保存点
    void clearSavedPoints();                                  //清空所有保存点
signals:
    void savedPointClicked(int id);                           //用户点击了某个保存点
protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
private:
    QPointF worldToPixel(double x,double y) const;           //世界坐标→像素坐标
    int hitTest(const QPointF& pixelPt) const;                //点击检测，返回保存点id或-1
    // 机器人当前位置
    double robotX = 0.0, robotY = 0.0, robotZ = 0.0;
    bool hasRobotPosition = false;
    // 保存点列表
    struct SavedPoint 
    {
        int id;
        double x, y;
    };
    QVector<SavedPoint> savedPoints;
    // 地图参数
    double worldRange = 15.0;                  // 地图显示的世界范围（米），正方形
    int baseStationMargin = 40;                // 基站标记离底部的像素距离
    static constexpr double HIT_RADIUS = 8.0;  // 点击检测半径（像素）
    static constexpr double OVERLAP_PIXEL = 6.0; // 保存点重叠判定（像素）
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:  //
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;

private slots:   //槽函数，包括页面1/页面2切换，手柄/键盘切换,
    void onModeStop();
    void onModeKeyboard();
    void onModeJoystick();
    void onEmergencyStop();
    void onIMUreset();
    void onPIDcontrol();
    void updateWASDButtonColor(int key, bool pressed);

public:
    struct JoystickState
    {
        float axes[8]={0};
        int buttons[16]={0};
    };

private:
    void setupUI();
    void initROS();
 
    // 页面切换相关的控件
    QWidget *centralWidget;
    QHBoxLayout *mainLayout;
    QWidget *sideBar;
    QVBoxLayout *sideBarLayout;
    QPushButton *btnPage;
    QStackedWidget *stackedWidget;

    QWidget *page;

    // 键盘响应控制反馈
    QPushButton *btnW;
    QPushButton *btnA;
    QPushButton *btnS;
    QPushButton *btnD;
    QPushButton *btnI;
    QPushButton *btnJ;
    QPushButton *btnK;
    QPushButton *btnL;

    //模式选择按钮(手柄A)
    QPushButton *btnModeStop;
    QPushButton *btnModeKeyboard;
    QPushButton *btnModeJoystick;
    int keyboardMode;  // 0为手柄模式，1为键盘模式,2为锁定模式
    //急停（手柄B），重置IMU（手柄X），PID控制（手柄Y）
    QPushButton *btnEmergencyStop;
    QPushButton *btnIMUreset;
    QPushButton *btnPIDcontrol;
    bool pidstage=0;
    QPushButton *btnNormal;
    QPushButton *btnYolo;

    //键盘互斥
    bool WPressd=0;
    bool SPressd=0;
    bool APressd=0;
    bool DPressd=0;
    bool IPressd=0;
    bool KPressd=0;
    bool JPressd=0;
    bool LPressd=0;
    QSet<int> pressedKeys;
//ros2相关
private:
    //ros2通信相关
    rclcpp::Node::SharedPtr guinode;
    rclcpp::Publisher<cabin_interface::msg::ControlMove>::SharedPtr move_publisher;
    rclcpp::Publisher<cabin_interface::msg::ControlCmd>::SharedPtr cmd_publisher;
    rclcpp::Subscription<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr sonar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ugps_sub_;
    QTimer *rosSpinTimer;
    //控制状态
    float joy_force[3]={0.0f};
    float joy_moment[3]={0.0f};
    int robot_state=1;
    //按钮防抖
    bool prev_button_a=0;
    bool prev_button_b=0;
    bool prev_button_x=0;
    bool prev_button_y=0;
private:
    void updateControl();
    void publishControlCmd(int lock,bool imu_reset,bool pid_enable);
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void ugpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void controlTimerCallback();
//TCP协议视频接收相关
private:
    QWidget *videoPage;
    QLabel  *videoLabel;
    QTcpSocket *tcpSocket;  //节套字
    QByteArray recvbuffer; //缓冲区
    quint32 expectbyte;     //期望长度
    bool isReceiving;       //是否正在接收数据体
    quint16 currentPort;
    QString serverIp; 
    QTcpServer *tcpServer;
    QTcpSocket *clientSocket;
private:
    void initVideo(quint16 port);
    void switchPort(int port);
private slots:
    void displayImage(const QImage &image);
    void onTcpConnected();
    void onTcpReadyRead();
    void onTcpError(QAbstractSocket::SocketError error);
    void onNewConnection();
    void onNormal();
    void onYolo();
    void onSonarPopOut();
//声呐相关
private:
    SonarWidget *sonarWidget; 
    QVBoxLayout *rightColumnLayout=nullptr;   //新增：保存布局引用
    QWidget *sonarPopupWindow=nullptr;        //弹出窗口指针
    int sonarOriginalIndex=-1;
//地图相关：
private:
    void onSaveCurrent();              //保存当前图像+坐标
    void onLoadSaved();                //打开已保存列表
    void onSavedPointClicked(int id);  //地图上的保存点被点击
    void ensureSaveDir();              //确保保存目录存在
    void loadSavedDataFromDisk();      //从磁盘加载所有保存条目
    void saveIndexToDisk();            //将内存索引写回磁盘
    void showImagePreview(int id);     //弹出预览窗口
    void deleteSavedEntry(int id);     //删除按钮
private:
    MapWidget *mapWidget=nullptr;
    QPushButton *btnSave;              //保存
    QPushButton *btnLoad;              //读取
    QString m_saveDir;                 //保存目录路径
    QJsonArray m_savedEntries;         //内存中的索引数组
    int m_nextSaveId=0;              //下一个保存ID
    QImage m_currentVideoImage;        //当前摄像头画面（最新一帧）
    double m_currentRobotX = 0.0;      
    double m_currentRobotY = 0.0;       
    double m_currentRobotZ = 0.0;       
};

#endif // MAINWINDOW_H