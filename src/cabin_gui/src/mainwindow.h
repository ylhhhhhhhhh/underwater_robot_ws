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

#include <sensor_msgs/msg/joy.hpp>
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
protected:
    void paintEvent(QPaintEvent *event) override;       //重写成绘制基础图像
    void resizeEvent(QResizeEvent *event) override;     //重写成调整缩放按钮的位置
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
    float displayScale = 1.0f;     //显示缩放比例
    static constexpr float MIN_SCALE = 0.2f;  //最小缩放限制
    static constexpr float MAX_SCALE = 5.0f;  //最大缩放限制

    float currentAngle = 0.0f;      //最近更新的角度
    bool hasNewData = false;        //是否收到数据
    QVector<QColor> colorTable;     //伪RGB查找表

    QPushButton *zoomInBtn;         //放大
    QPushButton *zoomOutBtn;        //缩小
};

QVector<QColor> generateJetColorTable();

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
//声呐相关
private:
    SonarWidget *sonarWidget; 
};

#endif // MAINWINDOW_H