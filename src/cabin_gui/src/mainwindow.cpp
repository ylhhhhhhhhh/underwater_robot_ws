#include "mainwindow.h"
#include <QMessageBox>
#include <QTimer>
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), keyboardMode(1)
{
    setupUI();
    initROS();
    currentPort=6001;
    initVideo(currentPort); 
}
MainWindow::~MainWindow()
{
    if(rclcpp::ok())
    {
      rclcpp::shutdown();
    }
}
void MainWindow::setupUI()
{
    //主界面
    setWindowTitle("水下机器人控制系统");
    resize(1600,1200);
    setFocusPolicy(Qt::StrongFocus);  //确保接收键盘事件
    centralWidget=new QWidget(this);
    setCentralWidget(centralWidget);
    mainLayout = new QHBoxLayout(centralWidget);
    mainLayout->setContentsMargins(0,0,0,0);
    mainLayout->setSpacing(0);
    //左侧侧边栏
    sideBar=new QWidget();
    sideBar->setStyleSheet("background-color: black;");
    sideBar->setFixedWidth(150);
    sideBarLayout = new QVBoxLayout(sideBar);
    sideBarLayout->setAlignment(Qt::AlignTop);
    sideBarLayout->setSpacing(10);
    sideBarLayout->setContentsMargins(10,20,10,10);
    btnNormal = new QPushButton("普通模式");
    btnYolo = new QPushButton("YOLO模式");
    btnNormal->setCheckable(true);
    btnYolo->setCheckable(true);
    btnNormal->setChecked(true);
    // 设置按钮样式（白字，背景色自定义）
    btnNormal->setStyleSheet("color: white; background-color: #2E7D32;");
    btnYolo->setStyleSheet("color: white; background-color: #4c49d5;");
    sideBarLayout->addWidget(btnNormal);
    sideBarLayout->addWidget(btnYolo);
    sideBarLayout->addStretch();
    //右侧页面容器
    stackedWidget = new QStackedWidget();
    page = new QWidget();
    stackedWidget->setStyleSheet("QStackedWidget { background-color: #ffffff; }");
    QHBoxLayout *pageLayout = new QHBoxLayout(page);
    pageLayout->setContentsMargins(20, 20, 20, 20);
    pageLayout->setSpacing(10);
    // 左列（视频）——固定尺寸，顶部对齐
    QVBoxLayout *leftLayout = new QVBoxLayout();
    leftLayout->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    videoLabel = new QLabel();
    videoLabel->setAlignment(Qt::AlignCenter);
    videoLabel->setStyleSheet("background-color: black;");
    videoLabel->setFixedSize(800, 600);     // 放大视频尺寸（4:3）
    leftLayout->addWidget(videoLabel);
    // 左侧列底部留空，不添加额外拉伸
    pageLayout->addLayout(leftLayout, 1);
    // 右列（声呐 + 按钮组）
    QVBoxLayout *rightColumnLayout = new QVBoxLayout();
    rightColumnLayout->setAlignment(Qt::AlignTop);   // 整体靠上，但弹簧会将按钮推到底部
    rightColumnLayout->setSpacing(10);
    // 声呐在上方，固定正方形
    sonarWidget = new SonarWidget(this);
    sonarWidget->setFixedSize(400, 400);
    sonarWidget->setRange(0.0f, 10.0f);
    sonarWidget->setPixelsPerMeter(25.0f);
    sonarWidget->setColorTable(generateJetColorTable());
    rightColumnLayout->addWidget(sonarWidget, 0, Qt::AlignTop);
    // 添加弹性空间，将后续按钮组推到底部
    rightColumnLayout->addStretch();   // 关键：弹簧将按钮组压到底部
    // 按钮组（WASD/IJKL）放在声呐下方，右下对齐
    QWidget *buttonGroupWidget1 = new QWidget();
    QHBoxLayout *buttonGroupLayout1 = new QHBoxLayout(buttonGroupWidget1);
    buttonGroupLayout1->setAlignment(Qt::AlignRight | Qt::AlignBottom);
    // WASD 区域
    QWidget *wasdWidget1 = new QWidget();
    QGridLayout *wasdLayout1 = new QGridLayout(wasdWidget1);
    btnW = new QPushButton("W");
    btnA = new QPushButton("A");
    btnS = new QPushButton("S");
    btnD = new QPushButton("D");
    btnW->setFixedSize(60, 60);
    btnA->setFixedSize(60, 60);
    btnS->setFixedSize(60, 60);
    btnD->setFixedSize(60, 60);
    wasdLayout1->addWidget(btnW, 0, 1);
    wasdLayout1->addWidget(btnA, 1, 0);
    wasdLayout1->addWidget(btnS, 1, 1);
    wasdLayout1->addWidget(btnD, 1, 2);
    wasdLayout1->setAlignment(Qt::AlignCenter);
    // IJKL 区域
    QWidget *ijklWidget1 = new QWidget();
    QGridLayout *ijklLayout1 = new QGridLayout(ijklWidget1);
    btnI = new QPushButton("I");
    btnJ = new QPushButton("J");
    btnK = new QPushButton("K");
    btnL = new QPushButton("L");
    btnI->setFixedSize(60, 60);
    btnJ->setFixedSize(60, 60);
    btnK->setFixedSize(60, 60);
    btnL->setFixedSize(60, 60);
    ijklLayout1->addWidget(btnI, 0, 1);
    ijklLayout1->addWidget(btnJ, 1, 0);
    ijklLayout1->addWidget(btnK, 1, 1);
    ijklLayout1->addWidget(btnL, 1, 2);
    ijklLayout1->setAlignment(Qt::AlignCenter);
    buttonGroupLayout1->addWidget(wasdWidget1);
    buttonGroupLayout1->addWidget(ijklWidget1);
    buttonGroupLayout1->setSpacing(20);
    rightColumnLayout->addWidget(buttonGroupWidget1, 0, Qt::AlignRight | Qt::AlignBottom);
    pageLayout->addLayout(rightColumnLayout, 1);
    // 设置按钮样式（统一设置）
    btnW->setStyleSheet("background-color: lightgray;");
    btnA->setStyleSheet("background-color: lightgray;");
    btnS->setStyleSheet("background-color: lightgray;");
    btnD->setStyleSheet("background-color: lightgray;");
    btnI->setStyleSheet("background-color: lightgray;");
    btnJ->setStyleSheet("background-color: lightgray;");
    btnK->setStyleSheet("background-color: lightgray;");
    btnL->setStyleSheet("background-color: lightgray;");

    stackedWidget->addWidget(page);
    // 左下角模式选择按钮
    QWidget *modeWidget = new QWidget();
    modeWidget->setFixedHeight(80);
    QHBoxLayout *modeLayout = new QHBoxLayout(modeWidget);
    btnModeKeyboard = new QPushButton("键盘");
    btnModeJoystick = new QPushButton("手柄");
    btnModeStop     = new QPushButton("锁定");
    btnModeKeyboard->setCheckable(true);
    btnModeJoystick->setCheckable(true);
    btnModeStop->setCheckable(true);
    btnModeKeyboard->setChecked(true);   // 默认键盘模式
    btnModeJoystick->setChecked(false);
    btnModeStop->setChecked(false);
    btnModeKeyboard->setStyleSheet("QPushButton { background-color: #3cd83c; color: white; }"
                                   "QPushButton:checked { background-color: #2E7D32; }");
    btnModeJoystick->setStyleSheet("QPushButton { background-color: #f44336; color: white; }"
                                   "QPushButton:checked { background-color: #c62828; }");
    btnModeStop->setStyleSheet("QPushButton { background-color: #3692cf; color: white; }"
                               "QPushButton:checked { background-color: #4c49d5; }");
    modeLayout->addWidget(btnModeKeyboard);
    modeLayout->addWidget(btnModeJoystick);
    modeLayout->addWidget(btnModeStop);
    modeLayout->setAlignment(Qt::AlignLeft);
    modeLayout->setContentsMargins(10, 10, 10, 10);  
    //右下角模式选择按钮
    QWidget *toolWidget = new QWidget();
    toolWidget->setFixedHeight(80);
    QHBoxLayout *toolLayout=new QHBoxLayout(toolWidget);
    btnEmergencyStop =new QPushButton("急停");
    btnIMUreset=new QPushButton("重置IMU");
    btnPIDcontrol=new QPushButton("启用PID");
    btnEmergencyStop->setFixedSize(80,40);
    btnIMUreset->setFixedSize(80,40);
    btnPIDcontrol->setFixedSize(80,40);
    btnPIDcontrol->setCheckable(true);
    btnPIDcontrol->setChecked(false);   //初始PID关闭
    btnEmergencyStop->setStyleSheet("background-color: #ff4444; color: white; font-weight: bold;");
    btnIMUreset->setStyleSheet("background-color: #ffaa44;");
    btnPIDcontrol->setStyleSheet("background-color: #a525e4;");
    toolLayout->addStretch();        //左侧弹性，使按钮靠右
    toolLayout->addWidget(btnEmergencyStop);
    toolLayout->addWidget(btnIMUreset);
    toolLayout->addWidget(btnPIDcontrol);
    // 组装主布局
    mainLayout->addWidget(sideBar);
    QVBoxLayout *rightLayout = new QVBoxLayout();
    rightLayout->addWidget(stackedWidget);
    //创建水平底部栏
    QWidget *bottomBar = new QWidget();
    QHBoxLayout *bottomBarLayout = new QHBoxLayout(bottomBar);
    bottomBarLayout->setContentsMargins(0, 0, 0, 0);
    bottomBarLayout->setSpacing(0);
    //将现有的 modeWidget 和 toolWidget 放入 bottomBarLayout
    bottomBarLayout->addWidget(modeWidget);
    bottomBarLayout->addStretch();            
    bottomBarLayout->addWidget(toolWidget);   
    rightLayout->addWidget(bottomBar);
    QWidget *rightContainer = new QWidget();
    rightContainer->setLayout(rightLayout);
    mainLayout->addWidget(rightContainer, 1);
    //连接信号和槽
    connect(btnModeKeyboard, &QPushButton::clicked,this,&MainWindow::onModeKeyboard);
    connect(btnModeJoystick, &QPushButton::clicked,this,&MainWindow::onModeJoystick);
    connect(btnModeStop,     &QPushButton::clicked,this,&MainWindow::onModeStop);
    connect(btnEmergencyStop,&QPushButton::clicked,this,&MainWindow::onEmergencyStop);
    connect(btnIMUreset,     &QPushButton::clicked,this,&MainWindow::onIMUreset);
    connect(btnPIDcontrol,   &QPushButton::clicked,this,&MainWindow::onPIDcontrol);
    connect(btnNormal,       &QPushButton::clicked,this,&MainWindow::onNormal);
    connect(btnYolo,         &QPushButton::clicked,this,&MainWindow::onYolo);
}
void MainWindow::initROS()
{
    if(!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    guinode=std::make_shared<rclcpp::Node>("cabin_gui");
    move_publisher=guinode->create_publisher<cabin_interface::msg::ControlMove>("/red/command/move",10);
    cmd_publisher =guinode->create_publisher<cabin_interface::msg::ControlCmd>("/red/command/cmd",10);
    joy_sub=guinode->create_subscription<sensor_msgs::msg::Joy>(
      "joy",10,std::bind(&MainWindow::joyCallback,this,std::placeholders::_1)
    );
    rosSpinTimer = new QTimer(this);
    connect(rosSpinTimer, &QTimer::timeout, this, [this]() {
        //static int count = 0;
        //if (count++ % 50 == 0)   // 每秒打印一次（50*20ms=1s）
            //std::cout << "rosSpinTimer running..." << std::endl;
        rclcpp::spin_some(guinode);
        controlTimerCallback();
    });
    rosSpinTimer->start(20); 

    float param_range_max = 2.0f;
    guinode->get_parameter_or("range_max", param_range_max, 2.0f);

    sonar_sub_ = guinode->create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
    "/scan_echo",
    rclcpp::SensorDataQoS(),
    [this](const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {
        //RCLCPP_INFO(guinode->get_logger(), 
          //          "Sonar received: angle=%.2f, range=%d, samples=%d",
            //        msg->angle, msg->range, msg->number_of_samples);
        std::vector<uint8_t> intensities;
        intensities.assign(msg->intensities.begin(), msg->intensities.end());
        float rangeMax = static_cast<float>(msg->range);
        if (rangeMax < 0.1f) rangeMax = 0.1f;
        sonarWidget->updateSonarData(msg->angle, msg->number_of_samples,
                                     0.0f, rangeMax, intensities);
    });
    connect(rosSpinTimer, &QTimer::timeout, this, [this]() {
        if (rclcpp::ok()) {
            rclcpp::spin_some(guinode);
        }
        if (rclcpp::ok()) {
            controlTimerCallback();
        }
    });
}

void MainWindow::updateControl()
{
    if(WPressd&&!SPressd)
        joy_force[0]=33.0f;
    else  if(SPressd&&!WPressd)
        joy_force[0]=-33.0f;
    else
        joy_force[0]=0.0f;
    if(DPressd&&!APressd)
        joy_force[1]=33.0f;
    else  if(APressd&&!DPressd)
        joy_force[1]=-33.0f;
    else
        joy_force[1]=0.0f;
    if(IPressd&&!KPressd)
        joy_force[2]=15.0f;
    else  if(KPressd&&!IPressd)
        joy_force[2]=-15.0f;
    else 
        joy_force[2]=0.0f;
    if (LPressd && !JPressd)
        joy_moment[2] = 33.0f;
    else if (JPressd && !LPressd)
        joy_moment[2] = -33.0f;
    else
        joy_moment[2] = 0.0f;
}

void MainWindow::publishControlCmd(int lock, bool imu_reset, bool pid_enable)
{
  auto cmd_msg =cabin_interface::msg::ControlCmd();
  cmd_msg.header.stamp=guinode->now();
  cmd_msg.lock=lock;
  cmd_msg.imu_reset=imu_reset;
  cmd_msg.pid_enable=pid_enable;
  cmd_publisher->publish(cmd_msg);
  if(imu_reset)
  {
    cmd_msg.imu_reset=false;
    cmd_publisher->publish(cmd_msg);
  }
}

void MainWindow::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    //1.摇杆控制（仅在手柄模式下更新运动输出）
    if(keyboardMode==0)
    {
        joy_force[0]=joy_force[1]=joy_force[2]=0.0f;
        joy_moment[2]=0.0f;
        // 清零
        joy_force[0]=joy_force[1]=joy_force[2]=0.0f;
        joy_moment[2]=0.0f;
        //左摇杆上下->force.x（前进/后退）
        float left_ud=msg->axes[1];
        if(std::fabs(left_ud)>0.05)
        {
            joy_force[0]=left_ud*33.0f;
        }
        //左摇杆左右->force.y（横移）
        float left_lr=msg->axes[0];
        if(std::fabs(left_lr)>0.05)
        {
            joy_force[1]=-left_lr*33.0f;
        }
        //右摇杆左右->moment.z（偏航）
        float right_lr = msg->axes[2];
        if(std::fabs(right_lr)>0.05)
        {
            joy_moment[2]=right_lr*33.0f;
        }
        //十字键上下->force.z（升沉）
        float cross_ud=msg->axes[5];
        if (cross_ud>0.5)
            joy_force[2]=15.0f;
        else if(cross_ud<-0.5)
            joy_force[2]=-15.0f;
    }
    //2. 按钮功能（不受模式限制，任何模式下都有效）
    //A 按钮：循环切换 keyboardMode（0->1->2->0）
    bool a_pressed=(msg->buttons[1]==1);
    if(a_pressed&&!prev_button_a)
    {
        keyboardMode=(keyboardMode+1)%3;
        //同步GUI模式按钮状态
        if(keyboardMode==0) 
        {
            btnModeJoystick->setChecked(true);
            btnModeKeyboard->setChecked(false);
            btnModeStop->setChecked(false);
        }
        else if(keyboardMode==1)
        {
            btnModeKeyboard->setChecked(true);
            btnModeJoystick->setChecked(false);
            btnModeStop->setChecked(false);
        }
        else 
        {
            btnModeStop->setChecked(true);
            btnModeKeyboard->setChecked(false);
            btnModeJoystick->setChecked(false);
            // 锁定模式下立即清零运动输出并发布一次零指令
            std::fill(std::begin(joy_force),std::end(joy_force),0.0f);
            std::fill(std::begin(joy_moment),std::end(joy_moment),0.0f);
            if(move_publisher)
            {
                auto move_msg=cabin_interface::msg::ControlMove();
                move_msg.header.stamp=guinode->now();
                move_publisher->publish(move_msg);
            }
        }
        // 发布 lock 状态变化
        publishControlCmd(keyboardMode,false,pidstage);
    }
    prev_button_a=a_pressed;
    //B按钮：急停（清零）
    bool b_pressed=(msg->buttons[2]==1);
    if(b_pressed&&!prev_button_b)
    {
        onEmergencyStop();
    }
    prev_button_b= b_pressed;
    //X按钮：重置 IMU
    bool x_pressed=(msg->buttons[0]==1);
    if(x_pressed&&!prev_button_x)
    {
        onIMUreset();
    }
    prev_button_x=x_pressed;
    //Y按钮：切换 PID
    bool y_pressed=(msg->buttons[3]==1);
    if (y_pressed&&!prev_button_y) 
    {
        onPIDcontrol();
    }
    prev_button_y= y_pressed;
}

void MainWindow::controlTimerCallback()
{
    //RCLCPP_INFO(guinode->get_logger(), "joyCallback received");
    if(keyboardMode==2)
      return;
    auto move_msg=cabin_interface::msg::ControlMove();
    move_msg.header.stamp=guinode->now();
    move_msg.force.x=joy_force[0];
    move_msg.force.y=joy_force[1];
    move_msg.force.z=joy_force[2];
    move_msg.moment.x=joy_moment[0];
    move_msg.moment.y=joy_moment[1];
    move_msg.moment.z=joy_moment[2];
    move_publisher->publish(move_msg);
}

void MainWindow::onModeKeyboard()
{
    keyboardMode = 1;
    btnModeKeyboard->setChecked(true);
    btnModeJoystick->setChecked(false);
    btnModeStop->setChecked(false);
    // 清除所有WASDIJKL按钮的高亮
    updateWASDButtonColor(Qt::Key_W, false);
    updateWASDButtonColor(Qt::Key_A, false);
    updateWASDButtonColor(Qt::Key_S, false);
    updateWASDButtonColor(Qt::Key_D, false);
    updateWASDButtonColor(Qt::Key_I, false);
    updateWASDButtonColor(Qt::Key_K, false);
    updateWASDButtonColor(Qt::Key_J, false);
    updateWASDButtonColor(Qt::Key_L, false);
    publishControlCmd(keyboardMode, false, pidstage);
}

void MainWindow::onModeJoystick()
{
    keyboardMode = 0;
    btnModeJoystick->setChecked(true);
    btnModeKeyboard->setChecked(false);
    btnModeStop->setChecked(false);
    // 清除所有WASDIJKL按钮的高亮
    updateWASDButtonColor(Qt::Key_W, false);
    updateWASDButtonColor(Qt::Key_A, false);
    updateWASDButtonColor(Qt::Key_S, false);
    updateWASDButtonColor(Qt::Key_D, false);
    updateWASDButtonColor(Qt::Key_I, false);
    updateWASDButtonColor(Qt::Key_K, false);
    updateWASDButtonColor(Qt::Key_J, false);
    updateWASDButtonColor(Qt::Key_L, false);
    publishControlCmd(keyboardMode, false, pidstage);
}

void MainWindow::onModeStop()
{
    keyboardMode = 2;
    btnModeJoystick->setChecked(false);
    btnModeKeyboard->setChecked(false);
    btnModeStop->setChecked(true);
    // 清除所有WASDIJKL按钮的高亮
    updateWASDButtonColor(Qt::Key_W, false);
    updateWASDButtonColor(Qt::Key_A, false);
    updateWASDButtonColor(Qt::Key_S, false);
    updateWASDButtonColor(Qt::Key_D, false);
    updateWASDButtonColor(Qt::Key_I, false);
    updateWASDButtonColor(Qt::Key_K, false);
    updateWASDButtonColor(Qt::Key_J, false);
    updateWASDButtonColor(Qt::Key_L, false);
    publishControlCmd(keyboardMode, false, pidstage);
}

void MainWindow::onEmergencyStop()
{
    //清零力和力矩
    std::fill(std::begin(joy_force),std::end(joy_force),0.0f);
    std::fill(std::begin(joy_moment),std::end(joy_moment),0.0f);
    if(move_publisher)
    {
      auto move_msg=cabin_interface::msg::ControlMove();
      move_msg.header.stamp=guinode->now();
      move_msg.force.x=move_msg.force.y=move_msg.force.z=0.0f;
      move_msg.moment.x=move_msg.moment.y=move_msg.moment.z=0.0f;
      move_publisher->publish(move_msg);
    }
}

void MainWindow::onIMUreset()
{
  publishControlCmd(keyboardMode,true,pidstage);
}

void MainWindow::onPIDcontrol()
{
  pidstage=!pidstage;
    if(pidstage)
    {
      btnPIDcontrol->setText("关闭PID");
      btnPIDcontrol->setStyleSheet("background-color: #00aa00;");
    }
    else
    {
      btnPIDcontrol->setText("启用PID");
      btnPIDcontrol->setStyleSheet("background-color: #a525e4;");
    }
    publishControlCmd(keyboardMode,false,pidstage);
}

void MainWindow::updateWASDButtonColor(int key, bool pressed)
{
    QPushButton *targetBtn = nullptr;
    if(key==Qt::Key_W) targetBtn=btnW;
    else if(key==Qt::Key_A) targetBtn=btnA;
    else if(key==Qt::Key_S) targetBtn=btnS;
    else if(key==Qt::Key_D) targetBtn=btnD;
    else if(key==Qt::Key_I) targetBtn=btnI;
    else if(key==Qt::Key_J) targetBtn=btnJ;
    else if(key==Qt::Key_K) targetBtn=btnK;
    else if(key==Qt::Key_L) targetBtn=btnL;
    if(!targetBtn) return;
    if(pressed==1)
      targetBtn->setStyleSheet("background-color: yellow;");
    else
      targetBtn->setStyleSheet("background-color: lightgray;");
}
// 键盘按下事件重写
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    int key=event->key();
    if(keyboardMode!=1) //手柄模式关闭键盘响应
    {
      QMainWindow::keyPressEvent(event);
      return;
    }
    bool need_update=0;
    if((key == Qt::Key_W && SPressd==0) || (key == Qt::Key_A && DPressd==0) || (key == Qt::Key_S && WPressd==0) || (key == Qt::Key_D && APressd==0) || (key == Qt::Key_I && KPressd==0) || (key == Qt::Key_K && IPressd==0) || (key == Qt::Key_J && LPressd==0) || (key == Qt::Key_L && JPressd==0) )  
    {
      if(key==Qt::Key_W)
        WPressd=1,need_update=1;
      if(key==Qt::Key_S)
        SPressd=1,need_update=1;
      if(key==Qt::Key_A)
        APressd=1,need_update=1;
      if(key==Qt::Key_D)
        DPressd=1,need_update=1;
      if(key==Qt::Key_I)
        IPressd=1,need_update=1;
      if(key==Qt::Key_K)
        KPressd=1,need_update=1;
      if(key==Qt::Key_J)
        JPressd=1,need_update=1;
      if(key==Qt::Key_L)
        LPressd=1,need_update=1;
    }
    if(need_update)
    {
      updateWASDButtonColor(key,true);
      updateControl();
    }
    QMainWindow::keyPressEvent(event);
}
//键盘松开事件重写
void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    int key = event->key();
    if(keyboardMode!=1) 
    {
      QMainWindow::keyReleaseEvent(event);
      return;
    }
    bool need_update=0;
    if(key == Qt::Key_W||key == Qt::Key_A||key == Qt::Key_S||key == Qt::Key_D||key == Qt::Key_I||key == Qt::Key_J||key == Qt::Key_K||key == Qt::Key_L) 
    {
      if(key==Qt::Key_W)
        WPressd=0,need_update=1;
      if(key==Qt::Key_S)
        SPressd=0,need_update=1;
      if(key==Qt::Key_A)
        APressd=0,need_update=1;
      if(key==Qt::Key_D)
        DPressd=0,need_update=1;
      if(key==Qt::Key_I)
        IPressd=0,need_update=1;
      if(key==Qt::Key_J)
        JPressd=0,need_update=1;
      if(key==Qt::Key_K)
        KPressd=0,need_update=1;
      if(key==Qt::Key_L)
        LPressd=0,need_update=1;
    }
    if(need_update)
    {
      updateWASDButtonColor(key,0);
      updateControl();
    }
    QMainWindow::keyReleaseEvent(event);
}

void MainWindow::initVideo(quint16 port)
{
    currentPort = port;
    tcpServer = new QTcpServer(this);
    connect(tcpServer, &QTcpServer::newConnection, this, &MainWindow::onNewConnection);
    if (!tcpServer->listen(QHostAddress::Any, port)) {
        qDebug() << "Server could not start!";
        return;
    }
    recvbuffer.clear();
    isReceiving = false;
    expectbyte = 0;
}
void MainWindow::onNewConnection()
{
    if (tcpSocket) {
        tcpSocket->disconnectFromHost();
        tcpSocket->deleteLater();
        tcpSocket = nullptr;
    }
    tcpSocket = tcpServer->nextPendingConnection();
    connect(tcpSocket, &QTcpSocket::readyRead, this, &MainWindow::onTcpReadyRead);
    connect(tcpSocket, &QTcpSocket::disconnected, this, [this]() {
        tcpSocket->deleteLater();
        tcpSocket = nullptr;
    });
    qDebug() << "Video client connected";
}

void MainWindow::switchPort(int newPort)
{
    if(currentPort==newPort)
        return;
    currentPort=newPort;
    if(tcpServer->isListening())
    {
        tcpServer->close();
    }
    if(!tcpServer->listen(QHostAddress::Any,newPort))
    {
        qDebug()<<"Failed to switch to port"<<newPort;
        return;
    }
    if(clientSocket)
    {
        clientSocket->disconnectFromHost();
        clientSocket=nullptr;
    }
    recvbuffer.clear();
    isReceiving=false;
    expectbyte=0;
}

void MainWindow::onTcpConnected()
{
    if(!tcpSocket)
        return;
    qDebug()<<"Video connected";
}
void MainWindow::onTcpReadyRead()
{
    while(tcpSocket->bytesAvailable()>0)
    {
        if(!isReceiving)
        {
            //读取四字节长度
            if(tcpSocket->bytesAvailable()<4)
                return ;
            QByteArray lenData=tcpSocket->read(4);
            expectbyte=qFromBigEndian<quint32>((const uchar*)lenData.constData());
            recvbuffer.clear();
            isReceiving=1;
        }
        //读数据体
        qint64 bytesToRead=qMin<qint64>(expectbyte-recvbuffer.size(),tcpSocket->bytesAvailable());
        if(bytesToRead>0)
        {
            recvbuffer.append(tcpSocket->read(bytesToRead));
        }
        //检查完整接收
        if(recvbuffer.size()==(int)expectbyte)
        {
            QImage image;
            if(image.loadFromData(recvbuffer,"JPEG"))
            {
                displayImage(image);
            }
            else
            {
                qDebug()<<"Failed to decode JPEG";
            }
            isReceiving=0;
            recvbuffer.clear();
        }
    }
}
void MainWindow::onTcpError(QAbstractSocket::SocketError error)
{
    qDebug()<<"Tcp error:"<<error;
}

void MainWindow::onNormal()
{
    btnNormal->setChecked(true);
    btnYolo->setChecked(false);
    switchPort(6001);
}

void MainWindow::onYolo()
{
    btnNormal->setChecked(false);
    btnYolo->setChecked(true);
    switchPort(6002);
}

void MainWindow::displayImage(const QImage &image)
{
    QPixmap pixmap=QPixmap::fromImage(image);
    QSize labelSize=videoLabel->size();
    if (labelSize.width()>0&&labelSize.height()>0)
    {
        QPixmap scaled=pixmap.scaled(labelSize, Qt::KeepAspectRatio,Qt::SmoothTransformation);
        videoLabel->setPixmap(scaled);
    } 
    else
    {
        videoLabel->setPixmap(pixmap);
    }
}




SonarWidget::SonarWidget(QWidget *parent)
    :QWidget(parent)
{
    setMinimumSize(200,200);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QVector<QColor> defaultColors(256);
    setColorTable(generateJetColorTable());
    zoomInBtn=new QPushButton("+",this);
    zoomOutBtn=new QPushButton("-",this);
    zoomInBtn->setFixedSize(30,30);
    zoomOutBtn->setFixedSize(30,30);
    zoomInBtn->setStyleSheet("QPushButton { background-color: rgba(0,0,0,150); color: white; border: 1px solid white; border-radius: 4px; }"
                               "QPushButton:hover { background-color: rgba(255,255,255,150); color: black; }");
    zoomOutBtn->setStyleSheet(zoomInBtn->styleSheet());
    connect(zoomInBtn,&QPushButton::clicked,this,&SonarWidget::onZoomIn);
    connect(zoomOutBtn,&QPushButton::clicked,this,&SonarWidget::onZoomOut);
    imageSize=400;
    initImage();
}

void SonarWidget::setRange(float new_rangeMin, float new_rangeMax)
{
    if(new_rangeMin>=new_rangeMax)
        return ;
    rangeMin=new_rangeMin;
    rangeMax=new_rangeMax;
    initImage();
    update();
}

void SonarWidget::setPixelsPerMeter(float ppm)
{
    if(ppm<=0)
        return ;
    pixelsPerMeter=ppm;
    initImage();
    update();
}

void SonarWidget::setColorTable(const QVector<QColor> &colors)
{
    if(colors.size()!=256)
        return ;
    colorTable=colors;
}

void SonarWidget::updateSonarData(float new_angle, uint16_t new_num_samples, float new_rangeMin, float new_rangeMax, const std::vector<uint8_t> &intensities)
{
    if(new_rangeMin!=rangeMin||new_rangeMax!=rangeMax)
    {
        rangeMax=new_rangeMax;
        rangeMin=new_rangeMin;
        initImage();
    }
    if((int)intensities.size()!=new_num_samples)
        qWarning()<<"intensities size mismatch";
    updateSingleRay(new_angle,intensities,new_rangeMin,new_rangeMax);
    currentAngle=new_angle;
    hasNewData=1;
    update();
}

void SonarWidget::paintEvent(QPaintEvent *event)
{
    (void)event;
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing,true);
    //int drawSize=imageSize*displayScale;
    painter.save();
    painter.translate(width()/2.0,height()/2.0);
    painter.scale(displayScale,displayScale);
    {
        QMutexLocker locker(&mutex);
        painter.drawImage(-imageSize/2,-imageSize/2,sonarImage);
    }
    drawGrid(painter);
    drawScanLine(painter);
    painter.restore();
    drawCenterMark(painter);
}

void SonarWidget::resizeEvent(QResizeEvent *event)
{
    updateButtonPositions();
    QWidget::resizeEvent(event);
}

void SonarWidget::updateButtonPositions()
{
    int w=width();
    int h=height();
    int margin=10,btnSize=30;
    zoomInBtn->move(w-margin-btnSize,h-margin-btnSize);
    zoomOutBtn->move(w-margin-btnSize,h-margin-2*btnSize-5);
}

void SonarWidget::initImage()
{
    QMutexLocker locker(&mutex);
    sonarImage=QImage(imageSize,imageSize,QImage::Format_RGB32);  //正方形，RGB32格式
    sonarImage.fill(Qt::black);
    hasNewData=0;
}

void SonarWidget::updateSingleRay(float new_angle, const std::vector<uint8_t> &intensities, float new_rangeMin, float new_rangeMax)
{
    if(intensities.empty())
        return;
    QMutexLocker locker(&mutex);
    int size=imageSize;
    int center=size/2;
    float rangeSpan=new_rangeMax-new_rangeMin;
    if(rangeSpan<=0)
        return;
    float sinA=std::sin(new_angle);
    float cosA=std::cos(new_angle);
    for(size_t i=0;i<intensities.size();i++)
    {
        float r_real=new_rangeMin+(i/(float)intensities.size())*rangeSpan;
        float px=center+r_real*pixelsPerMeter*cosA;
        float py=center-r_real*pixelsPerMeter*sinA;
        int x=qRound(px);
        int y=qRound(py);
        if(x<0||x>=size||y<0||y>=size)
            continue;
        int intensity=intensities[i];
        sonarImage.setPixelColor(x,y,colorTable[intensity]);
    }
}

void SonarWidget::drawGrid(QPainter &painter)
{
    painter.save();
    painter.setPen(QPen(QColor(100, 100, 100, 80), 1, Qt::DashLine));
    painter.setBrush(Qt::NoBrush);
    int center=0;
    float maxRadius=rangeMax*pixelsPerMeter;
    float displayRadius=qMin(maxRadius,(float)imageSize/2);
    for(int i=1;i<=3;i++)
    {
        float radius=(displayRadius/3.0f)*i;
        painter.drawEllipse(QPointF(center,center),radius,radius);
    }
    for(int i=0;i<8;i++)
    {
        float angle=i*M_PI/4;
        float x=center+displayRadius*std::cos(angle);
        float y=center-displayRadius*std::sin(angle);
        painter.drawLine(center,center,x,y);
    }
    painter.restore();
}

void SonarWidget::drawCenterMark(QPainter &painter)
{
    painter.save();
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(255, 0, 0, 200));
    painter.drawEllipse(QPointF(width()/2.0, height()/2.0), 5, 5);
    painter.restore();
}

void SonarWidget::drawScanLine(QPainter &painter)
{
    if(!hasNewData)
        return;
    painter.save();
    painter.setPen(QPen(QColor(0, 255, 0, 180), 2));
    painter.setBrush(Qt::NoBrush);
    float maxRadius=rangeMax*pixelsPerMeter;
    float displayRadius=qMin(maxRadius,(float)imageSize/2);
    float x=displayRadius*std::cos(currentAngle);
    float y=-displayRadius*std::sin(currentAngle);
    painter.drawLine(0,0,x,y);
    painter.restore();
}
void SonarWidget::onZoomIn()
{
    displayScale=qMin(MAX_SCALE,displayScale*1.2f);
    update();
    emit scaleChanged(displayScale);
}
void SonarWidget::onZoomOut()
{
    displayScale=qMax(MIN_SCALE,displayScale/1.2f);
    update();
    emit scaleChanged(displayScale);
}
QVector<QColor> generateJetColorTable()
{
    QVector<QColor> table(256);
    for(int i=0;i<256;i++)
    {
        double t=i/255.0;
        int r=qBound(0,(int)(255*(1.5-fabs(4*t-3))),255);
        int g=qBound(0,(int)(255*(1.5-fabs(4*t-1))),255);
        int b=qBound(0,(int)(255*(1.5-fabs(4*t+1))),255);
        table[i] = QColor(r, g, b);
    }
    return table;
}