/**
 * @file main.cpp
 * @brief InsightAT 应用入口点
 */

#include <QApplication>
#include <glog/logging.h>
#include "ui/MainWindow.h"

int main(int argc, char* argv[]) {
    // 初始化 glog
    google::InitGoogleLogging(argv[0]);
    
    // 创建应用
    QApplication app(argc, argv);
    
    // 设置应用元信息
    app.setApplicationName("InsightAT");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("InsightAT");
    app.setOrganizationDomain("com.insightat");
    
    LOG(INFO) << "InsightAT application started";
    
    // 创建并显示主窗口
    insight::ui::MainWindow window;
    window.show();
    
    // 运行应用事件循环
    int result = app.exec();
    
    // 关闭 glog
    google::ShutdownGoogleLogging();
    
    return result;
}
