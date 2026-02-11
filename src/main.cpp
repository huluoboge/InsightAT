/**
 * @file main.cpp
 * @brief InsightAT 应用入口点
 */

#include <QApplication>
#include <QDir>
#include <glog/logging.h>
#include "ui/MainWindow.h"

int main(int argc, char* argv[]) {
    // 配置 GLOG 默认标志
    // 默认输出到终端，便于通过命令行查看调试信息
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true; // 开启带颜色的日志
    
    // 如果需要保存到文件，可以取消下面注释
    // QDir().mkpath("logs");
    // FLAGS_log_dir = "logs";
    // FLAGS_alsologtostderr = true; // 同时输出到 stderr 和文件

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
