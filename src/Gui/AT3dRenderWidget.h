#ifndef AT3DRENDERWIDGET_H
#define AT3DRENDERWIDGET_H

#include <array>
#include <functional>

#include "InsightATGlobal.h"
#include "SubWidget.h"
#include "render/render_tracks.h"
#include "render/render_widget.h"
#include <QWidget>

#include "Common/Project.h"
#include "Common/db_types.h"
namespace Ui {
class AT3dRenderWidget;
}

namespace insight{

class AT3dRenderWidget : public QWidget {
    Q_OBJECT

public:
    explicit AT3dRenderWidget(QWidget* parent = nullptr);
    ~AT3dRenderWidget();

    void refreshDatas(const ATTask& task);

    void refreshDatas(const ModelTask& task);

public slots:
    void on_pushButton_cameraSmaller_clicked();
    void on_pushButton_cameraBigger_clicked();
    void on_pushButton_showBall_clicked(bool);
    void on_pushButton_vertexSmaller_clicked();
    void on_pushButton_vertexBigger_clicked();
    void on_pushButton_showCamera_clicked(bool);
    void on_pushButton_showVertex_clicked(bool);
    void on_pushButton_home_clicked();

    void onSetGridCount(int xcount, int ycount, int zcount);

public:
    void setGridCallBack(std::function<void(int, int, int)> setGridFunc)
    {
        _setGridFunction = setGridFunc;
    }

private:
    static void gridFunction(int, int, int) { }
    Ui::AT3dRenderWidget* ui;
    render::RenderWidget* _renderWidget;
    render::RenderTracks* _tracks;
    std::function<void(int, int, int)> _setGridFunction;
    // std::array<double,3> _center;
};

}//name space insight
#endif // AT3DRENDERWIDGET_H
