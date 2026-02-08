/**
 * @file CoordinateSystemWidget.h
 * @brief 坐标系设置小部件
 * 
 * 功能：
 * 1. 通过 SpatialReferenceTool 加载所有 EPSG 坐标系
 * 2. 显示常见坐标系快捷列表
 * 3. 支持关键字搜索坐标系
 * 4. 支持手动输入 WKT 字符串
 * 5. 实时显示选择的坐标系详情
 */

#ifndef UI_COORDINATESYSTEMWIDGET_H
#define UI_COORDINATESYSTEMWIDGET_H

#include <QWidget>
#include <QString>
#include <memory>

class QLineEdit;
class QListWidget;
class QListWidgetItem;
class QLabel;
class QPlainTextEdit;
class QComboBox;

namespace insight {

struct Coordinate;

namespace ui {

class SpatialReferenceTool;

/**
 * @class CoordinateSystemWidget
 * @brief 坐标系选择和管理小部件
 */
class CoordinateSystemWidget : public QWidget {
    Q_OBJECT

public:
    explicit CoordinateSystemWidget(QWidget* parent = nullptr);
    ~CoordinateSystemWidget();

    /**
     * 加载坐标系数据库（地理和投影）
     * 
     * @param[in] configPath 配置目录路径（包含 GEOGCS_Database.csv 和 PROJCS_Database.csv）
     * @return 成功返回true
     */
    bool loadCoordinateDatabases(const std::string& configPath);

    /**
     * 获取当前选择的 EPSG 代码
     * 
     * @return EPSG 代码（如4326），未选择返回0
     */
    int getSelectedEPSG() const;

    /**
     * 获取当前选择的 WKT 字符串
     * 
     * @return WKT 字符串
     */
    QString getSelectedWKT() const;

    /**
     * 获取当前选择的坐标系名称
     * 
     * @return 坐标系名称
     */
    QString getSelectedName() const;

    /**
     * 设置选择的坐标系（按 EPSG 代码）
     * 
     * @param[in] epsg EPSG 代码
     * @return 成功返回true
     */
    bool setSelectedEPSG(int epsg);

    /**
     * 清空当前选择
     */
    void clearSelection();

signals:
    /**
     * 坐标系被选择时发出
     * 
     * @param epsg EPSG 代码
     * @param name 坐标系名称
     * @param wkt WKT 字符串
     */
    void coordinateSystemSelected(int epsg, const QString& name, const QString& wkt);

private slots:
    /**
     * 处理常见坐标系列表选择
     */
    void onCommonCoordinateSelected(int index);

    /**
     * 处理搜索框文本改变
     */
    void onSearchTextChanged(const QString& text);

    /**
     * 处理搜索结果列表选择
     */
    void onSearchResultSelected(QListWidgetItem* item);

    /**
     * 处理 WKT 手动输入（选项卡切换）
     */
    void onWKTInputChanged();

private:
    /**
     * 初始化UI
     */
    void initializeUI();

    /**
     * 更新搜索结果显示
     */
    void updateSearchResults(const std::string& keyword);

    /**
     * 更新坐标系详情显示
     */
    void updateCoordinateDetails(const Coordinate& coord);

    /**
     * 从 WKT 字符串验证并选择坐标系
     */
    void selectFromWKT(const QString& wktString);

    // 工具对象
    std::unique_ptr<SpatialReferenceTool> m_tool;

    // UI 组件
    QComboBox* m_commonCoordsCombo;          ///< 常见坐标系下拉框
    QLineEdit* m_searchEdit;                 ///< 搜索输入框
    QListWidget* m_searchResultList;         ///< 搜索结果列表
    QPlainTextEdit* m_wktInputEdit;          ///< WKT 输入框
    
    // 显示细节
    QLabel* m_selectedEPSGLabel;             ///< 显示选择的 EPSG
    QLabel* m_selectedNameLabel;             ///< 显示选择的名称
    QLabel* m_selectedProjectedLabel;        ///< 显示是否为投影坐标系
    QPlainTextEdit* m_selectedWKTDisplay;    ///< 显示选择的 WKT

    // 状态
    int m_selectedEPSG = 0;                  ///< 当前选择的 EPSG 代码
    QString m_selectedWKT;                   ///< 当前选择的 WKT
    QString m_selectedName;                  ///< 当前选择的名称
};

}  // namespace ui
}  // namespace insight

#endif  // UI_COORDINATESYSTEMWIDGET_H
