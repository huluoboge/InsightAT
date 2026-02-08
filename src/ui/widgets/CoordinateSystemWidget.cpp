/**
 * @file CoordinateSystemWidget.cpp
 * @brief 坐标系设置小部件 - 实现
 */

#include "CoordinateSystemWidget.h"
#include "SpatialReferenceDialog.h"

#include "Common/Coordinates.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPlainTextEdit>
#include <QComboBox>
#include <QTabWidget>
#include <QGroupBox>
#include <glog/logging.h>

namespace insight {
namespace ui {

CoordinateSystemWidget::CoordinateSystemWidget(QWidget* parent)
    : QWidget(parent)
    , m_tool(std::make_unique<SpatialReferenceTool>())
{
    initializeUI();
}

CoordinateSystemWidget::~CoordinateSystemWidget() = default;

void CoordinateSystemWidget::initializeUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    // ─────────────────────────────────────────────────────────
    // 上半部分：常见坐标系和搜索
    // ─────────────────────────────────────────────────────────
    
    // 常见坐标系选择
    QGroupBox* commonGroup = new QGroupBox("常见坐标系", this);
    QVBoxLayout* commonLayout = new QVBoxLayout(commonGroup);
    
    m_commonCoordsCombo = new QComboBox(this);
    m_commonCoordsCombo->addItem("-- 选择常见坐标系 --", -1);
    commonLayout->addWidget(m_commonCoordsCombo);
    
    mainLayout->addWidget(commonGroup);

    // 搜索部分
    QGroupBox* searchGroup = new QGroupBox("搜索坐标系", this);
    QVBoxLayout* searchLayout = new QVBoxLayout(searchGroup);
    
    QLabel* searchLabel = new QLabel("按 EPSG 码或名称搜索：", this);
    searchLayout->addWidget(searchLabel);
    
    m_searchEdit = new QLineEdit(this);
    m_searchEdit->setPlaceholderText("例：4326 或 WGS84");
    searchLayout->addWidget(m_searchEdit);
    
    QLabel* resultsLabel = new QLabel("搜索结果：", this);
    searchLayout->addWidget(resultsLabel);
    
    m_searchResultList = new QListWidget(this);
    m_searchResultList->setMaximumHeight(150);
    searchLayout->addWidget(m_searchResultList);
    
    mainLayout->addWidget(searchGroup);

    // ─────────────────────────────────────────────────────────
    // 中间部分：WKT 输入选项
    // ─────────────────────────────────────────────────────────
    
    QGroupBox* wktGroup = new QGroupBox("WKT 字符串输入", this);
    QVBoxLayout* wktLayout = new QVBoxLayout(wktGroup);
    
    QLabel* wktLabel = new QLabel("直接输入或粘贴 WKT 字符串：", this);
    wktLayout->addWidget(wktLabel);
    
    m_wktInputEdit = new QPlainTextEdit(this);
    m_wktInputEdit->setPlaceholderText("例：GEOGCS[\"WGS 84\",...");
    m_wktInputEdit->setMaximumHeight(80);
    wktLayout->addWidget(m_wktInputEdit);
    
    mainLayout->addWidget(wktGroup);

    // ─────────────────────────────────────────────────────────
    // 下半部分：选择结果显示
    // ─────────────────────────────────────────────────────────
    
    QGroupBox* detailsGroup = new QGroupBox("选择的坐标系", this);
    QVBoxLayout* detailsLayout = new QVBoxLayout(detailsGroup);
    
    // EPSG 信息
    QHBoxLayout* epsgLayout = new QHBoxLayout();
    epsgLayout->addWidget(new QLabel("EPSG 代码：", this));
    m_selectedEPSGLabel = new QLabel("未选择", this);
    m_selectedEPSGLabel->setStyleSheet("font-weight: bold;");
    epsgLayout->addWidget(m_selectedEPSGLabel);
    epsgLayout->addStretch();
    detailsLayout->addLayout(epsgLayout);
    
    // 名称信息
    QHBoxLayout* nameLayout = new QHBoxLayout();
    nameLayout->addWidget(new QLabel("坐标系名称：", this));
    m_selectedNameLabel = new QLabel("未选择", this);
    m_selectedNameLabel->setStyleSheet("font-weight: bold;");
    nameLayout->addWidget(m_selectedNameLabel);
    nameLayout->addStretch();
    detailsLayout->addLayout(nameLayout);
    
    // 坐标系类型
    QHBoxLayout* typeLayout = new QHBoxLayout();
    typeLayout->addWidget(new QLabel("坐标系类型：", this));
    m_selectedProjectedLabel = new QLabel("未选择", this);
    m_selectedProjectedLabel->setStyleSheet("font-weight: bold;");
    typeLayout->addWidget(m_selectedProjectedLabel);
    typeLayout->addStretch();
    detailsLayout->addLayout(typeLayout);
    
    // WKT 显示
    QLabel* wktDisplayLabel = new QLabel("WKT 字符串：", this);
    detailsLayout->addWidget(wktDisplayLabel);
    
    m_selectedWKTDisplay = new QPlainTextEdit(this);
    m_selectedWKTDisplay->setReadOnly(true);
    m_selectedWKTDisplay->setMaximumHeight(100);
    m_selectedWKTDisplay->setFont(QFont("Courier", 9));
    detailsLayout->addWidget(m_selectedWKTDisplay);
    
    mainLayout->addWidget(detailsGroup);

    // 连接信号槽
    connect(m_commonCoordsCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &CoordinateSystemWidget::onCommonCoordinateSelected);
    
    connect(m_searchEdit, &QLineEdit::textChanged,
            this, &CoordinateSystemWidget::onSearchTextChanged);
    
    connect(m_searchResultList, &QListWidget::itemSelectionChanged,
            this, [this]() {
                if (m_searchResultList->currentItem()) {
                    onSearchResultSelected(m_searchResultList->currentItem());
                }
            });
    
    connect(m_wktInputEdit, &QPlainTextEdit::textChanged,
            this, &CoordinateSystemWidget::onWKTInputChanged);
}

bool CoordinateSystemWidget::loadCoordinateDatabases(const std::string& configPath)
{
    // 加载两个数据库（地理和投影）
    if (!m_tool->loadCoordinateDatabases(configPath)) {
        LOG(ERROR) << "Failed to load coordinate databases from: " << configPath;
        return false;
    }

    // 填充常见坐标系下拉框
    auto commonCoords = m_tool->getCommonCoordinates();
    for (const auto& coord : commonCoords) {
        bool ok = false;
        int epsg = coord.EPSG(&ok);
        if (ok) {
            QString itemText = QString::fromStdString(coord.CoordinateName) + 
                             " (EPSG:" + QString::number(epsg) + ")";
            m_commonCoordsCombo->addItem(itemText, epsg);
        }
    }

    return true;
}

int CoordinateSystemWidget::getSelectedEPSG() const
{
    return m_selectedEPSG;
}

QString CoordinateSystemWidget::getSelectedWKT() const
{
    return m_selectedWKT;
}

QString CoordinateSystemWidget::getSelectedName() const
{
    return m_selectedName;
}

bool CoordinateSystemWidget::setSelectedEPSG(int epsg)
{
    Coordinate coord = m_tool->findByEPSG(epsg);
    if (coord.EPSGName.empty()) {
        LOG(WARNING) << "EPSG code not found: " << epsg;
        return false;
    }

    updateCoordinateDetails(coord);
    return true;
}

void CoordinateSystemWidget::clearSelection()
{
    m_selectedEPSG = 0;
    m_selectedWKT.clear();
    m_selectedName.clear();
    
    m_selectedEPSGLabel->setText("未选择");
    m_selectedNameLabel->setText("未选择");
    m_selectedProjectedLabel->setText("未选择");
    m_selectedWKTDisplay->clear();
    
    m_commonCoordsCombo->setCurrentIndex(0);
    m_searchEdit->clear();
    m_searchResultList->clear();
    m_wktInputEdit->clear();
}

void CoordinateSystemWidget::onCommonCoordinateSelected(int index)
{
    if (index <= 0) {
        clearSelection();
        return;
    }

    int epsg = m_commonCoordsCombo->currentData().toInt();
    setSelectedEPSG(epsg);
}

void CoordinateSystemWidget::onSearchTextChanged(const QString& text)
{
    updateSearchResults(text.toStdString());
}

void CoordinateSystemWidget::onSearchResultSelected(QListWidgetItem* item)
{
    if (!item) {
        return;
    }

    // 从项目数据中提取 EPSG（存储在 Qt::UserRole）
    int epsg = item->data(Qt::UserRole).toInt();
    setSelectedEPSG(epsg);
    
    // 更新其他 UI 元素
    m_commonCoordsCombo->blockSignals(true);
    for (int i = 0; i < m_commonCoordsCombo->count(); ++i) {
        if (m_commonCoordsCombo->itemData(i).toInt() == epsg) {
            m_commonCoordsCombo->setCurrentIndex(i);
            break;
        }
    }
    m_commonCoordsCombo->blockSignals(false);
}

void CoordinateSystemWidget::onWKTInputChanged()
{
    QString wkt = m_wktInputEdit->toPlainText().trimmed();
    if (!wkt.isEmpty()) {
        selectFromWKT(wkt);
    }
}

void CoordinateSystemWidget::updateSearchResults(const std::string& keyword)
{
    m_searchResultList->clear();

    if (keyword.empty()) {
        return;
    }

    auto results = m_tool->searchByKeyword(keyword);
    for (const auto& coord : results) {
        bool ok = false;
        int epsg = coord.EPSG(&ok);
        
        if (ok) {
            QString itemText = QString::fromStdString(coord.CoordinateName) + 
                             " (EPSG:" + QString::number(epsg) + ")";
            QListWidgetItem* item = new QListWidgetItem(itemText, m_searchResultList);
            item->setData(Qt::UserRole, epsg);  // 存储 EPSG 便于后续检索
        }
    }
}

void CoordinateSystemWidget::updateCoordinateDetails(const Coordinate& coord)
{
    bool ok = false;
    int epsg = coord.EPSG(&ok);
    
    if (!ok || epsg == 0) {
        LOG(WARNING) << "Invalid coordinate system";
        return;
    }

    // 更新状态
    m_selectedEPSG = epsg;
    m_selectedWKT = QString::fromStdString(coord.WKT);
    m_selectedName = QString::fromStdString(coord.CoordinateName);

    // 更新 UI 显示
    m_selectedEPSGLabel->setText(QString::number(epsg));
    m_selectedNameLabel->setText(m_selectedName);
    
    // 检查是否为投影坐标系
    bool isProjected = false;
    if (const_cast<Coordinate&>(coord).isProject(&ok)) {
        isProjected = true;
    }
    m_selectedProjectedLabel->setText(isProjected ? "投影坐标系" : "地理坐标系");
    
    m_selectedWKTDisplay->setPlainText(m_selectedWKT);

    // 发出信号
    emit coordinateSystemSelected(epsg, m_selectedName, m_selectedWKT);
}

void CoordinateSystemWidget::selectFromWKT(const QString& wktString)
{
    // 注：这是一个简化实现
    // 实际应用中可能需要根据 WKT 字符串在数据库中查找对应的 EPSG
    // 或使用 OGRSpatialReference 进行解析
    
    m_selectedWKT = wktString;
    m_selectedWKTDisplay->setPlainText(wktString);
    
    // 尝试从 WKT 提取 EPSG（简单的正则匹配）
    // 完整实现需要使用 GDAL/OGR 库
    
    LOG(INFO) << "WKT input: " << wktString.toStdString();
}

}  // namespace ui
}  // namespace insight
