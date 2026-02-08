/**
 * ProjectCoordinateWidget 使用指南
 * ================================
 * 
 * ProjectCoordinateWidget 是用于项目坐标系选择和配置的 UI Widget。
 * 支持两种坐标系模式：
 * 
 * 1. Local Coordinate System（本地坐标系）
 *    - 使用参考点（纬度、经度、高度）定义
 *    - 使用原点（X、Y、Z）定义本地坐标系原点
 *    - 适用于小范围局部处理
 * 
 * 2. Geodetic Coordinate System（大地坐标系）
 *    - 通过 SpatialReferenceDialog 选择标准坐标系
 *    - 支持 EPSG 代码和 WKT 定义
 *    - 适用于大范围全球处理
 * 
 * 使用方式 1：直接在对话框中使用
 * ==================================
 * 
 * #include "dialogs/ProjectCoordinateDialog.h"
 * 
 * ProjectCoordinateDialog dialog(this);
 * if (dialog.exec() == QDialog::Accepted) {
 *     auto coordSys = dialog.GetCoordinateSystem();
 *     // 设置到项目
 *     m_project->input_coordinate_system = coordSys;
 * }
 * 
 * 
 * 使用方式 2：在自定义对话框中嵌入 Widget
 * =========================================
 * 
 * #include "widgets/ProjectCoordinateWidget.h"
 * 
 * ProjectCoordinateWidget* coordWidget = new ProjectCoordinateWidget();
 * // 添加到你的对话框 layout 中
 * mainLayout->addWidget(coordWidget);
 * 
 * // 从用户选择中获取坐标系
 * auto coordSys = coordWidget->GetCoordinateSystem();
 * m_project->input_coordinate_system = coordSys;
 * 
 * 
 * API 详解
 * ========
 * 
 * GetCoordinateSystem() 
 *   返回 database::CoordinateSystem 对象
 *   根据用户选择的模式生成对应的坐标系定义
 * 
 * SetCoordinateSystem(const CoordinateSystem&)
 *   设置初始的坐标系值（用于编辑现有项目）
 *   自动检测坐标系类型并更新 UI
 * 
 * IsValid()
 *   验证坐标系配置的有效性
 *   LOCAL 模式总是有效的
 *   大地坐标系需要选择过坐标
 * 
 * 
 * 实现细节
 * ========
 * 
 * LOCAL 坐标系存储格式（在 definition 字段中）：
 *   LOCAL|lat,lon,alt|x,y,z
 * 
 * 示例：
 *   LOCAL|39.9,-116.4,50|0,0,0
 *   (表示参考点在北京，原点在参考点)
 * 
 * 大地坐标系：
 *   - EPSG 优先使用 EPSG:4326 等格式
 *   - 没有 EPSG 时使用完整 WKT 字符串
 * 
 * 
 * 与 SpatialReferenceDialog 集成
 * ==============================
 * 
 * 当用户点击"Select..."按钮时：
 *   1. 打开 SpatialReferenceDialog
 *   2. 用户选择坐标系（EPSG 或 WKT）
 *   3. Coordinate 对象返回
 *   4. 提取 name, EPSG, WKT 信息
 *   5. 更新 UI 显示
 *   6. 保存到内部结构
 * 
 * 
 * 转换到 database::CoordinateSystem
 * ==================================
 * 
 * database::CoordinateSystem 结构：
 * {
 *     Type type;                          // 坐标系类型
 *     RotationConvention rotation_conv;   // 旋转约定
 *     std::string definition;             // 定义字符串
 *     std::optional<Origin> origin;       // 原点
 *     std::optional<ReferencePoint> ref;  // 参考点
 * }
 * 
 * ProjectCoordinateWidget 的转换逻辑：
 * 
 * LOCAL 模式：
 *   - type = kLocal
 *   - definition = "LOCAL|lat,lon,alt|x,y,z"
 *   - origin = {x, y, z}（如果非零）
 *   - reference = {lat, lon, alt}（如果非零）
 * 
 * 大地坐标系模式：
 *   - type = kEPSG（如有 EPSG） 或 kWKT
 *   - definition = EPSG 代码或 WKT 字符串
 */
