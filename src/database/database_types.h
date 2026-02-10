/**
 * @file database_types.h
 * @brief InsightAT数据库核心类型定义
 * @author InsightAT Team
 * @date 2026-02-08
 * 
 * 统一的数据库类型定义包括：
 * - CoordinateSystem: 坐标系描述和管理
 * - InputPose: 输入位姿（GNSS/IMU数据）
 * - Measurement: 测量框架（GNSS/IMU/GCP/SLAM）
 * - ATTask: 空三任务及其快照设计
 */

#ifndef INSIGHT_DATABASE_TYPES_H
#define INSIGHT_DATABASE_TYPES_H

#include <string>
#include <optional>
#include <vector>
#include <map>
#include <cereal/cereal.hpp>

// ─────────────────────────────────────────────────────────────
// Cereal support for std::optional
// ─────────────────────────────────────────────────────────────
namespace cereal {
    // Cereal 对 std::optional 的序列化支持
    template <class Archive, class T>
    void save(Archive& ar, const std::optional<T>& opt) {
        if (opt.has_value()) {
            ar(true, *opt);
        } else {
            ar(false);
        }
    }

    template <class Archive, class T>
    void load(Archive& ar, std::optional<T>& opt) {
        bool has_value;
        ar(has_value);
        if (has_value) {
            T value;
            ar(value);
            opt = value;
        } else {
            opt = std::nullopt;
        }
    }
}  // namespace cereal

namespace insight {
namespace database {

// Forward declarations
struct GCPMeasurement;

// ─────────────────────────────────────────────────────────────
// CoordinateSystem: 坐标系定义
// ─────────────────────────────────────────────────────────────

/**
 * @struct CoordinateSystem
 * @brief 坐标系描述符 - 支持EPSG/WKT/ENU/Local
 */
struct CoordinateSystem {
    enum class Type : int {
        kEPSG = 0,      ///< EPSG代码
        kWKT = 1,       ///< OGC WKT字符串
        kENU = 2,       ///< ENU本地坐标系
        kLocal = 3      ///< Local本地/未知坐标系
    };

    enum class RotationConvention : int {
        kOmegaPhiKappa = 0,     ///< 摄影测量 (ω,φ,κ)
        kYawPitchRoll = 1       ///< 航空学 (Y,P,R)
    };

    struct Origin {
        double x = 0.0, y = 0.0, z = 0.0;

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const /*version*/) {
            ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
        }
    };

    struct ReferencePoint {
        double lat = 0.0, lon = 0.0, alt = 0.0;

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const /*version*/) {
            ar(CEREAL_NVP(lat), CEREAL_NVP(lon), CEREAL_NVP(alt));
        }
    };

    Type type = Type::kLocal;
    RotationConvention rotation_convention = RotationConvention::kOmegaPhiKappa;
    std::string definition;
    std::optional<Origin> origin;
    std::optional<ReferencePoint> reference;

    std::string ToString() const;
    bool IsValid() const;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(type), CEREAL_NVP(rotation_convention), CEREAL_NVP(definition));
        if (version > 0) {
            // 使用 has_value() 检查 optional 是否有值
            bool has_origin = origin.has_value();
            bool has_reference = reference.has_value();
            
            ar(CEREAL_NVP(has_origin));
            if (has_origin) {
                ar(CEREAL_NVP(origin));
            }
            
            ar(CEREAL_NVP(has_reference));
            if (has_reference) {
                ar(CEREAL_NVP(reference));
            }
        }
    }
};

// ─────────────────────────────────────────────────────────────
// InputPose: 输入位姿
// ─────────────────────────────────────────────────────────────

/**
 * @struct InputPose
 * @brief 输入位姿 - 轻量级测量表示
 */
struct InputPose {
    enum class AngleUnit : int {
        kDegrees = 0,
        kRadians = 1
    };

    double x = 0.0, y = 0.0, z = 0.0;
    bool has_position = false;

    double omega = 0.0, phi = 0.0, kappa = 0.0;
    bool has_rotation = false;

    AngleUnit angle_unit = AngleUnit::kDegrees;

    void Reset();
    bool HasData() const;
    std::string ToString() const;
    bool IsValid() const;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const /*version*/) {
        ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z), CEREAL_NVP(has_position));
        ar(CEREAL_NVP(omega), CEREAL_NVP(phi), CEREAL_NVP(kappa), CEREAL_NVP(has_rotation));
        ar(CEREAL_NVP(angle_unit));
    }
};

// ─────────────────────────────────────────────────────────────
// GCPMeasurement: 地面控制点（独立存储）
// ─────────────────────────────────────────────────────────────

/**
 * @struct GCPMeasurement
 * @brief 地面控制点(GCP)数据 - 独立存储于Project中
 * 
 * 设计说明：
 * - GCP数据单一来源（在Project.gcp_database中集中管理）
 * - 不散布在Measurement中，避免数据冗余
 * - 包含完整的3D坐标和多图像观测信息
 * - 图像中根据需要通过缓存索引关联GCP
 */
struct GCPMeasurement {
    /**
     * GCP观测 - 一个GCP在一个图像中的观测
     * 
     * 设计说明：
     * - 一个GCP可在多张图像中被观测
     * - 每个观测记录图像ID和对应的像素坐标
     * - 像素坐标的不确定度可选（用于加权约束）
     */
    struct Observation {
        uint32_t image_id = static_cast<uint32_t>(-1);  ///< 观测该GCP的图像ID
        double pixel_x = 0.0, pixel_y = 0.0;            ///< 像素坐标
        double pixel_cov_x = 0.0, pixel_cov_y = 0.0;    ///< 像素坐标方差（可选）
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const /*version*/) {
            ar(CEREAL_NVP(image_id), CEREAL_NVP(pixel_x), CEREAL_NVP(pixel_y));
            ar(CEREAL_NVP(pixel_cov_x), CEREAL_NVP(pixel_cov_y));
        }
    };
    
    uint32_t gcp_id = static_cast<uint32_t>(-1);        ///< 控制点ID（全局唯一）
    std::string gcp_name;                               ///< 控制点名称（可选）
    
    // 3D坐标（参考坐标系）
    double x = 0.0, y = 0.0, z = 0.0;                   ///< 3D坐标值
    double cov_xx = 0.0, cov_yy = 0.0, cov_zz = 0.0;   ///< 坐标方差
    double cov_xy = 0.0, cov_xz = 0.0, cov_yz = 0.0;   ///< 坐标协方差
    
    // 图像观测列表（多图像观测）
    std::vector<Observation> observations;               ///< 该GCP在各个图像中的观测

    bool IsValid() const;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const /*version*/) {
        ar(CEREAL_NVP(gcp_id), CEREAL_NVP(gcp_name));
        ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
        ar(CEREAL_NVP(cov_xx), CEREAL_NVP(cov_yy), CEREAL_NVP(cov_zz));
        ar(CEREAL_NVP(cov_xy), CEREAL_NVP(cov_xz), CEREAL_NVP(cov_yz));
        ar(CEREAL_NVP(observations));
    }
};

// ─────────────────────────────────────────────────────────────
// Measurement: 测量框架
// ─────────────────────────────────────────────────────────────

/**
 * @struct Measurement
 * @brief 统一的测量数据框架 - GNSS/IMU/SLAM
 * 
 * 注意：GCP数据不再在此包含，而是独立存储于Project.gcp_database中
 */
struct Measurement {
    enum class Type : int {
        kGNSS = 0,
        kIMU = 1,
        kGCP = 2,       ///< 已弃用：GCP现在在Project中集中管理
        kSLAM = 3,
        kOther = 255
    };

    // GNSS测量
    struct GNSSMeasurement {
        double x = 0.0, y = 0.0, z = 0.0;
        double cov_xx = 0.0, cov_yy = 0.0, cov_zz = 0.0;
        double cov_xy = 0.0, cov_xz = 0.0, cov_yz = 0.0;
        uint8_t num_satellites = 0;
        double hdop = 0.0, vdop = 0.0;

        bool IsValid() const;

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const /*version*/) {
            ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
            ar(CEREAL_NVP(cov_xx), CEREAL_NVP(cov_yy), CEREAL_NVP(cov_zz));
            ar(CEREAL_NVP(cov_xy), CEREAL_NVP(cov_xz), CEREAL_NVP(cov_yz));
            ar(CEREAL_NVP(num_satellites), CEREAL_NVP(hdop), CEREAL_NVP(vdop));
        }
    };

    // IMU测量
    struct IMUMeasurement {
        bool has_attitude = false;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        double cov_att_xx = 0.0, cov_att_yy = 0.0, cov_att_zz = 0.0;

        bool has_accel = false;
        double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
        double cov_acc_xx = 0.0, cov_acc_yy = 0.0, cov_acc_zz = 0.0;

        bool has_gyro = false;
        double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
        double cov_gyr_xx = 0.0, cov_gyr_yy = 0.0, cov_gyr_zz = 0.0;

        bool IsValid() const;

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const /*version*/) {
            ar(CEREAL_NVP(has_attitude), CEREAL_NVP(roll), CEREAL_NVP(pitch), CEREAL_NVP(yaw));
            ar(CEREAL_NVP(cov_att_xx), CEREAL_NVP(cov_att_yy), CEREAL_NVP(cov_att_zz));
            ar(CEREAL_NVP(has_accel), CEREAL_NVP(accel_x), CEREAL_NVP(accel_y), CEREAL_NVP(accel_z));
            ar(CEREAL_NVP(cov_acc_xx), CEREAL_NVP(cov_acc_yy), CEREAL_NVP(cov_acc_zz));
            ar(CEREAL_NVP(has_gyro), CEREAL_NVP(gyro_x), CEREAL_NVP(gyro_y), CEREAL_NVP(gyro_z));
            ar(CEREAL_NVP(cov_gyr_xx), CEREAL_NVP(cov_gyr_yy), CEREAL_NVP(cov_gyr_zz));
        }
    };

    // SLAM相对位姿测量
    struct SLAMMeasurement {
        uint32_t reference_image_id = static_cast<uint32_t>(-1);
        double rel_x = 0.0, rel_y = 0.0, rel_z = 0.0;
        double rel_qx = 0.0, rel_qy = 0.0, rel_qz = 0.0, rel_qw = 1.0;
        double confidence = 0.0;

        bool IsValid() const;

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const /*version*/) {
            ar(CEREAL_NVP(reference_image_id));
            ar(CEREAL_NVP(rel_x), CEREAL_NVP(rel_y), CEREAL_NVP(rel_z));
            ar(CEREAL_NVP(rel_qx), CEREAL_NVP(rel_qy), CEREAL_NVP(rel_qz), CEREAL_NVP(rel_qw));
            ar(CEREAL_NVP(confidence));
        }
    };

    Type type = Type::kOther;
    uint32_t image_id = static_cast<uint32_t>(-1);
    int64_t timestamp = 0;  // milliseconds

    std::optional<GNSSMeasurement> gnss;
    std::optional<IMUMeasurement> imu;
    std::optional<SLAMMeasurement> slam;

    std::string ToString() const;
    bool IsValid() const;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const /*version*/) {
        ar(CEREAL_NVP(type), CEREAL_NVP(image_id), CEREAL_NVP(timestamp));
        
        // 处理可选的 GNSS/IMU/SLAM 测量
        bool has_gnss = gnss.has_value();
        bool has_imu = imu.has_value();
        bool has_slam = slam.has_value();
        
        ar(CEREAL_NVP(has_gnss));
        if (has_gnss) ar(CEREAL_NVP(gnss));
        
        ar(CEREAL_NVP(has_imu));
        if (has_imu) ar(CEREAL_NVP(imu));
        
        ar(CEREAL_NVP(has_slam));
        if (has_slam) ar(CEREAL_NVP(slam));
    }
};

// ─────────────────────────────────────────────────────────────
// ATTask: 空三任务
// ─────────────────────────────────────────────────────────────

/**
 * @struct ATTask
 * @brief 空三任务及其快照设计
 */
struct OptimizedPose {
    double x = 0.0, y = 0.0, z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const /*version*/) {
        ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
        ar(CEREAL_NVP(qx), CEREAL_NVP(qy), CEREAL_NVP(qz), CEREAL_NVP(qw));
    }
};

// ─────────────────────────────────────────────────────────────
// CameraModel & Image: 相机参数和图像数据结构
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// CameraRig: 相机配置（多相机固定几何关系）
// ─────────────────────────────────────────────────────────────

/**
 * @struct CameraRig
 * @brief 相机配置 - 表示多个相机的固定几何关系
 * 
 * 设计场景：
 * - 倾斜摄影：5个相机固定安装（垂直+4斜视）
 * - 多视角系统：多个同步采集的相机
 * - 立体对：左右相机的相对位置
 * 
 * 设计原则：
 * - Rig定义了多个相机之间的相对位置/姿态关系
 * - 相对关系可能已标定（固定）、未知（待优化）或部分已知
 * - 在BA中根据calib_status应用相应的约束
 */
struct CameraRig {
    enum class CalibrationStatus : int {
        kUnknown = 0,      ///< 相对位置未知，需在BA中优化
        kKnown = 1,        ///< 相对位置已标定，在BA中固定
        kPartial = 2       ///< 部分参数已知，部分待优化
    };
    
    /**
     * @struct CameraMount
     * @brief Rig中的单个相机挂载点
     */
    struct CameraMount {
        uint32_t camera_id = 0;            ///< 相机ID（在该Rig内唯一）
        std::string position_name;         ///< 位置名称（如"Nadir", "Forward", "Port"）
        
        // ── 相对于Rig基准坐标系的位置和姿态
        double rel_x = 0.0, rel_y = 0.0, rel_z = 0.0;           ///< 相对位置 (单位：m)
        double rel_qx = 0.0, rel_qy = 0.0, rel_qz = 0.0, rel_qw = 1.0;  ///< 相对姿态 (四元数)
        
        // ── 位置和姿态的不确定度（如果已标定）
        double cov_pos_xx = 0.0, cov_pos_yy = 0.0, cov_pos_zz = 0.0;  ///< 位置方差
        double cov_rot_xx = 0.0, cov_rot_yy = 0.0, cov_rot_zz = 0.0;  ///< 旋转方差
        
        bool IsValid() const;
        std::string ToString() const;
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const /*version*/) {
            ar(CEREAL_NVP(camera_id), CEREAL_NVP(position_name));
            ar(CEREAL_NVP(rel_x), CEREAL_NVP(rel_y), CEREAL_NVP(rel_z));
            ar(CEREAL_NVP(rel_qx), CEREAL_NVP(rel_qy), CEREAL_NVP(rel_qz), CEREAL_NVP(rel_qw));
            ar(CEREAL_NVP(cov_pos_xx), CEREAL_NVP(cov_pos_yy), CEREAL_NVP(cov_pos_zz));
            ar(CEREAL_NVP(cov_rot_xx), CEREAL_NVP(cov_rot_yy), CEREAL_NVP(cov_rot_zz));
        }
    };
    
    uint32_t rig_id = 0;                   ///< 配置ID（全局唯一）
    std::string rig_name;                  ///< 配置名称（如"DJI-Zenmuse-5"）
    CalibrationStatus calib_status = CalibrationStatus::kUnknown;  ///< 标定状态
    
    std::vector<CameraMount> mounts;       ///< 所有相机挂载点
    
    std::string description;               ///< Rig描述
    
    // ── 方法
    /**
     * 验证Rig的有效性
     * - 必须至少有一个相机
     * - 所有相机ID唯一
     * - 所有挂载点有效
     */
    bool IsValid() const;
    
    /**
     * 根据camera_id查找挂载点
     * 
     * @param[in] camera_id 相机ID
     * @return 挂载点指针，未找到返回nullptr
     */
    const CameraMount* FindCameraMount(uint32_t camera_id) const;
    
    /**
     * 获取详细描述
     */
    std::string ToString() const;
    
    /**
     * 获取简洁摘要
     */
    std::string GetSummary() const;
    
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const /*version*/) {
        ar(CEREAL_NVP(rig_id), CEREAL_NVP(rig_name), CEREAL_NVP(calib_status));
        ar(CEREAL_NVP(mounts), CEREAL_NVP(description));
    }
};

// ─────────────────────────────────────────────────────────────
// OptimizationFlags: 参数优化标记
// ─────────────────────────────────────────────────────────────

/**
 * @struct OptimizationFlags
 * @brief 相机参数优化标记 - 标明在BA中哪些参数需要优化，哪些固定
 * 
 * 设计原则：数据和算法分离
 * - OptimizationFlags 仅作为数据容器，标记参数状态
 * - 实际的优化算法独立实现于外部工具模块
 * 
 * 使用场景：
 * - 简单参数化：只优化k1（广角镜头常见）
 * - 完全参数化：优化所有参数（精密测量）
 * - 固定内参：仅优化位姿（已标定的相机）
 */
struct OptimizationFlags {
    // ─────────────────────────────────────────
    // 内参优化标记
    // ─────────────────────────────────────────
    
    bool focal_length = false;              ///< 是否优化焦距 f
    bool principal_point_x = false;         ///< 是否优化主点X (ppx/cx)
    bool principal_point_y = false;         ///< 是否优化主点Y (ppy/cy)
    bool aspect_ratio = false;              ///< 是否优化宽高比 (fy/fx)
    bool skew = false;                      ///< 是否优化像素偏斜
    
    // ─────────────────────────────────────────
    // Brown畸变参数优化标记
    // ─────────────────────────────────────────
    
    bool k1 = false;                        ///< 1阶径向畸变
    bool k2 = false;                        ///< 2阶径向畸变
    bool k3 = false;                        ///< 3阶径向畸变
    bool k4 = false;                        ///< 4阶径向畸变（可选）
    
    bool p1 = false;                        ///< 1阶切向畸变
    bool p2 = false;                        ///< 2阶切向畸变
    
    bool b1 = false;                        ///< 薄棱镜畸变 X
    bool b2 = false;                        ///< 薄棱镜畸变 Y

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const /*version*/) {
        ar(CEREAL_NVP(focal_length), CEREAL_NVP(principal_point_x), 
           CEREAL_NVP(principal_point_y), CEREAL_NVP(aspect_ratio), CEREAL_NVP(skew));
        ar(CEREAL_NVP(k1), CEREAL_NVP(k2), CEREAL_NVP(k3), CEREAL_NVP(k4));
        ar(CEREAL_NVP(p1), CEREAL_NVP(p2), CEREAL_NVP(b1), CEREAL_NVP(b2));
    }
};

/**
 * @struct CameraModel
 * @brief 相机内参模型 - 基于Brown-Conrady模型的完整参数
 * 
 * 设计原则：数据和算法分离
 * - CameraModel 是纯数据结构，包含相机的所有参数
 * - 不包含算法方法（如参数估计、优化等）
 * - 算法独立实现于 camera_utils.h 模块中
 * 
 * 支持的相机模型：
 * - Pinhole：标准针孔相机（无畸变）
 * - Brown-Conrady：完整的8参数畸变模型（k1,k2,k3,p1,p2,b1,b2）
 * - SimpleDistortion：简化模型（仅k1,k2）
 * - Fisheye：鱼眼镜头
 * 
 * Brown-Conrady畸变模型定义：
 *   r² = u_norm² + v_norm²
 *   
 *   r_dist = r × (1 + k1×r² + k2×r⁴ + k3×r⁶ + k4×r⁸)
 *   
 *   u_distorted = u_norm × r_dist + p1×(2×u_norm×v_norm + p2×(r² + 2×u_norm²))
 *               + b1×u_norm + b2×v_norm
 *   
 *   v_distorted = v_norm × r_dist + p2×(2×u_norm×v_norm + p1×(r² + 2×v_norm²))
 *               + b2×u_norm + b1×v_norm
 * 
 * 参数估计和优化算法独立实现于：src/camera/camera_utils.h
 */
struct CameraModel {
    enum class Type : int {
        kPinhole = 0,           ///< 标准针孔相机（无畸变）
        kBrownConrady = 1,      ///< Brown-Conrady完整畸变模型
        kSimpleDistortion = 2,  ///< 简化畸变（仅k1,k2）
        kFisheye = 3,           ///< 鱼眼镜头
        kOther = 255
    };

    Type type = Type::kBrownConrady;
    
    // ─────────────────────────────────────────────────────────
    // 1. 传感器物理信息
    // ─────────────────────────────────────────────────────────
    
    uint32_t width = 0;                     ///< 图像宽度（像素）
    uint32_t height = 0;                    ///< 图像高度（像素）
    
    double sensor_width_mm = 0.0;           ///< 传感器物理宽度 (mm)
    double sensor_height_mm = 0.0;          ///< 传感器物理高度 (mm)
    double pixel_size_um = 0.0;             ///< 像素大小 (微米)
    double focal_length_35mm = 0.0;         ///< 35mm等效焦距 (mm)
    
    // ─────────────────────────────────────────────────────────
    // 2. 内参（标准Pinhole模型）
    // ─────────────────────────────────────────────────────────
    
    double focal_length = 0.0;              ///< 焦距 (像素) - f
    double principal_point_x = 0.0;         ///< 主点X坐标 (像素) - ppx/cx
    double principal_point_y = 0.0;         ///< 主点Y坐标 (像素) - ppy/cy
    double aspect_ratio = 1.0;              ///< 焦距宽高比 fy/fx
    double skew = 0.0;                      ///< 像素偏斜系数
    
    // ─────────────────────────────────────────────────────────
    // 3. Brown-Conrady畸变参数
    // ─────────────────────────────────────────────────────────
    
    double k1 = 0.0;                       ///< 1阶径向畸变
    double k2 = 0.0;                       ///< 2阶径向畸变
    double k3 = 0.0;                       ///< 3阶径向畸变
    double k4 = 0.0;                       ///< 4阶径向畸变
    
    double p1 = 0.0;                       ///< 1阶切向畸变
    double p2 = 0.0;                       ///< 2阶切向畸变
    
    double b1 = 0.0;                       ///< 薄棱镜畸变 X
    double b2 = 0.0;                       ///< 薄棱镜畸变 Y
    
    // ─────────────────────────────────────────────────────────
    // 4. 元数据和优化标记
    // ─────────────────────────────────────────────────────────
    
    std::string camera_name;                ///< 相机型号名称
    std::string make;                       ///< 制造商
    std::string model;                      ///< 具体型号
    std::string lens_model;                 ///< 镜头型号
    uint32_t serial_number = 0;             ///< 相机序列号
    
    OptimizationFlags optimization_flags;   ///< BA中的优化标记
    
    // ─────────────────────────────────────────────────────────
    // 5. 方法（仅数据验证和格式化，不包含算法）
    // ─────────────────────────────────────────────────────────
    
    /**
     * @brief 验证相机参数的有效性
     * @return 所有关键参数有效返回true
     * 
     * 验证项：
     * - 分辨率 > 0
     * - 焦距 > 0 且合理
     * - 主点在图像范围内
     */
    bool IsValid() const;
    
    /**
     * @brief 获取详细的相机参数描述
     * @return 多行格式化字符串
     */
    std::string ToString() const;
    
    /**
     * @brief 获取简洁的相机摘要
     * @return 单行摘要
     */
    std::string GetSummary() const;
    
    /**
     * @brief 检查是否有任何畸变参数非零
     * @return 如果任何畸变参数非零返回true
     */
    bool HasDistortion() const;
    
    /**
     * @brief 重置所有参数为默认值
     */
    void Reset();

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(type), CEREAL_NVP(width), CEREAL_NVP(height));
        ar(CEREAL_NVP(sensor_width_mm), CEREAL_NVP(sensor_height_mm), 
           CEREAL_NVP(pixel_size_um), CEREAL_NVP(focal_length_35mm));
        ar(CEREAL_NVP(focal_length), CEREAL_NVP(principal_point_x), 
           CEREAL_NVP(principal_point_y), CEREAL_NVP(aspect_ratio), CEREAL_NVP(skew));
        ar(CEREAL_NVP(k1), CEREAL_NVP(k2), CEREAL_NVP(k3), CEREAL_NVP(k4));
        ar(CEREAL_NVP(p1), CEREAL_NVP(p2), CEREAL_NVP(b1), CEREAL_NVP(b2));
        ar(CEREAL_NVP(camera_name), CEREAL_NVP(make), CEREAL_NVP(model), 
           CEREAL_NVP(lens_model), CEREAL_NVP(serial_number));
        if (version > 0) {
            ar(CEREAL_NVP(optimization_flags));
        }
    }
};

/**
 * @struct Image
 * @brief 单张图像的基本信息
 */
struct Image {
    uint32_t image_id = 0;                  ///< 图像ID（唯一）
    std::string filename;                   ///< 图像文件名
    InputPose input_pose;                   ///< 初始位姿估计
    std::optional<CameraModel> camera;      ///< 图像级相机参数（可选，仅在图像级模式时使用）
    std::optional<Measurement::GNSSMeasurement> gnss_data;  ///< v2: GNSS测量数据（可选）

    std::string ToString() const;
    bool IsValid() const;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(image_id), CEREAL_NVP(filename));
        ar(CEREAL_NVP(input_pose));
        
        // 处理可选的 camera
        bool has_camera = camera.has_value();
        ar(CEREAL_NVP(has_camera));
        if (has_camera) {
            ar(CEREAL_NVP(camera));
        }
        
        // v2+: 处理可选的 gnss_data
        if (version >= 2) {
            bool has_gnss = gnss_data.has_value();
            ar(CEREAL_NVP(has_gnss));
            if (has_gnss) {
                ar(CEREAL_NVP(gnss_data));
            }
        }
    }
};

/**
 * @struct ImageGroup
 * @brief 图像分组 - 支持组级和图像级两种相机参数管理模式
 * 
 * 设计灵感：ContextCapture ImageGroup
 * 
 * 两种工作模式：
 * 1. GroupLevel: 所有图像共享一个相机参数（group_camera）
 *    - 优点：参数管理简洁，内存占用低
 *    - 适用：同一相机采集的图像集合
 * 
 * 2. ImageLevel: 每个图像有自己的相机参数（image.camera）
 *    - 优点：支持混合相机、相机漂移等复杂场景
 *    - 缺点：参数管理复杂，内存占用高
 */
struct ImageGroup {
    enum class CameraMode : int {
        kGroupLevel = 0,    ///< 组级模式：所有图像共享一个相机参数
        kImageLevel = 1,    ///< 图像级模式：每个图像有自己的相机参数
        kRigBased = 2       ///< Rig模式：图像来自多相机配置中的特定相机
    };

    /**
     * @struct RigMountInfo
     * @brief Rig挂载点信息 - 用于Rig模式
     * 
     * 当camera_mode == kRigBased时，此结构体指定该分组内的图像
     * 来自指定Rig的哪个相机
     */
    struct RigMountInfo {
        uint32_t rig_id = 0;        ///< 所属的Rig ID
        uint32_t camera_id = 0;     ///< 该Rig内的相机ID
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const /*version*/) {
            ar(CEREAL_NVP(rig_id), CEREAL_NVP(camera_id));
        }
    };

    // ── 分组信息
    uint32_t group_id = 0;                  ///< 分组ID（唯一）
    std::string group_name;                 ///< 分组名称（可选）
    CameraMode camera_mode = CameraMode::kGroupLevel;  ///< 相机参数模式

    // ── 组级相机（仅在kGroupLevel模式时使用）
    std::optional<CameraModel> group_camera;            ///< 组共享的相机参数
    
    // ── Rig信息（仅在kRigBased模式时使用）
    std::optional<RigMountInfo> rig_mount_info;         ///< Rig挂载点信息

    // ── 图像集合
    std::vector<Image> images;              ///< 分组内的所有图像

    // ── 元数据
    std::string description;                ///< 分组描述
    int64_t creation_time = 0;              ///< 创建时间戳

    // ── 方法
    /**
     * 为组内所有图像应用相机参数
     * 
     * @param[in] camera 相机参数
     * @param[in] mode 模式（组级或图像级）
     * 
     * 在组级模式下：直接设置 group_camera
     * 在图像级模式下：为所有未设置相机的图像应用此参数
     */
    void ApplyCameraModel(const CameraModel& camera, CameraMode mode);

    /**
     * 获取特定图像的相机参数
     * 
     * @param[in] image_id 图像ID
     * @return 相机参数指针，未找到返回nullptr
     * 
     * - 在组级模式下：返回 group_camera（如果存在）
     * - 在图像级模式下：返回图像自己的参数（如果存在）
     * - 在Rig模式下：需在Project中查询对应的Rig和相机参数
     */
    const CameraModel* GetCameraForImage(uint32_t image_id) const;

    /**
     * 添加图像到分组
     * 
     * @param[in] image 图像对象
     * @return 添加成功返回true
     */
    bool AddImage(const Image& image);

    /**
     * 找到图像在分组中的索引
     * 
     * @param[in] image_id 图像ID
     * @return 索引位置，未找到返回 -1
     */
    int FindImageIndex(uint32_t image_id) const;

    /**
     * 验证分组的一致性
     * 
     * @return 有效返回true
     * 
     * 检查项：
     * - 所有图像ID唯一
     * - 在组级模式下group_camera必须存在
     * - 在图像级模式下每个图像必须有相机参数
     */
    bool IsValid() const;

    /**
     * 获取人类可读的描述
     * 
     * @return 分组的格式化描述字符串
     */
    std::string ToString() const;

    /**
     * 从组级模式转换到图像级模式
     * 
     * 会将 group_camera 复制到所有未设置相机的图像
     * 返回后可安全清空 group_camera
     */
    bool ConvertToImageLevel();

    /**
     * 从图像级模式转换到组级模式
     * 
     * @return 转换成功返回true（要求所有图像的相机参数相同）
     */
    bool ConvertToGroupLevel();

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(group_id), CEREAL_NVP(group_name), CEREAL_NVP(camera_mode));
        
        // 处理可选的 group_camera
        bool has_group_camera = group_camera.has_value();
        ar(CEREAL_NVP(has_group_camera));
        if (has_group_camera) {
            ar(CEREAL_NVP(group_camera));
        }
        
        ar(CEREAL_NVP(images));
        ar(CEREAL_NVP(description), CEREAL_NVP(creation_time));
        
        if (version > 1) {
            // 处理可选的 rig_mount_info
            bool has_rig_mount_info = rig_mount_info.has_value();
            ar(CEREAL_NVP(has_rig_mount_info));
            if (has_rig_mount_info) {
                ar(CEREAL_NVP(rig_mount_info));
            }
        }
    }
};

// ─────────────────────────────────────────────────────────────
// OptimizationConfig: 空三任务的优化参数配置
// ─────────────────────────────────────────────────────────────

/**
 * @struct OptimizationConfig
 * @brief 空三任务的优化参数配置 - 控制BA中如何优化相机参数和约束条件
 * 
 * 包含：
 * - 按相机ID映射的参数优化标记（每个相机独立控制）
 * - GNSS约束开关和权重
 * - 迭代优化的其他通用参数
 */
struct OptimizationConfig {
    /// 相机参数优化标记，按camera_id映射
    std::map<uint32_t, OptimizationFlags> camera_optimization;
    
    /// GNSS约束是否启用
    bool enable_gnss_constraint = true;
    
    /// GNSS权重（通常为 1/sigma²）
    double gnss_weight = 1.0;
    
    /// 最大重投影误差阈值（像素）
    double max_reprojection_error = 10.0;
    
    /// 备注
    std::string description;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const /*version*/) {
        ar(CEREAL_NVP(camera_optimization), CEREAL_NVP(enable_gnss_constraint),
           CEREAL_NVP(gnss_weight), CEREAL_NVP(max_reprojection_error),
           CEREAL_NVP(description));
    }
};

struct ATTask {
    struct InputSnapshot {
        CoordinateSystem input_coordinate_system;
        std::vector<Measurement> measurements;
        std::vector<ImageGroup> image_groups;  ///< 输入图像分组（可选，但推荐）

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(input_coordinate_system), CEREAL_NVP(measurements));
            if (version > 0) {
                ar(CEREAL_NVP(image_groups));
            }
        }
    };

    struct Initialization {
        uint32_t prev_task_id = static_cast<uint32_t>(-1);
        std::map<uint32_t, OptimizedPose> initial_poses;

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const /*version*/) {
            ar(CEREAL_NVP(prev_task_id), CEREAL_NVP(initial_poses));
        }
    };

    std::string id;                                 ///< 任务UUID（唯一标识）
    uint32_t task_id = 0;                           ///< 任务整型ID（用于持久化引用）
    std::string task_name;                          ///< 任务名称，e.g. "AT_0", "AT_1"（用户友好）
    InputSnapshot input_snapshot;
    std::optional<Initialization> initialization;
    CoordinateSystem output_coordinate_system;
    std::map<uint32_t, OptimizedPose> optimized_poses;
    OptimizationConfig optimization_config;         ///< 优化参数配置（v3新增）

    /**
     * 从输入图像分组获取指定图像的相机参数
     * 
     * @param[in] group_id 分组ID
     * @param[in] image_id 图像ID
     * @return 相机参数指针，未找到返回nullptr
     */
    const CameraModel* GetCameraForImage(uint32_t group_id, uint32_t image_id) const;

    /**
     * 查找包含指定图像的分组
     * 
     * @param[in] image_id 图像ID
     * @return 分组指针，未找到返回nullptr
     */
    const ImageGroup* FindGroupByImageId(uint32_t image_id) const;

    /**
     * 获取所有有效图像总数（跨所有分组）
     * 
     * @return 图像总数
     */
    size_t GetTotalImageCount() const;

    std::string ToString() const;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(id));
        if (version >= 4) {
            ar(CEREAL_NVP(task_id));
        }
        if (version >= 1) {
            ar(CEREAL_NVP(task_name));
        }
        ar(CEREAL_NVP(input_snapshot));
        ar(CEREAL_NVP(initialization), CEREAL_NVP(output_coordinate_system));
        ar(CEREAL_NVP(optimized_poses));
        if (version >= 3) {
            ar(CEREAL_NVP(optimization_config));
        }
    }
};

// ─────────────────────────────────────────────────────────────
// Project: 项目元数据和输入数据容器
// ─────────────────────────────────────────────────────────────

/**
 * @struct Project
 * @brief 项目结构体 - 包含项目基本信息和所有输入数据
 * 
 * Project 是一个高层容器，包含：
 * 1. 项目元数据（名称、UUID、创建时间、描述、作者等）
 * 2. 输入数据（坐标系、测量数据、图像分组）
 * 3. 初始位姿信息（可选）
 * 4. 一个或多个空三任务
 */
struct Project {
    // ── 项目元数据（基本信息）
    std::string name;                       ///< 项目名称
    std::string uuid;                       ///< 项目唯一标识符（UUID v4）
    int64_t creation_time = 0;              ///< 项目创建时间（Unix 时间戳，秒）
    std::string description;                ///< 项目描述
    std::string author;                     ///< 项目作者/操作员
    
    // ── 项目版本管理
    std::string project_version = "1.0";    ///< 项目文件格式版本
    int64_t last_modified_time = 0;         ///< 最后修改时间（Unix 时间戳，秒）
    
    // ── 项目标签（可选）
    std::vector<std::string> tags;          ///< 项目标签（用于分类和搜索）
    
    // ── 输入数据（Inputs）
    CoordinateSystem input_coordinate_system;  ///< 输入坐标系
    std::vector<Measurement> measurements;     ///< 所有输入测量数据（GNSS/IMU/SLAM）
    std::vector<ImageGroup> image_groups;      ///< 所有输入图像分组
    
    // ── GCP数据库（单一来源）
    std::map<uint32_t, GCPMeasurement> gcp_database;  ///< GCP集中存储（gcp_id -> GCPMeasurement）
    
    // ── 缓存索引（懒加载）
    mutable std::map<uint32_t, std::vector<uint32_t>> image_to_gcp_cache;  ///< 图像ID -> GCP IDs（缓存）
    mutable bool cache_valid = false;  ///< 缓存有效性标志
    
    // ── 相机配置（多相机固定几何关系）
    std::map<uint32_t, CameraRig> camera_rigs;  ///< 相机配置库（rig_id -> CameraRig）
    
    // ── 初始位姿（可选）
    std::optional<InputPose> initial_pose;     ///< 初始位姿估计（可选，用于初始化）
    
    // ── 空三任务（通常一个项目只有一个主任务）
    std::vector<ATTask> at_tasks;              ///< 空三处理任务列表
    
    // ── ID 计数器（用于生成唯一 ID，保持持久化）
    uint32_t next_image_id = 1;
    uint32_t next_image_group_id = 1;
    uint32_t next_rig_id = 1;
    uint32_t next_gcp_id = 1;
    uint32_t next_at_task_id = 0;  ///< 用于生成 AT_0, AT_1 这样的名称

    // ── 项目统计信息
    /**
     * 获取项目中所有图像的总数
     * 
     * @return 所有分组中的总图像数
     */
    size_t GetTotalImageCount() const;
    
    /**
     * 获取项目中的总测量数据条数
     * 
     * @return 测量数据的总条数
     */
    size_t GetTotalMeasurementCount() const {
        return measurements.size();
    }
    
    /**
     * 获取特定类型的测量数据个数
     * 
     * @param[in] type 测量类型
     * @return 该类型测量数据的个数
     */
    size_t GetMeasurementCountByType(Measurement::Type type) const;
    
    /**
     * 验证项目数据的一致性
     * 
     * @return 有效返回true
     * 
     * 检查项目中的：
     * - 项目元数据完整性
     * - 输入坐标系有效性
     * - 所有测量数据有效性
     * - 所有图像分组有效性
     * - 初始位姿有效性（如果存在）
     */
    bool IsValid() const;
    
    /**
     * 获取人类可读的项目描述
     * 
     * @return 格式化的项目信息字符串
     */
    std::string ToString() const;
    
    /**
     * 获取项目摘要统计信息
     * 
     * @return 包含项目关键统计的字符串
     */
    std::string GetSummary() const;
    
    /**
     * 根据图像 ID 查找所属的分组
     * 
     * @param[in] image_id 图像ID
     * @return 分组指针，未找到返回nullptr
     */
    const ImageGroup* FindGroupByImageId(uint32_t image_id) const;
    
    /**
     * 获取指定图像观测到的所有GCP IDs
     * 
     * @param[in] image_id 图像ID
     * @return GCP IDs列表，未找到返回空向量
     * 
     * 使用缓存机制提高性能：
     * - 首次调用时构建索引
     * - 后续调用直接返回缓存结果
     * - 修改gcp_database后需调用InvalidateGCPCache()
     */
    std::vector<uint32_t> GetGCPsForImage(uint32_t image_id) const;
    
    /**
     * 获取指定GCP的完整信息
     * 
     * @param[in] gcp_id GCP ID
     * @return GCPMeasurement指针，未找到返回nullptr
     */
    const GCPMeasurement* GetGCP(uint32_t gcp_id) const;
    
    /**
     * 在修改gcp_database后调用此方法使缓存失效
     * 
     * 设计说明：
     * - 缓存索引在GetGCPsForImage()调用时懒加载
     * - 每次修改GCP数据库后，需调用此方法
     * - 这样确保数据一致性，避免两边计算不一致
     */
    void InvalidateGCPCache() const;
    
    /**
     * 重建GCP缓存索引
     * 
     * @return 成功返回true
     * 
     * 遍历所有GCP及其observations，构建image_id -> gcp_ids映射
     */
    bool RebuildGCPCache() const;
    
    /**
     * 根据图像 ID 查找对应的相机参数
     * 
     * @param[in] image_id 图像ID
     * @return 相机模型指针，未找到返回nullptr
     * 
     * 自动搜索所有分组中的图像
     */
    const CameraModel* GetCameraForImageId(uint32_t image_id) const;
    
    // ── CameraRig 相关方法
    
    /**
     * 获取指定Rig的配置信息
     * 
     * @param[in] rig_id Rig ID
     * @return Rig指针，未找到返回nullptr
     */
    const CameraRig* GetCameraRig(uint32_t rig_id) const;
    
    /**
     * 为某个Rig中的特定相机获取相机参数
     * 
     * @param[in] rig_id Rig ID
     * @param[in] camera_id Rig内的相机ID
     * @return 相机参数指针，未找到返回nullptr
     * 
     * 使用方法：
     * 1. 从ImageGroup中获取 rig_mount_info
     * 2. 调用此方法获取对应的相机参数
     */
    const CameraModel* GetCameraForRigMount(uint32_t rig_id, uint32_t camera_id) const;
    
    /**
     * 验证Rig的一致性
     * 
     * @param[in] rig_id Rig ID
     * @return 有效返回true
     * 
     * 检查：
     * - Rig本身有效
     * - 所有挂载的相机都有对应的相机参数
     */
    bool ValidateRig(uint32_t rig_id) const;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(name), CEREAL_NVP(uuid), CEREAL_NVP(creation_time));
        ar(CEREAL_NVP(description), CEREAL_NVP(author));
        ar(CEREAL_NVP(project_version), CEREAL_NVP(last_modified_time));
        
        if (version > 0) {
            ar(CEREAL_NVP(tags));
        }
        
        ar(CEREAL_NVP(input_coordinate_system), CEREAL_NVP(measurements));
        ar(CEREAL_NVP(image_groups));
        
        // 处理可选的 initial_pose
        bool has_initial_pose = initial_pose.has_value();
        ar(CEREAL_NVP(has_initial_pose));
        if (has_initial_pose) {
            ar(CEREAL_NVP(initial_pose));
        }
        
        if (version > 1) {
            ar(CEREAL_NVP(gcp_database), CEREAL_NVP(camera_rigs));
        }
        
        ar(CEREAL_NVP(at_tasks));

        if (version > 3) {
            ar(CEREAL_NVP(next_image_id), CEREAL_NVP(next_image_group_id));
            ar(CEREAL_NVP(next_rig_id), CEREAL_NVP(next_gcp_id));
        }

        if (version > 4) {
            ar(CEREAL_NVP(next_at_task_id));
        }
    }
};

}  // namespace database
}  // namespace insight

// ─────────────────────────────────────────────────────────────
// Cereal版本控制
// ─────────────────────────────────────────────────────────────
CEREAL_CLASS_VERSION(insight::database::CoordinateSystem, 1);
CEREAL_CLASS_VERSION(insight::database::InputPose, 1);
CEREAL_CLASS_VERSION(insight::database::GCPMeasurement, 1);  ///< v1: 完整GCP结构
CEREAL_CLASS_VERSION(insight::database::GCPMeasurement::Observation, 1);
CEREAL_CLASS_VERSION(insight::database::Measurement, 1);  ///< v1: 移除GCP字段
CEREAL_CLASS_VERSION(insight::database::Measurement::GNSSMeasurement, 1);
CEREAL_CLASS_VERSION(insight::database::Measurement::IMUMeasurement, 1);
CEREAL_CLASS_VERSION(insight::database::Measurement::SLAMMeasurement, 1);
CEREAL_CLASS_VERSION(insight::database::OptimizedPose, 1);
CEREAL_CLASS_VERSION(insight::database::OptimizationFlags, 1);
CEREAL_CLASS_VERSION(insight::database::OptimizationConfig, 1);  ///< v1: 新增优化参数配置
CEREAL_CLASS_VERSION(insight::database::CameraModel, 2);  ///< v2: 增加OptimizationFlags字段
CEREAL_CLASS_VERSION(insight::database::CameraRig, 1);
CEREAL_CLASS_VERSION(insight::database::CameraRig::CameraMount, 1);
CEREAL_CLASS_VERSION(insight::database::Image, 2);  ///< v2: 增加gnss_data字段
CEREAL_CLASS_VERSION(insight::database::ImageGroup, 2);  ///< v2: 增加rig_mount_info字段
CEREAL_CLASS_VERSION(insight::database::ImageGroup::RigMountInfo, 1);
CEREAL_CLASS_VERSION(insight::database::ATTask, 4);  ///< v4: 添加 task_id
CEREAL_CLASS_VERSION(insight::database::ATTask::InputSnapshot, 2);
CEREAL_CLASS_VERSION(insight::database::ATTask::Initialization, 1);
CEREAL_CLASS_VERSION(insight::database::Project, 5);  ///< v5: 添加 next_at_task_index

#endif  // INSIGHT_DATABASE_TYPES_H
