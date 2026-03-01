#ifndef INSIGHT_COMMON_DB_TYPES_H
#define INSIGHT_COMMON_DB_TYPES_H

#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "stlplus3/filesystemSimplified/file_system.hpp"

#include "cereal/cereal.hpp"
#include <glog/logging.h>

#include "common_global.h"
#include "exif_IO.hpp"
#include "numeric.h"
#include "string_utils.h"

namespace insight {

typedef uint32_t KeyType;


typedef uint64_t image_pair_t; // match pair id
// Each image pair gets a unique ID

struct PriKey {
    KeyType seed = 0;
    KeyType generate() { return seed++; }
    void reset() { seed = 0; }
};
// ============================================================================
// COORDINATE SYSTEM DESCRIPTORS (坐标系描述)
// ============================================================================

/**
 * Coordinate system descriptor with support for EPSG, WKT, ENU, and Local.
 *
 * DESIGN: 用户通过UI选择坐标系，自动包含必要的参数
 *
 * - EPSG: "EPSG:4326" (椭球和投影参数自动包含)
 *   包括投影坐标系 (e.g., EPSG:3857)、地理坐标系 (e.g., EPSG:4978 for ECEF)
 *
 * - WKT: "PROJCS[...]" 或 "GEOGCS[...]" OGC WKT定义 (椭球参数自动包含)
 *
 * - ENU: "ENU:lat,lon,alt" 格式，自动提取参考点
 *   例如: "ENU:39.9045,116.4074,50.0" (北京坐标)
 *
 * - LOCAL: 本地/未知坐标系，用户指定参数，无自动转换
 *
 * 参考: ContextCapture metadata.xml 结构、OSGB瓦片格式
 */
struct CoordinateSystemDescriptor {
    enum Type {
        kEPSG = 0, ///< EPSG code (e.g., "EPSG:4326", "EPSG:4978")
        kWKT = 1, ///< OGC WKT string (e.g., "PROJCS[...]", "GEOGCS[...]")
        kENU = 2, ///< ENU with reference point (e.g., "ENU:39.9045,116.4074,50.0")
        kLocal = 3 ///< Local/unknown coordinate system
    };

    /**
     * 旋转约定 - 定义rotation的表达方式
     * 用于解释InputPose中的(omega, phi, kappa)或(yaw, pitch, roll)
     * 在整个Project中应该一致
     */
    enum RotationConvention {
        kNone = 0, ///< 无旋转信息
        kOmegaPhiKappa = 1, ///< 摄影测量 (Z-Y-X extrinsic)
        kYawPitchRoll = 2 ///< 航空学 (Z-Y-X intrinsic)
    };

    Type type = kEPSG;
    std::string definition; ///< EPSG code, WKT string, ENU string, or local name
    RotationConvention rotation_convention = kNone; ///< P1 NEW: 旋转约定

    /**
     * Coordinate origin for projected coordinate systems.
     *
     * 当使用投影坐标系时，坐标值可能很大（如UTM: 500000m以上）。
     * 设置坐标原点可以显著改善浮点精度，类似于ContextCapture的tile中心概念。
     *
     * 参考OSGB metadata.xml的tile中心定义。
     * 示例: 对于UTM投影，可以设置为tile中心的投影坐标。
     */
    struct Origin {
        double x = 0.0; ///< X坐标偏移
        double y = 0.0; ///< Y坐标偏移
        double z = 0.0; ///< Z坐标偏移 (可选)

        bool IsZero() const
        {
            return x == 0.0 && y == 0.0 && z == 0.0;
        }

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version)
        {
            ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
        }
    };

    /**
     * Reference point for ENU coordinate system.
     * 从 "ENU:lat,lon,alt" 字符串自动解析，或手动指定。
     */
    struct ReferencePoint {
        double lat = 0.0; ///< WGS84 latitude (degrees)
        double lon = 0.0; ///< WGS84 longitude (degrees)
        double alt = 0.0; ///< WGS84 ellipsoidal height (meters)

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version)
        {
            ar(CEREAL_NVP(lat), CEREAL_NVP(lon), CEREAL_NVP(alt));
        }
    };

    std::optional<Origin> origin; ///< 坐标原点，可选
    std::optional<ReferencePoint> reference; ///< ENU参考点或其他用途

    /**
     * 从ENU字符串解析参考点。
     * 输入: "ENU:39.9045,116.4074,50.0"
     * 输出: ReferencePoint{lat=39.9045, lon=116.4074, alt=50.0}
     *
     * @return 成功返回true，失败返回false
     */
    bool ParseENUReference()
    {
        if (type != kENU || definition.empty()) {
            return false;
        }

        size_t pos = definition.find(':');
        if (pos == std::string::npos) {
            return false;
        }

        std::string coords_str = definition.substr(pos + 1);
        std::vector<std::string> parts;
        if (!insight::split(coords_str, ',', parts) || parts.size() != 3) {
            return false;
        }

        try {
            ReferencePoint ref;
            ref.lat = std::stod(parts[0]);
            ref.lon = std::stod(parts[1]);
            ref.alt = std::stod(parts[2]);

            // Validate ranges
            if (ref.lat < -90.0 || ref.lat > 90.0 || ref.lon < -180.0 || ref.lon > 180.0) {
                return false;
            }

            reference = ref;
            return true;
        } catch (...) {
            return false;
        }
    }

    /**
     * 获取可读的坐标系描述。
     */
    std::string ToString() const
    {
        std::ostringstream oss;

        if (type == kEPSG) {
            oss << definition << " (EPSG)";
        } else if (type == kWKT) {
            oss << "[WKT] " << definition.substr(0, 50);
            if (definition.length() > 50)
                oss << "...";
        } else if (type == kENU) {
            oss << definition << " (ENU)";
            if (reference) {
                oss << " ref[" << reference->lat << ","
                    << reference->lon << "," << reference->alt << "]";
            }
        } else if (type == kLocal) {
            oss << definition << " (Local)";
        }

        if (origin && !origin->IsZero()) {
            oss << " origin[" << origin->x << ","
                << origin->y << "," << origin->z << "]";
        }

        return oss.str();
    }

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        ar(CEREAL_NVP(type), CEREAL_NVP(definition));
        if (version > 0) {
            // Version 1+: includes rotation_convention
            ar(CEREAL_NVP(rotation_convention));
            ar(CEREAL_NVP(origin), CEREAL_NVP(reference));
        }
    }
};

// Note: CEREAL_CLASS_VERSION declarations moved outside namespace
// See end of file after closing namespace bracket

struct VersionHead {
    int version = 1;
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const ver)
    {
        ar(CEREAL_NVP(version));
    }
};

static const KeyType UndefinedKey = std::numeric_limits<KeyType>::max();

struct DBCamera;

/**
 * Camera pose in photogrammetric convention (OmegaPhiKappa).
 *
 * STANDARD: ISPRS Photogrammetry + doc/rotation/ROTATION_STANDARDS.md
 *
 * COORDINATE SYSTEMS:
 *   - (x, y, z): Camera position in World coordinate system
 *     Current assumption: Projected coordinate system (UTM) or ECEF
 *     TODO: Make coordinate system type explicit (see doc/rotation/CODE_ANALYSIS_ROTATION.md)
 *   - (enuX, enuY, enuZ): Derived from (x,y,z), redundant storage
 *     TODO: Remove redundancy, compute on-demand
 *
 * ROTATION DEFINITION (Critical - World → Camera, Passive Rotation):
 *   R = R_z(κ) · R_y(φ) · R_x(ω)  [Z-Y-X extrinsic order, fixed axes]
 *
 *   p_camera = R · p_world
 *
 *   Where:
 *   - R is a passive rotation (coordinate transform)
 *   - p_world: point in world coordinate system
 *   - p_camera: same point in camera coordinate system
 *
 * EULER ANGLE ORDER (Extrinsic Z-Y-X):
 *   1. ω (omega):  Rotation around fixed World X-axis  [range: -π to π]
 *   2. φ (phi):    Rotation around fixed World Y-axis  [range: -π/2 to π/2]
 *   3. κ (kappa):  Rotation around fixed World Z-axis  [range: -π to π]
 *
 * GIMBAL LOCK WARNING:
 *   Gimbal lock occurs at φ = ±π/2
 *   When φ approaches these values:
 *   - Numerical instability increases
 *   - Multiple Euler angle combinations give same rotation
 *   - SOLUTION: Use quaternion representation instead
 *   - USE: rotation_utils.h IsGimbalLockRisk() function for detection
 *
 * RECOMMENDED USAGE:
 *   For OmegaPhiKappa conversions and validation, use src/Common/rotation_utils.h:
 *
 *   Example:
 *   ```cpp
 *   #include "rotation_utils.h"
 *
 *   // Check gimbal lock risk
 *   if (IsGimbalLockRisk(pose.phi)) {
 *       // Use quaternion instead
 *       Quaternion q = OPK_to_Quaternion(pose.omega, pose.phi, pose.kappa);
 *   }
 *
 *   // Convert to rotation matrix
 *   Mat3 R = OPK_to_RotationMatrix(pose.omega, pose.phi, pose.kappa);
 *   ```
 *
 * REFERENCES:
 *   - ISPRS Photogrammetry Standards (OmegaPhiKappa definition)
 *   - doc/rotation/ROTATION_STANDARDS.md (comprehensive explanation)
 *   - doc/rotation/CODE_ANALYSIS_ROTATION.md (known issues and improvements)
 *   - doc/rotation/ROTATION_QUICK_REFERENCE.md (quick lookup)
 *   - src/Common/ROTATION_UTILS_IMPLEMENTATION.md (implementation status)
 *   - src/Common/rotation_utils.h (API reference)
 */

/**
 * 输入位姿 - 原始测量数据的轻量化表示
 *
 * 存储在Image中，表示该image的原始GNSS/IMU等测量数据
 *
 * 坐标解释：
 * - (x, y, z) 按 Project.input_coordinate_system 解释
 *   例如：EPSG:4326 → (lon, lat, alt)
 *        UTM → (easting, northing, height)
 *
 * 旋转解释：
 * - (omega, phi, kappa) 或 (yaw, pitch, roll)
 *   按 Project.input_crs.rotation_convention 解释
 */
struct InputPose {
    // ─────────────────────────────────────────
    // 位置信息
    // ─────────────────────────────────────────
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    bool has_position = false;

    // ─────────────────────────────────────────
    // 旋转信息 (按rotation_convention解释)
    // ─────────────────────────────────────────
    double omega = 0.0;
    double phi = 0.0;
    double kappa = 0.0;
    bool has_rotation = false;

    // ─────────────────────────────────────────
    // 角度单位
    // ─────────────────────────────────────────
    int angle_unit = 0; // 0=degrees, 1=radians

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z), CEREAL_NVP(has_position));
        ar(CEREAL_NVP(omega), CEREAL_NVP(phi), CEREAL_NVP(kappa), CEREAL_NVP(has_rotation));
        ar(CEREAL_NVP(angle_unit));
    }
};

// Note: CEREAL_CLASS_VERSION moved to end of file, outside namespace

struct DBPose {
    KeyType image_id = UndefinedKey;

    // ─────────────────────────────────────────────────────────────
    // ENUMERATIONS
    // ─────────────────────────────────────────────────────────────

    /**
     * Euler angle convention for input/output interaction.
     * 内部使用四元数，欧拉角仅用于与用户交互。
     */
    enum class EulerAngleConvention : int {
        kNone = 0, ///< 不使用欧拉角
        kOmegaPhiKappa = 1, ///< ISPRS photogrammetry (Z-Y-X extrinsic)
        kYawPitchRoll = 2 ///< Aviation (Z-Y-X intrinsic)
    };

    /**
     * Rotation matrix transform direction.
     * 说明四元数/欧拉角代表的旋转方向。
     */
    enum class RotationMatrixType : int {
        kWorld_to_Camera = 0, ///< Standard: p_camera = R · p_world
        kCamera_to_World = 1 ///< Inverse: p_world = R · p_camera
    };

    // ─────────────────────────────────────────────────────────────
    // POSITION (坐标)
    // ─────────────────────────────────────────────────────────────

    /// Camera position in the coordinate system specified by input_coordinate_system
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    /**
     * 输入坐标系描述。
     * 指定 (x, y, z) 在哪个坐标系中。
     * P1 新增字段：支持用户自定义坐标系。
     */
    CoordinateSystemDescriptor input_coordinate_system;

    // ─────────────────────────────────────────────────────────────
    // ROTATION - QUATERNION (推荐)
    // ─────────────────────────────────────────────────────────────

    /**
     * Unit quaternion representation (推荐).
     * 格式: (x, y, z, w) 其中 w 是标量部分
     * 初始值: (0, 0, 0, 1) = 单位四元数（无旋转）
     *
     * 优点：
     * - 避免gimbal lock
     * - 数值稳定
     * - 适合数值优化
     */
    double quaternion_x = 0.0;
    double quaternion_y = 0.0;
    double quaternion_z = 0.0;
    double quaternion_w = 1.0; ///< Initial = identity quaternion

    // ─────────────────────────────────────────────────────────────
    // ROTATION - EULER ANGLES (用户交互)
    // ─────────────────────────────────────────────────────────────

    /**
     * Euler angles (可选，仅用于用户交互).
     * 当 euler_convention != kNone 时有效。
     *
     * 注意：这些值在序列化时可能被转换为四元数，
     * 不应作为算法的主要输入。
     *
     * ω (omega): X-axis rotation
     * φ (phi): Y-axis rotation
     * κ (kappa): Z-axis rotation
     */
    double omega = 0.0;
    double phi = 0.0;
    double kappa = 0.0;

    /// Euler angle convention type
    EulerAngleConvention euler_convention = EulerAngleConvention::kNone;

    /// angleUnit: 0=degrees, 1=radians (仅适用于欧拉角的I/O)
    int angleUnit = 0;

    // ─────────────────────────────────────────────────────────────
    // ROTATION METADATA
    // ─────────────────────────────────────────────────────────────

    /// 旋转矩阵方向（四元数和欧拉角都遵循此约定）
    RotationMatrixType rotation_type = RotationMatrixType::kWorld_to_Camera;

    // ─────────────────────────────────────────────────────────────
    // MEASUREMENT WEIGHTS
    // ─────────────────────────────────────────────────────────────

    /// GPS measurement weights for bundle adjustment
    float weight_x = 1.0;
    float weight_y = 1.0;
    float weight_z = 1.0;

    // ─────────────────────────────────────────────────────────────
    // DEPRECATED/LEGACY FIELDS
    // ─────────────────────────────────────────────────────────────

    /**
     * Equivalent position in ENU (已弃用，为向后兼容保留).
     * 这些字段是冗余的，应该按需计算而非存储。
     * TODO: 在P2中移除这些字段
     */
    double enuX = 0.0;
    double enuY = 0.0;
    double enuZ = 0.0;

    /**
     * Metadata (已弃用，为向后兼容保留).
     * 这些字段在P1中被更清晰的枚举字段替代。
     */
    int coordinate = 0; // 0=x-right,y-down,z-forward; 1=x-right,y-up,z-backward
    int eulerAngle = 0; // 0=OmegaPhiKappa, 1=PhiOmegaKappa (已被 euler_convention 替代)

    // ─────────────────────────────────────────────────────────────
    // METHODS
    // ─────────────────────────────────────────────────────────────

    void reset()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        omega = 0.0;
        phi = 0.0;
        kappa = 0.0;
        quaternion_x = 0.0;
        quaternion_y = 0.0;
        quaternion_z = 0.0;
        quaternion_w = 1.0;
    }

    bool centerValid() const
    {
        return (x != 0.0 && y != 0.0 && z != 0.0);
    }

    bool rotationValid() const
    {
        // Check quaternion validity (not all zeros, and magnitude ~= 1)
        double quat_magnitude = std::sqrt(quaternion_x * quaternion_x + quaternion_y * quaternion_y + quaternion_z * quaternion_z + quaternion_w * quaternion_w);
        if (std::abs(quat_magnitude - 1.0) < 0.01) {
            return true; // Quaternion is valid
        }

        // Fallback: check Euler angles
        return (omega != 0.0 && phi != 0.0 && kappa != 0.0);
    }

    Vec3 center() const { return Vec3(x, y, z); }

    // angle (x,y,z) degs
    Vec3 rotationDeg() const { return Vec3(omega, phi, kappa); }

    // ─────────────────────────────────────────────────────────────
    // SERIALIZATION
    // ─────────────────────────────────────────────────────────────

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        // Version 3 with quaternion support
        if (version >= 3) {
            ar(CEREAL_NVP(image_id));
            ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
            ar(CEREAL_NVP(input_coordinate_system));
            ar(CEREAL_NVP(quaternion_x), CEREAL_NVP(quaternion_y),
                CEREAL_NVP(quaternion_z), CEREAL_NVP(quaternion_w));
            ar(CEREAL_NVP(omega), CEREAL_NVP(phi), CEREAL_NVP(kappa));
            ar(CEREAL_NVP(euler_convention), CEREAL_NVP(angleUnit));
            ar(CEREAL_NVP(rotation_type));
            ar(CEREAL_NVP(weight_x), CEREAL_NVP(weight_y), CEREAL_NVP(weight_z));
        }
        // Version 2 legacy format (for backward compatibility)
        else {
            ar(CEREAL_NVP(image_id));
            ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
            ar(CEREAL_NVP(omega), CEREAL_NVP(phi), CEREAL_NVP(kappa));
            ar(CEREAL_NVP(weight_x), CEREAL_NVP(weight_y), CEREAL_NVP(weight_z));
            ar(CEREAL_NVP(enuX), CEREAL_NVP(enuY), CEREAL_NVP(enuZ));
            ar(CEREAL_NVP(angleUnit), CEREAL_NVP(coordinate), CEREAL_NVP(eulerAngle));
        }
    }
}; // End of DBPose structure

// simple object for IO
struct DBImage {
    KeyType id = UndefinedKey;
    KeyType camera_id = UndefinedKey;
    std::string image_name; // Ӱ������������չ��
    std::string image_full_path; // ����Ӱ����

    DBPose pose;

    bool exif_valid = false;
    SimpleExifHeader exif_header; // exif ͷ��Ϣ

    void readExif();
    void readExif(const std::vector<unsigned char>& buf);
    size_t cameraHashCode() const;

    bool getCameraByExif(DBCamera& camera) const;
    bool getCameraByWH(DBCamera& camera) const;

    bool pose_valid = false; // pose �Ƿ���Ч
private:
    bool init_pose_center_valid = false; // pose��ʼֵ�Ƿ���Ч
    bool init_pose_rotation_valid = false;
};

class DBImageList {
public:
    const std::map<uint32_t, DBImage>& Image_list() const { return _image_list; }
    std::map<uint32_t, DBImage>& Image_list() { return _image_list; }

    bool saveToASCIIFile(const std::string& file) const;
    bool readFromASCIIFile(const std::string& file);
    void clear() { Image_list().clear(); }
    bool hasImage(uint32_t image_id) const;
    bool hasImage(const std::string& fullPathName) const;
    void addImage(const DBImage& dbimage);
    void resetUnregisted();
    void buildPatchCache();

private:
    std::map<uint32_t, DBImage> _image_list;
    std::set<std::string> _imagePathCacheList;
};

struct DBCamera {
    enum DISTORT_TYPE {
        eAddDistort = 0,
        eRemoveDistort = 1 // not implement yet
    };
    KeyType id = UndefinedKey;
    std::string camera_name;
    std::string Make; // the company
    std::string Model; // the camera model
    int w = 0;
    int h = 0;
    float focalmm = 0.f;
    float focal35mm = 0.f;
    float sensor_size_x = 0.f;
    float sensor_size_y = 0.f;
    int distort_type = eAddDistort;

    float focalpx = 0.f;
    float ppx = 0.f;
    float ppy = 0.f;
    float k1 = 0.f;
    float k2 = 0.f;
    float k3 = 0.f;
    float p1 = 0.f;
    float p2 = 0.f;
    float b1 = 0.f;
    float b2 = 0.f;
    float gps_offset_x = 0.f;
    float gps_offset_y = 0.f;
    float gps_offset_z = 0.f;

    size_t exif_hash_code = 0;
    struct AdjustFlag {
        bool f = false;
        bool ppxy = false;
        bool k1 = false;
        bool k2 = false;
        bool k3 = false;
        bool p1 = false;
        bool p2 = false;
        bool b1 = false;
        bool b2 = false;
        void getParams(std::vector<bool>& params) const
        {
            params.resize(11, false);
            params[0] = f;
            params[1] = false; // focal ratio always constant
            params[2] = ppxy;
            params[3] = ppxy;
            params[4] = k1;
            params[5] = k2;
            params[6] = k3;
            params[7] = p1;
            params[8] = p2;
            params[9] = b1;
            params[10] = b2;
        }
        void getConstantParams(std::vector<int>& constantParams) const
        {
            std::vector<bool> flags;
            getParams(flags);
            constantParams.clear();
            for (int i = 0; i < flags.size(); ++i) {
                if (!flags[i]) {
                    constantParams.push_back(i);
                }
            }
        }
        bool adjustAny() const
        {
            return (f || ppxy || k1 || k2 || k3
                || p1 || p2 || b1 || b2);
        }

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version)
        {
            ar(CEREAL_NVP(f));
            ar(CEREAL_NVP(ppxy));
            ar(CEREAL_NVP(k1));
            ar(CEREAL_NVP(k2));
            ar(CEREAL_NVP(k3));
            ar(CEREAL_NVP(p1));
            ar(CEREAL_NVP(p2));
            ar(CEREAL_NVP(b1));
            ar(CEREAL_NVP(b2));
        }
    };

    struct GPSAdjustFlag {
        bool b_adjust = false;
        float x_weight = 1.f;
        float y_weight = 1.f;
        float z_weight = 1.f;
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version)
        {
            if (version > 0) {
                ar(CEREAL_NVP(b_adjust));
                ar(CEREAL_NVP(x_weight));
                ar(CEREAL_NVP(y_weight));
                ar(CEREAL_NVP(z_weight));
            } else {
                // do something else
                CHECK(false) << "not implement version " << version;
            }
        }
    };
    AdjustFlag adjustFlag;
    GPSAdjustFlag gpsAdjustFlag;

    void generateHashCode();
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        ar(CEREAL_NVP(id));
        ar(CEREAL_NVP(camera_name));
        ar(CEREAL_NVP(w));
        ar(CEREAL_NVP(h));
        ar(CEREAL_NVP(focalmm));
        ar(CEREAL_NVP(focal35mm));
        ar(CEREAL_NVP(sensor_size_x));
        ar(CEREAL_NVP(sensor_size_y));
        ar(CEREAL_NVP(distort_type));
        ar(CEREAL_NVP(focalpx));
        ar(CEREAL_NVP(ppx));
        ar(CEREAL_NVP(ppy));
        ar(CEREAL_NVP(k1));
        ar(CEREAL_NVP(k2));
        ar(CEREAL_NVP(k3));
        ar(CEREAL_NVP(p1));
        ar(CEREAL_NVP(p2));
        ar(CEREAL_NVP(b1));
        ar(CEREAL_NVP(b2));
        ar(CEREAL_NVP(gps_offset_x));
        ar(CEREAL_NVP(gps_offset_y));
        ar(CEREAL_NVP(gps_offset_z));
        ar(CEREAL_NVP(adjustFlag));
        if (version >= 2) {
            ar(CEREAL_NVP(exif_hash_code));
        }
    }
};

class DBCameraList {
public:
    bool saveToJson(const std::string& file) const;
    bool readFromJson(const std::string& file);

    const std::map<uint32_t, DBCamera>& Camera_list() const { return _camera_list; }
    std::map<uint32_t, DBCamera>& Camera_list() { return _camera_list; }

    void clear() { _camera_list.clear(); }

private:
    std::map<uint32_t, DBCamera> _camera_list;
    VersionHead _head;
};

/*
ÿһ��ͼ����һ��pose������pose id����image id
*/
class DBPoseList {
public:
    enum {
        VERSION = 2
    };
    std::map<uint32_t, DBPose>& Pose_list() { return _pose_list; }
    const std::map<uint32_t, DBPose>& Pose_list() const { return _pose_list; }

    bool saveToASCIIFile(const std::string& file) const;
    bool readFromASCIIFile(const std::string& file);

private:
    std::map<uint32_t, DBPose> _pose_list;
};

struct DBTrack {
public:
    struct X {
        double x = 0;
        double y = 0;
        double z = 0;
        uint8_t r = 255;
        uint8_t g = 255;
        uint8_t b = 255;
    };

    struct V {
        KeyType image_id = UndefinedKey;
        // u v coordinate of image
        float u = 0;
        float v = 0;
        float scale = 1; // feature scale;
    };

    Vec3 position() const { return Vec3(landmark.x, landmark.y, landmark.z); }

public:
    KeyType track_id;
    X landmark;
    std::vector<V> views;
};

class DBTrackList {
public:
    enum {
        VERSION = 2,
    };
    const std::map<uint32_t, DBTrack>& TrackList() const { return _track_lsit; }
    std::map<uint32_t, DBTrack>& TrackList() { return _track_lsit; }

    bool saveToAsciiFile(const std::string& file) const;
    bool readFromAsciiFile(const std::string& file);

    bool saveToBinFile(const std::string& file) const;
    bool readFromBinFile(const std::string& file);

private:
    std::map<uint32_t, DBTrack> _track_lsit;
};

struct DBGCP {
    enum TYPE {
        GCP_CHECK = 0, // ����
        GCP_CONTROL = 1, // ���Ƶ�
    };
    struct X {
        double x = 0;
        double y = 0;
        double z = 0;
        Vec3 toEigen() const { return Vec3(x, y, z); }
    };

    struct V {
        KeyType image_id = UndefinedKey;
        // u v coordinate of image
        // the u,v coordinates are  edit by user
        float u = 0;
        float v = 0;
        // reproject coordinate of image
        float ru = 0;
        float rv = 0;
        bool enabled = false;
    };
    KeyType track_id;
    X landmark;
    std::map<KeyType, V> views;
    std::string name; // ���Ƶ�����
    int type; // ���㻹�ǿ��Ƶ�
    int enabled = 1; // �Ƿ����
};

class DBGCPList {
public:
    const std::map<uint32_t, DBGCP>& GCP_List() const { return _gcp_lsit; }
    std::map<uint32_t, DBGCP>& GCP_List() { return _gcp_lsit; }

    bool saveToAsciiFile(const std::string& file) const;
    bool readFromAsciiFile(const std::string& file);

private:
    std::map<uint32_t, DBGCP> _gcp_lsit;
};

struct Resource {
    PriKey imageSeed;
    PriKey cameraSeed;
    PriKey gcpSeed;
    PriKey taskSeed;
    PriKey modelSeed;
    Resource()
    {
        reset();
    }
    void reset()
    {
        cameraSeed.reset();
        imageSeed.reset();
        gcpSeed.reset();
        taskSeed.reset();
        modelSeed.reset();
    }

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        // You can choose different behaviors depending on the version
        // This is useful if you need to support older variants of your codebase
        // interacting with newer ones
        ar(cereal::make_nvp("image_id", imageSeed.seed));
        ar(cereal::make_nvp("camera_id", cameraSeed.seed));
        if (version >= 2) {
            ar(cereal::make_nvp("gcp_id", gcpSeed.seed));
        }
        if (version >= 3) {
            ar(cereal::make_nvp("task_id", taskSeed.seed));
        }
        if (version >= 4) {
            ar(cereal::make_nvp("model_id", modelSeed.seed));
        }
    }
};

// ============================================================================
// MEASUREMENT DATA STRUCTURES (P1 NEW - 测量数据)
// ============================================================================

/**
 * 通用测量数据结构 - 支持GNSS、IMU、GCP、SLAM等多种传感器
 *
 * 设计原则：
 * - 所有测量都有时间标记或关联
 * - 所有测量都有不确定度（协方差）
 * - 在Bundle Adjustment中作为先验约束
 * - 易于扩展新的测量类型
 */
struct Measurement {
    enum Type {
        kGNSS = 0, // GPS/RTK 位置测量
        kIMU = 1, // IMU 姿态（加速度、角速度）
        kGCP = 2, // 地面控制点
        kSLAM = 3, // Visual SLAM 相对位姿
        kOther = 4
    };

    Type type;
    KeyType image_id; // 关联到哪个图像
    double timestamp = -1.0; // 可选：测量时间戳

    // ─────────────────────────────────────────
    // GNSS 位置测量
    // ─────────────────────────────────────────
    struct GNSSMeasurement {
        double x, y, z; // 位置（坐标系由Project.input_crs定义）
        double cov_xx, cov_yy, cov_zz; // 对角线方差
        double cov_xy = 0.0, cov_xz = 0.0, cov_yz = 0.0; // 非对角线协方差

        int num_satellites = -1; // GPS卫星数量
        double hdop = -1.0, vdop = -1.0; // 几何精度因子

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version)
        {
            ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
            ar(CEREAL_NVP(cov_xx), CEREAL_NVP(cov_yy), CEREAL_NVP(cov_zz));
            ar(CEREAL_NVP(cov_xy), CEREAL_NVP(cov_xz), CEREAL_NVP(cov_yz));
            if (version > 0) {
                ar(CEREAL_NVP(num_satellites), CEREAL_NVP(hdop), CEREAL_NVP(vdop));
            }
        }
    };

    // ─────────────────────────────────────────
    // IMU 测量 (惯导)
    // ─────────────────────────────────────────
    struct IMUMeasurement {
        // 姿态（按Project.input_crs.rotation_convention）
        bool has_attitude = false;
        double omega = 0.0, phi = 0.0, kappa = 0.0; // 或 yaw, pitch, roll
        double cov_omega = 0.0, cov_phi = 0.0, cov_kappa = 0.0;

        // 加速度
        bool has_acceleration = false;
        double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
        double cov_accel_x = 0.0, cov_accel_y = 0.0, cov_accel_z = 0.0;

        // 角速度
        bool has_angular_velocity = false;
        double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
        double cov_gyro_x = 0.0, cov_gyro_y = 0.0, cov_gyro_z = 0.0;

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version)
        {
            ar(CEREAL_NVP(has_attitude), CEREAL_NVP(omega), CEREAL_NVP(phi),
                CEREAL_NVP(kappa), CEREAL_NVP(cov_omega), CEREAL_NVP(cov_phi),
                CEREAL_NVP(cov_kappa));
            ar(CEREAL_NVP(has_acceleration), CEREAL_NVP(accel_x),
                CEREAL_NVP(accel_y), CEREAL_NVP(accel_z));
            ar(CEREAL_NVP(cov_accel_x), CEREAL_NVP(cov_accel_y), CEREAL_NVP(cov_accel_z));
            ar(CEREAL_NVP(has_angular_velocity), CEREAL_NVP(gyro_x),
                CEREAL_NVP(gyro_y), CEREAL_NVP(gyro_z));
            ar(CEREAL_NVP(cov_gyro_x), CEREAL_NVP(cov_gyro_y), CEREAL_NVP(cov_gyro_z));
        }
    };

    // ─────────────────────────────────────────
    // GCP 地面控制点
    // ─────────────────────────────────────────
    struct GCPMeasurement {
        std::string gcp_id; // 控制点标识符
        double x, y, z; // 三维坐标
        double cov_xx, cov_yy, cov_zz; // 坐标方差
        double cov_xy = 0.0, cov_xz = 0.0, cov_yz = 0.0;

        // 可选：图像上的观测
        double pixel_x = -1.0, pixel_y = -1.0;
        double pixel_cov_x = -1.0, pixel_cov_y = -1.0;

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version)
        {
            ar(CEREAL_NVP(gcp_id), CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
            ar(CEREAL_NVP(cov_xx), CEREAL_NVP(cov_yy), CEREAL_NVP(cov_zz));
            ar(CEREAL_NVP(cov_xy), CEREAL_NVP(cov_xz), CEREAL_NVP(cov_yz));
            if (version > 0) {
                ar(CEREAL_NVP(pixel_x), CEREAL_NVP(pixel_y),
                    CEREAL_NVP(pixel_cov_x), CEREAL_NVP(pixel_cov_y));
            }
        }
    };

    // ─────────────────────────────────────────
    // SLAM 相对位姿
    // ─────────────────────────────────────────
    struct SLAMMeasurement {
        KeyType reference_image_id; // 参考图像ID

        // 相对位置
        double dx, dy, dz;
        double cov_dx, cov_dy, cov_dz;

        // 相对旋转（四元数）
        double qx, qy, qz, qw;
        double cov_qx, cov_qy, cov_qz; // 旋转不确定度

        double confidence = 1.0; // 匹配置信度 [0, 1]

        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version)
        {
            ar(CEREAL_NVP(reference_image_id));
            ar(CEREAL_NVP(dx), CEREAL_NVP(dy), CEREAL_NVP(dz));
            ar(CEREAL_NVP(cov_dx), CEREAL_NVP(cov_dy), CEREAL_NVP(cov_dz));
            ar(CEREAL_NVP(qx), CEREAL_NVP(qy), CEREAL_NVP(qz), CEREAL_NVP(qw));
            ar(CEREAL_NVP(cov_qx), CEREAL_NVP(cov_qy), CEREAL_NVP(cov_qz));
            ar(CEREAL_NVP(confidence));
        }
    };

    // ─────────────────────────────────────────
    // 测量数据存储 (Union-like)
    // ─────────────────────────────────────────
    std::optional<GNSSMeasurement> gnss;
    std::optional<IMUMeasurement> imu;
    std::optional<GCPMeasurement> gcp;
    std::optional<SLAMMeasurement> slam;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        ar(CEREAL_NVP(type), CEREAL_NVP(image_id), CEREAL_NVP(timestamp));

        // 根据type序列化相应的数据
        if (type == kGNSS)
            ar(CEREAL_NVP(gnss));
        if (type == kIMU)
            ar(CEREAL_NVP(imu));
        if (type == kGCP)
            ar(CEREAL_NVP(gcp));
        if (type == kSLAM)
            ar(CEREAL_NVP(slam));
    }
};

// Note: CEREAL_CLASS_VERSION moved to end of file, outside namespace

class ImageListGenerator {
public:
    std::vector<int> importImages(const std::set<std::string>& imageFiles,
        const KeyType& cameraKey,
        Resource* rc, progress_fun progress = nullptr);

    void removeImages(const std::set<KeyType>& imageFiles)
    {
        for (auto itr = imageFiles.begin(); itr != imageFiles.end(); ++itr) {
            imageList.Image_list().erase(*itr);
            // poseList.Pose_list().erase(*itr);
        }
    }
    DBImageList imageList;

    void mergePoseList(const DBPoseList& poseList);
    // DBPoseList poseList;
    void clear()
    {
        imageList.clear();
        // poseList.Pose_list().clear();
    }
};

bool saveImageNeighbors(
    const std::map<int, std::vector<int>>& neighbors,
    const std::string& file);

bool readImageNeighbors(
    std::map<int, std::vector<int>>& neighbors,
    const std::string& file);

} // name space insight

// ============================================================================
// CEREAL CLASS VERSION DECLARATIONS (Outside namespace)
// ============================================================================

CEREAL_CLASS_VERSION(insight::VersionHead, 1);
CEREAL_CLASS_VERSION(insight::Resource, 4);
CEREAL_CLASS_VERSION(insight::DBCamera::AdjustFlag, 1);
CEREAL_CLASS_VERSION(insight::DBCamera::GPSAdjustFlag, 1);
CEREAL_CLASS_VERSION(insight::DBCamera, 2);
CEREAL_CLASS_VERSION(insight::CoordinateSystemDescriptor, 1); // P1 NEW
CEREAL_CLASS_VERSION(insight::InputPose, 1); // P1 NEW
CEREAL_CLASS_VERSION(insight::DBPose, 3); ///< Version 3: Added quaternion and coordinate system
CEREAL_CLASS_VERSION(insight::Measurement::GCPMeasurement, 1); // P1 NEW
CEREAL_CLASS_VERSION(insight::Measurement, 1); // P1 NEW

#define DBGCPListVersion 1

#endif // TASK_LIST_IMAGE_LIST_H
