#include "CameraSensorDatabase.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <glog/logging.h>

namespace insight {
namespace database {

CameraSensorDatabase& CameraSensorDatabase::instance() {
    static CameraSensorDatabase instance;
    return instance;
}

bool CameraSensorDatabase::load(const std::string& dbPath) {
    std::ifstream file(dbPath);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open camera sensor database: " << dbPath;
        return false;
    }

    m_sensors.clear();
    m_lookupTable.clear();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string manufacturer, model, widthStr;

        if (std::getline(ss, manufacturer, ';') &&
            std::getline(ss, model, ';') &&
            std::getline(ss, widthStr, ';')) {
            
            try {
                double width = std::stod(widthStr);
                SensorInfo info{manufacturer, model, width};
                m_sensors.push_back(info);

                std::string key = normalizeString(manufacturer) + "|" + normalizeString(model);
                m_lookupTable[key] = width;
            } catch (...) {
                LOG(WARNING) << "Failed to parse sensor width in line: " << line;
            }
        }
    }

    LOG(INFO) << "Loaded " << m_sensors.size() << " sensors from " << dbPath;
    return !m_sensors.empty();
}

bool CameraSensorDatabase::querySensorWidth(const std::string& manufacturer, const std::string& model, double& widthMm) const {
    std::string key = normalizeString(manufacturer) + "|" + normalizeString(model);
    auto it = m_lookupTable.find(key);
    if (it != m_lookupTable.end()) {
        widthMm = it->second;
        return true;
    }
    return false;
}

std::string CameraSensorDatabase::normalizeString(const std::string& s) const {
    std::string res = s;
    // 转小写并去除两端空格
    std::transform(res.begin(), res.end(), res.begin(), ::tolower);
    res.erase(0, res.find_first_not_of(" \t\r\n"));
    res.erase(res.find_last_not_of(" \t\r\n") + 1);
    return res;
}

} // namespace database
} // namespace insight
