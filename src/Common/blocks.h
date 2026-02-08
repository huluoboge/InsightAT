/************************************************************************/
/* 
@file: blocks.h
@author: Jones
@date: 2017-10-1
@version:1.0
*/
/************************************************************************/
#ifndef INSIGHT_COMMON_BLOCKS_H_
#define INSIGHT_COMMON_BLOCKS_H_

#include "Eigen/Core"
#include <fstream>
#include <glog/logging.h>
#include <string>
#include <vector>

namespace insight {
typedef Eigen::Vector3d Vec3;
struct Box {
    bool valid = false;
    Vec3 min;
    Vec3 max;

    Box(const Vec3& minPoint, const Vec3& maxPoint)
        : min(minPoint)
        , max(maxPoint)
    {
    }

    Box()
        : min(1, 1, 1)
        , max(-1, -1, -1)
    {
    }

    bool isValid() const
    {
        return valid;
    }

    Vec3 center() const
    {
        return (max + min) / 2.0;
    }

    double xSize() const { return max.x() - min.x(); }
    double ySize() const { return max.y() - min.y(); }
    double zSize() const { return max.z() - min.z(); }

    void setMinMax(const Vec3& _min, const Vec3& _max)
    {
        max = _max;
        min = _min;
    }

    //包括边界
    bool isPointIn(float x, float y, float z) const
    {

        if (x <= max.x() && y <= max.y() && z <= max.z() && x >= min.x() && y >= min.y() && z >= min.z()) {
            return true;
        } else
            return false;
    }
    //包括边界
    bool isPointIn(const Vec3& p) const
    {

        if (p.x() <= max.x() && p.y() <= max.y() && p.z() <= max.z() && p.x() >= min.x() && p.y() >= min.y() && p.z() >= min.z()) {
            return true;
        } else
            return false;
    }

    //不包括边界
    bool isAbsolutePointIn(const Vec3& p) const
    {

        if (p.x() < max.x() && p.y() < max.y() && p.z() < max.z() && p.x() > min.x() && p.y() > min.y() && p.z() > min.z()) {
            return true;
        } else
            return false;
    }

    void expand(float dx, float dy, float dz)
    {
        min.x() -= dx;
        max.x() += dx;
        min.y() -= dy;
        max.y() += dy;
        min.z() -= dz;
        max.z() += dz;
    }

    bool isPointIn2(float* p) const
    {
        if (p[0] <= max[0] && p[1] <= max[1] && p[0] >= min[0] && p[1] >= min[1]) {
            return true;
        } else {
            return false;
        }
    }
    void expand(float dx, float dy)
    {
        min[0] -= dx;
        min[1] -= dy;
        max[0] += dx;
        max[1] += dy;
    }

    void print()
    {
        std::cout << "\t" << min[0] << "\t" << min[1] << "\t" << min[2]
                  << "\n\t" << max[0] << "\t" << max[1] << "\t" << max[2] << std::endl;
    }

    template <typename T>
    void update(T begin, T end)
    {
        while (begin != end) {
            update(*begin);
            ++begin;
        }
    }

    template <typename T>
    void update(T& pt)
    {
        if (valid) {
            min.x() = std::min<double>(pt.x, min.x());
            min.y() = std::min<double>(pt.y, min.y());
            min.z() = std::min<double>(pt.z, min.z());
            max.x() = std::max<double>(pt.x, max.x());
            max.y() = std::max<double>(pt.y, max.y());
            max.z() = std::max<double>(pt.z, max.z());
        } else {
            min.x() = pt.x;
            min.y() = pt.y;
            min.z() = pt.z;
            max = min;
            valid = true;
        }
    }

    void update(double x, double y, double z)
    {
        if (valid) {
            min.x() = std::min<double>(x, min.x());
            min.y() = std::min<double>(y, min.y());
            min.z() = std::min<double>(z, min.z());
            max.x() = std::max<double>(x, max.x());
            max.y() = std::max<double>(y, max.y());
            max.z() = std::max<double>(z, max.z());
        } else {
            min.x() = x;
            min.y() = y;
            min.z() = z;
            max = min;
            valid = true;
        }
    }

    template <typename T>
    void update2(T& pt)
    {
        if (valid) {
            min.x() = std::min<double>(pt.x(), min.x());
            min.y() = std::min<double>(pt.y(), min.y());
            min.z() = std::min<double>(pt.z(), min.z());
            max.x() = std::max<double>(pt.x(), max.x());
            max.y() = std::max<double>(pt.y(), max.y());
            max.z() = std::max<double>(pt.z(), max.z());
        } else {
            min.x() = pt.x();
            min.y() = pt.y();
            min.z() = pt.z();
            max = min;
            valid = true;
        }
    }

    bool collide(const Box& b) const
    {
        return b.min.x() < max.x() && b.max.x() > min.x() && b.min.y() < max.y() && b.max.y() > min.y() && b.min.z() < max.z() && b.max.z() > min.z();
    }

    Vec3 P(const int& i) const
    {
        return Vec3(
            min[0] + (i % 2) * xSize(),
            min[1] + ((i / 2) % 2) * ySize(),
            min[2] + (i > 3) * zSize());
    }
};
struct Block {
    size_t id = 0;
    Box box;
    bool operator<(const Block& b) const { return id < b.id; }
};

struct Blocks {
    Blocks()
        : columns(1)
        , rows(1)
        , transeCoord(0, 0, 0)
    {
    }
    Box box;
    Vec3 transeCoord; //block box ԭ��ʵ������ƽ����
    int columns; //�ֿ�������
    int rows; //�ֿ�������

    Vec3 structureToBlocks(Vec3 landmark) { return landmark - transeCoord; }
    Vec3 blocksToStructure(Vec3 blockPt) { return blockPt + transeCoord; }

    std::vector<Block> computeBlockDataToGeoBlock() const
    {
        std::vector<Block> bloks = blockData;
        for (int i = 0; i < bloks.size(); ++i) {
            bloks[i].box.min += transeCoord;
            bloks[i].box.max += transeCoord;
        }
        return bloks;
    }

    void print()
    {
        // 			logger() << std::fixed;
        // 			logger() << "Rows X cols " << rows << " X " << columns << "\n";
        // 			logger() << blockData.size() << std::endl;
        // 			for (int i = 0; i < blockData.size(); ++i)
        // 			{
        // 				logger() << blockData[i].id << std::endl;
        // 				logger() << "\t" << blockData[i].box.min.x() << "\t" << blockData[i].box.min.y() << "\t" << blockData[i].box.min.z()
        // 					<< "\n\t" << blockData[i].box.max.x() << "\t" << blockData[i].box.max.y() << "\t" << blockData[i].box.max.z() << std::endl;
        // 				logger() << "--------------------------------------------\n";
        // 			}
    }
    std::vector<Block> blockData;
};

inline void writeBlocksBoxText(const Blocks& blocks, const std::string& file)
{
    std::ofstream ofs(file);
    ofs << std::fixed;
    ofs << blocks.rows << std::endl
        << blocks.columns << std::endl
        << blocks.transeCoord.x() << " " << blocks.transeCoord.y() << " " << blocks.transeCoord.z() << std::endl;
    ofs
        << blocks.box.min.x() << " " << blocks.box.min.y() << " " << blocks.box.min.z() << std::endl
        << blocks.box.max.x() << " " << blocks.box.max.y() << " " << blocks.box.max.z() << std::endl;

    ofs << blocks.blockData.size() << std::endl;
    for (size_t i = 0; i < blocks.blockData.size(); ++i) {
        const auto& block = blocks.blockData[i];
        ofs << block.id << std::endl;
        ofs << block.box.min.x() << " " << block.box.min.y() << " " << block.box.min.z() << std::endl
            << block.box.max.x() << " " << block.box.max.y() << " " << block.box.max.z() << std::endl;
    }
}

inline void readBlocksBoxText(Blocks& blocks, const std::string& file)
{
    std::ifstream ifs(file);
    ifs >> blocks.rows >> blocks.columns;
    ifs >> blocks.transeCoord.x() >> blocks.transeCoord.y() >> blocks.transeCoord.z();
    ifs
        >> blocks.box.min.x() >> blocks.box.min.y() >> blocks.box.min.z()
        >> blocks.box.max.x() >> blocks.box.max.y() >> blocks.box.max.z();

    int nBlock = 0;
    ifs >> nBlock;
    blocks.blockData.resize(nBlock);

    for (size_t i = 0; i < blocks.blockData.size(); ++i) {
        auto& block = blocks.blockData[i];
        int id;
        ifs >> id
            >> block.box.min.x() >> block.box.min.y() >> block.box.min.z()
            >> block.box.max.x() >> block.box.max.y() >> block.box.max.z();
        block.id = id;
    }
}
}

#endif //
