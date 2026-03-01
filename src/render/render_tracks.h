#pragma once

#include "render_global.h"
#include "render_object.h"
#include "render_types.h"
#include <QString>
#include <array>

namespace insight{

namespace render {

class  RenderTracks : public RenderObject {
public:
    RenderTracks();
    virtual ~RenderTracks();
    struct Photo {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Photo()
        {
            id = 0;
            focal = 0;
            w = 0;
            h = 0;
        }
        int id;
        float focal;
        float w;
        float h;

        struct Pose {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Pose()
            {
                memset(data, 0, sizeof(double) * 6);
                centerValid = false;
                rotationValid = false;
                openglMat.setIdentity();
                color.setOnes();
            }
            double data[6]; //C and OmegaPhiKappa
            bool centerValid;
            bool rotationValid;
            Mat4 openglMat;
            Vec3 color;
        };

        Pose initPose;
        Pose refinedPose;
        QString name;
    };
    /*
		*/
    struct Observe {
        Observe()
        {
            photoId = 0;
            featX = 0;
            featY = 0;
        }
        int photoId;
        float featX, featY;
    };
    /*
		*/
    struct Track {
        int trackId = 0;
        double x = 0;
        double y = 0;
        double z = 0;
        Vec3 color;
        std::vector<Observe> obs;
    };

    struct Grid {
        Grid()
            : minx(0)
            , miny(0)
            , minz(0)
            , maxx(1)
            , maxy(1)
            , maxz(1)
        {
        }
        int xcount = 1;
        int ycount = 1;
        int zcount = 1;
        double minx, miny, minz;
        double maxx, maxy, maxz;
        std::vector<double> xs;
        std::vector<double> ys;
        std::vector<double> zs;
        void generate(double low, double high, int count, std::vector<double>& datas)
        {
            if (count == 1) {
                datas.push_back(low);
                datas.push_back(high);
            } else if (count == 2) {
                double mid = (low + high) * 0.5;
                datas.emplace_back(low);
                datas.emplace_back(mid);
                datas.emplace_back(high);
            } else {
                int n = count - 1;
                double space = (high - low) / n;
                double half = space * 0.5;
                datas.emplace_back(low);
                double start = low + half;
                for (int i = 0; i < n; ++i) {
                    double d = start + i * space;
                    datas.emplace_back(d);
                }
                datas.emplace_back(high);
            }
        }
        void generateDatas()
        {
            //两边都是0.5，中间都是1
            xs.clear();
            ys.clear();
            zs.clear();
            generate(minx, maxx, xcount, xs);
            generate(miny, maxy, ycount, ys);
            generate(minz, maxz, zcount, zs);
        }
    };
    struct RenderOptions {
        RenderOptions()
        {
        }
        float photoScale = 1.f;
        float poseSize = 3;
        float vetexSize = 1;
        uint8_t gridcolor[3] = { 10, 150, 100 };
    };

    virtual void draw(RenderContext* rc);

    typedef std::vector<Track> Tracks;
    typedef std::vector<Photo> Photos;

    void setRenderOptions(const RenderOptions& opt) { m_renderOptions = opt; }

    void setTracks(const Tracks& t) { m_tracks = t; }
    void setPhotos(const Photos& p) { m_photos = p; }
    void setGCPs(const Tracks& gcp) { m_gcps = gcp; }
    void setCenter(double x, double y, double z);

    void clear()
    {
        m_tracks.clear();
        m_photos.clear();
    }
    void photoLarger() { m_renderOptions.photoScale *= 1.1f; }
    void photoSmaller() { m_renderOptions.photoScale *= 0.9f; }

    void vertexLarge() { m_renderOptions.vetexSize *= 1.1f; };
    void vertexSmaller() { m_renderOptions.vetexSize *= 0.9f; };

    void setPhotoVisible(bool vis) { m_showPhoto = vis; }
    void setVertexVisible(bool vis) { m_showVertex = vis; }

    bool isPhotoVisible() const { return m_showPhoto; }
    bool isVertexVisible() const { return m_showVertex; }

    void setGridVisible(bool vis) { m_showGrid = vis; }

    void setGrid(const Grid& grid) { m_grid = grid; }
    Grid& getGrid() { return m_grid; }

protected:
    void renderPhoto(const Photo& p);

protected:
    bool m_showPhoto;
    bool m_showVertex;
    bool m_showGrid = false;
    bool m_showAxis = true;

    Tracks m_tracks;
    Tracks m_gcps;
    Photos m_photos;
    Grid m_grid;
    RenderOptions m_renderOptions;

    std::array<double, 3> m_center;
};
} // namespace render

}//name space insight
