
#include "render_tracks.h"
#include "render_types.h"

#include "render_context.h"
#include <array>

namespace insight{

namespace render {
RenderTracks::RenderTracks()
    : m_showPhoto(true)
    , m_showVertex(true)
{
}

RenderTracks::~RenderTracks()
{
}

void RenderTracks::renderPhoto(const Photo& p)
{
    float photoscale = 6000;
    if (p.initPose.centerValid) {
        photoscale = 50.f;
        glColor3dv(p.initPose.color.data());
        glBegin(GL_POINTS);
        glVertex3d(p.initPose.data[0],
            p.initPose.data[1],
            p.initPose.data[2]);
        glEnd();
#ifndef SHOW_GPS_ROTATION
#define SHOW_GPS_ROTATION
#endif
#ifdef SHOW_GPS_ROTATION
        if (p.initPose.rotationValid) {
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glTranslated(p.initPose.data[0],
                p.initPose.data[1],
                p.initPose.data[2]);
            glMultMatrixd(p.initPose.openglMat.transpose().data());
            {
                float scale = 1.f / photoscale;
                glScalef(scale, scale, scale);
                //draw camera
                glScalef(m_renderOptions.photoScale,
                    m_renderOptions.photoScale,
                    m_renderOptions.photoScale);
                glBegin(GL_LINE_STRIP);
                glVertex3d(0, 0, 0);
                glVertex3d(0, 0, -p.focal);
                glEnd();

                glBegin(GL_LINE_LOOP);
                glVertex3d(-p.w * 0.5, -p.h * 0.5, -p.focal);
                glVertex3d(p.w * 0.5, -p.h * 0.5, -p.focal);
                glVertex3d(p.w * 0.5, p.h * 0.5, -p.focal);
                glVertex3d(-p.w * 0.5, p.h * 0.5, -p.focal);
                glEnd();
            }
            glMatrixMode(GL_MODELVIEW);
            glPopMatrix();
        }
#endif
    }

    if (p.refinedPose.centerValid) {
        glColor3dv(p.refinedPose.color.data());
        glBegin(GL_POINTS);
        glVertex3d(p.refinedPose.data[0],
            p.refinedPose.data[1],
            p.refinedPose.data[2]);
        glEnd();

        if (p.refinedPose.rotationValid) {
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glTranslated(p.refinedPose.data[0],
                p.refinedPose.data[1],
                p.refinedPose.data[2]);
            glMultMatrixd(p.refinedPose.openglMat.transpose().data());
            {
                //draw camera
                float scale = 1.f / photoscale;
                glScalef(scale, scale, scale);
                glScalef(m_renderOptions.photoScale,
                    m_renderOptions.photoScale,
                    m_renderOptions.photoScale);
                glBegin(GL_LINE_STRIP);
                glVertex3d(0, 0, 0);
                glVertex3d(0, 0, -p.focal);
                glEnd();

                glBegin(GL_LINE_LOOP);
                glVertex3d(-p.w * 0.5, -p.h * 0.5, -p.focal);
                glVertex3d(p.w * 0.5, -p.h * 0.5, -p.focal);
                glVertex3d(p.w * 0.5, p.h * 0.5, -p.focal);
                glVertex3d(-p.w * 0.5, p.h * 0.5, -p.focal);
                glEnd();
            }
            glMatrixMode(GL_MODELVIEW);
            glPopMatrix();
        }
    }
}

void RenderTracks::draw(RenderContext* rc)
{
    //draw photos
    glPointSize(m_renderOptions.poseSize);
    //draw imported pose
    if (m_showPhoto) {
        for (size_t i = 0; i < m_photos.size(); ++i) {
            renderPhoto(m_photos[i]);
        }
    }

    if (m_showVertex) {

        //draw 3d points
        glPointSize(m_renderOptions.vetexSize);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < m_tracks.size(); ++i) {
            glColor3dv(m_tracks[i].color.data());
            glVertex3d(m_tracks[i].x, m_tracks[i].y, m_tracks[i].z);
        }
        glEnd();

        glPointSize(m_renderOptions.vetexSize * 5);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < m_gcps.size(); ++i) {
            glColor3dv(m_gcps[i].color.data());
            glVertex3d(m_gcps[i].x, m_gcps[i].y, m_gcps[i].z);
        }
        glEnd();
    }
    double xsize = 1, ysize = 1, zsize = 1;
    if (m_showGrid) {
        glColor3ubv(m_renderOptions.gridcolor);
        for (int j = 0; j < m_grid.xs.size(); ++j) {
            glBegin(GL_LINE_LOOP);
            glVertex3d(m_grid.xs[j], m_grid.miny, m_grid.minz);
            glVertex3d(m_grid.xs[j], m_grid.maxy, m_grid.minz);
            glVertex3d(m_grid.xs[j], m_grid.maxy, m_grid.maxz);
            glVertex3d(m_grid.xs[j], m_grid.miny, m_grid.maxz);
            glEnd();
        }
        for (int j = 0; j < m_grid.ys.size(); ++j) {
            glBegin(GL_LINE_LOOP);
            glVertex3d(m_grid.minx, m_grid.ys[j], m_grid.minz);
            glVertex3d(m_grid.maxx, m_grid.ys[j], m_grid.minz);
            glVertex3d(m_grid.maxx, m_grid.ys[j], m_grid.maxz);
            glVertex3d(m_grid.minx, m_grid.ys[j], m_grid.maxz);
            glEnd();
        }
        for (int j = 0; j < m_grid.zs.size(); ++j) {
            glBegin(GL_LINE_LOOP);
            glVertex3d(m_grid.minx, m_grid.miny, m_grid.zs[j]);
            glVertex3d(m_grid.maxx, m_grid.miny, m_grid.zs[j]);
            glVertex3d(m_grid.maxx, m_grid.maxy, m_grid.zs[j]);
            glVertex3d(m_grid.minx, m_grid.maxy, m_grid.zs[j]);
            glEnd();
        }
        xsize = (m_grid.maxx - m_grid.minx) * 0.5;
        ysize = (m_grid.maxy - m_grid.miny) * 0.5;
        zsize = (m_grid.maxz - m_grid.minz) * 0.5;
    }

    if (m_showAxis) {
        glColor3ub(255, 0, 0);
        glBegin(GL_LINES);
        glVertex3d(0, 0, 0);
        glVertex3d(xsize, 0, 0);
        glEnd();
        glColor3ub(0, 255, 0);
        glBegin(GL_LINES);
        glVertex3d(0, 0, 0);
        glVertex3d(0, ysize, 0);
        glEnd();
        glColor3ub(0, 0, 255);
        glBegin(GL_LINES);
        glVertex3d(0, 0, 0);
        glVertex3d(0, 0, zsize);
        glEnd();
    }
}

void RenderTracks::setCenter(double x, double y, double z)
{
    m_center.at(0) = x;
    m_center.at(1) = y;
    m_center.at(2) = z;
}

} // namespace render

}//name space insight
