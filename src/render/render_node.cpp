#include "render_node.h"
#include <list>
#include <algorithm>
#include <cassert>
#include "opengl_mat.h"
#include "render_object.h"
#include "render_context.h"

namespace insight{

namespace render
{
	RenderNode::RenderNode(RenderNode *parent /*= NULL*/)
	{
		m_parentNode = parent;
		m_modelMatrix = Mat4::Identity();
		if (parent != NULL)
		{
			parent->addChildNode(this);
		}
	}

	void RenderNode::rotate(double angle, double x, double y, double z)
	{
		m_modelMatrix *= toMat4(vmath::rotation_matrix<double>(angle, x, y, z));
	}

	void RenderNode::rotateX(double angle)
	{
		m_modelMatrix *= toMat4(vmath::rotation_matrix<double>(angle, 1, 0, 0));
	}

	void RenderNode::rotateY(double angle)
	{
		m_modelMatrix *= toMat4(vmath::rotation_matrix<double>(angle, 0, 1, 0));
	}

	void RenderNode::rotateZ(double angle)
	{
		m_modelMatrix *= toMat4(vmath::rotation_matrix<double>(angle, 0, 0, 1));
	}

	void RenderNode::rotateXByPos(double angle, double y, double z)
	{
		Vec3 pos1 = localToWorld(0.0, y, z);
		m_modelMatrix *= toMat4(vmath::rotation_matrix<double>(angle, 1, 0, 0));
		Vec3 pos2 = localToWorld(0.0, y, z);
		translate(pos1 - pos2);
	}

	void RenderNode::rotateYByPos(double angle, double x, double z)
	{
		Vec3 pos1 = localToWorld(x, 0.0, z);
		m_modelMatrix *= toMat4(vmath::rotation_matrix<double>(angle, 0, 1, 0));
		Vec3 pos2 = localToWorld(x, 0.0, z);
		translate(pos1 - pos2);
	}

	void RenderNode::rotateZByPos(double angle, double x, double y)
	{
		Vec3 pos1 = localToWorld(x, y, 0.0);
		m_modelMatrix *= toMat4(vmath::rotation_matrix<double>(angle, 0, 0, 1));
		Vec3 pos2 = localToWorld(x, y, 0.0);
		translate(pos1 - pos2);
	}

	void RenderNode::scale(double ratio)
	{
		Vec3 pos = position();
		translate(-pos);
		m_modelMatrix *= toMat4(vmath::scaling_matrix<double>(ratio, ratio, ratio));
		translate(pos);
		//m_scale *= ratio;
	}

	void RenderNode::scale(double sx, double sy, double sz)
	{
		Vec3 pos = position();
		translate(-pos);
		m_modelMatrix *= toMat4(vmath::scaling_matrix<double>(sx, sy, sz));
		translate(pos);
	}

	Vec3 RenderNode::getScale() const
	{
		double sx = x().norm();
		double sy = y().norm();
		double sz = z().norm();
		return Vec3(sx, sy, sz);
	}

	void RenderNode::setScale(double sx, double sy, double sz)
	{
		Vec3 s = getScale();
		sx = sx / s.x();
		sy = sy / s.y();
		sz = sz / s.z();
		scale(sx, sy, sz);
	}


	void RenderNode::scaleByPos(double ratio, double x, double y, double z)
	{
		Vec3 pos = localToWorld(x, y, z);
		m_modelMatrix *= toMat4(vmath::scaling_pos_matrix(pos.x(), pos.y(), pos.z(), ratio, ratio, ratio));
	}

	Mat4 RenderNode::localToWorldMat() const
	{
		std::vector<RenderNode*> parents = parentNodes();
		Mat4 mat = Mat4::Identity();
		for (int i = 0; i < parents.size(); ++i)
		{
			mat *= parents[i]->refModelMatrix();
		}
		mat *= m_modelMatrix;
		return mat;
	}


	Vec3 RenderNode::localToWorld(double x, double y, double z)
	{
		std::vector<RenderNode*> parents = parentNodes();
		Mat4 mat = Mat4::Identity();
		for (int i = 0; i < parents.size(); ++i)
		{
			mat *= parents[i]->refModelMatrix();
		}
		mat *= m_modelMatrix;

		return (mat * Vec4(x, y, z, 1.0)).head<3>();
	}

	Vec3 RenderNode::worldToLocal(double x, double y, double z)
	{
		std::vector<RenderNode*> parents = parentNodes();
		Mat4 mat = Mat4::Identity();
		for (int i = 0; i < parents.size(); ++i)
		{
			mat *= parents[i]->refModelMatrix();
		}
		mat *= m_modelMatrix;
		return (mat.inverse() * Vec4(x, y, z, 1.0)).head<3>();
	}

	Vec3 RenderNode::fastWorldToLocal(double x, double y, double z)
	{
		std::vector<RenderNode*> parents = parentNodes();
		Mat4 mat = Mat4::Identity();
		for (int i = 0; i < parents.size(); ++i)
		{
			mat *= parents[i]->refModelMatrix();
		}
		mat *= m_modelMatrix;
		return 	(toMat4(vmath::fast_inverse(toVMat4(mat))) *  Vec4(x, y, z, 1.0)).head<3>();
	}

	Vec3 RenderNode::localToParent(double x, double y, double z)
	{
		return (m_modelMatrix * Vec4(x, y, z, 1.0)).head<3>();
	}

	Vec3 RenderNode::parentToLocal(double x, double y, double z)
	{
		return (m_modelMatrix.inverse() * Vec4(x, y, z, 1.0)).head<3>();
	}

	Vec3 RenderNode::fastParentToLocal(double x, double y, double z)
	{
		return (toMat4(vmath::fast_inverse(toVMat4(m_modelMatrix))) *  Vec4(x, y, z, 1.0)).head<3>();
	}

	void RenderNode::render(RenderContext *rc)
	{
		if (!isVisible()) {
			return;
		}
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		rc->push();
		rc->modelview *= m_modelMatrix;
		glLoadMatrixd(rc->modelview.data());
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		draw(rc);
		for (int i = 0; i < (int)m_childNodes.size(); ++i)
		{
			m_childNodes[i]->render(rc);
		}
		glPopAttrib();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		rc->pop();
	}

	void RenderNode::addChildNode(RenderNode *childNode)
	{
		m_childNodes.push_back(childNode);
	}

	RenderNode::~RenderNode()
	{
		destroyAllChildNodes();
		destroyAllObjects();
	}

	void RenderNode::destroyAllChildNodes()
	{
		for (int i = 0; i < (int)m_childNodes.size(); ++i)
		{
			delete m_childNodes[i];
		}
		m_childNodes.clear();
	}

	std::vector<RenderNode *> RenderNode::parentNodes() const
	{
		std::list<RenderNode *> parents;
		RenderNode *parent = m_parentNode;
		while (parent != NULL)
		{
			parents.push_front(parent);
			parent = parent->m_parentNode;
		}

		std::vector<RenderNode *> parentObjs;
		parentObjs.resize(parents.size());
		std::copy(parents.begin(), parents.end(), parentObjs.begin());
		return parentObjs;
	}

	void RenderNode::destroyAllObjects()
	{
		for (int i = 0; i < (int)m_renderObjects.size(); ++i)
		{
			delete m_renderObjects[i];
		}
		m_renderObjects.clear();
	}

	void RenderNode::setParentNode(RenderNode *parent)
	{
		if (m_parentNode != NULL)
		{
			m_parentNode->removeChildNode(this);
		}
		if (parent != NULL)
		{
			parent->addChildNode(this);
		}
		m_parentNode = parent;
	}

	void RenderNode::removeChildNode(RenderNode *childObj)
	{
		std::vector<RenderNode*>::iterator itr = std::find(m_childNodes.begin(), m_childNodes.end(), childObj);
		if (itr != m_childNodes.end())
		{
			m_childNodes.erase(itr);
		}
	}

	void RenderNode::clearAllChildNodes()
	{
		m_childNodes.clear();
	}

	void RenderNode::identityAll()
	{
		m_modelMatrix = Mat4::Identity();
		for (int i = 0; i < m_childNodes.size(); ++i)
		{
			m_childNodes[i]->identityAll();
		}
	}

	void RenderNode::draw(RenderContext *rc)
	{
		for (size_t i = 0; i < m_renderObjects.size(); ++i) {
			m_renderObjects[i]->draw(rc);
		}
	}

}

}//name space insight


