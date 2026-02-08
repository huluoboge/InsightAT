#pragma once

#include "render_global.h"

#include <vector>
#include <QString>
#include <QVariant>
#include "render_types.h"
#include "render_object.h"

namespace insight{

namespace render
{
	class RenderContext;
	class RenderObject;
	/**
 * @brief 3D��Ⱦ����Ⱦ�ڵ㣬�ڲ���ģ�;���
 */
	class  RenderNode : public RenderObject
	{
	public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		explicit RenderNode(RenderNode *parent = NULL);
	public:
		virtual ~RenderNode();

	public:

		Mat4 modelMatrix() const { return m_modelMatrix; }
		Mat4 &refModelMatrix() { return m_modelMatrix; }
		const Mat4 &refModelMatrix() const { return m_modelMatrix; }

        void identityAll();
		inline Vec3 x() const { return m_modelMatrix.col(0).head<3>(); }
		inline Vec3 y() const { return m_modelMatrix.col(1).head<3>(); }
		inline Vec3 z() const { return m_modelMatrix.col(2).head<3>(); }


		inline Vec3 position() { return m_modelMatrix.col(3).head<3>(); }
		inline void setPosition(const Vec3 &pos)
		{
			m_modelMatrix(0, 3) = pos.x();
			m_modelMatrix(1, 3) = pos.y();
			m_modelMatrix(2, 3) = pos.z();
		}

		inline void setPosition(double x, double y, double z)
		{
			setPosition(Vec3(x, y, z));
		}

		inline void position(double &x, double &y, double &z)
		{
			Vec3 pos = position();
			x = pos.x();
			y = pos.y();
			z = pos.z();
		}


		void translate(const Vec3 &v) { translate(v.x(), v.y(), v.z()); }

		void translate(double x, double y, double z)
		{
			m_modelMatrix(0,3) += x;
			m_modelMatrix(1,3) += y;
			m_modelMatrix(2,3) += z;
		}


		void rotateX(double angle);
		void rotateY(double angle);
		void rotateZ(double angle);
		void rotate(double angle, double x, double y, double z); // ��x,y,z��������������תangle�Ƕ�


		void rotateXByPos(double angle, double y, double z);


		void rotateYByPos(double angle, double x, double z);


		void rotateZByPos(double angle, double x, double y);


		void scale(double ratio);

		void scale(double sx, double sy, double sz);

		//������������ϵ��
		Vec3 getScale() const;

		//�������������ϵ��
		void setScale(double sx, double sy, double sz);


		void scaleByPos(double ratio, double x, double y, double z);

		inline void scaleByPos(double ratio, const Vec3 &pos) { return scaleByPos(ratio, pos.x(), pos.y(), pos.z()); }

		Vec3 localToWorld(double x, double y, double z);
		inline Vec3 localToWorld(const Vec3 &pos) { return localToWorld(pos.x(), pos.y(), pos.z()); }

		Vec3 worldToLocal(double x, double y, double z);
		inline Vec3 worldToLocal(const Vec3 &pos) { return worldToLocal(pos.x(), pos.y(), pos.z()); }

		//���û�����ţ���ô�����ÿ���ת����������Ҫ�������˾���Ŀ��������ʺϸ��Ա任
		Vec3 fastWorldToLocal(double x, double y, double z);
		inline Vec3 fastWorldToLocal(const Vec3 &pos) { return fastWorldToLocal(pos.x(), pos.y(), pos.z()); }


		Vec3 localToParent(double x, double y, double z);
		inline Vec3 localToParent(const Vec3 &pos) { return localToParent(pos.x(), pos.y(), pos.z()); }

		Vec3 parentToLocal(double x, double y, double z);
		inline Vec3 parentToLocal(const Vec3 &pos) { return parentToLocal(pos.x(), pos.y(), pos.z()); }

		//���û�����ţ���ô�����ÿ���ת����������Ҫ�������˾���Ŀ�������
		Vec3 fastParentToLocal(double x, double y, double z);
		inline Vec3 fastParentToLocal(const Vec3 &pos) { return fastParentToLocal(pos.x(), pos.y(), pos.z()); }

		// mat * vec(local) = vec(world)
		Mat4 localToWorldMat() const;

		std::vector<RenderObject*> &renderObjects() { return m_renderObjects; }

	public:


		void addChildNode(RenderNode *child);

		RenderNode *parentNode() { return m_parentNode; }
		void setParentNode(RenderNode *parent);

		const std::vector<RenderNode*> & childNodes() const { return m_childNodes; }
		std::vector<RenderNode*> childNodes() { return m_childNodes; }

		int childNodeCount() const { return int(m_childNodes.size()); }
		bool haveChildNode() const { return !m_childNodes.empty(); }

		void destroyAllChildNodes();//delete all child
		void removeChildNode(RenderNode *childNode);//just remove ,not delete
		void clearAllChildNodes();//remove all

		std::vector<RenderNode *> parentNodes() const;

		void destroyAllObjects();
	public:


		virtual void render(RenderContext *rc);

		virtual void draw(RenderContext *rc);
	protected:

		friend class RenderContext;
		Mat4 m_modelMatrix;
		std::vector<RenderNode *> m_childNodes;
		std::vector<RenderObject*> m_renderObjects;
		RenderNode *m_parentNode;

	};


}

}//name space insight

