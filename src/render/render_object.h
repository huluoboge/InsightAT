#pragma once

#include "render_global.h"

#include <QVariant>

#include <unordered_map>

namespace insight{

namespace render
{
	class RenderContext;

	class  RenderObject 
	{
	public:

		void show() { m_bVisible = true; }


		void hide() { m_bVisible = false; }


		void setVisible(bool visible) { m_bVisible = visible; }

		bool isVisible() const { return m_bVisible; }


		QVariant data(int role) const {
			if (m_roleData.end() == m_roleData.find(role)) return QVariant();
			return m_roleData.at(role);
		}
		void setData(int role, QVariant data) {
			m_roleData[role] = data;
		}

		virtual void draw(RenderContext *rc) {}
	protected:
		std::unordered_map<int, QVariant> m_roleData;
		bool m_bVisible = true;
	};


	 void exitRender();
	 void startRender();
}

}//name space insight
