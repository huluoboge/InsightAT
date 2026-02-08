#pragma once
#include "render_global.h"
#include "render_map2.h"

#include <QTreeWidget>
#include <QList>
#include <QPushButton>

namespace insight{

namespace render
{
	class  RenderLayerManage : public QWidget
	{
		Q_OBJECT
	public:
		RenderLayerManage(QWidget *parent = nullptr);

		void setMap(RenderMap2 *map) { m_map = map; }

		void refreshDatas();
	public slots:
		void	itemChanged(QTreeWidgetItem * item, int column);
		void moveSelectItemTop();
		void moveSelectItemDown();
		void zoomToItem();
		void removeItem();
		void itemSelectionChanged();
	protected:
		QTreeWidget *m_treeWidget;
		RenderMap2 *m_map;
		QList<QPushButton*> m_itemButtons;
	};
}

}//name space insight
