#include "render_layer_manage.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>

#include <memory>

namespace insight{

namespace render
{
	RenderLayerManage::RenderLayerManage(QWidget *parent /*= nullptr*/)
		:QWidget(parent)
	{
		m_treeWidget = new QTreeWidget(this);
		m_treeWidget->setColumnCount(1);
		QHBoxLayout *buttonLayout = new QHBoxLayout;
		QPushButton *moveTopBtn = new QPushButton(this);
		moveTopBtn->setText(tr("move up"));
		moveTopBtn->setEnabled(false);
		QPushButton *moveDownBtn = new QPushButton(this);
		moveDownBtn->setText(tr("move down"));
		moveDownBtn->setEnabled(false);
		QPushButton *zoomToBtn = new QPushButton(this);
		zoomToBtn->setText(tr("zoom to"));
		zoomToBtn->setEnabled(false);
		QPushButton *removeBtn = new QPushButton(this);
		removeBtn->setText(tr("remove"));
		removeBtn->setEnabled(false);

		buttonLayout->addWidget(moveTopBtn);
		buttonLayout->addWidget(moveDownBtn);
		buttonLayout->addWidget(zoomToBtn);
		buttonLayout->addWidget(removeBtn);
		buttonLayout->addStretch();

		QVBoxLayout *layout = new QVBoxLayout(this);
		layout->addLayout(buttonLayout);
		layout->addWidget(m_treeWidget);
		m_treeWidget->setHeaderHidden(true);
		m_treeWidget->setSelectionMode(QAbstractItemView::SingleSelection);
		connect(m_treeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(itemChanged(QTreeWidgetItem*, int)));
		connect(moveTopBtn, SIGNAL(clicked()), this, SLOT(moveSelectItemTop()));
		connect(moveDownBtn, SIGNAL(clicked()), this, SLOT(moveSelectItemDown()));
		connect(zoomToBtn, SIGNAL(clicked()), this, SLOT(zoomToItem()));
		connect(removeBtn, SIGNAL(clicked()), this, SLOT(removeItem()));
		connect(m_treeWidget, SIGNAL(itemSelectionChanged()), this, SLOT(itemSelectionChanged()));
		m_itemButtons.push_back(moveTopBtn);
		m_itemButtons.push_back(moveDownBtn);
		m_itemButtons.push_back(zoomToBtn);
		m_itemButtons.push_back(removeBtn);
	}

	void RenderLayerManage::refreshDatas()
	{
		m_treeWidget->clear();
		if (m_map == nullptr) return;
		QList<RenderLayer *> layers = m_map->layers();
		QList<QTreeWidgetItem *> items;
		for (int i = 0; i < layers.size(); ++i)
		{
			QTreeWidgetItem *item = new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("%1").arg(layers[i]->name())));
			item->setData(0, Qt::UserRole, i);
			Qt::ItemFlags flags = item->flags();
			flags |= Qt::ItemIsUserCheckable;
			flags &= ~Qt::ItemIsEditable;
			item->setFlags(flags);
			if (layers[i]->isVisible()) {
				item->setCheckState(0, Qt::Checked);
			}
			else {
				item->setCheckState(0, Qt::Unchecked);
			}
			
			items.append(item);
		}
		m_treeWidget->insertTopLevelItems(0, items);
	}

	void RenderLayerManage::itemChanged(QTreeWidgetItem * item, int column)
	{
		QList<RenderLayer *> layers = m_map->layers();
		int id = item->data(column, Qt::UserRole).toInt();
		Qt::CheckState state = item->checkState(column);
		
		if (state == Qt::Checked) {
			layers[id]->setVisible(true);
			
		}
		else {
			layers[id]->setVisible(false);
		}
		m_map->repaint();
		m_map->update();
	}

	void RenderLayerManage::moveSelectItemTop()
	{
		QList<QTreeWidgetItem*>items = m_treeWidget->selectedItems();
		if (items.count() == 1) {
			int id = items[0]->data(0, Qt::UserRole).toInt();
			if (id == 0) return;
			int id_top = id - 1;
			std::swap(m_map->layers()[id], m_map->layers()[id_top]);
			refreshDatas();
			m_map->update();
		}
	}

	void RenderLayerManage::moveSelectItemDown()
	{
		QList<QTreeWidgetItem*>items = m_treeWidget->selectedItems();
		if (items.count() == 1) {
			int id = items[0]->data(0, Qt::UserRole).toInt();
			if (id >= m_map->layers().size()-1) return;
			int id_down = id +1;
			std::swap(m_map->layers()[id], m_map->layers()[id_down]);
			refreshDatas();
			m_map->update();
		}
	}
	void RenderLayerManage::zoomToItem()
	{
		QList<QTreeWidgetItem*>items = m_treeWidget->selectedItems();
		if (items.count() == 1) {
			int id = items[0]->data(0, Qt::UserRole).toInt();
			m_map->zoomToExtent(m_map->layers()[id]->extent());
			m_map->update();
		}
	}
	void RenderLayerManage::removeItem()
	{
		QList<QTreeWidgetItem*>items = m_treeWidget->selectedItems();
		if (items.count() == 1) {
			int id = items[0]->data(0, Qt::UserRole).toInt();
			delete m_map->layers()[id];
			m_map->layers().removeAt(id);
			refreshDatas();
			m_map->update();
		}
	}

	void RenderLayerManage::itemSelectionChanged()
	{
		QList<QTreeWidgetItem*>items = m_treeWidget->selectedItems();
		if (items.empty()) {
			for (int i = 0; i < m_itemButtons.size(); ++i)
			{
				m_itemButtons[i]->setEnabled(false);
			}
		}
		else {
			for (int i = 0; i < m_itemButtons.size(); ++i)
			{
				m_itemButtons[i]->setEnabled(true);
			}
		}
	}
}

}//name space insight
