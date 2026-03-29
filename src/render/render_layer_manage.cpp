#include "render_layer_manage.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

#include <memory>

namespace insight {

namespace render {
RenderLayerManage::RenderLayerManage(QWidget* parent /*= nullptr*/) : QWidget(parent) {
  m_treeWidget = new QTreeWidget(this);
  m_treeWidget->setColumnCount(1);
  QHBoxLayout* buttonLayout = new QHBoxLayout;
  QPushButton* moveTopBtn = new QPushButton(this);
  moveTopBtn->setText(tr("move up"));
  moveTopBtn->setEnabled(false);
  QPushButton* moveDownBtn = new QPushButton(this);
  moveDownBtn->setText(tr("move down"));
  moveDownBtn->setEnabled(false);
  QPushButton* zoomToBtn = new QPushButton(this);
  zoomToBtn->setText(tr("zoom to"));
  zoomToBtn->setEnabled(false);
  QPushButton* removeBtn = new QPushButton(this);
  removeBtn->setText(tr("remove"));
  removeBtn->setEnabled(false);

  buttonLayout->addWidget(moveTopBtn);
  buttonLayout->addWidget(moveDownBtn);
  buttonLayout->addWidget(zoomToBtn);
  buttonLayout->addWidget(removeBtn);
  buttonLayout->addStretch();

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->addLayout(buttonLayout);
  layout->addWidget(m_treeWidget);
  m_treeWidget->setHeaderHidden(true);
  m_treeWidget->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(m_treeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this,
          SLOT(on_tree_item_changed(QTreeWidgetItem*, int)));
  connect(moveTopBtn, SIGNAL(clicked()), this, SLOT(move_select_item_top()));
  connect(moveDownBtn, SIGNAL(clicked()), this, SLOT(move_select_item_down()));
  connect(zoomToBtn, SIGNAL(clicked()), this, SLOT(zoom_to_item()));
  connect(removeBtn, SIGNAL(clicked()), this, SLOT(remove_item()));
  connect(m_treeWidget, SIGNAL(itemSelectionChanged()), this, SLOT(item_selection_changed()));
  m_itemButtons.push_back(moveTopBtn);
  m_itemButtons.push_back(moveDownBtn);
  m_itemButtons.push_back(zoomToBtn);
  m_itemButtons.push_back(removeBtn);
}

void RenderLayerManage::refresh_datas() {
  m_treeWidget->clear();
  if (map_ == nullptr)
    return;
  QList<RenderLayer*> layers = map_->layers();
  QList<QTreeWidgetItem*> items;
  for (int i = 0; i < layers.size(); ++i) {
    QTreeWidgetItem* item =
        new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("%1").arg(layers[i]->name())));
    item->setData(0, Qt::UserRole, i);
    Qt::ItemFlags flags = item->flags();
    flags |= Qt::ItemIsUserCheckable;
    flags &= ~Qt::ItemIsEditable;
    item->setFlags(flags);
    if (layers[i]->isVisible()) {
      item->setCheckState(0, Qt::Checked);
    } else {
      item->setCheckState(0, Qt::Unchecked);
    }

    items.append(item);
  }
  m_treeWidget->insertTopLevelItems(0, items);
}

void RenderLayerManage::on_tree_item_changed(QTreeWidgetItem* item, int column) {
  QList<RenderLayer*> layers = map_->layers();
  int id = item->data(column, Qt::UserRole).toInt();
  Qt::CheckState state = item->checkState(column);

  if (state == Qt::Checked) {
    layers[id]->setVisible(true);

  } else {
    layers[id]->setVisible(false);
  }
  map_->repaint();
  map_->update();
}

void RenderLayerManage::move_select_item_top() {
  QList<QTreeWidgetItem*> items = m_treeWidget->selectedItems();
  if (items.count() == 1) {
    int id = items[0]->data(0, Qt::UserRole).toInt();
    if (id == 0)
      return;
    int id_top = id - 1;
    std::swap(map_->layers()[id], map_->layers()[id_top]);
    refresh_datas();
    map_->update();
  }
}

void RenderLayerManage::move_select_item_down() {
  QList<QTreeWidgetItem*> items = m_treeWidget->selectedItems();
  if (items.count() == 1) {
    int id = items[0]->data(0, Qt::UserRole).toInt();
    if (id >= map_->layers().size() - 1)
      return;
    int id_down = id + 1;
    std::swap(map_->layers()[id], map_->layers()[id_down]);
    refresh_datas();
    map_->update();
  }
}
void RenderLayerManage::zoom_to_item() {
  QList<QTreeWidgetItem*> items = m_treeWidget->selectedItems();
  if (items.count() == 1) {
    int id = items[0]->data(0, Qt::UserRole).toInt();
    map_->zoom_to_extent(map_->layers()[id]->extent());
    map_->update();
  }
}
void RenderLayerManage::remove_item() {
  QList<QTreeWidgetItem*> items = m_treeWidget->selectedItems();
  if (items.count() == 1) {
    int id = items[0]->data(0, Qt::UserRole).toInt();
    delete map_->layers()[id];
    map_->layers().removeAt(id);
    refresh_datas();
    map_->update();
  }
}

void RenderLayerManage::item_selection_changed() {
  QList<QTreeWidgetItem*> items = m_treeWidget->selectedItems();
  if (items.empty()) {
    for (int i = 0; i < m_itemButtons.size(); ++i) {
      m_itemButtons[i]->setEnabled(false);
    }
  } else {
    for (int i = 0; i < m_itemButtons.size(); ++i) {
      m_itemButtons[i]->setEnabled(true);
    }
  }
}
} // namespace render

} // namespace insight
