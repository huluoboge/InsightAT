#pragma once
#include "render_global.h"
#include "render_map2.h"

#include <QList>
#include <QPushButton>
#include <QTreeWidget>

namespace insight {

namespace render {
class RENDER_EXPORT RenderLayerManage : public QWidget {
  Q_OBJECT
public:
  RenderLayerManage(QWidget* parent = nullptr);

  void set_map(RenderMap2* map) { map_ = map; }

  void refresh_datas();

public slots:
  void on_tree_item_changed(QTreeWidgetItem* item, int column);
  void move_select_item_top();
  void move_select_item_down();
  void zoom_to_item();
  void remove_item();
  void item_selection_changed();

protected:
  QTreeWidget* m_treeWidget;
  RenderMap2* map_;
  QList<QPushButton*> m_itemButtons;
};
} // namespace render

} // namespace insight
