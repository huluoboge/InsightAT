#include "observation_list_widget.h"

#include <QHeaderView>
#include <QVBoxLayout>

namespace insight {

ObservationListWidget::ObservationListWidget(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  header_label_ = new QLabel(tr("Observations of selected 3D point"), this);
  header_label_->setStyleSheet(QStringLiteral("font-weight: bold;"));
  layout->addWidget(header_label_);

  table_ = new QTableWidget(0, 3, this);
  table_->setHorizontalHeaderLabels({tr("Image"), tr("u (px)"), tr("v (px)")});
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->verticalHeader()->setVisible(false);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->setColumnWidth(0, 180);
  table_->setColumnWidth(1, 80);
  table_->setColumnWidth(2, 80);

  connect(table_, &QTableWidget::cellDoubleClicked,
          this, &ObservationListWidget::on_double_click);

  layout->addWidget(table_);
}

void ObservationListWidget::set_observations(
    const std::vector<ObservationRecord>& observations) {
  records_ = observations;
  table_->setRowCount(0);

  if (observations.empty()) {
    header_label_->setText(tr("Observations of selected 3D point (0)"));
    return;
  }

  // Use the first record's track_id to identify which point we're showing.
  header_label_->setText(
      tr("Observations of 3D point #%1 (%2 images)")
          .arg(observations[0].track_id)
          .arg(observations.size()));

  table_->setRowCount(static_cast<int>(observations.size()));
  for (size_t i = 0; i < observations.size(); ++i) {
    const auto& rec = observations[i];
    const int row   = static_cast<int>(i);

    // Image name
    auto* name_item = new QTableWidgetItem(
        QString::fromStdString(rec.image_name));
    name_item->setData(Qt::UserRole, QString::fromStdString(rec.image_path));
    table_->setItem(row, 0, name_item);

    // u coordinate
    auto* u_item = new QTableWidgetItem(
        QString::number(rec.u, 'f', 1));
    table_->setItem(row, 1, u_item);

    // v coordinate
    auto* v_item = new QTableWidgetItem(
        QString::number(rec.v, 'f', 1));
    table_->setItem(row, 2, v_item);
  }
}

void ObservationListWidget::clear() {
  records_.clear();
  table_->setRowCount(0);
  header_label_->setText(tr("Observations of selected 3D point"));
}

int ObservationListWidget::observation_count() const {
  return table_->rowCount();
}

void ObservationListWidget::on_double_click(int row, int /*col*/) {
  if (row < 0 || row >= static_cast<int>(records_.size()))
    return;
  emit observation_selected(records_[row]);
}

} // namespace insight
