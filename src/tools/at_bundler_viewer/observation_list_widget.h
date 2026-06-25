#pragma once

#include <QWidget>

#include <QLabel>
#include <QTableWidget>

#include <string>
#include <vector>

namespace insight {

/// Lightweight record linking a 3D-point observation to its image.
struct ObservationRecord {
  int photo_id      = -1;
  int track_id      = -1;
  std::string image_path;
  std::string image_name; // basename for display
  double u           = 0.0;
  double v           = 0.0;
  double image_width  = 0.0;
  double image_height = 0.0;
};

/// Table listing all images that observe a selected 3D point.
/// Columns: image name, u, v.  Double-clicking a row emits observation_selected.
class ObservationListWidget : public QWidget {
  Q_OBJECT

signals:
  void observation_selected(const ObservationRecord& rec);

public:
  explicit ObservationListWidget(QWidget* parent = nullptr);

  /// Populate the table from a list of observation records.
  void set_observations(const std::vector<ObservationRecord>& observations);

  /// Clear the table.
  void clear();

  /// Number of rows currently in the table.
  int observation_count() const;

private slots:
  void on_double_click(int row, int col);

private:
  QTableWidget* table_        = nullptr;
  QLabel*       header_label_ = nullptr;
  std::vector<ObservationRecord> records_;
};

} // namespace insight
