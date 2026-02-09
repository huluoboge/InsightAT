#ifndef INSIGHT_UI_IMAGE_EDITOR_DIALOG_H
#define INSIGHT_UI_IMAGE_EDITOR_DIALOG_H

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QWidget>
#include <vector>
#include <string>
#include <cstdint>
#include "database/database_types.h"

namespace insight {

// Forward declarations
namespace ui {
class ProjectDocument;
}

namespace database {
    struct ImageGroup;
}

namespace ui {

/**
 * ImageEditorDialog - Non-modal dialog for image management in an ImageGroup
 *
 * Features:
 * - QTableWidget with columns: [Status] [Image ID] [Filename] [Delete]
 * - File existence checking with visual indicators (✓ green, ⚠️ red)
 * - Row coloring: white for valid files, light red for missing files
 * - Add individual images or entire folders
 * - Multi-select deletion
 * - Image preview integration
 * - Right panel: GNSS measurement details display
 * - GNSS import/clear operations
 *
 * Signals:
 * - imagesChanged(uint32_t group_id) - emitted when images are modified
 *
 * Usage:
 *   ImageEditorDialog editor(m_projectDocument, this);
 *   editor.LoadGroup(group_pointer);
 *   editor.show();
 *   connect(&editor, &ImageEditorDialog::imagesChanged, this, &MySlot);
 */
class ImageEditorDialog : public QDialog {
    Q_OBJECT

public:
    explicit ImageEditorDialog(ProjectDocument* projectDoc, QWidget* parent = nullptr);
    ~ImageEditorDialog() = default;

    /**
     * Load an image group for editing
     * @param group Pointer to ImageGroup to edit (must be valid)
     */
    void LoadGroup(database::ImageGroup* group);

    /**
     * Get list of modified images (if needed for external processing)
     * @return Vector of image IDs that were added or modified
     */
    std::vector<uint32_t> GetModifiedImages() const;

    /**
     * Refresh table display and file status indicators
     */
    void RefreshTable();

signals:
    /**
     * Emitted when images are added, removed, or modified
     * @param group_id ID of the modified group
     */
    void imagesChanged(uint32_t group_id);

private slots:
    void OnAddImagesClicked();
    void OnAddFolderClicked();
    void OnDeleteSelectedClicked();
    void OnPreviewClicked();
    void OnCloseClicked();
    void OnTableSelectionChanged();
    void OnShowGNSSDataToggled();
    void OnImportGNSSClicked();
    void OnClearGNSSClicked();
    void OnGNSSDataChanged();
    void OnEditGNSSClicked();
    void OnSetAllCovarianceClicked();

private:
    void InitializeUI();
    void PopulateTable();
    void UpdateTableRowStatus(int row);
    void SetRowFileExistsStatus(int row, bool exists);
    void UpdateGNSSDetailsPanel();
    void DisplayGNSSDetails(const database::Image* image);
    std::string GetImageFilename(const std::string& fullPath) const;
    bool FileExists(const std::string& fullPath) const;
    void AddImageToGroup(const std::string& fullPath);
    void DeleteSelectedRows();

    // UI Components - Main layout
    QWidget* m_centralWidget = nullptr;
    QHBoxLayout* m_mainLayout = nullptr;

    // Left panel - Image list
    QWidget* m_leftPanel = nullptr;
    QVBoxLayout* m_leftLayout = nullptr;
    QTableWidget* m_imageTable = nullptr;
    QHBoxLayout* m_leftButtonLayout = nullptr;
    QPushButton* m_addImagesButton = nullptr;
    QPushButton* m_addFolderButton = nullptr;
    QPushButton* m_deleteButton = nullptr;
    QPushButton* m_previewButton = nullptr;

    // Right panel - GNSS details
    QWidget* m_rightPanel = nullptr;
    QVBoxLayout* m_rightLayout = nullptr;
    QPushButton* m_showGNSSDataButton = nullptr;
    QGroupBox* m_gnssDetailsGroup = nullptr;
    QWidget* m_gnssDetailsWidget = nullptr;
    QVBoxLayout* m_gnssDetailsLayout = nullptr;

    // GNSS detail display
    QLabel* m_gnssImageIdLabel = nullptr;
    QLabel* m_gnssImageIdValue = nullptr;
    QLabel* m_gnssCoordLabel = nullptr;
    QLabel* m_gnssXValue = nullptr;
    QLabel* m_gnssYValue = nullptr;
    QLabel* m_gnssZValue = nullptr;
    QLabel* m_gnssCovLabel = nullptr;
    QLabel* m_gnssCovXXValue = nullptr;
    QLabel* m_gnssCovYYValue = nullptr;
    QLabel* m_gnssCovZZValue = nullptr;
    QLabel* m_gnssStatusLabel = nullptr;

    // GNSS action buttons
    QHBoxLayout* m_gnssButtonLayout = nullptr;
    QPushButton* m_importGNSSButton = nullptr;
    QPushButton* m_clearGNSSButton = nullptr;
    QPushButton* m_editGNSSButton = nullptr;
    QPushButton* m_setAllCovarianceButton = nullptr;
    QPushButton* m_closeButton = nullptr;

    // Data
    ProjectDocument* m_projectDocument = nullptr;
    database::ImageGroup* m_currentGroup = nullptr;
    std::vector<uint32_t> m_modifiedImages;
    bool m_gnssDataVisible = false;
    int m_selectedImageIndex = -1;

    // Constants
    static constexpr int COLUMN_STATUS = 0;
    static constexpr int COLUMN_ID = 1;
    static constexpr int COLUMN_FILENAME = 2;
    static constexpr int COLUMN_DELETE = 3;
    static constexpr int NUM_COLUMNS = 4;
};

} // namespace ui
} // namespace insight

#endif // INSIGHT_UI_IMAGE_EDITOR_DIALOG_H
