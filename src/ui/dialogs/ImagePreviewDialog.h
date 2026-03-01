#ifndef INSIGHT_UI_IMAGE_PREVIEW_DIALOG_H
#define INSIGHT_UI_IMAGE_PREVIEW_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <vector>
#include <string>

namespace insight {
namespace ui {

/**
 * ImagePreviewDialog - Modal dialog for previewing images with navigation
 *
 * Features:
 * - Display image thumbnail in center
 * - Navigation buttons (Previous, Next)
 * - Show current image index and filename
 * - Close button
 *
 * Usage:
 *   ImagePreviewDialog preview(this);
 *   preview.SetImageList({"/path/to/img1.jpg", "/path/to/img2.jpg"});
 *   preview.ShowImage(0); // Show first image
 *   preview.exec();
 */
class ImagePreviewDialog : public QDialog {
    Q_OBJECT

public:
    explicit ImagePreviewDialog(QWidget* parent = nullptr);
    ~ImagePreviewDialog() = default;

    /**
     * Set the list of image paths to preview
     * @param image_paths Vector of full image paths
     */
    void SetImageList(const std::vector<std::string>& image_paths);

    /**
     * Display image at specified index
     * @param index Index in the image list (0-based)
     */
    void ShowImage(size_t index);

    /**
     * Get current image index being displayed
     */
    size_t GetCurrentIndex() const { return m_currentIndex; }

    /**
     * Get current image path being displayed
     */
    std::string GetCurrentImagePath() const;

private slots:
    void OnPreviousClicked();
    void OnNextClicked();
    void OnCloseClicked();

private:
    void UpdateDisplay();
    void UpdateNavigationButtons();

    // UI Components
    QLabel* m_imageLabel = nullptr; ///< Display image thumbnail
    QLabel* m_infoLabel = nullptr; ///< Display filename and index info
    QPushButton* m_prevButton = nullptr;
    QPushButton* m_nextButton = nullptr;
    QPushButton* m_closeButton = nullptr;

    // Data
    std::vector<std::string> m_imagePaths;
    size_t m_currentIndex = 0;

    // Constants
    static constexpr int MAX_PREVIEW_WIDTH = 600;
    static constexpr int MAX_PREVIEW_HEIGHT = 500;
};

} // namespace ui
} // namespace insight

#endif // INSIGHT_UI_IMAGE_PREVIEW_DIALOG_H
