#include "ImagePreviewDialog.h"

#include <QPixmap>
#include <QFileInfo>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <sstream>
#include <glog/logging.h>

namespace insight {
namespace ui {

ImagePreviewDialog::ImagePreviewDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("Image Preview");
    setModal(true);
    setMinimumSize(700, 600);

    // Create image display label
    m_imageLabel = new QLabel(this);
    m_imageLabel->setAlignment(Qt::AlignCenter);
    m_imageLabel->setMinimumSize(MAX_PREVIEW_WIDTH, MAX_PREVIEW_HEIGHT);
    m_imageLabel->setStyleSheet("QLabel { border: 1px solid #ccc; background-color: #f0f0f0; }");

    // Create info label (filename and index)
    m_infoLabel = new QLabel(this);
    m_infoLabel->setAlignment(Qt::AlignCenter);
    m_infoLabel->setStyleSheet("QLabel { padding: 5px; font-weight: bold; }");

    // Create navigation buttons
    m_prevButton = new QPushButton("Previous", this);
    m_nextButton = new QPushButton("Next", this);
    m_closeButton = new QPushButton("Close", this);

    // Set button sizes
    m_prevButton->setMaximumWidth(100);
    m_nextButton->setMaximumWidth(100);
    m_closeButton->setMaximumWidth(100);

    // Create layout
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Add image label
    mainLayout->addWidget(m_imageLabel, 1);

    // Add info label
    mainLayout->addWidget(m_infoLabel);

    // Add navigation buttons layout
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_prevButton);
    buttonLayout->addWidget(m_nextButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_closeButton);
    mainLayout->addLayout(buttonLayout);

    setLayout(mainLayout);

    // Connect signals
    connect(m_prevButton, &QPushButton::clicked, this, &ImagePreviewDialog::OnPreviousClicked);
    connect(m_nextButton, &QPushButton::clicked, this, &ImagePreviewDialog::OnNextClicked);
    connect(m_closeButton, &QPushButton::clicked, this, &ImagePreviewDialog::OnCloseClicked);

    UpdateNavigationButtons();
}

void ImagePreviewDialog::SetImageList(const std::vector<std::string>& image_paths)
{
    m_imagePaths = image_paths;
    m_currentIndex = 0;
    UpdateDisplay();
}

void ImagePreviewDialog::ShowImage(size_t index)
{
    if (index >= m_imagePaths.size()) {
        LOG(WARNING) << "Image index " << index << " out of range";
        return;
    }
    m_currentIndex = index;
    UpdateDisplay();
}

std::string ImagePreviewDialog::GetCurrentImagePath() const
{
    if (m_currentIndex < m_imagePaths.size()) {
        return m_imagePaths[m_currentIndex];
    }
    return "";
}

void ImagePreviewDialog::OnPreviousClicked()
{
    if (m_currentIndex > 0) {
        m_currentIndex--;
        UpdateDisplay();
    }
}

void ImagePreviewDialog::OnNextClicked()
{
    if (m_currentIndex < m_imagePaths.size() - 1) {
        m_currentIndex++;
        UpdateDisplay();
    }
}

void ImagePreviewDialog::OnCloseClicked()
{
    accept();
}

void ImagePreviewDialog::UpdateDisplay()
{
    if (m_imagePaths.empty()) {
        m_imageLabel->setText("No images to display");
        m_infoLabel->setText("");
        return;
    }

    const std::string& imagePath = m_imagePaths[m_currentIndex];

    // Load and display image
    QPixmap pixmap(QString::fromStdString(imagePath));
    if (pixmap.isNull()) {
        m_imageLabel->setText(QString("Failed to load image:\n%1").arg(QString::fromStdString(imagePath)));
        LOG(WARNING) << "Failed to load image: " << imagePath;
    } else {
        // Scale pixmap to fit in label while maintaining aspect ratio
        QPixmap scaledPixmap = pixmap.scaled(MAX_PREVIEW_WIDTH, MAX_PREVIEW_HEIGHT, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        m_imageLabel->setPixmap(scaledPixmap);
    }

    // Update info label
    QFileInfo fileInfo(QString::fromStdString(imagePath));
    std::ostringstream oss;
    oss << "[" << (m_currentIndex + 1) << "/" << m_imagePaths.size() << "] "
        << fileInfo.fileName().toStdString();
    m_infoLabel->setText(QString::fromStdString(oss.str()));

    UpdateNavigationButtons();
}

void ImagePreviewDialog::UpdateNavigationButtons()
{
    m_prevButton->setEnabled(m_currentIndex > 0);
    m_nextButton->setEnabled(m_currentIndex < m_imagePaths.size() - 1);
}

} // namespace ui
} // namespace insight
