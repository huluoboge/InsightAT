#include "ImageEditorDialog.h"
#include "ImagePreviewDialog.h"
#include "../models/ProjectDocument.h"
#include "../widgets/GNSSMeasurementImportDialog.h"
#include "database/database_types.h"

#include <QHeaderView>
#include <QFileDialog>
#include <QMessageBox>
#include <QFileInfo>
#include <QDir>
#include <QApplication>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleValidator>
#include <filesystem>
#include <glog/logging.h>
#include <cmath>

namespace fs = std::filesystem;

namespace insight {
namespace ui {

ImageEditorDialog::ImageEditorDialog(ProjectDocument* projectDoc, QWidget* parent)
    : QDialog(parent), m_projectDocument(projectDoc)
{
    setWindowTitle("Image Editor");
    setModal(false);
    setMinimumSize(900, 500);

    InitializeUI();
    
    // Connect ProjectDocument signals if needed
    if (m_projectDocument) {
        // Could connect to data change signals here
    }
}

void ImageEditorDialog::InitializeUI()
{
    // Create main central widget and horizontal layout (left + right panels)
    m_centralWidget = new QWidget(this);
    m_mainLayout = new QHBoxLayout(m_centralWidget);
    m_mainLayout->setContentsMargins(5, 5, 5, 5);

    // ========== LEFT PANEL - Image List ==========
    m_leftPanel = new QWidget(m_centralWidget);
    m_leftLayout = new QVBoxLayout(m_leftPanel);
    m_leftLayout->setContentsMargins(0, 0, 0, 0);

    // Create table widget
    m_imageTable = new QTableWidget(m_leftPanel);
    m_imageTable->setColumnCount(NUM_COLUMNS);
    m_imageTable->setHorizontalHeaderLabels({"Status", "Image ID", "Filename", "Action"});
    m_imageTable->horizontalHeader()->setStretchLastSection(false);
    m_imageTable->setColumnWidth(COLUMN_STATUS, 60);
    m_imageTable->setColumnWidth(COLUMN_ID, 80);
    m_imageTable->setColumnWidth(COLUMN_FILENAME, 300);
    m_imageTable->setColumnWidth(COLUMN_DELETE, 80);
    m_imageTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_imageTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_imageTable->setAlternatingRowColors(true);
    m_leftLayout->addWidget(m_imageTable, 1);

    // Create buttons for left panel
    m_addImagesButton = new QPushButton("Add Images", m_leftPanel);
    m_addFolderButton = new QPushButton("Add Folder", m_leftPanel);
    m_deleteButton = new QPushButton("Delete Selected", m_leftPanel);
    m_previewButton = new QPushButton("Preview", m_leftPanel);

    // Set button sizes
    m_addImagesButton->setMaximumWidth(120);
    m_addFolderButton->setMaximumWidth(120);
    m_deleteButton->setMaximumWidth(120);
    m_previewButton->setMaximumWidth(120);

    // Button layout for left panel
    m_leftButtonLayout = new QHBoxLayout();
    m_leftButtonLayout->addWidget(m_addImagesButton);
    m_leftButtonLayout->addWidget(m_addFolderButton);
    m_leftButtonLayout->addWidget(m_deleteButton);
    m_leftButtonLayout->addWidget(m_previewButton);
    m_leftButtonLayout->addStretch();
    m_leftLayout->addLayout(m_leftButtonLayout);

    m_mainLayout->addWidget(m_leftPanel, 2);  // Left panel takes 2/3 space

    // ========== RIGHT PANEL - GNSS Details ==========
    m_rightPanel = new QWidget(m_centralWidget);
    m_rightLayout = new QVBoxLayout(m_rightPanel);
    m_rightLayout->setContentsMargins(0, 0, 0, 0);
    m_rightPanel->setMaximumWidth(300);
    m_rightPanel->setMinimumWidth(250);

    // Show GNSS Data button
    m_showGNSSDataButton = new QPushButton("Show GNSS Data", m_rightPanel);
    m_showGNSSDataButton->setCheckable(true);
    m_showGNSSDataButton->setMaximumHeight(30);
    m_rightLayout->addWidget(m_showGNSSDataButton);

    // GNSS Details Group Box
    m_gnssDetailsGroup = new QGroupBox("GNSS Details", m_rightPanel);
    m_gnssDetailsGroup->setVisible(false);
    m_gnssDetailsLayout = new QVBoxLayout(m_gnssDetailsGroup);

    // Create GNSS detail labels
    m_gnssImageIdLabel = new QLabel("Image ID:", m_gnssDetailsGroup);
    m_gnssImageIdValue = new QLabel("None", m_gnssDetailsGroup);
    m_gnssImageIdValue->setStyleSheet("font-weight: bold;");

    m_gnssCoordLabel = new QLabel("Coordinates:", m_gnssDetailsGroup);
    m_gnssXValue = new QLabel("X: --", m_gnssDetailsGroup);
    m_gnssYValue = new QLabel("Y: --", m_gnssDetailsGroup);
    m_gnssZValue = new QLabel("Z: --", m_gnssDetailsGroup);

    m_gnssCovLabel = new QLabel("Covariance (m²):", m_gnssDetailsGroup);
    m_gnssCovXXValue = new QLabel("σ_xx: --", m_gnssDetailsGroup);
    m_gnssCovYYValue = new QLabel("σ_yy: --", m_gnssDetailsGroup);
    m_gnssCovZZValue = new QLabel("σ_zz: --", m_gnssDetailsGroup);

    m_gnssStatusLabel = new QLabel("Status: No data", m_gnssDetailsGroup);
    m_gnssStatusLabel->setStyleSheet("color: #888;");

    // Add labels to layout
    m_gnssDetailsLayout->addWidget(m_gnssImageIdLabel);
    m_gnssDetailsLayout->addWidget(m_gnssImageIdValue);
    m_gnssDetailsLayout->addSpacing(10);
    m_gnssDetailsLayout->addWidget(m_gnssCoordLabel);
    m_gnssDetailsLayout->addWidget(m_gnssXValue);
    m_gnssDetailsLayout->addWidget(m_gnssYValue);
    m_gnssDetailsLayout->addWidget(m_gnssZValue);
    m_gnssDetailsLayout->addSpacing(10);
    m_gnssDetailsLayout->addWidget(m_gnssCovLabel);
    m_gnssDetailsLayout->addWidget(m_gnssCovXXValue);
    m_gnssDetailsLayout->addWidget(m_gnssCovYYValue);
    m_gnssDetailsLayout->addWidget(m_gnssCovZZValue);
    m_gnssDetailsLayout->addSpacing(10);
    m_gnssDetailsLayout->addWidget(m_gnssStatusLabel);
    m_gnssDetailsLayout->addStretch();

    m_rightLayout->addWidget(m_gnssDetailsGroup, 1);

    // GNSS action buttons - create two rows of buttons
    QVBoxLayout* gnssActionsLayout = new QVBoxLayout();
    
    // First row: Import, Clear, Edit
    m_gnssButtonLayout = new QHBoxLayout();
    m_importGNSSButton = new QPushButton("Import GNSS", m_rightPanel);
    m_clearGNSSButton = new QPushButton("Clear GNSS", m_rightPanel);
    m_editGNSSButton = new QPushButton("Edit GNSS", m_rightPanel);
    m_importGNSSButton->setMaximumWidth(110);
    m_clearGNSSButton->setMaximumWidth(110);
    m_editGNSSButton->setMaximumWidth(110);
    m_gnssButtonLayout->addWidget(m_importGNSSButton);
    m_gnssButtonLayout->addWidget(m_clearGNSSButton);
    m_gnssButtonLayout->addWidget(m_editGNSSButton);
    gnssActionsLayout->addLayout(m_gnssButtonLayout);
    
    // Second row: Set All Covariance
    QHBoxLayout* covarianceLayout = new QHBoxLayout();
    m_setAllCovarianceButton = new QPushButton("Set All Covariance", m_rightPanel);
    m_setAllCovarianceButton->setMaximumWidth(340);
    covarianceLayout->addWidget(m_setAllCovarianceButton);
    gnssActionsLayout->addLayout(covarianceLayout);
    
    m_rightLayout->addLayout(gnssActionsLayout);

    m_mainLayout->addWidget(m_rightPanel, 1);  // Right panel takes 1/3 space

    // Close button at the bottom (full width)
    QHBoxLayout* bottomLayout = new QHBoxLayout();
    m_closeButton = new QPushButton("Close", this);
    m_closeButton->setMaximumWidth(120);
    bottomLayout->addStretch();
    bottomLayout->addWidget(m_closeButton);

    // Create dialog layout
    QVBoxLayout* dialogLayout = new QVBoxLayout(this);
    dialogLayout->addWidget(m_centralWidget, 1);
    dialogLayout->addLayout(bottomLayout);
    setLayout(dialogLayout);

    // Connect signals
    connect(m_addImagesButton, &QPushButton::clicked, this, &ImageEditorDialog::OnAddImagesClicked);
    connect(m_addFolderButton, &QPushButton::clicked, this, &ImageEditorDialog::OnAddFolderClicked);
    connect(m_deleteButton, &QPushButton::clicked, this, &ImageEditorDialog::OnDeleteSelectedClicked);
    connect(m_previewButton, &QPushButton::clicked, this, &ImageEditorDialog::OnPreviewClicked);
    connect(m_showGNSSDataButton, &QPushButton::toggled, this, &ImageEditorDialog::OnShowGNSSDataToggled);
    connect(m_importGNSSButton, &QPushButton::clicked, this, &ImageEditorDialog::OnImportGNSSClicked);
    connect(m_clearGNSSButton, &QPushButton::clicked, this, &ImageEditorDialog::OnClearGNSSClicked);
    connect(m_editGNSSButton, &QPushButton::clicked, this, &ImageEditorDialog::OnEditGNSSClicked);
    connect(m_setAllCovarianceButton, &QPushButton::clicked, this, &ImageEditorDialog::OnSetAllCovarianceClicked);
    connect(m_closeButton, &QPushButton::clicked, this, &ImageEditorDialog::OnCloseClicked);
    connect(m_imageTable, &QTableWidget::itemSelectionChanged, this, &ImageEditorDialog::OnTableSelectionChanged);
}

void ImageEditorDialog::LoadGroup(database::ImageGroup* group)
{
    m_currentGroup = group;
    if (!m_currentGroup) {
        LOG(ERROR) << "ImageEditorDialog::LoadGroup - group pointer is null";
        return;
    }

    setWindowTitle(QString("Image Editor - Group: %1").arg(QString::fromStdString(m_currentGroup->group_name)));
    m_modifiedImages.clear();
    PopulateTable();
}

void ImageEditorDialog::PopulateTable()
{
    if (!m_currentGroup) return;

    m_imageTable->setRowCount(0);

    for (const auto& image : m_currentGroup->images) {
        int row = m_imageTable->rowCount();
        m_imageTable->insertRow(row);

        // Status column (will be updated by UpdateTableRowStatus)
        QTableWidgetItem* statusItem = new QTableWidgetItem();
        m_imageTable->setItem(row, COLUMN_STATUS, statusItem);

        // Image ID column
        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(image.image_id));
        idItem->setFlags(idItem->flags() & ~Qt::ItemIsEditable);
        m_imageTable->setItem(row, COLUMN_ID, idItem);

        // Filename column
        std::string filename = GetImageFilename(image.filename);
        QTableWidgetItem* filenameItem = new QTableWidgetItem(QString::fromStdString(filename));
        filenameItem->setFlags(filenameItem->flags() & ~Qt::ItemIsEditable);
        m_imageTable->setItem(row, COLUMN_FILENAME, filenameItem);

        // Store full path as tooltip for later display
        filenameItem->setToolTip(QString::fromStdString(image.filename));

        // Delete button
        QPushButton* deleteBtn = new QPushButton("Delete", this);
        deleteBtn->setMaximumWidth(70);
        m_imageTable->setCellWidget(row, COLUMN_DELETE, deleteBtn);

        // Connect delete button to a lambda that deletes this specific row
        connect(deleteBtn, &QPushButton::clicked, [this, row]() {
            if (row >= 0 && row < m_imageTable->rowCount()) {
                uint32_t imageId = m_imageTable->item(row, COLUMN_ID)->text().toUInt();
                m_imageTable->removeRow(row);
                
                // Remove from group
                if (m_currentGroup) {
                    auto it = m_currentGroup->images.begin();
                    while (it != m_currentGroup->images.end()) {
                        if (it->image_id == imageId) {
                            it = m_currentGroup->images.erase(it);
                        } else {
                            ++it;
                        }
                    }
                    if (m_projectDocument) {
                        m_projectDocument->saveProject();
                    }
                    emit imagesChanged(m_currentGroup->group_id);
                }
            }
        });

        // Update status
        UpdateTableRowStatus(row);
    }
}

void ImageEditorDialog::UpdateTableRowStatus(int row)
{
    if (row < 0 || row >= m_imageTable->rowCount()) return;

    // Get full path from tooltip or filename cell
    QString fullPath = m_imageTable->item(row, COLUMN_FILENAME)->toolTip();
    if (fullPath.isEmpty()) {
        fullPath = m_imageTable->item(row, COLUMN_FILENAME)->text();
    }

    std::string path = fullPath.toStdString();
    bool exists = FileExists(path);

    SetRowFileExistsStatus(row, exists);
}

void ImageEditorDialog::SetRowFileExistsStatus(int row, bool exists)
{
    QTableWidgetItem* statusItem = m_imageTable->item(row, COLUMN_STATUS);
    if (!statusItem) {
        statusItem = new QTableWidgetItem();
        m_imageTable->setItem(row, COLUMN_STATUS, statusItem);
    }

    if (exists) {
        // Green status: ✓
        statusItem->setText("✓");
        statusItem->setForeground(QColor(0, 128, 0)); // Green
        statusItem->setFont(QFont("Arial", 14, QFont::Bold));
        
        // White background for valid files
        for (int col = 0; col < NUM_COLUMNS; ++col) {
            QTableWidgetItem* item = m_imageTable->item(row, col);
            if (item) {
                item->setBackground(QColor(255, 255, 255)); // White
            }
        }
    } else {
        // Red status: ⚠️
        statusItem->setText("⚠");
        statusItem->setForeground(QColor(255, 0, 0)); // Red
        statusItem->setFont(QFont("Arial", 14, QFont::Bold));
        
        // Light red background for missing files
        for (int col = 0; col < NUM_COLUMNS; ++col) {
            QTableWidgetItem* item = m_imageTable->item(row, col);
            if (item) {
                item->setBackground(QColor(255, 200, 200)); // Light red
            }
        }
    }
}

std::string ImageEditorDialog::GetImageFilename(const std::string& fullPath) const
{
    QFileInfo fileInfo(QString::fromStdString(fullPath));
    return fileInfo.fileName().toStdString();
}

bool ImageEditorDialog::FileExists(const std::string& fullPath) const
{
    try {
        return fs::exists(fullPath);
    } catch (...) {
        return false;
    }
}

void ImageEditorDialog::OnAddImagesClicked()
{
    if (!m_projectDocument || !m_currentGroup) {
        QMessageBox::warning(this, "Error", "No group selected");
        return;
    }

    QString filter = "Image Files (*.jpg *.JPG *.jpeg *.JPEG *.png *.PNG *.tiff *.TIFF *.tif *.TIF *.bmp *.BMP);;All Files (*)";
    QStringList filePaths = QFileDialog::getOpenFileNames(
        this,
        "Select Images",
        QDir::homePath(),
        filter
    );

    for (const QString& filePath : filePaths) {
        AddImageToGroup(filePath.toStdString());
    }

    if (!filePaths.isEmpty()) {
        PopulateTable();
        if (m_projectDocument) {
            m_projectDocument->saveProject();
        }
        emit imagesChanged(m_currentGroup->group_id);
    }
}

void ImageEditorDialog::OnAddFolderClicked()
{
    if (!m_projectDocument || !m_currentGroup) {
        QMessageBox::warning(this, "Error", "No group selected");
        return;
    }

    QString folderPath = QFileDialog::getExistingDirectory(
        this,
        "Select Folder",
        QDir::homePath()
    );

    if (folderPath.isEmpty()) return;

    // Get all image files in folder
    QDir dir(folderPath);
    dir.setNameFilters({"*.jpg", "*.JPG", "*.jpeg", "*.JPEG", "*.png", "*.PNG", 
                        "*.tiff", "*.TIFF", "*.tif", "*.TIF", "*.bmp", "*.BMP"});
    dir.setFilter(QDir::Files | QDir::NoSymLinks);

    QStringList files = dir.entryList();
    if (files.isEmpty()) {
        QMessageBox::information(this, "Info", "No image files found in folder");
        return;
    }

    int addedCount = 0;
    for (const QString& filename : files) {
        QString fullPath = dir.filePath(filename);
        AddImageToGroup(fullPath.toStdString());
        addedCount++;
    }

    if (addedCount > 0) {
        PopulateTable();
        if (m_projectDocument) {
            m_projectDocument->saveProject();
        }
        emit imagesChanged(m_currentGroup->group_id);
        QMessageBox::information(this, "Success", QString("Added %1 images").arg(addedCount));
    }
}

void ImageEditorDialog::OnDeleteSelectedClicked()
{
    QList<QTableWidgetItem*> selectedItems = m_imageTable->selectedItems();
    if (selectedItems.isEmpty()) {
        QMessageBox::warning(this, "Warning", "No images selected");
        return;
    }

    int reply = QMessageBox::question(
        this,
        "Confirm Delete",
        QString("Delete %1 image(s)?").arg(m_imageTable->selectionModel()->selectedRows().count()),
        QMessageBox::Yes | QMessageBox::No
    );

    if (reply == QMessageBox::No) return;

    DeleteSelectedRows();
}

void ImageEditorDialog::OnPreviewClicked()
{
    QList<QTableWidgetItem*> selectedItems = m_imageTable->selectedItems();
    if (selectedItems.isEmpty()) {
        QMessageBox::warning(this, "Warning", "No image selected for preview");
        return;
    }

    // Get unique rows from selection
    QSet<int> selectedRows;
    for (auto item : selectedItems) {
        selectedRows.insert(m_imageTable->row(item));
    }

    // Build image path list
    std::vector<std::string> imagePaths;
    for (int row : selectedRows) {
        QTableWidgetItem* filenameItem = m_imageTable->item(row, COLUMN_FILENAME);
        if (filenameItem) {
            // Get full path from tooltip
            QString fullPath = filenameItem->toolTip();
            imagePaths.push_back(fullPath.toStdString());
        }
    }

    if (imagePaths.empty()) return;

    // Show preview dialog
    ImagePreviewDialog preview(this);
    preview.SetImageList(imagePaths);
    preview.ShowImage(0);
    preview.exec();
}

void ImageEditorDialog::OnCloseClicked()
{
    close();
}

void ImageEditorDialog::OnTableSelectionChanged()
{
    QList<int> selectedRows;
    for (int row = 0; row < m_imageTable->rowCount(); ++row) {
        if (m_imageTable->item(row, 0)->isSelected()) {
            selectedRows.append(row);
        }
    }
    m_previewButton->setEnabled(!selectedRows.isEmpty());

    // Update GNSS details for the first selected image
    if (!selectedRows.isEmpty()) {
        m_selectedImageIndex = selectedRows[0];
        if (m_currentGroup && m_selectedImageIndex >= 0 && m_selectedImageIndex < static_cast<int>(m_currentGroup->images.size())) {
            UpdateGNSSDetailsPanel();
        }
    } else {
        m_selectedImageIndex = -1;
        DisplayGNSSDetails(nullptr);
    }
}

void ImageEditorDialog::OnShowGNSSDataToggled()
{
    m_gnssDataVisible = m_showGNSSDataButton->isChecked();
    m_gnssDetailsGroup->setVisible(m_gnssDataVisible);
}

void ImageEditorDialog::OnImportGNSSClicked()
{
    if (!m_projectDocument || !m_currentGroup) {
        QMessageBox::warning(this, "Error", "No image group loaded");
        return;
    }

    // Check if there are any images in the group
    if (m_currentGroup->images.empty()) {
        QMessageBox::warning(this, "No Images", "Please add images to the group before importing GNSS data");
        return;
    }

    // First, let user select a GNSS file
    QString gnssFilePath = QFileDialog::getOpenFileName(this,
        "Select GNSS Data File",
        "",
        "Text Files (*.txt *.csv);;All Files (*)");

    if (gnssFilePath.isEmpty()) {
        return;  // User cancelled
    }

    // Create and configure GNSS import dialog
    insight::ui::widgets::GNSSMeasurementImportDialog gnssDialog(this);
    gnssDialog.setWindowTitle("Import GNSS Measurements for: " + QString::fromStdString(m_currentGroup->group_name));

    // Set the GNSS file
    gnssDialog.setFile(gnssFilePath);

    // Default settings: Geographic coordinates (Lat/Lon) with uniform covariance
    gnssDialog.setCoordinateType(true);  // true = geographic (Lat/Lon), false = projected (X/Y/Z)
    gnssDialog.setUseUniformCovariance(true);
    gnssDialog.setUniformCovariance(1.0, 2.0);  // σ_xy=1m, σ_z=2m
    gnssDialog.setImportRotation(false);  // Initially disabled

    // Show the dialog modally
    if (gnssDialog.exec() == QDialog::Accepted) {
        try {
            // Get the GNSS measurements from the dialog
            std::vector<database::Measurement::GNSSMeasurement> gnssData = gnssDialog.getGNSSMeasurements();

            if (gnssData.empty()) {
                QMessageBox::warning(this, "No Data", "No GNSS data was extracted from the file");
                return;
            }

            // Check if data count matches image count
            if (gnssData.size() != m_currentGroup->images.size()) {
                QString msg = QString("Warning: Data count (%1) does not match image count (%2).\n\n")
                    .arg(gnssData.size())
                    .arg(m_currentGroup->images.size());
                msg += "Only the first " + QString::number(std::min(gnssData.size(), m_currentGroup->images.size())) + " images will be updated.";
                QMessageBox::warning(this, "Count Mismatch", msg);
            }

            // Apply GNSS data to images in the group
            m_projectDocument->applyGNSSToImages(gnssData, m_currentGroup->group_id);

            // Refresh the table to show GNSS status
            RefreshTable();
            UpdateGNSSDetailsPanel();

            // Show success message
            QMessageBox::information(this, "Success",
                QString("Successfully imported GNSS data for %1 image(s)").arg(gnssData.size()));

        } catch (const std::exception& e) {
            QMessageBox::critical(this, "Error", QString("Failed to import GNSS data: %1").arg(e.what()));
        }
    }
}

void ImageEditorDialog::OnClearGNSSClicked()
{
    if (!m_currentGroup || m_selectedImageIndex < 0 || m_selectedImageIndex >= static_cast<int>(m_currentGroup->images.size())) {
        QMessageBox::warning(this, "Error", "No image selected");
        return;
    }

    auto& image = m_currentGroup->images[m_selectedImageIndex];
    if (!image.gnss_data.has_value()) {
        QMessageBox::information(this, "Info", "Selected image has no GNSS data");
        return;
    }

    image.gnss_data.reset();
    UpdateGNSSDetailsPanel();

    if (m_projectDocument) {
        m_projectDocument->saveProject();
        emit imagesChanged(m_currentGroup->group_id);
    }

    QMessageBox::information(this, "Success", "GNSS data cleared");
}

void ImageEditorDialog::UpdateGNSSDetailsPanel()
{
    if (!m_currentGroup || m_selectedImageIndex < 0 || m_selectedImageIndex >= static_cast<int>(m_currentGroup->images.size())) {
        DisplayGNSSDetails(nullptr);
        return;
    }

    DisplayGNSSDetails(&m_currentGroup->images[m_selectedImageIndex]);
}

void ImageEditorDialog::DisplayGNSSDetails(const database::Image* image)
{
    if (!image) {
        m_gnssImageIdValue->setText("None");
        m_gnssXValue->setText("X: --");
        m_gnssYValue->setText("Y: --");
        m_gnssZValue->setText("Z: --");
        m_gnssCovXXValue->setText("σ_xx: --");
        m_gnssCovYYValue->setText("σ_yy: --");
        m_gnssCovZZValue->setText("σ_zz: --");
        m_gnssStatusLabel->setText("Status: No data");
        m_gnssStatusLabel->setStyleSheet("color: #888;");
        m_clearGNSSButton->setEnabled(false);
        return;
    }

    m_gnssImageIdValue->setText(QString("IMG_%1").arg(image->image_id));

    if (image->gnss_data.has_value()) {
        const auto& gnss = image->gnss_data.value();
        m_gnssXValue->setText(QString("X: %1 m").arg(gnss.x, 0, 'f', 2));
        m_gnssYValue->setText(QString("Y: %1 m").arg(gnss.y, 0, 'f', 2));
        m_gnssZValue->setText(QString("Z: %1 m").arg(gnss.z, 0, 'f', 2));

        m_gnssCovXXValue->setText(QString("σ_xx: %1 m²").arg(gnss.cov_xx, 0, 'f', 6));
        m_gnssCovYYValue->setText(QString("σ_yy: %1 m²").arg(gnss.cov_yy, 0, 'f', 6));
        m_gnssCovZZValue->setText(QString("σ_zz: %1 m²").arg(gnss.cov_zz, 0, 'f', 6));

        m_gnssStatusLabel->setText("Status: Valid GNSS data");
        m_gnssStatusLabel->setStyleSheet("color: #00aa00; font-weight: bold;");
        m_clearGNSSButton->setEnabled(true);
    } else {
        m_gnssXValue->setText("X: --");
        m_gnssYValue->setText("Y: --");
        m_gnssZValue->setText("Z: --");
        m_gnssCovXXValue->setText("σ_xx: --");
        m_gnssCovYYValue->setText("σ_yy: --");
        m_gnssCovZZValue->setText("σ_zz: --");
        m_gnssStatusLabel->setText("Status: No GNSS data");
        m_gnssStatusLabel->setStyleSheet("color: #888;");
        m_clearGNSSButton->setEnabled(false);
    }
}

void ImageEditorDialog::OnGNSSDataChanged()
{
    UpdateGNSSDetailsPanel();
}

std::vector<uint32_t> ImageEditorDialog::GetModifiedImages() const
{
    return m_modifiedImages;
}

void ImageEditorDialog::RefreshTable()
{
    PopulateTable();
}

void ImageEditorDialog::AddImageToGroup(const std::string& fullPath)
{
    if (!m_projectDocument || !m_currentGroup) return;

    // Generate new image ID
    uint32_t newImageId = m_projectDocument->generateImageId();

    // Create image record
    database::Image newImage;
    newImage.image_id = newImageId;
    newImage.filename = fullPath;

    // Add to group
    m_currentGroup->images.push_back(newImage);
    m_modifiedImages.push_back(newImageId);
}

void ImageEditorDialog::DeleteSelectedRows()
{
    // Collect rows to delete (in reverse order to maintain indices)
    QList<int> rowsToDelete;
    QSet<int> selectedRowSet;
    
    for (int row = 0; row < m_imageTable->rowCount(); ++row) {
        if (m_imageTable->item(row, 0)->isSelected()) {
            selectedRowSet.insert(row);
        }
    }

    rowsToDelete = selectedRowSet.values();
    std::sort(rowsToDelete.rbegin(), rowsToDelete.rend()); // Sort descending

    // Delete rows and corresponding images
    for (int row : rowsToDelete) {
        uint32_t imageId = m_imageTable->item(row, COLUMN_ID)->text().toUInt();
        m_imageTable->removeRow(row);

        if (m_currentGroup) {
            auto it = m_currentGroup->images.begin();
            while (it != m_currentGroup->images.end()) {
                if (it->image_id == imageId) {
                    it = m_currentGroup->images.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    if (m_projectDocument) {
        m_projectDocument->saveProject();
    }
    
    emit imagesChanged(m_currentGroup->group_id);
}

void ImageEditorDialog::OnEditGNSSClicked()
{
    if (!m_currentGroup || m_selectedImageIndex < 0 || m_selectedImageIndex >= static_cast<int>(m_currentGroup->images.size())) {
        QMessageBox::warning(this, "Error", "No image selected");
        return;
    }

    auto& image = m_currentGroup->images[m_selectedImageIndex];
    if (!image.gnss_data.has_value()) {
        QMessageBox::information(this, "Info", "Selected image has no GNSS data to edit");
        return;
    }

    auto& gnss = image.gnss_data.value();

    // Create edit dialog
    QDialog editDialog(this);
    editDialog.setWindowTitle(QString("Edit GNSS Data - Image %1").arg(image.image_id));
    editDialog.setMinimumWidth(350);

    QVBoxLayout* layout = new QVBoxLayout(&editDialog);

    // Coordinate fields
    QGroupBox* coordGroup = new QGroupBox("Coordinates", &editDialog);
    QVBoxLayout* coordLayout = new QVBoxLayout(coordGroup);

    QHBoxLayout* xLayout = new QHBoxLayout();
    QLabel* xLabel = new QLabel("X (m):", coordGroup);
    QLineEdit* xEdit = new QLineEdit(QString::number(gnss.x, 'f', 6), coordGroup);
    xLayout->addWidget(xLabel);
    xLayout->addWidget(xEdit);
    coordLayout->addLayout(xLayout);

    QHBoxLayout* yLayout = new QHBoxLayout();
    QLabel* yLabel = new QLabel("Y (m):", coordGroup);
    QLineEdit* yEdit = new QLineEdit(QString::number(gnss.y, 'f', 6), coordGroup);
    yLayout->addWidget(yLabel);
    yLayout->addWidget(yEdit);
    coordLayout->addLayout(yLayout);

    QHBoxLayout* zLayout = new QHBoxLayout();
    QLabel* zLabel = new QLabel("Z (m):", coordGroup);
    QLineEdit* zEdit = new QLineEdit(QString::number(gnss.z, 'f', 6), coordGroup);
    zLayout->addWidget(zLabel);
    zLayout->addWidget(zEdit);
    coordLayout->addLayout(zLayout);

    layout->addWidget(coordGroup);

    // Covariance fields
    QGroupBox* covGroup = new QGroupBox("Covariance (m²)", &editDialog);
    QVBoxLayout* covLayout = new QVBoxLayout(covGroup);

    QHBoxLayout* covXXLayout = new QHBoxLayout();
    QLabel* covXXLabel = new QLabel("σ_xx:", covGroup);
    QLineEdit* covXXEdit = new QLineEdit(QString::number(gnss.cov_xx, 'f', 6), covGroup);
    covXXLayout->addWidget(covXXLabel);
    covXXLayout->addWidget(covXXEdit);
    covLayout->addLayout(covXXLayout);

    QHBoxLayout* covYYLayout = new QHBoxLayout();
    QLabel* covYYLabel = new QLabel("σ_yy:", covGroup);
    QLineEdit* covYYEdit = new QLineEdit(QString::number(gnss.cov_yy, 'f', 6), covGroup);
    covYYLayout->addWidget(covYYLabel);
    covYYLayout->addWidget(covYYEdit);
    covLayout->addLayout(covYYLayout);

    QHBoxLayout* covZZLayout = new QHBoxLayout();
    QLabel* covZZLabel = new QLabel("σ_zz:", covGroup);
    QLineEdit* covZZEdit = new QLineEdit(QString::number(gnss.cov_zz, 'f', 6), covGroup);
    covZZLayout->addWidget(covZZLabel);
    covZZLayout->addWidget(covZZEdit);
    covLayout->addLayout(covZZLayout);

    layout->addWidget(covGroup);
    layout->addStretch();

    // Buttons
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    QPushButton* okButton = new QPushButton("OK", &editDialog);
    QPushButton* cancelButton = new QPushButton("Cancel", &editDialog);
    buttonLayout->addStretch();
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);
    layout->addLayout(buttonLayout);

    connect(okButton, &QPushButton::clicked, &editDialog, &QDialog::accept);
    connect(cancelButton, &QPushButton::clicked, &editDialog, &QDialog::reject);

    if (editDialog.exec() == QDialog::Accepted) {
        try {
            gnss.x = xEdit->text().toDouble();
            gnss.y = yEdit->text().toDouble();
            gnss.z = zEdit->text().toDouble();
            gnss.cov_xx = covXXEdit->text().toDouble();
            gnss.cov_yy = covYYEdit->text().toDouble();
            gnss.cov_zz = covZZEdit->text().toDouble();

            if (m_projectDocument) {
                m_projectDocument->saveProject();
                emit imagesChanged(m_currentGroup->group_id);
            }

            UpdateGNSSDetailsPanel();
            QMessageBox::information(this, "Success", "GNSS data updated successfully");
        } catch (const std::exception& e) {
            QMessageBox::critical(this, "Error", QString("Failed to update GNSS data: %1").arg(e.what()));
        }
    }
}

void ImageEditorDialog::OnSetAllCovarianceClicked()
{
    if (!m_currentGroup || m_currentGroup->images.empty()) {
        QMessageBox::warning(this, "Error", "No images in group");
        return;
    }

    // Create dialog to input uniform covariance
    QDialog covDialog(this);
    covDialog.setWindowTitle("Set Covariance for All Images");
    covDialog.setMinimumWidth(350);

    QVBoxLayout* layout = new QVBoxLayout(&covDialog);
    QLabel* infoLabel = new QLabel(
        QString("Apply uniform covariance to %1 images in this group").arg(m_currentGroup->images.size()),
        &covDialog);
    layout->addWidget(infoLabel);

    QGroupBox* covGroup = new QGroupBox("Covariance Values (m²)", &covDialog);
    QVBoxLayout* covLayout = new QVBoxLayout(covGroup);

    QHBoxLayout* covXXLayout = new QHBoxLayout();
    QLabel* covXXLabel = new QLabel("σ_xx (XY variance):", covGroup);
    QLineEdit* covXXEdit = new QLineEdit("1.0", covGroup);
    covXXEdit->setValidator(new QDoubleValidator(0.0, 10000.0, 6, covXXEdit));
    covXXLayout->addWidget(covXXLabel);
    covXXLayout->addWidget(covXXEdit);
    covLayout->addLayout(covXXLayout);

    QHBoxLayout* covZZLayout = new QHBoxLayout();
    QLabel* covZZLabel = new QLabel("σ_zz (Z variance):", covGroup);
    QLineEdit* covZZEdit = new QLineEdit("4.0", covGroup);
    covZZEdit->setValidator(new QDoubleValidator(0.0, 10000.0, 6, covZZEdit));
    covZZLayout->addWidget(covZZLabel);
    covZZLayout->addWidget(covZZEdit);
    covLayout->addLayout(covZZLayout);

    layout->addWidget(covGroup);
    layout->addStretch();

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    QPushButton* applyButton = new QPushButton("Apply", &covDialog);
    QPushButton* cancelButton = new QPushButton("Cancel", &covDialog);
    buttonLayout->addStretch();
    buttonLayout->addWidget(applyButton);
    buttonLayout->addWidget(cancelButton);
    layout->addLayout(buttonLayout);

    connect(applyButton, &QPushButton::clicked, &covDialog, &QDialog::accept);
    connect(cancelButton, &QPushButton::clicked, &covDialog, &QDialog::reject);

    if (covDialog.exec() == QDialog::Accepted) {
        try {
            double covXX = covXXEdit->text().toDouble();
            double covZZ = covZZEdit->text().toDouble();

            int updatedCount = 0;
            for (auto& image : m_currentGroup->images) {
                if (image.gnss_data.has_value()) {
                    auto& gnss = image.gnss_data.value();
                    gnss.cov_xx = covXX;
                    gnss.cov_yy = covXX;
                    gnss.cov_zz = covZZ;
                    gnss.cov_xy = 0.0;
                    gnss.cov_xz = 0.0;
                    gnss.cov_yz = 0.0;
                    updatedCount++;
                }
            }

            if (updatedCount > 0) {
                if (m_projectDocument) {
                    m_projectDocument->saveProject();
                    emit imagesChanged(m_currentGroup->group_id);
                }
                UpdateGNSSDetailsPanel();
                QMessageBox::information(this, "Success",
                    QString("Updated covariance for %1 image(s)").arg(updatedCount));
            } else {
                QMessageBox::information(this, "Info", "No images with GNSS data to update");
            }
        } catch (const std::exception& e) {
            QMessageBox::critical(this, "Error", QString("Failed to update covariance: %1").arg(e.what()));
        }
    }
}

} // namespace ui
} // namespace insight
