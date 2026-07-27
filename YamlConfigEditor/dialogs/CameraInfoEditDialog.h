#ifndef CAMERAINFOEDITDIALOG_H
#define CAMERAINFOEDITDIALOG_H

#include <QDialog>
#include "AllConfig.h"

class QLineEdit;
class QSpinBox;
class QCheckBox;
class QComboBox;

class CameraInfoEditDialog : public QDialog {
    Q_OBJECT
public:
    explicit CameraInfoEditDialog(const CameraInfo& info, bool isNew, QWidget* parent = nullptr);
    CameraInfo getCameraInfo() const;

private:
    CameraInfo m_info;
    bool m_isNew;

    QSpinBox* m_indexSpin;
    QLineEdit* m_descEdit;
    QComboBox* m_typeCombo;
    QLineEdit* m_urlEdit;
    QSpinBox* m_widthSpin;
    QSpinBox* m_heightSpin;
    QCheckBox* m_undistortCheck;
    QComboBox* m_modelCombo;
    QSpinBox* m_fpsSpin;
};

#endif
