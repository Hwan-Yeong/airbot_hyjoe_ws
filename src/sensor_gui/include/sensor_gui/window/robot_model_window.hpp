#pragma once

#include <QDialog>
#include <map>
#include <string>
#include <array>

namespace Ui {
class RobotModelWindow;
}

class RobotModelWindow : public QDialog {
    Q_OBJECT

public:
    explicit RobotModelWindow(QWidget *parent = nullptr);
    ~RobotModelWindow();

    // Initialize the table with current physics parameters
    void setParameters(const std::map<std::string, float>& params);
    
    // Get the updated parameters from the table
    std::map<std::string, float> getParameters() const;

    // Initialize the color tab with current materials
    void setColors(const std::map<std::string, std::array<float, 4>>& colors);

signals:
    // Emitted when the user clicks 'Apply'
    void parametersApplied(const std::map<std::string, float>& new_params);

    // Emitted when the user changes a color via color picker
    void colorChanged(const std::string& name, float r, float g, float b, float a);

private slots:
    void onApplyClicked();
    void onCloseClicked();

private:
    Ui::RobotModelWindow *ui;
};
