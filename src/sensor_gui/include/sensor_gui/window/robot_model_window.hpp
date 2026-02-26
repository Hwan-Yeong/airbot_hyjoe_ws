#pragma once

#include <QDialog>
#include <map>
#include <string>

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

signals:
    // Emitted when the user clicks 'Apply'
    void parametersApplied(const std::map<std::string, float>& new_params);

private slots:
    void onApplyClicked();
    void onCloseClicked();

private:
    Ui::RobotModelWindow *ui;
};
