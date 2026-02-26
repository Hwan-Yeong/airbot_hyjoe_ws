#include "sensor_gui/window/robot_model_window.hpp"
#include "ui_robot_model_window.h"

RobotModelWindow::RobotModelWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RobotModelWindow)
{
    ui->setupUi(this);
    
    // Connect Signals
    connect(ui->btn_apply, &QPushButton::clicked, this, &RobotModelWindow::onApplyClicked);
    connect(ui->btn_close, &QPushButton::clicked, this, &RobotModelWindow::onCloseClicked);
}

RobotModelWindow::~RobotModelWindow()
{
    delete ui;
}

void RobotModelWindow::setParameters(const std::map<std::string, float>& params)
{
    ui->table_params->setRowCount(params.size());
    int row = 0;
    for (const auto& [key, value] : params) {
        QTableWidgetItem* keyItem = new QTableWidgetItem(QString::fromStdString(key));
        keyItem->setFlags(keyItem->flags() & ~Qt::ItemIsEditable); // Key should be read-only
        
        QTableWidgetItem* valueItem = new QTableWidgetItem(QString::number(value, 'f', 4));
        
        std::string unit = "";
        if (key == "robot_mass" || key == "wheel_mass") unit = "kg";
        else if (key == "wheel_friction") unit = "μ";
        else if (key == "max_motor_impulse") unit = "Nm";
        else if (key == "solver_iterations") unit = "iter";
        else if (key == "damping") unit = "ratio";
        else unit = "";
        
        QTableWidgetItem* unitItem = new QTableWidgetItem(QString::fromStdString(unit));
        unitItem->setFlags(unitItem->flags() & ~Qt::ItemIsEditable); // Unit should be read-only
        
        ui->table_params->setItem(row, 0, keyItem);
        ui->table_params->setItem(row, 1, valueItem);
        ui->table_params->setItem(row, 2, unitItem);
        row++;
    }
}

std::map<std::string, float> RobotModelWindow::getParameters() const
{
    std::map<std::string, float> new_params;
    for (int row = 0; row < ui->table_params->rowCount(); ++row) {
        QTableWidgetItem* keyItem = ui->table_params->item(row, 0);
        QTableWidgetItem* valueItem = ui->table_params->item(row, 1);
        if (keyItem && valueItem) {
            std::string key = keyItem->text().toStdString();
            bool ok;
            float val = valueItem->text().toFloat(&ok);
            if (ok) {
                new_params[key] = val;
            }
        }
    }
    return new_params;
}

void RobotModelWindow::onApplyClicked()
{
    emit parametersApplied(getParameters());
}

void RobotModelWindow::onCloseClicked()
{
    close();
}
