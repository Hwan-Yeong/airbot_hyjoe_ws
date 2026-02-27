#include "sensor_gui/window/robot_model_window.hpp"
#include "ui_robot_model_window.h"
#include <QColorDialog>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QRadioButton>
#include <QButtonGroup>
#include <QLabel>
#include <QHBoxLayout>

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
        if (key == "robot_mass" || key == "wheel_mass" || key == "caster_mass") unit = "kg";
        else if (key == "wheel_friction") unit = "μ";
        else if (key == "max_motor_impulse") unit = "Nm";
        else if (key == "solver_iterations") unit = "iter";
        else if (key == "damping") unit = "ratio";
        else if (key == "suspension_stiffness") unit = "N/m";
        else if (key == "suspension_damping") unit = "Ns/m";
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

void RobotModelWindow::setColors(const std::map<std::string, std::array<float, 4>>& colors)
{
    // 기존에 있던 위젯 청소 (안전하게 삭제하여 segfault 방지)
    QLayoutItem* item;
    while ((item = ui->layout_colors->takeAt(0)) != nullptr) {
        if (QWidget* w = item->widget()) {
            delete w;
        } else if (QLayout* l = item->layout()) {
            QLayoutItem* subItem;
            while ((subItem = l->takeAt(0)) != nullptr) {
                if (QWidget* sw = subItem->widget()) {
                    delete sw;
                }
                delete subItem;
            }
        }
        delete item;
    }

    QButtonGroup* colorGroup = new QButtonGroup(this);

    for (const auto& [name, rgba] : colors) {
        if (name == "body_color") continue; // body_color 매크로 자체는 목록에서 제외합니다.

        QHBoxLayout* rowLayout = new QHBoxLayout();
        QRadioButton* radioBtn = new QRadioButton(QString::fromStdString(name));
        
        // 현재 body_color가 이 색상과 동일한지 확인
        bool isCurrentColor = false;
        if (colors.count("body_color")) {
            const auto& body_rgba = colors.at("body_color");
            if (body_rgba[0] == rgba[0] && body_rgba[1] == rgba[1] &&
                body_rgba[2] == rgba[2] && body_rgba[3] == rgba[3]) {
                isCurrentColor = true;
            }
        }
        radioBtn->setChecked(isCurrentColor);
        colorGroup->addButton(radioBtn);

        // 색상 미리보기용 컬러 박스
        QLabel* colorBox = new QLabel();
        colorBox->setFixedSize(24, 24);
        QColor initColor(rgba[0] * 255, rgba[1] * 255, rgba[2] * 255, rgba[3] * 255);
        QString css = QString("background-color: rgba(%1, %2, %3, %4); border: 1px solid #777; border-radius: 4px;")
                          .arg(initColor.red()).arg(initColor.green()).arg(initColor.blue()).arg(initColor.alpha());
        colorBox->setStyleSheet(css);

        connect(radioBtn, &QRadioButton::toggled, this, [this, rgba](bool checked) {
            if (checked) {
                // 특정 색상 라디오버튼이 선택되면, 해당 값을 전역 `body_color`로 전달합니다.
                emit colorChanged("body_color", rgba[0], rgba[1], rgba[2], rgba[3]);
            }
        });

        rowLayout->addWidget(radioBtn);
        rowLayout->addWidget(colorBox);
        rowLayout->addStretch();
        ui->layout_colors->addLayout(rowLayout);
    }

    // ==== Custom Color 옵션 추가 ====
    QHBoxLayout* customRowLayout = new QHBoxLayout();
    QRadioButton* customRadioBtn = new QRadioButton("Custom Color");
    colorGroup->addButton(customRadioBtn);

    QPushButton* customColorBtn = new QPushButton();
    customColorBtn->setFixedSize(24, 24);
    
    // 초기 커스텀 색상은 body_color 혹은 흰색으로 설정
    QColor customInitColor = Qt::white;
    if (colors.count("body_color") && !colorGroup->checkedButton()) {
        const auto& bc = colors.at("body_color");
        customInitColor = QColor(bc[0]*255, bc[1]*255, bc[2]*255, bc[3]*255);
        customRadioBtn->setChecked(true); // 아무것도 안골라져있으면 Custom이 선택된 상태로 간주
    }
    
    QString customCss = QString("background-color: rgba(%1, %2, %3, %4); border: 1px solid #777; border-radius: 4px;")
                      .arg(customInitColor.red()).arg(customInitColor.green()).arg(customInitColor.blue()).arg(customInitColor.alpha());
    customColorBtn->setStyleSheet(customCss);

    connect(customColorBtn, &QPushButton::clicked, this, [this, customRadioBtn, customColorBtn]() mutable {
        QColor current_color = customColorBtn->palette().color(QPalette::Window);
        QColor c = QColorDialog::getColor(current_color, this, "Select Custom Color", QColorDialog::ShowAlphaChannel);
        if (c.isValid()) {
            QString updated_css = QString("background-color: rgba(%1, %2, %3, %4); border: 1px solid #333; border-radius: 4px;")
                                      .arg(c.red()).arg(c.green()).arg(c.blue()).arg(c.alpha());
            customColorBtn->setStyleSheet(updated_css);
            
            // 커스텀 색상 라디오버튼이 켜져있거나 켜지게 만들고 즉시 적용
            customRadioBtn->setChecked(true);
            emit colorChanged("body_color", c.redF(), c.greenF(), c.blueF(), c.alphaF());
        }
    });
    
    connect(customRadioBtn, &QRadioButton::toggled, this, [this, customColorBtn](bool checked) {
        if (checked) {
            QColor c = customColorBtn->palette().color(QPalette::Window);
            emit colorChanged("body_color", c.redF(), c.greenF(), c.blueF(), c.alphaF());
        }
    });

    customRowLayout->addWidget(customRadioBtn);
    customRowLayout->addWidget(customColorBtn);
    customRowLayout->addStretch();
    ui->layout_colors->addLayout(customRowLayout);

    ui->layout_colors->addStretch();
}

void RobotModelWindow::onApplyClicked()
{
    emit parametersApplied(getParameters());
}

void RobotModelWindow::onCloseClicked()
{
    close();
}
