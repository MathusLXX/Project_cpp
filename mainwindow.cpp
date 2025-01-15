#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "drawarea.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->draw_area = new DrawingArea(this);
    this->draw_area->setFixedSize(400, 400);
    ui->verticalLayout->addWidget(draw_area);
}

MainWindow::~MainWindow() {
    delete ui;
}

