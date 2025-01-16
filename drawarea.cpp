#include "drawarea.h"
#include <QMouseEvent>
#include <QPainter>

DrawingArea::DrawingArea(QWidget* parent)
    : QOpenGLWidget(parent), timer(new QTimer(this)) {
    connect(timer, &QTimer::timeout, this, &DrawingArea::animate);
    timer->start(16);
}

void DrawingArea::mouseReleaseEvent(QMouseEvent *event) {
    context.addParticle({float(event->x()), float(event->y())}, {0.0f, 0.0f}, 20.0f, 100000000.0F);
    update();
}

void DrawingArea::animate() {
    context.updatePhysicalSystem(0.1f); // update tous les 100 ms
    update();
}

void DrawingArea::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    context.display(&painter, event);
}
