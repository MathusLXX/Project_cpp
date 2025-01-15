#ifndef DRAWINGAREA_H
#define DRAWINGAREA_H

#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QTimer>
#include "context.h"

class DrawingArea : public QOpenGLWidget {
    Q_OBJECT

public:
    explicit DrawingArea(QWidget* parent = nullptr);
    ~DrawingArea() override = default;

protected:
    void paintEvent(QPaintEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void animate();

private:
    Context context;
    QTimer* timer;
};

#endif
