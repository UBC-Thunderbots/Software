#pragma once

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QWheelEvent>
#include <QKeyEvent>

class MySceneView : public QGraphicsView {
Q_OBJECT
public:
    explicit MySceneView(QWidget *parent = 0);

protected slots:

    void wheelEvent(QWheelEvent *event);

    void keyPressEvent(QKeyEvent *event);
};
