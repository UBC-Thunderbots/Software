#pragma once

#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QKeyEvent>
#include <QWheelEvent>

class MySceneView : public QGraphicsView
{
    Q_OBJECT
   public:
    explicit MySceneView(QWidget *parent = 0);

   protected slots:

    void wheelEvent(QWheelEvent *event);

    void keyPressEvent(QKeyEvent *event);
};
