#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QWheelEvent>
#include <QKeyEvent>

class SceneView : public QGraphicsView {
Q_OBJECT
public:
    explicit SceneView(QWidget *parent = 0);

protected slots:

    void wheelEvent(QWheelEvent *event);

    void keyPressEvent(QKeyEvent *event);
};
