#ifndef MYLABEL_H
#define MYLABEL_H

#include <QWidget>

#include "analogclockl_global.h"

class ANALOGCLOCKSHARED_EXPORT AnalogClock : public QWidget
{
    Q_OBJECT

public:
    AnalogClock(QWidget *parent=0);
protected:
    virtual void paintEvent (QPaintEvent *event) ;
};

#endif // MYLABEL_H
