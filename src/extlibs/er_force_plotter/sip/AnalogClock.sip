%Import QtGui/QtGuimod.sip
%Import QtWidgets/QtWidgetsmod.sip

%If (Qt_5_0_0 -)

class AnalogClock : public QWidget{

%TypeHeaderCode
#include "analogclock.h"
%End

public:
    AnalogClock(QWidget *parent /TransferThis/ = 0);

protected:
    virtual void resizeEvent(QResizeEvent *);
    virtual void paintEvent(QPaintEvent *e); 
};

%End