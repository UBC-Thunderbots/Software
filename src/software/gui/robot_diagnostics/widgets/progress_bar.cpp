#include "progress_bar.h"

void updateProgressBar(QProgressBar *progress_bar, float value)
{
    progress_bar->setFormat(QString(progress_bar->format()).arg(value, 0, 'f', 2));
    // we don't use this value, it's just for visuals
    progress_bar->setValue(static_cast<int>(value));
}
