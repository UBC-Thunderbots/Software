#include "decimal_progress_bar.h"

DecimalProgressBar::DecimalProgressBar(QWidget *parent) : QProgressBar(parent) {}

void DecimalProgressBar::setupProgressBar(int max, QString format)
{
    format_ = format;
    updateProgressBar(0.0f);
    setMaximum(max);
}

void DecimalProgressBar::updateProgressBar(float value)
{
    setFormat(format_.arg(value, 0, 'f', 2));
    // we don't use this value, it's just for visuals
    setValue(static_cast<int>(value));
}
