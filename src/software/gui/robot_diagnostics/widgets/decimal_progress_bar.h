#pragma once

#include <QtWidgets/QProgressBar>
#include <QtWidgets/QWidget>

/**
 * A custom QProgressBar designed to display values up to 2 decimal points. This class
 * manages the displayed value and format of the progress bar
 */
class DecimalProgressBar : public QProgressBar
{
    Q_OBJECT
   public:
    /**
     * Creates a new DecimalProgressBar
     *
     * @param parent A pointer to the parent widget for this DecimalProgressBar
     */
    explicit DecimalProgressBar(QWidget *parent = nullptr);

    /**
     * Setups the DecimalProgressBar widget. Sets the displayed value to 0
     *
     * @param max The maximum allowed value
     * @param format The format to display values with
     */
    void setupProgressBar(int max, QString format);

    /**
     * Updates the value displayed on the progress bar
     *
     * @param value The value to display
     */
    void updateProgressBar(float value);

   private:
    QString format_;
};
