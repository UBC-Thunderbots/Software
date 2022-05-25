#include "software/gui/generic_widgets/robot_status/robot_status.h"

#include <QtCore/QList>
#include <QtWidgets/QHeaderView>

void setupRobotStatusTable(QTableWidget* table)
{
    QList<QString> horizontal_header_labels({"Age (Seconds)", "Message"});
    // The column count must be set before the labels are set
    table->setColumnCount(horizontal_header_labels.size());
    table->setHorizontalHeaderLabels(horizontal_header_labels);
    table->horizontalHeader()->setVisible(true);
    table->horizontalHeader()->setStretchLastSection(true);

    table->verticalHeader()->setVisible(false);
    table->setShowGrid(true);
    table->setVisible(true);

    table->update();
}
