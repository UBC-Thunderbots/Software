#include "software/gui/main_widget.h"
#include "ui_main_widget.h" // TODO: find the real path

MainWidget::MainWidget(QWidget *parent) : QWidget(parent), main_widget(new Ui::MainWidget()) {
    // Handles all the setup of the generated UI components and adds the components
    // to this widget
    main_widget->setupUi(this);
}

MainWidget::~MainWidget() {
    delete main_widget;
}
