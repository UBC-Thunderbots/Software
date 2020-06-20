#include "software/gui/simulator/widgets/simulator_gui.h"

#include "software/gui/drawing/ssl_wrapper_packet.h"
#include "software/gui/geometry_conversion.h"

SimulatorGUI::SimulatorGUI(
    std::shared_ptr<ThreadSafeBuffer<SSL_WrapperPacket>> ssl_wrapper_packet_buffer,
    std::shared_ptr<ThreadSafeBuffer<Rectangle>> view_area_buffer)
    : QMainWindow(),
      main_widget(new MainWidget(this)),
      update_timer(new QTimer(this)),
      ssl_wrapper_packet_buffer(ssl_wrapper_packet_buffer),
      view_area_buffer(view_area_buffer)
{
    setCentralWidget(main_widget);
    connect(update_timer, &QTimer::timeout, this, &SimulatorGUI::drawSimulatorContents);
    update_timer->start(static_cast<int>(
        Duration::fromSeconds(DRAW_UPDATE_INTERVAL_SECONDS).getMilliseconds()));
}

void SimulatorGUI::drawSimulatorContents()
{
    auto ssl_wrapper_packet = ssl_wrapper_packet_buffer->popMostRecentlyAddedValue();
    if (ssl_wrapper_packet)
    {
        auto draw_function = getDrawSSLWrapperPacketFunction(*ssl_wrapper_packet);
        main_widget->draw({draw_function});
    }

    updateDrawViewArea();
}

void SimulatorGUI::updateDrawViewArea()
{
    std::optional<Rectangle> view_area = view_area_buffer->popLeastRecentlyAddedValue();
    if (view_area)
    {
        main_widget->setDrawViewArea(createQRectF(view_area.value()));
    }
}
