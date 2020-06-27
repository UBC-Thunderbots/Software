#include "software/backend/grsim_backend.h"

#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string GrSimBackend::name = "grsim";

GrSimBackend::GrSimBackend(std::shared_ptr<const NetworkConfig> network_config)
    : network_config(network_config),
    network_input(setupNetworkClient(network_config,
    DynamicParameters->getAIControlConfig()->getRefboxConfig(),
    DynamicParameters->getCameraConfig(), [this](World world) {this->receiveWorld(world);})),
      grsim_output(GRSIM_COMMAND_NETWORK_ADDRESS, GRSIM_COMMAND_NETWORK_PORT,
                   DynamicParameters->getAIControlConfig()->getRefboxConfig())
{
}

void GrSimBackend::onValueReceived(ConstPrimitiveVectorPtr primitives)
{
    setMostRecentlyReceivedPrimitives(primitives);
    updateGrSim();
}

void GrSimBackend::receiveWorld(World world)
{
    setMostRecentlyReceivedWorld(world);
    Subject<World>::sendValueToObservers(world);
    updateGrSim();
}

void GrSimBackend::setMostRecentlyReceivedWorld(World world)
{
    std::scoped_lock lock(most_recently_received_world_mutex);
    most_recently_received_world = world;
}

void GrSimBackend::setMostRecentlyReceivedPrimitives(ConstPrimitiveVectorPtr primitives)
{
    std::scoped_lock lock(most_recently_received_primitives_mutex);
    most_recently_received_primitives = std::move(primitives);
}

void GrSimBackend::updateGrSim()
{
    std::scoped_lock lock(most_recently_received_world_mutex,
                          most_recently_received_primitives_mutex);

    // Update GrSim if we have all the information we need
    if (most_recently_received_world && most_recently_received_primitives)
    {
        grsim_output.sendPrimitives(*most_recently_received_primitives,
                                    most_recently_received_world->friendlyTeam(),
                                    most_recently_received_world->ball());
    }
}

NetworkClient GrSimBackend::setupNetworkClient(std::shared_ptr<const NetworkConfig> network_config,
                                               std::shared_ptr<const RefboxConfig> refbox_config,
                                               std::shared_ptr<const CameraConfig> camera_config,
                                               const std::function<void(World)>& received_world_callback) {
    return NetworkClient(network_config->VisionIP()->value(),
                         network_config->VisionPort()->value(),
                         network_config->GamecontrollerIP()->value(),
                         network_config->GamecontrollerPort()->value(),
                         received_world_callback,
                         refbox_config, camera_config);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Backend, GrSimBackend> factory;
