#pragma once

#include "software/backend/backend.h"
#include "software/backend/output/wifi/wifi_output.h"
#include "software/parameter/dynamic_parameters.h"

class HandheldControllerBackend : public Backend
{
   public:
    static const std::string name;

    HandheldControllerBackend();

    /**
     * Creates a new PrimitiveGenerator
     *
     * @param controller_input_config The config for the HandheldControllerBackend
     */
    HandheldControllerBackend(
        std::shared_ptr<const HandheldControllerInputConfig> controller_input_config);

   private:
    static constexpr int DEFAULT_RADIO_CONFIG = 0;

    void onValueReceived(ConstPrimitiveVectorPtr primitives) override;

    // The interface that lets us send primitives to the robots over wifi
    WifiOutput wifi_output;
    std::shared_ptr<const HandheldControllerInputConfig> controller_input_config;
};
