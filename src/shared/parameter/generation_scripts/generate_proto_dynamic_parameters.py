import yaml
import os
from pathlib import Path
import argparse
from config_yaml_loader import (
    ConfigYamlLoader,
    ConfigYamlException,
    ConfigYamlCycleDetected,
    ConfigYamlMalformed,
)

from proto_writer import ProtoWriter

# Path relative to the bazel WORKSPACE root
# This path is included in the data for the py_binary bazel target
PARAMETER_CONFIG_PATH = Path(os.path.dirname(__file__), "../config_definitions")


def generate_proto_dynamic_parameters(output_proto):
    yamls = list(PARAMETER_CONFIG_PATH.glob("*.yaml"))
    config_metadata = ConfigYamlLoader.get_config_metadata(yamls)

    ProtoWriter.write_config_metadata_proto(
        output_proto, "ThunderbotsConfig", config_metadata
    )


def main():
    parser = argparse.ArgumentParser(description="Generate DynamicParameters")
    parser.add_argument(
        "--output_proto",
        type=str,
        required=True,
        help="The .proto file that will be generated",
    )

    args = parser.parse_args()
    generate_proto_dynamic_parameters(args.output_proto)


#######################################################################
#                                Main                                 #
#######################################################################

if __name__ == "__main__":
    main()
