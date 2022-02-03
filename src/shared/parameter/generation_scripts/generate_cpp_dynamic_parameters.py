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

from cpp_writer import CppWriter

# Path relative to the bazel WORKSPACE root
# This path is included in the data for the py_binary bazel target
PARAMETER_CONFIG_PATH = Path(os.path.dirname(__file__), "../config_definitions")


def generate_cpp_dynamic_parameters(output_header, output_source, include_headers):
    yamls = list(PARAMETER_CONFIG_PATH.glob("*.yaml"))
    config_metadata = ConfigYamlLoader.get_config_metadata(yamls)

    CppWriter.write_config_metadata_header(
        output_header, include_headers, "ThunderbotsConfig", config_metadata
    )
    CppWriter.write_config_metadata_source(
        output_source, output_header, "ThunderbotsConfig", config_metadata
    )


def main():
    parser = argparse.ArgumentParser(description="Generate DynamicParameters")
    parser.add_argument(
        "--output_header",
        type=str,
        required=True,
        help="The header file that will be generated",
    )
    parser.add_argument(
        "--output_source",
        type=str,
        required=True,
        help="The source file that will be generated",
    )
    parser.add_argument(
        "--include_headers",
        nargs="+",
        required=False,
        help=(
            "Filepaths (relative to the bazel WORKSPACE) for any header "
            "files that should be included at the top of the generated code"
        ),
    )

    args = parser.parse_args()
    generate_cpp_dynamic_parameters(
        args.output_header, args.output_source, args.include_headers,
    )


#######################################################################
#                                Main                                 #
#######################################################################

if __name__ == "__main__":
    main()
