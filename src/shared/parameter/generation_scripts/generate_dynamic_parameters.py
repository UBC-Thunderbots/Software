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
from c_writer import CWriter

# Path relative to the bazel WORKSPACE root
# This path is included in the data for the py_binary bazel target
PARAMETER_CONFIG_PATH = Path(os.path.dirname(__file__), "../config_definitions")
PARAMETER_TEST_CONFIG_PATH = Path(
    os.path.dirname(__file__), "../config_definitions/test"
)


def generate_dynamic_parameters(output_header, output_source, include_headers, generate_for_cpp):
    if generate_for_cpp:

        yamls = list(PARAMETER_CONFIG_PATH.glob("*.yaml"))
        config_metadata = ConfigYamlLoader.get_config_metadata(yamls)

        CppWriter.write_config_metadata(
            output_header, include_headers, "ThunderbotsConfig", config_metadata
        )
        CppWriter.write_config_metadata(
            output_source, include_headers, "ThunderbotsConfig", config_metadata
        )
    else:
        # Genernate this properly as part of https://github.com/UBC-Thunderbots/Software/issues/1731
        test_yamls = list(PARAMETER_TEST_CONFIG_PATH.glob("*.yaml"))
        test_config_metadata = ConfigYamlLoader.get_config_metadata(test_yamls)
        CWriter.write_config_metadata(
            output_header, "ThunderbotsConfig", test_config_metadata
        )
        CWriter.write_config_metadata(
            output_source, "ThunderbotsConfig", test_config_metadata
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
    output_type = parser.add_mutually_exclusive_group(required=True)
    output_type.add_argument(
        "-c",
        dest="generate_for_cpp",
        action="store_false",
        help="Generate code that is compatible with C",
    )
    output_type.add_argument(
        "-cpp",
        dest="generate_for_cpp",
        action="store_true",
        help="Generate code that is compatible with C++ (not compatible with C)",
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
    generate_dynamic_parameters(
        args.output_header, args.output_source, args.include_headers, args.generate_for_cpp
    )


#######################################################################
#                                Main                                 #
#######################################################################

if __name__ == "__main__":
    main()
