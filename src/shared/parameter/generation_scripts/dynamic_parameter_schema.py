INCLUDE_KEY = "include"
PARAMETER_KEY = "parameters"
CONSTANT_KEY = "constant"
OPTIONAL_KEY = "optional"

SUPPORTED_TYPES = [
    "bool",
    "int",
    "float",
    "double",
    "string",
    "factory",
    "enum",
]

INCLUDE_DEF_SCHEMA = {
    "type": "array",
    "items": [{"type": "string"}, {"additionalProperties": False}],
    "uniqueItems": True,
    "minItems": 1,
}

PARAM_DEF_SCHEMA = {
    "type": "array",
    "uniqueItems": True,
    "minItems": 1,
    "items": [
        {
            "type": "object",
            "properties": {
                "bool": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "value": {"type": "boolean"},
                        "description": {"type": "string"},
                        CONSTANT_KEY: {"type": "boolean"},
                        OPTIONAL_KEY: {"type": "boolean"},
                    },
                    "required": ["name", "value", "description"],
                    "additionalProperties": False,
                }
            },
        },
        {
            "type": "object",
            "properties": {
                "int": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "min": {"type": "integer"},
                        "max": {"type": "integer"},
                        "value": {"type": "integer"},
                        "description": {"type": "string"},
                        CONSTANT_KEY: {"type": "boolean"},
                        OPTIONAL_KEY: {"type": "boolean"},
                    },
                    "required": ["name", "min", "max", "value", "description"],
                    "additionalProperties": False,
                }
            },
        },
        {
            "type": "object",
            "properties": {
                "float": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "min": {"type": "number"},
                        "max": {"type": "number"},
                        "value": {"type": "number"},
                        "description": {"type": "string"},
                        CONSTANT_KEY: {"type": "boolean"},
                        OPTIONAL_KEY: {"type": "boolean"},
                    },
                    "required": ["name", "min", "max", "value", "description"],
                    "additionalProperties": False,
                }
            },
        },
        {
            "type": "object",
            "properties": {
                "std::string": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "value": {"type": "string"},
                        "description": {"type": "string"},
                        CONSTANT_KEY: {"type": "boolean"},
                        OPTIONAL_KEY: {"type": "boolean"},
                    },
                    "required": ["name", "value", "description"],
                    "additionalProperties": False,
                }
            },
        },
        {
            "type": "object",
            "properties": {
                "enum": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "enum": {"type": "string"},
                        "value": {"type": "string"},
                        "description": {"type": "string"},
                        CONSTANT_KEY: {"type": "boolean"},
                        OPTIONAL_KEY: {"type": "boolean"},
                    },
                    "required": ["name", "enum", "value", "description"],
                    "additionalProperties": False,
                }
            },
        },
        {
            "type": "object",
            "properties": {
                "factory": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "index_type": {"type": "string"},
                        "type_to_create": {"type": "string"},
                        "value": {"type": "string"},
                        "description": {"type": "string"},
                        CONSTANT_KEY: {"type": "boolean"},
                        OPTIONAL_KEY: {"type": "boolean"},
                    },
                    "required": [
                        "name",
                        "index_type",
                        "type_to_create",
                        "value",
                        "description",
                    ],
                    "additionalProperties": False,
                }
            },
        },
    ],
}
