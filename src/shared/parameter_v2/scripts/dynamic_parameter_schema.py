DYNAMIC_PARAMETER_YAML_SCHEMA = {
    "type": "array",
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
                    },
                    "required": ["name", "value", "description"],
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
                    },
                    "required": ["name", "min", "max", "value", "description"],
                }
            },
        },
        {
            "type": "object",
            "properties": {
                "uint": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "min": {"type": "integer"},
                        "max": {"type": "integer"},
                        "value": {"type": "integer"},
                        "description": {"type": "string"},
                    },
                    "required": ["name", "min", "max", "value", "description"],
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
                    },
                    "required": ["name", "min", "max", "value", "description"],
                }
            },
        },
        {
            "type": "object",
            "properties": {
                "string": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "value": {"type": "string"},
                        "description": {"type": "string"},
                    },
                    "required": ["name", "value", "description"],
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
                    },
                    "required": ["name", "enum", "value", "description"],
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
                    },
                    "required": [
                        "name",
                        "index_type",
                        "type_to_create",
                        "value",
                        "description",
                    ],
                }
            },
        },
    ],
}
