import re


def to_pascal_case(snake_str):
    return "".join(x.title() for x in snake_str.split("_"))


def to_camel_case(snake_str):
    upper_camel_case = to_pascal_case(snake_str)
    return upper_camel_case[0].lower() + upper_camel_case[1:]


def to_snake_case(camel_str):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", camel_str).lower()
