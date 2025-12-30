from typing import Any
import click
import typer
import typer.core
import typer.main
from typer.models import OptionInfo


# ======= Patch to support nargs functionality in typer =======
# nargs usage is considered bad practice in typer and many CLIs,
# but since we likely want to be able to parse many optional args at once, this patch allows us to do so.
# See https://github.com/fastapi/typer/issues/110
# ==============================================================

class OptionEatAll(typer.core.TyperOption):
    """Click option that consumes arguments until next option flag."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        kwargs["multiple"] = True
        kwargs["is_flag"] = False
        super().__init__(*args, **kwargs)
        self._previous_parser_process = None
        self._eat_all_parser = None

    def add_to_parser(self, parser: Any, ctx: click.Context) -> Any:
        def parser_process(value: str, state: Any) -> None:
            values = [value]
            done = False
            while state.rargs and not done:
                for prefix in self._eat_all_parser.prefixes:
                    if state.rargs[0].startswith(prefix):
                        done = True
                        break
                if not done:
                    values.append(state.rargs.pop(0))
            for val in values:
                self._previous_parser_process(val, state)

        retval = super().add_to_parser(parser, ctx)
        for name in self.opts:
            our_parser = parser._long_opt.get(name) or parser._short_opt.get(name)
            if our_parser:
                self._eat_all_parser = our_parser
                self._previous_parser_process = our_parser.process
                our_parser.process = parser_process
                break
        return retval


class MultiOptionInfo(OptionInfo):
    """Marker for multi-argument options."""
    is_multi_option: bool = True

    def __init__(self, option_info: OptionInfo) -> None:
        super().__init__(**option_info.__dict__)


def MultiOption(default: Any = ..., *param_decls: str, **kwargs: Any) -> MultiOptionInfo:
    """Create option that accepts multiple values."""
    return MultiOptionInfo(typer.Option(default, *param_decls, **kwargs))


# Monkey-patch to detect MultiOptionInfo and use OptionEatAll
_original_get_click_param = typer.main.get_click_param


def _patched_get_click_param(param: typer.models.ParamMeta):
    click_param, converter = _original_get_click_param(param)
    if isinstance(param.default, MultiOptionInfo):
        option_eat_all = object.__new__(OptionEatAll)
        option_eat_all.__dict__.update(click_param.__dict__)
        option_eat_all._previous_parser_process = None
        option_eat_all._eat_all_parser = None
        return (option_eat_all, converter)
    return (click_param, converter)


typer.main.get_click_param = _patched_get_click_param
