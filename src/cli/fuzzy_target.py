import itertools
import sys
from subprocess import PIPE, run

import iterfzf
from cli.action_argument import ActionArgument
from thefuzz import process

THEFUZZ_MATCH_RATIO_THRESHOLD = 65


def fuzzy_find_target(
    action: "ActionArgument", search_query: str, interactive_search: bool
) -> str:
    """Resolve a search query to a concrete Bazel target via fuzzy matching.

    Queries Bazel for the candidate targets relevant to the action (tests,
    binaries, and/or libraries) and fuzzy-matches the search query against
    their names. If interactive search is requested, or the best match falls
    below the confidence threshold, the user picks from the top matches via an
    fzf prompt; otherwise the best match is used directly.

    :param action: the Bazel action, which determines the candidate target kinds
    :param search_query: the query to match against target names
    :param interactive_search: force the interactive fzf picker
    :return: the fully-qualified Bazel target label
    """
    bazel_query = [
        "bazel",
        "--quiet",
        "query",
        "--noshow_progress",
        "--noshow_loading_progress",
    ]  # Keep Bazel status messages out of the target picker.
    test_query = [*bazel_query, "tests(//...)"]
    binary_query = [*bazel_query, "kind(.*_binary,//...)"]
    library_query = [*bazel_query, "kind(.*_library,//...)"]

    bazel_queries = {
        ActionArgument.test: [test_query],
        ActionArgument.run: [test_query, binary_query],
        ActionArgument.build: [library_query, test_query, binary_query],
    }

    targets = list(
        itertools.chain.from_iterable(
            run(q, stdout=PIPE).stdout.rstrip(b"\n").split(b"\n")
            for q in bazel_queries[action]
        )
    )
    target_dict = {target.split(b":")[-1]: target for target in targets}
    target_names = list(target_dict.keys())

    needs_selection = interactive_search or not search_query
    if not needs_selection:
        most_similar_target_name, confidence = process.extract(
            search_query, target_names, limit=1
        )[0]
        needs_selection = confidence < THEFUZZ_MATCH_RATIO_THRESHOLD

    if needs_selection:
        selected_name = iterfzf.iterfzf(
            target_names,
            header="Search and select target"
            if interactive_search
            else "No match was strong enough, search and select target",
            prompt="Type to search > ",
            query=search_query,
        )
        if selected_name is None:
            print("Cancelled.")
            sys.exit(0)
        target = target_dict[selected_name].decode("utf-8")
    else:
        target = target_dict[most_similar_target_name].decode("utf-8")
        print(f"Found target {target} (confidence {confidence})")

    return target
