#!/opt/tbotspython/bin/python3.8

import re
import os
from pathlib import Path

OUTPUT_FILE_PATH = "docs/fsm-diagrams.md"


def generate_diagram(fsm):
    """
    Generates a Mermaid text definition for a state diagram 
    representing the FSM.

    Details on Mermaid UML syntax:
    https://mermaid.js.org/syntax/stateDiagram.html

    :param fsm: the FSM code containing its transition table
    :returns: the mermaid.js text definition for FSM diagram
    """

    # Regex match to extract transition table from the FSM
    transition_table_match = re.search(r"make_transition_table\(([\s\S]*?)\);", fsm)
    if not transition_table_match:
        return
    transition_table = transition_table_match.group(1)

    # Remove comment lines
    transition_table = re.sub(r"//.*$", "", transition_table, flags=re.MULTILINE)

    # Remove all whitespace
    transition_table = re.sub(r"\s+", "", transition_table, flags=re.MULTILINE)

    diagram = ""

    transitions = transition_table.split(",")
    for transition in transitions:

        # Transitions have the following format:
        # src_state + event [guard] / action = dest_state

        # Extract dest_state from transition, which is after '=' sign
        transition, *rest = transition.split("=")
        dest_state = rest[0] if rest else ""

        # Extract action from transition, which is after '/' sign
        transition, *rest = transition.split("/")
        action = rest[0] if rest else ""

        # Extract guard contained between square brackets
        guard = next(iter(re.findall(r"\[(.*?)\]", transition)), "")

        # Extract src_state from transition, which is before '+' sign
        src_state = transition.split("+")[0]

        def remove_suffix(str, suffix):
            return str[: -len(suffix)] if str.endswith(suffix) else str

        # Remove suffixes from states, guard, action
        src_state = remove_suffix(src_state, "_S")
        dest_state = remove_suffix(dest_state, "_S")
        guard = " && ".join([remove_suffix(g, "_G") for g in guard.split("&&")])
        action = remove_suffix(action, "_A")

        # Terminate state is marked with X in transition table.
        # Give terminate state custom styling with the
        # :::terminate classDef style
        if dest_state == "X":
            dest_state = "Terminate:::terminate"
        if src_state == "X":
            src_state = "Terminate:::terminate"

        # Initial state is marked with '*' prefix, so need to add
        # special transition from start to initial state in our diagram
        if src_state.startswith("*"):
            src_state = src_state[1:]
            diagram += f"[*] --> {src_state}\n"

        # If dest_state is omitted, it is an internal transition
        if not dest_state:
            dest_state = src_state

        diagram += f"{src_state} --> {dest_state}"

        if guard:
            diagram += f" : [{guard}]"

            if action:
                diagram += f"\\n<i>{action}</i>"

        elif action:
            diagram += f" : <i>{action}</i>"

        diagram += "\n"

    return diagram


if __name__ == "__main__":

    root_dir = Path(os.path.abspath(__file__)).parents[3]

    fsm_diagrams = {}
    fsm_file_paths = {}

    ai_dir = os.path.join(root_dir, "src/software/ai")
    for root, dirs, files in os.walk(ai_dir):
        dirs.sort()
        for file in sorted(files):

            if not file.endswith("fsm.h"):
                continue

            # Read contents of FSM header file
            file_path = os.path.join(root, file)
            with open(file_path, "r") as f:
                fsm = f.read()

            diagram = generate_diagram(fsm)

            if diagram:

                # Regex match to extract FSM struct name
                fsm_name = re.findall(r"struct\s+(\w+)\s*\{", fsm)[0]

                # Get file path to FSM header file relative to project root
                file_path = file_path[file_path.find("src/") :]

                fsm_diagrams[fsm_name] = diagram
                fsm_file_paths[fsm_name] = file_path

    output_file_path = os.path.join(root_dir, OUTPUT_FILE_PATH)
    with open(output_file_path, "w") as output_file:

        print("# Play and Tactic FSM Diagrams\n", file=output_file)

        for fsm_name, diagram in fsm_diagrams.items():

            file_path = fsm_file_paths[fsm_name]

            # Format diagram in Markdown syntax and write to output file
            print(
                f"## [{fsm_name}](/{file_path})\n\n"
                + "```mermaid\n\n"
                + "stateDiagram-v2\n"
                + "classDef terminate fill:white,color:black,font-weight:bold\n"
                + "direction LR\n"
                + diagram
                + "\n```\n",
                file=output_file,
            )

    # Print out confirmation message to console
    print(
        f"Generated {len(fsm_diagrams)} FSM diagrams. "
        + f"View output in {OUTPUT_FILE_PATH}"
    )
