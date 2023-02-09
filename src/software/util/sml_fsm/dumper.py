import re
import os

OUTPUT_FILEPATH = "docs/fsm-diagrams.md"

def generateDiagram(fsm):
    """
    Generates a mermaid.js text definition for a state diagram 
    representing the FSM.

    Details on mermaid.js UML syntax:
    https://mermaid.js.org/syntax/stateDiagram.html

    :param fsm: the FSM code containing its transition table
    :returns: the mermaid.js text definition for FSM diagram
    """

    # Regex match to extract transition table from the FSM
    transition_table_match = re.search(r'make_transition_table\(([\s\S]*?)\);', fsm);
    if not transition_table_match: 
        return
    transition_table = transition_table_match.group(1)

    # Remove comment lines
    transition_table = re.sub(r'//.*$', '', transition_table, flags=re.MULTILINE)
    
    # Remove all whitespace
    transition_table = re.sub(r'\s+', '', transition_table, flags=re.MULTILINE)

    transitions = transition_table.split(',')

    diagram_uml = ''

    for transition in transitions:

        # Transitions have the following format:
        # src_state + event [guard] / action = dest_state

        # Extract dest_state from transition, which is after '=' sign
        transition, *rest = transition.split('=')
        dest_state = rest[0] if rest else ''

        # Extract action from transition, which is after '/' sign
        transition, *rest = transition.split('/')
        action = rest[0] if rest else ''

        # Extract guard contained between square brackets
        guard = next(iter(re.findall(r'\[(.*?)\]', transition)), '')

        # Extract src_state from transition, which is before '+' sign
        src_state = transition.split('+')[0]

        # Remove suffixes from states, guard, action
        def remove_suffix(str, suffix):
            return str[:-len(suffix)] if str.endswith(suffix) else str
        src_state = remove_suffix(src_state, '_S')
        dest_state = remove_suffix(dest_state, '_S')
        guard = remove_suffix(guard, '_G')
        action = remove_suffix(action, '_A')
        
        if dest_state == 'X':
            dest_state = '[*]'

        if src_state.startswith('*'):
            src_state = src_state[1:]
            diagram_uml += f"[*] --> {src_state}\n"
        
        if not dest_state:
            dest_state = src_state

        if src_state == 'X':
            src_state = 'Terminate:::terminate'

        diagram_uml += f"{src_state} --> {dest_state}"

        if guard:
            diagram_uml += f" : [{guard}]"

            if action:
                diagram_uml += f"\\n<i>{action}</i>"
        
        elif action:
            diagram_uml += f" : <i>{action}</i>"
        
        diagram_uml += "\n"

    return (
        "stateDiagram-v2\n" +
        "classDef terminate fill:white,color:black,font-weight:bold\n" + 
        "direction LR\n" +
        diagram_uml
    )

if __name__ == "__main__":

    with open(OUTPUT_FILEPATH, "w") as output_file:

        print("# Play and Tactic FSM Diagrams\n", file=output_file)
        
        diagrams_generated = 0
        for root, dirs, files in os.walk("src/software/ai"):
            for file in files:
                if not file.endswith("fsm.h"):
                    continue

                filepath = os.path.join(root, file)
                with open(filepath, "r") as f:
                    fsm = f.read()

                # Regex match to extract FSM struct name
                fsm_name = re.findall(r'struct\s+(\w+)\s*\{', fsm)[0]

                diagram = generateDiagram(fsm)
                
                # Format diagram in Markdown syntax and write to output file
                if diagram:
                    print(f"## [{fsm_name}](/{filepath})\n", file=output_file)
                    print(f"```mermaid\n\n{diagram}\n```\n", file=output_file)
                    diagrams_generated += 1
    
        # Print out confirmation message to console
        print(
            f"Generated {diagrams_generated} FSM diagrams. " +
            f"View output in {OUTPUT_FILEPATH}"
        )
                    