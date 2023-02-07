import re
import os

def generateDiagram(filepath):

    with open(filepath, "r") as f:
        lines = f.read()
    
    match = re.search(r'make_transition_table\(([\s\S]*?)\);', lines);
    if match:
        transition_table = match.group(1)

        # Removes comments in the code
        transition_table = re.sub(r'//.*$', '', transition_table, flags=re.MULTILINE)
        
        # Remove all whitespace
        transition_table = re.sub(r'\s+', '', transition_table, flags=re.MULTILINE)

        transitions = transition_table.split(',')

        body = ''
        for transition in transitions:

            result = transition.split('=')
            dest_state = result[1] if len(result) > 1 else ''

            if dest_state == 'X':
                dest_state = '[*]'

            result = result[0].split('/')
            action = result[1] if len(result) > 1 else ''

            # Extract guard contained between square brackets
            guard = next(iter(re.findall(r'\[(.*?)\]', result[0])), '')

            src_state = result[0].split('+')[0]

            # Remove suffixes
            src_state = src_state.replace('_S', '')
            dest_state = dest_state.replace('_S', '')
            guard = guard.replace('_G', '')
            action = action.replace('_A', '')
            
            if src_state.startswith('*'):
                src_state = src_state[1:]
                body += f"[*] --> {src_state}\n"
            
            if not dest_state:
                dest_state = src_state

            if src_state == 'X':
                src_state = 'Terminate:::terminate'

            body += f"{src_state} --> {dest_state}"

            if guard or action:
                body += " : "

                if guard:
                    body += f"[{guard}]"

                if guard and action:
                    body += "\\n"

                if action:
                    body += f"<i>{action}</i>"
            
            body += "\n"

        uml = "```mermaid\nstateDiagram-v2\nclassDef terminate fill:white,color:black,font-weight:bold\n" + body + "\n```\n";
        print(uml)

if __name__ == "__main__":
    
    for root, dirs, files in os.walk("src/software/ai"):
        for file in files:
            if file.endswith("fsm.h"):
                filepath = os.path.join(root, file)
                generateDiagram(filepath)