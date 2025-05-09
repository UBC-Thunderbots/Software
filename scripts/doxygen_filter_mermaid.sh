#!/bin/bash

# Usage: ./doxygen_filter_mermaid.sh <md-file>
# 
# This script reads the file <md-file> in, replaces all fenced code blocks
# with html <pre> </pre> tags and prints the result into stdout. 
# So, for example following content in input file <md-file>
#
# ```mermaid
#   <mermaid syntax>
# ```
#
# would be replaced by
#
# <pre class="mermaid">
#   <mermaid syntax>
# </pre>
#
# additionally a couple of newlines are added right before and after of <pre> tags.

fence_start="(^\`\`\`mermaid)(.*)"
fence_end="(\`\`\`)"
in_fence=0
while IFS= read -r line
do 
  if [[ $line =~ $fence_start ]]; then
        
        printf '\n<pre class=\"mermaid\">%s\n' "${BASH_REMATCH[2]}" 
        in_fence=1
  elif [ $in_fence -eq 1 ]; then
    if [[ $line =~ $fence_end ]]; then
      printf '</pre>\n\n'      
      in_fence=0
    else 
       printf '%s\n' "${line}"
    fi
  else
        printf '%s\n' "${line}"
  fi
done < "${1}"
