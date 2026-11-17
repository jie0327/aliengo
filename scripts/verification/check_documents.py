"""Check delivered Markdown local targets and code/detail block pairing."""
import json
import re
from pathlib import Path
root=Path(__file__).resolve().parents[2]
files=[root/'README.md', *sorted((root/'docs').rglob('*.md'))]
issues=[]
links=0
for path in files:
 text=path.read_text()
 link_text=re.sub(r'^```.*?^```[^\n]*', '', text, flags=re.M|re.S)
 link_text=re.sub(r'`[^`\n]*`', '', link_text)
 for target in re.findall(r'\[[^\]]*\]\(([^\n)]+)\)',link_text):
  target=target.strip('<>')
  if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:',target) or target.startswith('#'):
   continue
  relative=target.split('#')[0]
  if relative and not (path.parent/relative).exists():
   issues.append({'file':str(path.relative_to(root)),'missing':target})
  links+=1
 if len(re.findall(r'^```',text,re.M))%2:
  issues.append({'file':str(path.relative_to(root)),'problem':'unpaired code fence'})
 if text.count('<details>') != text.count('</details>'):
  issues.append({'file':str(path.relative_to(root)),'problem':'unpaired details'})
result={'files':len(files),'local_links':links,'issues':issues}
(root/'logs/rl').mkdir(parents=True, exist_ok=True)
(root/'logs/rl/document-check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
raise SystemExit(bool(issues))
