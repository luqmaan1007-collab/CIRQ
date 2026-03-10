import re

class Module:
    def __init__(self, name):
        self.name = name
        self.body = []

def parse_file(path):
    with open(path) as f:
        src = f.read()
    m = re.search(r'module\s+(\w+)', src)
    if not m: raise Exception("Module name missing")
    module = Module(m.group(1))
    body = re.search(r'\{(.*)\}', src, re.S)
    module.body = [l.strip() for l in body.group(1).splitlines() if l.strip()]
    return module
