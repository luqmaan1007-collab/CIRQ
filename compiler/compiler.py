from parser import parse_file
from codegen import execute_module
import sys

if len(sys.argv) < 2:
    print("Usage: python3 compiler.py <file.cirq>")
    sys.exit(1)

mod = parse_file(sys.argv[1])
execute_module(mod)
