from compiler import parse_file, execute_module

mods = ["../examples/alu.cirq","../examples/ram.cirq"]
for mfile in mods:
    print("=== TEST", mfile)
    m = parse_file(mfile)
    execute_module(m)
