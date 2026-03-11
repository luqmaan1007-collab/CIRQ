def execute_module(module):
    print(f"Running module {module.name}")
    for line in module.body:
        print("EXEC>", line)  # placeholder for real virtual hardware logic
