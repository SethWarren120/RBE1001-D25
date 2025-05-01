# All credit to Seth for the code to condense the project into a single file
import json
import os

processed = []
queue = [[]]
files = {}
filesDep = {}

dependencies = [[]]

dir = ""
out = ""

built = []
imports = ""
output = ""

with open("tools/condenser_settings.json", "r") as file:
    settings = json.loads(file.read())
    dir = settings["projectPath"]
    queue[0].append(settings["main"])
    out = settings["output"]

i = 0
dependenceReached = False

while not dependenceReached:
    queue.append([])
    dependencies.append([])
    while len(queue[i]) > 0:
        f = queue[i][0]
        queue[i].remove(f)
        dependencies[i].append(f)
        if f in processed:
            queue[i + 1] = queue[i + 1] + filesDep[f]
            continue
        processed.append(f)

        with open(dir + f) as file:
            
            files[f] = ""
            filesDep[f] = []
            lines = file.read().splitlines()

            for line in lines:
                content = line.strip()
                if content == "" or content.startswith("#"):
                    continue
                if content.startswith("from"):
                    c2 = content.removeprefix("from").strip().split(' ', 1)[0].replace('.', '/') + ".py"
                    if os.path.exists(dir + c2):
                        queue[i+1].append(c2)
                        filesDep[f].append(c2)
                    elif not content in imports:
                        imports = imports + content + "\n"
                    continue
                files[f] = files[f] + line + "\n"
    i += 1
    if len(queue[i]) == 0:
        dependenceReached = True

while i >= 0:
    for dependence in dependencies[i]:
        if dependence in built:
            continue
        
        built.append(dependence)
        output = output + files[dependence]
    i -= 1

with open(out, "w") as outputFile:
    outputFile.write("# Automatically generated deployment script\n# Edits to this file will be overwritten\n" + imports + output)