import subprocess

subprocess.run("python -m robotpy test --isolated -- -v -s --exitfirst", shell = True, check = True)
