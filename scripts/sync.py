import subprocess

subprocess.run("python -m robotpy sync --use-certifi --no-upgrade-project", shell = True, check = True)
