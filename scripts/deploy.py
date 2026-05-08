import subprocess

subprocess.run("python -m robotpy deploy --skip-tests --ignore-image-version", shell = True, check = True)
