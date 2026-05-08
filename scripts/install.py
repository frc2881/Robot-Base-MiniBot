import subprocess
import tomllib

with open("pyproject.toml", "rb") as f:
  toml = tomllib.load(f)
robotpy_version = toml["tool"]["robotpy"]["robotpy_version"]

subprocess.run(f'python -m pip install --upgrade robotpy=={ robotpy_version } certifi', shell = True, check = True)
