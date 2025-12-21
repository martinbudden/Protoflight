import subprocess

revision = (
    subprocess.check_output(["git", "rev-parse", "HEAD"])
    .strip()
    .decode("utf-8")
)
print("'-DGIT_REVISION=\"%s\"'" % revision)