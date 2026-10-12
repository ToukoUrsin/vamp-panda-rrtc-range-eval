#!/bin/sh
# Build VAMP v0.6.4 from the PyPI sdist (Release, native flags) in a fresh venv and convert the MBM tarballs.
set -e
export CMAKE_BUILD_PARALLEL_LEVEL=4
sudo apt-get -qq update >/dev/null && sudo DEBIAN_FRONTEND=noninteractive apt-get -qq install -y libeigen3-dev >/dev/null
uv venv -q -p 3.12 /home/daytona/venv
. /home/daytona/venv/bin/activate
uv pip install -q vamp-planner==0.6.4 numpy pyyaml fire tqdm
for r in ${ROBOTS:-panda fetch ur5 baxter}; do
  [ -f res/$r/problems.pkl ] || python res/problem_tar_to_pkl_json.py --robot $r >/dev/null 2>&1
  rm -f res/$r/problems.json
done
python -c "import vamp,sys; print('vamp ok', vamp.__file__, sys.version.split()[0])"
