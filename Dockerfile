# Docker images created using https://github.com/cntaylor/gtsam-python-docker-vscode
# Add pybind11 to the image (whichever one is being used)

FROM cntaylor/gtsam-python as release
RUN python3 -m pip install pybind11

FROM cntaylor/gtsam-python-debug as debug
RUN python3 -m pip install pybind11
