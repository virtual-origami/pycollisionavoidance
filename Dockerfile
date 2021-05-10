FROM python:3.8.3-slim-buster AS base

# Dedicated Workdir for App
WORKDIR /pycollisionavoidance

# Do not run as root
RUN useradd -m -r pycollisionavoidance && \
    chown pycollisionavoidance /pycollisionavoidance

COPY requirements.txt /pycollisionavoidance
RUN pip3 install -r requirements.txt

FROM base AS src
COPY . /pycollisionavoidance

# install pycollisionavoidance here as a python package
RUN pip3 install .

USER pycollisionavoidance

COPY scripts/docker-entrypoint.sh /entrypoint.sh

# Use the `collision-avoidance` binary as Application
FROM src AS prod
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["collision-avoidance", "-c", "config.yaml"]
