FROM python:3.8.3-slim-buster AS base

RUN apt-get update
RUN apt-get install -y netcat

# Dedicated Workdir for App
WORKDIR /pycollisionavoidance

# Do not run as root
RUN useradd -m -r pycollisionavoidance && \
    chown pycollisionavoidance /pycollisionavoidance

COPY requirements.txt /pycollisionavoidance
# RUN pip3 install -r requirements.txt

FROM base AS src
COPY . /pycollisionavoidance

# install pycollisionavoidance here as a python package
RUN pip3 install .

# USER pycollisionavoidance is commented to fix the bug related to permission
# USER pycollisionavoidance

COPY scripts/docker-entrypoint.sh /entrypoint.sh

# Use the `collision-avoidance` binary as Application
FROM src AS prod

# this is add to fix the bug related to permission
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["collision-avoidance", "-c", "config.yaml"]
