FROM ubuntu:focal
# skip interactive setup when installing packages
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update --yes
RUN apt-get install --yes build-essential cmake
RUN apt-get install --yes python3-dev python3-numpy
RUN apt-get install --yes libmodbus-dev libeigen3-dev libserialport-dev libopencv-dev