FROM debian:stable-20240110

RUN apt-get update \
    && apt install crossbuild-essential-arm64 meson ninja libopencv-dev libapriltag-dev git

COPY ./* /vision