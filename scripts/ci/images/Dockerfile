# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT
ARG BASE_IMAGE=git.in.encos.cn/container/ubuntu:emd-ci-portable-glibc2.28-20261027-v1-amd64
FROM ${BASE_IMAGE}
USER root
ARG VARIANT
ARG TARGETARCH
ARG WITH_JS=false
COPY eigen.tar.gz install.sh package.json pnpm-lock.yaml pnpm-workspace.yaml /tmp/jointsdk-image/
RUN --mount=type=cache,id=jointsdk-dnf-${VARIANT}-${TARGETARCH},target=/var/cache/dnf \
    bash /tmp/jointsdk-image/install.sh && rm -rf /tmp/jointsdk-image
ENV JOINT_SDK_EIGEN_PREFIX=/opt/jointsdk/eigen-3.3.4
ENV COREPACK_ENABLE_NETWORK=0
