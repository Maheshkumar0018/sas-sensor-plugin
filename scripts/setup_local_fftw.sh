#!/usr/bin/env bash
# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
#
# Install FFTW development files into <workspace>/.deps/fftw without sudo (ADR-0009).
# Needs the runtime library libfftw3-single3 already installed system-wide.
# Prefer `sudo apt install libfftw3-dev` when root is available; CMake then uses the system copy.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS="${WS}/.deps"
ARCH_LIB="usr/lib/x86_64-linux-gnu"

runtime_ver="$(dpkg-query -W -f='${Version}' libfftw3-single3 2>/dev/null || true)"
if [[ -z "${runtime_ver}" ]]; then
  echo "error: libfftw3-single3 (runtime) is not installed" >&2
  exit 1
fi

mkdir -p "${DEPS}/debs"
rm -rf "${DEPS}/fftw"
(cd "${DEPS}/debs" && apt download "libfftw3-dev=${runtime_ver}")
dpkg-deb -x "${DEPS}/debs/libfftw3-dev_${runtime_ver}_amd64.deb" "${DEPS}/fftw"

# Point the dev symlink at the system runtime and the pkg-config prefix at this tree.
ln -sfn "/${ARCH_LIB}/libfftw3f.so.3" "${DEPS}/fftw/${ARCH_LIB}/libfftw3f.so"
sed -i "s|^prefix=.*|prefix=${DEPS}/fftw/usr|" "${DEPS}/fftw/${ARCH_LIB}/pkgconfig/fftw3f.pc"

echo "FFTW ${runtime_ver} dev files in ${DEPS}/fftw"
