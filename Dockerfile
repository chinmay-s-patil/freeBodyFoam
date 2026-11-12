# syntax=docker/dockerfile:1
FROM docker.io/opencfd/openfoam-default:2412@sha256:1ba02114b1c025c370f2e269a07677c16c9bea8d990fcd75ac8378aff9d41b50

USER root

# Install Python 3 and verification plotting dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-pandas \
    python3-pil \
    && rm -rf /var/lib/apt/lists/*

# Copy freeBodyFoam source into container
ENV FREEBODYFOAM_SRC=/usr/local/src/freeBodyFoam
WORKDIR ${FREEBODYFOAM_SRC}
COPY . ${FREEBODYFOAM_SRC}

# Compile solvers, libraries, and utilities into OpenFOAM environment and system path
RUN bash -c '\
    FOAM_BASHRC="/usr/lib/openfoam/openfoam2412/etc/bashrc"; \
    set +e; \
    source "$FOAM_BASHRC"; \
    set -e; \
    ./Allwmake; \
    mkdir -p /usr/local/bin; \
    for app in freeBodyPimpleFoam freeBody6DoFPimpleFoam freeBodyRhoPimpleFoam freeBodyRhoCentralFoam writeFFDict; do \
        cp -f "${FOAM_USER_APPBIN}/${app}" "${FOAM_APPBIN}/"; \
        cp -f "${FOAM_USER_APPBIN}/${app}" /usr/local/bin/; \
        chmod 755 "/usr/local/bin/${app}"; \
    done; \
    for lib in libfallingInletVelocity.so libfreeFallingModels.so libfreeFallingFunctionObjects.so libsixDoFRotatingMotion.so libfreeBodyRotationalMotion.so; do \
        cp -f "${FOAM_USER_LIBBIN}/${lib}" "${FOAM_LIBBIN}/"; \
    done'

# Smoke test binaries to guarantee successful compilation and linking
RUN bash -c '\
    source /usr/lib/openfoam/openfoam2412/etc/bashrc && \
    freeBodyPimpleFoam -help && \
    freeBody6DoFPimpleFoam -help && \
    freeBodyRhoPimpleFoam -help && \
    freeBodyRhoCentralFoam -help && \
    writeFFDict -help'

# Switch to default non-root openfoam user
USER openfoam
WORKDIR /home/openfoam

CMD ["/bin/bash"]
