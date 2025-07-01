FROM ghcr.io/texasinstruments/ubuntu-distro:latest@sha256:7b47a9eaecfc2cc6a5b875c7f26632a40f98cf25dc63baba10eec42efacab812

# Default to root user for installs
USER root

ENV DEBIAN_FRONTEND=noninteractive

# Install GUI dependencies and basic tools
RUN apt-get update && \
    apt-get install -y \
        libx11-6 libxext6 libxrender1 libxtst6 libxi6 libgtk-3-0 \
        xauth x11-xserver-utils \
        wget sudo bash && \
    rm -rf /var/lib/apt/lists/*

# Create install directory under /home/tisdk/ti
RUN mkdir -p /home/tisdk/ti && chown -R tisdk:tisdk /home/tisdk/ti
RUN mkdir -p /home/tisdk/ti/edgeai && chown -R tisdk:tisdk /home/tisdk/ti
RUN mkdir -p /home/tisdk/ti/ti-cgt-c7000 && chown -R tisdk:tisdk /home/tisdk/ti
RUN mkdir -p /home/tisdk/ti/sysconfig && chown -R tisdk:tisdk /home/tisdk/ti


# ===== ARGs for downloading =====
ARG SDK_INSTALLER_URL="https://dr-download.ti.com/software-development/software-development-kit-sdk/MD-NQjfZVt1aJ/11.00.00.08/ti-processor-sdk-linux-edgeai-j722s-evm-11_00_00_08-Linux-x86-Install.bin"
ARG CGT_C7000_INSTALLER_URL="https://dr-download.ti.com/software-development/ide-configuration-compiler-or-debugger/MD-707zYe3Rik/5.0.0.LTS/ti_cgt_c7000_5.0.0.LTS_linux-x64_installer.bin"
ARG SYSCONFIG_INSTALLER_URL="https://dr-download.ti.com/software-development/ide-configuration-compiler-or-debugger/MD-nsUM6f7Vvb/1.24.0.4150/sysconfig-1.24.0_4150-setup.run"
ARG RTOS_SDK_URL="https://dr-download.ti.com/software-development/software-development-kit-sdk/MD-1bSfTnVt5d/11.00.00.06/ti-processor-sdk-rtos-j722s-evm-11_00_00_06.tar.gz"

WORKDIR /tmp

# ===== Install Linux EdgeAI SDK =====
RUN SDK_INSTALLER_FILENAME="${SDK_INSTALLER_URL##*/}" && \
    wget "${SDK_INSTALLER_URL}" -O "${SDK_INSTALLER_FILENAME}" && \
    chmod +x "${SDK_INSTALLER_FILENAME}" && \
    ./"${SDK_INSTALLER_FILENAME}" --mode unattended --prefix /home/tisdk/ti/edgeai && \
    rm "${SDK_INSTALLER_FILENAME}"

# ===== Install TI CGT C7000 Compiler =====
RUN CGT_FILENAME="${CGT_C7000_INSTALLER_URL##*/}" && \
    wget "${CGT_C7000_INSTALLER_URL}" -O "${CGT_FILENAME}" && \
    chmod +x "${CGT_FILENAME}" && \
    ./"${CGT_FILENAME}" --mode unattended --prefix /home/tisdk/ti/ti-cgt-c7000 && \
    rm "${CGT_FILENAME}"

# ===== Install SysConfig GUI tool =====
RUN SYSCONFIG_FILENAME="${SYSCONFIG_INSTALLER_URL##*/}" && \
    wget "${SYSCONFIG_INSTALLER_URL}" -O "${SYSCONFIG_FILENAME}" && \
    chmod +x "${SYSCONFIG_FILENAME}" && \
    ./"${SYSCONFIG_FILENAME}" --mode unattended --prefix /home/tisdk/ti/sysconfig && \
    rm "${SYSCONFIG_FILENAME}"

# ===== Install RTOS SDK =====
RUN RTOS_SDK_FILENAME="${RTOS_SDK_URL##*/}" && \
    wget "${RTOS_SDK_URL}" -O "${RTOS_SDK_FILENAME}" && \
    tar -xf "${RTOS_SDK_FILENAME}" -C /home/tisdk/ti && \
    rm "${RTOS_SDK_FILENAME}"

# ===== Set environment variables =====
ENV TI_INSTALL_BASE=/home/tisdk/ti

ENV CG_TOOL_ROOT=${TI_INSTALL_BASE}
ENV SYSCONFIG_ROOT=${TI_INSTALL_BASE}/sysconfig
ENV SDK_INSTALL_PATH=${TI_INSTALL_BASE}/ti-processor-sdk-rtos-j722s-evm-11_00_00_06
ENV TI_SDK_PATH=${SDK_INSTALL_PATH}

ENV PATH="${TI_INSTALL_BASE}/ti-cgt-c7000/ti-cgt-c7000/ti-cgt-c7000_5.0.0.LTS/bin:${SYSCONFIG_ROOT}:${PATH}"

# Optional: Set default working directory and copy helper scripts
COPY --chmod=755 uboot-env.sh /home/tisdk/
WORKDIR /home/tisdk

# ===== Return to tisdk user =====
USER tisdk
