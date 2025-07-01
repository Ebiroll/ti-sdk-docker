# ti-sdk-docker

[PROCESSOR-SDK-AM67A](https://www.ti.com/tool/PROCESSOR-SDK-AM67A) running on [TI Ubuntu](https://github.com/TexasInstruments/ti-docker-images)


## This will install the tools required to buld a simple DSP app to be run on the beagley-ai

## Docker setup

### Running the image


```
xhost +local:docker
docker run -it \
    -e DISPLAY=$DISPLAY \
    --name my-ti-container \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/olof/shared:/home/tisdk/shared \
    ghcr.io/ebiroll/ti-sdk-docker:latest

    sudo chown -R tisdk:tisdk /home/tisdk/shared
    sudo chown -R tisdk:tisdk /home/tisdk/ti/
```




### Building image

In same directory as `Dockerfile`:

```sh
docker build -t ghcr.io/ebiroll/ti-sdk-docker:latest .
```

Push to Github, using classic Github token:

```sh
docker login ghcr.io -u your-github-user
docker push ghcr.io/ebiroll/ti-sdk-docker:latest
```

# Devcontainer
On my machine it takes over an hour to add all SDK:s
But you can add "image": "ghcr.io/ebiroll/ti-sdk-docker:latest", to devcontainer.json or where you keep your images.


## Clone Hello Beagley


```
 cd ~/ti/ti-processor-sdk-rtos-j722s-evm-11_00_00_06
 sudo  chown -R tisdk:tisdk /home/tisdk/ti/ti-processor-sdk-rtos-j722s-evm-11_00_00_06
 $ sdk_builder/scripts/setup_psdk_rtos.sh

# ERROR: The following packages were not installed:
  libc6:i386
  It should probably work anyways.

cd ~/ti/ti-processor-sdk-rtos-j722s-evm-11_00_00_06/mcu_plus_sdk_j722s_11_00_00_12/examples
sudo  chown -R tisdk:tisdk /home/tisdk/ti/ti-processor-sdk-rtos-j722s-evm-11_00_00_06/mcu_plus_sdk_j722s_11_00_00_12/
git clone https://github.com/Ebiroll/hello_beagley


export PSDKR_PATH=${HOME}/ti/ti-processor-sdk-rtos-j722s-evm-11_00_00_06
cd ${PSDKR_PATH}
./sdk_builder/scripts/setup_psdk_rtos.sh  #Unless done already
cd sdk_builder
./make_sdk.sh
cd ~/ti/ti-processor-sdk-rtos-j722s-evm-11_00_00_06/mcu_plus_sdk_j722s_11_00_00_12
make -f makefile.j722s

~/ti/ti-processor-sdk-rtos-j722s-evm-11_00_00_06/mcu_plus_sdk_j722s_11_00_00_12$ make -s -C  examples/hello_world/j722s-evm/c75ss0-0_freertos/ti-c7000 syscfg-gui
```



## Regenerating  Hello beagly

```
If running make -s -C  examples/hello_world/j722s-evm/c75ss0-0_freertos/ti-c7000 syscfg-gui
And pressing save generated we get the following difference

Changes not staged for commit:
        modified:   j722s-evm/c75ss0-0_freertos/ti-c7000/generated/ti_dpl_config.c
        modified:   j722s-evm/c75ss0-0_freertos/ti-c7000/generated/ti_drivers_config.c

The changes are most likely related to this. https://www.ti.com/tool/WHIS-3P-SAFERTOS


diff --git a/j722s-evm/c75ss0-0_freertos/ti-c7000/generated/ti_dpl_config.c b/j722s-evm/c75ss0-0_freertos/ti-c7000/generated/ti_dpl_config.c
index 0706904..3202c3b 100644
--- a/j722s-evm/c75ss0-0_freertos/ti-c7000/generated/ti_dpl_config.c
+++ b/j722s-evm/c75ss0-0_freertos/ti-c7000/generated/ti_dpl_config.c
@@ -37,6 +37,29 @@
 #include <kernel/dpl/AddrTranslateP.h>
 #include "ti_dpl_config.h"
 #include "ti_drivers_config.h"
+#if defined(OS_SAFERTOS)
+#include "SafeRTOS_API.h"
+
+extern const xPORT_INIT_PARAMETERS xPortInit;
+
+portBaseType Dpl_kernelInit(void)
+{
+    portBaseType xInitSchedResult;
+
+    /* Initialise the kernel by passing in a pointer to the xPortInit structure
+     * and return the resulting error code. */
+    xInitSchedResult = xTaskInitializeScheduler( &xPortInit );
+
+    if( pdPASS == xInitSchedResult )
+    {
+#if ( configQUEUE_REGISTRY_SIZE > 0 )
+        vQueueAddToRegistry( acTimerCommandQueueBuffer, "Timer Command Q" );
+#endif
+    }
+
+    return xInitSchedResult;
+}
+#endif

One thing to note about the hello_beagley vs hello_world is that I have tried to add IPC Support. 
diff -urN  examples/hello_world/j722s-evm/c75ss0-0_freertos/ti-c7000/generated/  examples/hello_beagley/j722s-evm/c75ss0-0_freertos/ti-c7000/generated/

```

## Ideas for IPC

If we manage to align the carveouts, https://git.ti.com/cgit/ti-linux-kernel/ti-linux-kernel/tree/arch/arm64/boot/dts/ti/k3-am67a-beagley-ai.dts?h=11.00.09
One day we might patch the hello world program to eomething like this,

Looking at the device tree file, I can see the DMA carveout configurations are defined in the reserved-memory section. The DMA memory regions are configured as carveouts for each processor core (R5F, MCU R5F, Main R5F, and C7x DSPs) k3-am67a-beagley-ai.dts « ti « dts « boot « arm64 « arch - ti-linux-kernel/ti-linux-kernel - This repo contains a Linux kernel that has been integrated with outstanding TI open source patches based on the open source Linux kernel found at kernel.org. Contributions to this kernel need to be sent to the open source community for review..
Here are the DMA carveout configurations:

WKUP R5FSS0 Core0 DMA (lines 48-53):

Location: 0xa0000000
Size: 0x100000 (1MB)
Type: shared-dma-pool


MCU R5FSS0 Core0 DMA (lines 60-65):

Location: 0xa1000000
Size: 0x100000 (1MB)
Type: shared-dma-pool


Main R5FSS0 Core0 DMA (lines 72-77):

Location: 0xa2000000
Size: 0x100000 (1MB)
Type: shared-dma-pool


C7x_0 DMA (lines 84-89):

Location: 0xa3000000
Size: 0x100000 (1MB)
Type: shared-dma-pool


C7x_1 DMA (lines 96-101):

Location: 0xa4000000
Size: 0x100000 (1MB)
Type: shared-dma-pool



Each DMA region is paired with a larger memory region for the core itself (0xf00000 or 15MB).

```

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/ipc_notify.h>
#include <drivers/ipc_rpmsg.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

void hello_world_main(void *args)
{
    int32_t status;
    
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    
    DebugP_log("Hello World!\r\n");
    
    /* Initialize IPC Notify */
    status = IpcNotify_init();
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("IPC Notify init failed!\r\n");
    }
    else
    {
        DebugP_log("IPC Notify init done\r\n");
    }
    
    /* Initialize IPC RPMessage */
    status = IpcRPMessage_init();
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("IPC RPMessage init failed!\r\n");
    }
    else
    {
        DebugP_log("IPC RPMessage init done\r\n");
        
        /* Print IPC configuration */
        DebugP_log("\r\n==== IPC Configuration ====\r\n");
        DebugP_log("Self CPU ID: %d\r\n", IpcNotify_getSelfCoreId());
        DebugP_log("Number of cores: %d\r\n", IpcNotify_getNumCores());
        
        /* Print available remote cores */
        uint32_t numCores = IpcNotify_getNumCores();
        uint32_t selfCoreId = IpcNotify_getSelfCoreId();
        
        DebugP_log("Remote cores available for IPC:\r\n");
        for(uint32_t coreId = 0; coreId < numCores; coreId++)
        {
            if(coreId != selfCoreId)
            {
                if(IpcNotify_isCoreEnabled(coreId))
                {
                    DebugP_log("  - Core %d: Enabled\r\n", coreId);
                }
                else
                {
                    DebugP_log("  - Core %d: Disabled\r\n", coreId);
                }
            }
        }
        
        /* Print shared memory configuration if needed */
        DebugP_log("==========================\r\n\r\n");
    }
    
    /* Cleanup */
    Board_driversClose();
    Drivers_close();
}
```

## Fix link.cmd

From calude 4 opus I get,
Looking at your linker command file, there's a mismatch between the DMA carveout in the device tree and what's defined in the linker script.
Device Tree (DTS) says:

C7x_0 DMA memory region: 0xa3000000 with size 0x100000 (1MB)
C7x_0 memory region: 0xa3100000 with size 0xf00000 (15MB)

Linker script says:

DDR0_ALLOCATED_START: 0xA3000000 (matches the DTS)
C7X_IPC_D: starts at 0xA3000000, size 0x00100000 (1MB) - this matches
C7X_BOOT_BASE: starts at 0xA3200000 (0xA3000000 + 0x00200000)
C7X_DDR_SPACE: goes up to 0xA3410000 + 0x00BF0000 = 0xA4000000

The issue is that the linker script allocates memory up to 0xA4000000, but according to the DTS, the C7x_0 memory region should only go up to 0xA3100000 + 0xF00000 = 0xA4000000.
So technically they do align at the boundary, but there's a discrepancy in how the memory is being used:

DTS expects:

0xA3000000 - 0xA3100000: DMA region (1MB)
0xA3100000 - 0xA4000000: Memory region (15MB)


Linker script uses:

0xA3000000 - 0xA3100000: IPC region (1MB)
0xA3100000 - 0xA3100400: Resource table (1KB)
0xA3100400 - 0xA3200000: IPC trace buffer (~1MB)
0xA3200000 - 0xA3200400: Boot section
0xA3400000 - 0xA4000000: Main memory sections



The linker script seems to be using the memory regions differently than what the DTS expects. 

## Resource table in linux

https://www.mail-archive.com/linux-kernel@vger.kernel.org/msg2589080.html

```
// In ti_k3_dsp_remoteproc.c (now refactored to ti_k3_common.c)
static struct resource_table *k3_get_loaded_rsc_table(struct rproc *rproc,
                                                     size_t *rsc_table_sz)
{
    struct k3_rproc *kproc = rproc->priv;
    
    // Resource table is at the beginning of the first reserved memory region
    *rsc_table_sz = 256; // Typical size
    return (__force struct resource_table *)kproc->rmem[0].cpu_addr;
}
```

The resource table must be:

Placed in a specific section (.resource_table) in your DSP firmware
Located at the beginning of the DDR memory region allocated for the DSP
Properly formatted with the correct version and resource entries

This is why your linker script needs the .resource_table section placed at C7X_RESOURCE_TABLE_BASE (0xA3100000 in your case), which should match the DTS memory allocation.

## Running TI Edge AI SDK on BeagleY-AI

I have the TI Edge AI SDK [11.00.00.08](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-AM67A/11.00.00.08) running on BeagleY-AI with a few modifications.

- [UBoot - branch `ti-u-boot-2024.04-bb`](https://github.com/goat-hill/ti-u-boot/tree/ti-u-boot-2024.04-bb)
- [Linux kernel - branch `ti-linux-6.12.y-bb`](https://github.com/goat-hill/linux/tree/ti-linux-6.12.y-bb)
- [TI vision_apps - branch `11.00.00.06-beagley`](https://github.com/goat-hill/ti-vision-apps/tree/11.00.00.06-beagley)

## U-Boot development workflow

### Building U-Boot

#### r5

EVM configure:

```sh
make ARCH=arm O=/home/tisdk/uboot-build/r5 j722s_evm_r5_defconfig
```

BeagleY-AI configure:

```sh
make ARCH=arm O=/home/tisdk/uboot-build/r5 beagleyai_r5_defconfig
```

Compile:

```sh
make -j$(nproc) ARCH=arm O=/home/tisdk/uboot-build/r5 \
    CROSS_COMPILE="$CROSS_COMPILE_32" \
    BINMAN_INDIRS=${PREBUILT_IMAGES}
```

Copy output to shared volume:

```sh
cp /home/tisdk/uboot-build/r5/tiboot3-j722s-hs-fs-evm.bin /home/tisdk/shared/ti-uboot-build/tiboot3.bin
```

##### Other tasks

Clean:

```sh
make ARCH=arm O=/home/tisdk/uboot-build/r5 mrproper
```

Simplify defconfig:

```sh
make ARCH=arm O=/home/tisdk/uboot-build/r5 savedefconfig
cp /home/tisdk/uboot-build/r5/defconfig configs/beagleyai_r5_defconfig
```

#### a53

EVM configure:

```sh
make ARCH=arm O=/home/tisdk/uboot-build/a53 j722s_evm_a53_defconfig
```

BeagleY-AI configure:

```
make ARCH=arm O=/home/tisdk/uboot-build/a53 beagleyai_a53_defconfig
```

Compile:

```sh
make -j$(nproc) ARCH=arm O=/home/tisdk/uboot-build/a53 \
    CROSS_COMPILE="$CROSS_COMPILE_64" \
    CC="$CC_64" \
    BL31=${PREBUILT_IMAGES}/bl31.bin \
    TEE=${PREBUILT_IMAGES}/bl32.bin \
    BINMAN_INDIRS=${PREBUILT_IMAGES}
```

Copy output to shared volume:

```sh
cp /home/tisdk/uboot-build/a53/tispl.bin /home/tisdk/shared/ti-uboot-build/
cp /home/tisdk/uboot-build/a53/u-boot.img /home/tisdk/shared/ti-uboot-build/
```

##### Other tasks

Clean:

```sh
make ARCH=arm O=/home/tisdk/uboot-build/a53 mrproper
```

Simplify defconfig:

```sh
make ARCH=arm O=/home/tisdk/uboot-build/a53 savedefconfig
cp /home/tisdk/uboot-build/a53/defconfig configs/beagleyai_a53_defconfig
```

### Installing U-Boot

Based on [SDK instructions here](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-am67a/latest/exports/docs/linux/Foundational_Components/U-Boot/UG-General-Info.html#build-u-boot)

```sh
sudo cp tiboot3.bin tispl.bin u-boot.img /media/brady/BOOT
```

## Linux kernel development workflow

### Building Linux kernel

#### Image and modules

Must be on same git commit for `Image` + `modules` + `module_install` with no local changes to avoid `-dirty` flagging.

Clean and configure make:

```sh
make -j$(nproc) ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" distclean
make -j$(nproc) ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" defconfig ti_arm64_prune.config
```

Compile linux kernel image and modules:

```sh
make -j$(nproc) ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" Image
make -j$(nproc) ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" modules
```

#### Device tree DTBs

```sh
make -j$(nproc) ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE_64" dtbs
```

### Installing kernel

Instructions based on [SDK docs here](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-am67a/latest/exports/docs/linux/Foundational_Components_Kernel_Users_Guide.html#installing-the-kernel)

```sh
sudo cp arch/arm64/boot/Image /media/brady/rootfs/boot/
sudo cp arch/arm64/boot/dts/ti/k3-am67a-beagleyai.dtb /media/brady/rootfs/boot/dtb/
sudo cp arch/arm64/boot/dts/ti/k3-am67a-beagley-ai-edgeai-apps.dtbo /media/brady/rootfs/boot/dtb/ti/
sudo cp arch/arm64/boot/dts/ti/k3-am67a-beagley-ai-csi0-imx219.dtbo /media/brady/rootfs/boot/dtb/ti/
sudo make ARCH=arm64 INSTALL_MOD_PATH=/media/brady/rootfs/ modules_install
```

Make sure `boot` partition `uEnv.txt` indicates the overlays:
```
name_overlays=ti/k3-am67a-beagley-ai-edgeai-apps.dtbo ti/k3-am67a-beagley-ai-csi0-imx219.dtbo
```

## TI vision_apps development workflow

Install `PROCESSOR-SDK-RTOS-J722S` on TI Ubuntu docker image. Download SDK here:
https://www.ti.com/tool/PROCESSOR-SDK-J722S

I'm mostly following AM67A [firmware builder guide](https://software-dl.ti.com/jacinto7/esd/processor-sdk-rtos-j722s/11_00_00_06/exports/docs/psdk_rtos/docs/user_guide/firmware_builder.html)

To setup firmware builder run the following below. Exclude `--firmware-only` flag as we need Linux changes. Execute this in the root directory of PSDK RTOS.

```sh
./sdk_builder/scripts/setup_psdk_rtos.sh
```

### Swap to modified BeagleY-AI modified vision_apps

We need to use a modified verison of vision_apps to support memory map for BeagleY-AI with 4 GB DDR vs 8 GB EVM. Modifications are based on instructions [here](https://software-dl.ti.com/jacinto7/esd/processor-sdk-rtos-j722s/latest/exports/docs/psdk_rtos/docs/user_guide/developer_notes_memory_map.html)

```sh
mv vision_apps/ vision_apps_bak/
git clone https://github.com/goat-hill/ti-vision-apps.git vision_apps
cd vision_apps/
git checkout 11.00.00.06-beagley
```

### Compilation

Then:

```sh
cd sdk_builder/
TISDK_IMAGE=edgeai ./make_firmware.sh
```

### Gather build outputs

```sh
cp -r /tmp/tivision_apps_targetfs_stage /home/tisdk/shared/tivision_apps_targetfs_stage
```

### Installing vision_apps

```sh
export LINUX_FS_PATH=/media/brady/rootfs
export LINUX_FS_STAGE_PATH=/home/brady/host-shared/code/tivision_apps_targetfs_stage

# remove old remote files from filesystem
sudo rm -f $LINUX_FS_PATH/usr/lib/firmware/j722s-*-fw
sudo rm -f $LINUX_FS_PATH/usr/lib/firmware/j722s-*-fw-sec
sudo rm -rf $LINUX_FS_PATH/usr/lib/firmware/vision_apps_eaik
sudo rm -rf $LINUX_FS_PATH/opt/tidl_test/*
sudo rm -rf $LINUX_FS_PATH/opt/notebooks/*
sudo rm -rf $LINUX_FS_PATH/usr/include/processor_sdk/*

# create new directories
sudo mkdir -p $LINUX_FS_PATH/usr/include/processor_sdk

# copy full vision apps linux fs stage directory into linux fs
sudo cp -r $LINUX_FS_STAGE_PATH/* $LINUX_FS_PATH/.
```

## Edge AI development workflow

### Obtaining Edge AI image rootfs .tar.gz

Download same [TI SDK Edge AI](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-AM67A/11.00.00.08) .wic.gz used for flashing SD card.
Now we want to get the rootfs of the .wic available in the same directory. A bit tricky becuase it's a .wic image and not .tar.gz like TI SDK adas edition.

Steps if in a docker image:
```sh
sudo losetup -P $(losetup -f) ~/host-shared/code/tisdk-edgeai-image-j722s-evm.wic 
losetup -a
sudo mount /dev/loop13p2 /mnt/rootfs-edgeai
sudo tar -czpf targetfs.tar.gz -C /mnt/rootfs-edgeai/ .
```

Hold on to this .tar.gz. Now cleanup the mess:

```sh
sudo umount /mnt/rootfs-edgeai 
sudo losetup -d /dev/loop13
```

### Creating build environment

On TI Ubuntu docker image with Edge AI SDK installed:

```sh
git clone https://github.com/TexasInstruments/edgeai-app-stack
git submodule init
git submodule update
```

Extract EdgeAI rootfs to `targetfs/` local directory.

```sh
tar -zxf targetfs.tar.gz -C targetfs
```

### Compilation
Note: Required modifications to Makefile (point to local compile toolchain, targetfs/, and new targetfs-install/ dir)

Now compile:
```sh
ARCH=arm64 make -j$(nproc)
```
I had to compile it a few times...

### Copy to SD card

```sh
sudo cp -r targetfs-install/* /media/brady/rootfs/
```

## Testing TI Edge AI SDK

Kill the default app started at launch

```sh
killall edgeai-gui-app
```

Pipe OpenVX logs to shell:
```sh
source /opt/vision_apps/vision_apps_init.sh
```

Run IMX219 demo:

```sh
cd /opt/edgeai-gst-apps/apps_cpp/
./bin/Release/app_edgeai ../configs/imx219_cam_example.yaml
```

## Issues

- Need to add `libncurses-dev` apt package for menuconfig
- TI Ubuntu Docker build is [broken](https://github.com/TexasInstruments/ti-docker-images/pull/14#issuecomment-2970968176) (there was a gnutls package missing as a result)
