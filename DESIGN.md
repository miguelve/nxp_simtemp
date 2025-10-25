# NXP Simulated Temperature Driver — High-Level Architecture

This diagram shows how the `nxp_simtemp` platform driver interacts with:
- the **device tree**,  
- the **Linux kernel driver core**, and  
- a **user-space application**.

---

```mermaid
flowchart TD
    %% --- Device tree and platform driver binding ---
    DT["Device Tree Node<br/><code>compatible = 'nxp_simtemp'</code><br/><code>dev-index, sampling-ms, threshold-mC</code>"]
    KernelBus["Platform Bus<br/>(OF matching)"]
    Driver["nxp_simtemp Driver<br/>(module_platform_driver)<br/><code>simtemp_probe</code>"]

    %% --- Device internals ---
    Thread["Kernel Thread<br/><code>simtemp_fn</code><br/>Generates samples periodically"]
    SampleBuff["Sample Buffer<br/>(simtemp_smp_t)<br/>timestamp, temp_mC, flags"]

    %% --- User space side ---
    User["User-Space Application"]
    DevFile["Character Device<br/><code>/dev/nxp_simtempX</code>"]
    Sysfs["Sysfs Attributes<br/><code>/sys/class/nxp_simtemp/</code><br/>sampling, threshold, mode"]

    %% --- Connections ---
    DT --> KernelBus --> Driver
    Driver -->|"reads DT props"| Thread
    Driver -->|"creates char device"| DevFile
    Driver -->|"creates sysfs attrs"| Sysfs
    Driver -->|"manages buffer"| SampleBuff
    Thread -->|"updates samples"| SampleBuff

    %% User interactions
    User -->|"read(), poll(), ioctl()"| DevFile
    User -->|"echo/cat"| Sysfs
    DevFile -->|"returns struct simtemp_sample"| User
    Sysfs -->|"adjusts parameters"| Driver

    %% Styling
    classDef kernel fill:#f9f9f9,stroke:#333,stroke-width:1px;
    classDef device fill:#e6f7ff,stroke:#0b6fb6;
    classDef user fill:#fff3cd,stroke:#a67c00;
    class DT,KernelBus,Driver,DevFile,Sysfs kernel;
    class Thread,SampleBuff device;
    class User user;
