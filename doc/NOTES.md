## Data pathway

This is a map of the pathways traversed by sensor data in the firmware.

-----

#### Hardware (This is the easy part)

The two aspects of this sensor are pysically divided in the package, with separate data and chip-select lines.

  ```mermaid
  graph TD
    a(LSM9DS1 inertial aspect)
    b(LSM9DS1 magnetic aspect)
    c(CPLD)
    d(CPU)
    e(Transport hardware, eg: USB, Bluetooth)

    subgraph Hardware pathway
    a --> c
    b --> c
    c --> d
    d --> e
    end

    style a fill:#ccc,stroke:#222,stroke-width:2px;
    style b fill:#ccc,stroke:#222,stroke-width:2px;
    style c fill:#ccc,stroke:#222,stroke-width:2px;
    style d fill:#ccc,stroke:#222,stroke-width:2px;
    style e fill:#ccc,stroke:#222,stroke-width:2px;

  ```

-----

#### Software (Data path)

This graph is distinct from the control flow. The sensor data is scaled, upgraded to float, filtered, and fused before being deposited in a ManuLegend for transmission to other firmware modules (typically a counter-party via a comm link).

  ```mermaid
  graph TD
    subgraph Color Key
    key0[Asynchronous break]
    key2[Data abstraction point]
    key1[Sensor-specific]
    end

    style key0 fill:#8Cf,stroke:#222,stroke-width:2px;
    style key1 fill:#fac,stroke:#222,stroke-width:2px;
    style key2 fill:#e95,stroke:#222,stroke-width:2px;
  ```

  ```mermaid
  graph TD
    h(SPIBusOp)
    i(LSM9DS1_AG)
    j(LSM9DS1_M)
    k(LSM9DSx)
    m(InertialMeasurement)
    n(LegendManager)
    o(IIU)
    p(ManuLegend)

    q(Session driver)
    r(Transport driver)
    s(Internal condition triggers,<br /> power-management, etc)
    t(Error measurement<br /> and compensation)

    z{Data type}

    subgraph Replicated per-datum
    z -->|Inertial| i
    z -->|Magnetic| j
    i --> k
    j --> k
    k --> m
    m --> o
    end

    subgraph Single read
    t --> m
    p --> t
    h --> n
    n --> z
    o --> p
    p -.-> s
    p -.-> q
    q --> r
    end

    style h fill:#8Cf,stroke:#222,stroke-width:2px;
    style i fill:#fac,stroke:#222,stroke-width:2px;
    style j fill:#fac,stroke:#222,stroke-width:2px;
    style k fill:#fac,stroke:#222,stroke-width:2px;
    style m fill:#e95,stroke:#222,stroke-width:2px;
    style n fill:#fff,stroke:#222,stroke-width:2px;

    style o fill:#8Cf,stroke:#222,stroke-width:2px;
    style p fill:#e95,stroke:#222,stroke-width:2px;
    style q fill:#8Cf,stroke:#222,stroke-width:2px;
    style r fill:#8Cf,stroke:#222,stroke-width:2px;
    style s fill:#8Cf,stroke:#222,stroke-width:2px;
    style t fill:#fff,stroke:#222,stroke-width:2px;

    style z fill:#fac,stroke:#222,stroke-width:2px;
  ```

-----

#### Software (Control path)
