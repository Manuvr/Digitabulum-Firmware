## Data pathway

This is a map of the pathways traversed by sensor data in the firmware.

-----

#### Hardware (This is the easy part)

The two aspects of this sensor are physically divided in the IMU's package, with separate data and chip-select lines. The interface is unified by the CPLD. From that point, the CPU is responsible for processing the data and shuttling the results to a transport.

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
    key2[Data copy]
    key1[Data processing]
    end

    style key0 fill:#8Cf,stroke:#222,stroke-width:2px;
    style key1 fill:#fac,stroke:#222,stroke-width:2px;
    style key2 fill:#e95,stroke:#222,stroke-width:2px;
  ```

  ```mermaid
  graph TD
    a(Scaling, Float upgrade,<br />and application of error)
    b(Scaling, Float upgrade,<br />and application of error)
    h(SPIBusOp)
    i(Bias de-sat)
    j(Spherical aberration<br />correction)
    k(Orientation)
    m(SensorFrame)
    u(Gravity nullification)
    v(Velocity)
    w(Position)
    n(ManuManager)
    p(ManuLegend)

    q(Session driver)
    r(Transport driver)
    t(Error measurement<br /> and compensation)

    z{Data type}

    subgraph Integrator
    m --> z
    z -->|Inertial| i
    z -->|Magnetic| j
    z -->|Temperature| t
    z -->|Scale| t
    i --> a
    j --> b
    b --> k
    b --> p
    a --> p
    u --> p
    k --> p
    v --> p
    w --> p
    a --> k
    k --> u
    u --> v
    v --> w
    end

    subgraph Single read
    h --> n
    n --> m
    p -.-> q
    p --> t
    q --> r
    end

    style h fill:#8Cf,stroke:#222,stroke-width:2px;
    style i fill:#fac,stroke:#222,stroke-width:2px;
    style j fill:#fac,stroke:#222,stroke-width:2px;
    style k fill:#fac,stroke:#222,stroke-width:2px;
    style m fill:#e95,stroke:#222,stroke-width:2px;
    style n fill:#fff,stroke:#222,stroke-width:2px;

    style p fill:#e95,stroke:#222,stroke-width:2px;
    style q fill:#8Cf,stroke:#222,stroke-width:2px;
    style r fill:#8Cf,stroke:#222,stroke-width:2px;
    style t fill:#fff,stroke:#222,stroke-width:2px;

    style a fill:#fac,stroke:#222,stroke-width:2px;
    style b fill:#fac,stroke:#222,stroke-width:2px;
    style u fill:#fac,stroke:#222,stroke-width:2px;
    style v fill:#fac,stroke:#222,stroke-width:2px;
    style w fill:#fac,stroke:#222,stroke-width:2px;
  ```

-----
#### Software (Control path)

Data-driven demands from the integrator need to ultimately result in register changes in the IMUs.

  ```mermaid
  graph TD
    a(LSM9DS1)
    b(ManuManager)
    c(Integrator)
    d(ManuLegend)
    e(Client software)

    subgraph Control relationships
    a --- b
    b --- c
    c --- d
    d --- e
    end

    style a fill:#ccc,stroke:#222,stroke-width:2px;
    style b fill:#ccc,stroke:#222,stroke-width:2px;
    style c fill:#ccc,stroke:#222,stroke-width:2px;
    style d fill:#ccc,stroke:#222,stroke-width:2px;
    style e fill:#ccc,stroke:#222,stroke-width:2px;

  ```

-----
