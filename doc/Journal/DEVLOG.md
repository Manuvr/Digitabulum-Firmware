# This is not a CHANGELOG
It is a verbose running commentary with no relationship to versioning.
File should be read top-to-bottom for chronological order.
The contents of this file will be periodically archived and purged to keep the log related to the "current events" of the code base.

---J. Ian Lindsay 2017.02.11

------


### 2017.04.29

##### I think I fried my first digit. This is a running log of my postmortem.

Hooked up all flex circuits, and powered on. Ran rollcall, unaware of a problem.

Strange... the LEDs didn't come up as they should have...

IMU rollcall during failure....

    ==< ManuMgmt   Thread 0 >=======
    -- Intertial integration units: id(I/M)
    --
    -- Dgt      Prx        Imt        Dst            Reports
    -- 0(MC)    00(68/3d)     <N/A>   01(68/3d)   
    -- 1        02(68/3d)  03(68/3d)  04(68/3d)   
    -- 2        05(68/3d)  06(ff/3d)  07(ff/ff)   
    -- 3        08(68/3d)  09(68/3d)  10(68/3d)   
    -- 4        11(68/3d)  12(68/3d)  13(68/3d)  Y
    -- 5        14(68/3d)  15(68/3d)  16(68/00)  Y

    Guard bytes (00/00)

Unsure why I have that hanging 0 on the distal of digit5. Suspect DMA stupidity. But this is not my concern. What's going on with digit-2?

Smelled "the smell". The one that indicates a failed smoke test. But no smoke yet visible. I noted the draw (~1000mA!) and shut off the power. Not enough latent heat to localize the problem. So I powered back up.

Rollcall after power cycle yields...

    ==< ManuMgmt   Thread 0 >=======
    -- Intertial integration units: id(I/M)
    --
    -- Dgt      Prx        Imt        Dst            Reports
    -- 0(MC)    00(68/3d)     <N/A>   01(68/3d)   
    -- 1        02(68/3d)  03(68/3d)  04(68/3d)   
    -- 2        05(68/3d)  06(00/00)  07(00/00)   
    -- 3        08(68/3d)  09(68/3d)  10(68/3d)   
    -- 4        11(68/3d)  12(68/3d)  13(68/3d)  Y
    -- 5        14(68/3d)  15(68/3d)  16(68/00)  Y

    Guard bytes (00/00)

Proximal position still registers in both cases.

Distal position of digit-2 is melting into the support foam. Cut the juice and disconnect digit-2.

Suspect cause is faulty assembly of that particular digit (2), and not the CPLD board.

Bust out the microscope. Find missing charge-pump capacitor at distal position.

**Hypothesis:** Missing cap caused excessive draw on part, and voltage sag prevented intermediate IMU from reaching operating voltage. If this is the case, the experience had to be stressful for the digit, since the I/O voltages would not have sagged, allowing full drive current from the high signals to flood into the part.

I will attempt repair of this digit later to see what happens. In the mean time, replacing the failed digit fixed the fault. This means that the CPLD did its job with respect to electrical isolation of a failed digit, as the remaining digits continued to operate.

    ==< ManuMgmt   Thread 0 >=======
    -- Intertial integration units: id(I/M)
    --
    -- Dgt      Prx        Imt        Dst        Reports
    -- 0(MC)    00(68/3d)     <N/A>   01(68/3d)   
    -- 1        02(68/3d)  03(68/3d)  04(68/3d)   
    -- 2        05(68/3d)  06(68/3d)  07(68/3d)   
    -- 3        08(68/3d)  09(68/3d)  10(68/3d)   
    -- 4        11(68/3d)  12(68/3d)  13(68/3d)  Y
    -- 5        14(68/3d)  15(68/3d)  16(68/00)  Y

    Guard bytes (00/00)

**Confirmed:** Digit-2 was at fault. The CPLD and connector appear unaffected.

------

ESP32 is at full-throttle, the IMUs are uninit'd, and the LEDs are on at ~4%. All but one digit is connected. Power draw is 100mA at the USB port.

Since regulation is linear, that means a bit less than 100mA @3v3. Not bad for not yet giving a damn about power efficiency.

At least now I know that the power planes in the flex circuits are capable of a minimum of 900mA. This being the delta between the failure and normal cases.

------

All LEDs running at 33%. Everything works. Thermal load negligible. Draw is ~120ma @5v.
I used UV so I could coat the digits with phosphorescent silicone. But on my unit, I'm going to keep the natural ultraviolet. :slightly_smiling_face:

Going to kick the LEDs up to 100%. They will become too bright to look at. No one will run the LEDs that hot. If they do, they should expect a dead battery. These things are __BRIGHT__.

This test will have to repeated with the magnetometers engaged to measure the induced error. But right now, I am concerned with power measurements and stress testing.

Ran for 5 minutes. 380mA @5v. ~2 deg C increase over ambient.
I am convinced of the trace integrity and thermal relief.

------

##### Notes on ESP32 DMA



---J. Ian Lindsay 2017.04.29

------
