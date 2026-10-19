# Rojo-ecu-tool
Tuning and diagnostic tool for rojo E5 ECU'S. Uses arduino and can bus module

1. What you need

An Arduino Nano (5 V, ATmega328P) and an MCP2515 CAN module (the common blue board with an 8 MHz crystal), jumper wires, and access to the bike's ECU connector.
A USB cable that carries data (not a charge-only lead).
A computer running Google Chrome or Microsoft Edge - Windows, Linux or Mac. Firefox and Safari cannot use serial ports.
Tested on: ECU model SX-W125-M-210416, software number 402212071, CAN at 500 kbps. Other ROJO ECUs may behave differently.
What it does: live data, read and set idle speed, rev limit and run mode, read and clear fault codes, check the throttle, and (as an option you switch on) change the fuel factor. On ECU software below 400000000 it can also start and monitor the fast self-learning. What it does not do: fuel/ignition maps.

2. Wiring
MCP2515 module	Connects to
VCC	Nano 5V
GND	Nano GND
CS	Nano D10
SI (MOSI)	Nano D11
SO (MISO)	Nano D12
SCK	Nano D13
INT	not used
CANH	ECU connector: black/white wire
CANL	ECU connector: orange/black wire
GND (the module's other ground)	ECU connector: blue wire (signal ground)

The colours are those of the ROJO connector on the tested bike (from the ROJO manual). Check yours before connecting. Never connect the bike's red +12 V wires to the Nano or the module. If the ECU does not answer, swapping CANH and CANL is harmless and often the fix.
Termination (J1 jumper). A CAN bus needs 120 Ω at each end. With the ignition off and the module disconnected, measure between CANH and CANL at the ECU connector. About 120 Ω: the ECU has its own terminator, so close J1 on the module (this is what worked on the tested bike). About 60 Ω: leave J1 open.
Crystal. Read the silver crystal on the module. 8.000 needs nothing. If it says 16.000, change MCP_8MHZ to MCP_16MHZ near the top of the sketch.

3. Put the sketch on the Arduino

Plug the Nano in. If no new COM port appears in Device Manager (Ports), install the CH340 driver (most clone Nanos use that chip).
Linux (Ubuntu and similar)

4. Using the app

Live data: engine speed, throttle, battery, cylinder head temperature, O2 sensor, run mode and lock state, including two fuel numbers: Fuel factor (setting) is the value you can change (from the last read, it does not move with the throttle), and Fuel correction (?) is a live value the ECU sends that moves when the throttle moves; what exactly it is has not been confirmed. Labels with (?) are best guesses.

Read from ECU: shows what the ECU currently has. The first time an ECU is read, its idle, rev limit and mode are saved as your baseline.
Write changes…: change the boxes, press the button, check the list in the confirmation window, confirm. The app writes, re-reads, and tells you if the ECU reports the new values (verified).

Write-back test: writes the values the ECU already has. Nothing should change. Run it first to prove the write path works.

Throttle check: open and close the throttle fully a few times, then Stop. A good calibration reads about 0 % closed and about 100 % open.

Fuel factor (optional): tick Enable fuel factor changes, read the warning, and a fuel box appears. 1.000 means unchanged and higher adds fuel (up to 3.000)

Clear codes…: erases the stored fault codes and history in the ECU. A fault that is still present comes back, so it does not fix anything on its own, and the history can help a mechanic - use Log everything (diagnostics) first if you want a record.

Writing is switched off while the engine is running (rpm 50 or more), while the ECU reports itself locked, or with no live data.

Logs: Log live data records the live values as a CSV file - press it to start, press again to stop and save. Log everything (diagnostics) saves one text file with the ECU details, settings, fault codes, a fresh raw snapshot and every line exchanged with the Arduino. Send that file when asking for help.

5. Run modes

0 Comfort, 1 Sport, 2 Competition. Comfort is the initial setting from the dealer and is used for running in and fuel economy; on the author's bike it limited rpm in some or all gears. The maker does not document what Sport and Competition change (the author believes they alter the fuel tables and spark advance, but this has not been measured). Tests so far: the idle speed and rev limit values stay the same when the mode changes.

6. Fast self-learning

The ECU has a routine, called fast-learning in the original Android app, that lets the engine reach its best state in the shortest time. It is meant for the ECU's own recommended use, for example after the throttle has been recalibrated.

Only some ECUs. The original app treats ECU software numbers of 400000000 and above as "not implemented", and so does this tool: the Fast self-learning card shows whether yours is supported. The ECU the author built this on is 402212071, so the author could not test it. It is built from the original app's code.

Conditions (from the original app): bike stationary and unloaded - never on the road; engine fully warm (cylinder head temperature in the green zone); it may stop the engine; when it says done, switch the ignition off. It needs a valid O2 sensor reading (0 mV means invalid).

How it goes: press Start fast-learning…, confirm. Five circles show the learning points: grey waiting, red learning now, green learned. The status says "Not in learning point", "In learning point and doing learn" or "The learning of this point is done". The ECU decides when you are in a point, and the original app does not say which engine speed or throttle each point needs, so change the engine speed or throttle slowly and steadily until it says you are in a point, hold it until that point is learned, then look for the next. When all points are done the status says to close the power switch (ignition off).

The live-data CSV and the diagnostic log include the ECU mode (2 means learning), the learning point and the state bits, so you can review a session. Needs sketch v3.6 or newer.

7. Going back to your starting values

Use Restore baseline…. The first time the app sees an ECU it saves that ECU's idle, rev limit and mode as the baseline. Before you experiment, make sure the ECU holds the values you want to return to, then press Save current as baseline.

Some ECUs have a blue indicator lamp that tells you if idle air is too low (fast flashing, about 4 per second) or too high (slow flashing, about 1 per second) once warm; off means correct. If your ECU shows no lamp, use the engine speed in the app instead.

8. Safety and terms

Writing to an ECU can damage the engine or ECU and may make a vehicle illegal for road use. Engine off, battery charged, bike on a stand, small steps, write the old values down. Read the Terms & disclaimer (button at the bottom of the page): by using this tool you accept them.

