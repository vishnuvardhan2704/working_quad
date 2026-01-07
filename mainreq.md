i have the tx_commands.py on another laptop whihc acts as the ground station from which ill send a command to start the mission
now main.py is the controller file on this system that will do preflight chefks, run the missions and also can also take kml files input and plan mission etc . to understand that basic structure read in the readme.md file in nidar folder.

You are an expert Linux and embedded systems engineer.

I am working on a headless Raspberry Pi autonomous system that communicates via LoRa. The Raspberry Pi must automatically start its main Python program on boot without requiring a display, keyboard, mouse, or SSH.

Your task is to explain and implement a systemd service (lora.service) that ensures this behavior reliably.

Requirements:

The Raspberry Pi runs Raspberry Pi OS Lite.

The main application entry point is:

main.py


The working directory must be the current one.

The service must:

Start automatically on boot

Restart automatically if the program crashes

Run as the pi user (not root)

Run after the system reaches multi-user mode

Forward stdout and stderr to journalctl logs

The service file must be created exactly at:

/etc/systemd/system/lora.service

What you should output:

The exact contents of the lora.service file using correct systemd syntax

The exact terminal commands required to:

Reload systemd

Enable the service at boot

Start the service immediately

The command to monitor logs in real time using journalctl

A short explanation of what happens after reboot (boot → service start → autonomous execution)

Constraints:

Do not assume any GUI

Do not rely on Wi-Fi or SSH after initial setup

Do not include unrelated explanations

Focus only on systemd reliability and correctness

The final result should allow the Raspberry Pi to power on and immediately run the LoRa-based autonomous program without user interaction.








