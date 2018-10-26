import machine
import network

wlan = network.WLAN() # get current object, without changing the mode

if machine.reset_cause() != machine.SOFT_RESET:
    wlan.init(network.WLAN.STA)
    # configuration below MUST match your home router settings!!
    wlan.ifconfig(config=('192.168.2.151', '255.255.255.0', '192.168.2.1', '8.8.8.8'))

if not wlan.isconnected():
    # change the line below to match your network ssid, security and password
    wlan.connect('ziemodt', auth=(WLAN.WPA2, 'exc1t0n'), timeout=5000)
    while not wlan.isconnected():
        machine.idle() # save power while waiting