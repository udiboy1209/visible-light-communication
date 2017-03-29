import usb.core
import usb.util

# find our device
dev = usb.core.find(idVendor=0x1cbe, idProduct=0x0003)

# was it found?
if dev is None:
    raise ValueError('Device not found')

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.set_configuration()

# get an endpoint instance
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]

ep = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_OUT)

epin = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_IN)



assert ep is not None
assert epin is not None

# write the data

import time, sys

filename = sys.argv[1]

with open(filename) as f:
    while True:
        t = time.time()
        c = f.read(256)
        if not c:
            print "End of file"
            break
        ep.write(c)
        print('Avg speed: %f kbps' % (0.256/(time.time()-t)))
        # ret = epin.read(256);
