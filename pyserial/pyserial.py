import serial
import sys
from time import time

conn = serial.Serial(sys.argv[1],115200,timeout=0.01)

filename = sys.argv[2]
ec = 0
fc = 0

t = time()

with open(filename) as f:
  while True:
    chunk = f.read(256)

    if not chunk:
        break
    for c in chunk:
        if not c:
            print "End of file"
            break
        conn.write(c)
        r = conn.read(1)
        fc += 1
        if c != r:
            print('Error')
            ec += 1

tt = time()-t;

print('Sent %d bytes' % fc)
print('Errored in %d bytes' % ec)
print('Time taken %f s' % tt)
print('Speed %f bps' % (float(fc)*8/tt))
