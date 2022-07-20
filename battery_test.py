#!/usr/bin/env python3

import time, csv, datetime, json

import maynuo

class BatteryTest():
    port         = '/dev/ttyUSB0'
    speed        = 9600
    addr         = 1
    v_stop       = 2.975
    i_run        = 0.20
    o_fn         = "battlog_" + datetime.datetime.now().isoformat() + ".csv"
    log_interval = 5


    def __init__(self):
        self.m = maynuo.MaynuoLoad(self.port, self.speed, self.addr)
        self.m.inputOff()
        self.m.setReg('BATT', 0)
        self.ofh = open(self.o_fn, 'w')
        self.csv = csv.writer(self.ofh)

    def run(self):
        self.m.battTest(self.i_run, self.v_stop)
        self.m.inputOn()
        time.sleep(self.log_interval)
        op = self.m.getOperatingPoint()
        print(json.dumps(op, indent=2, default=str))
        self.csv.writerow(op.keys())
        self.csv.writerow([ str(x) for x in op.values()])
        while op['v'] > self.v_stop and op['i'] > 0:
            time.sleep(self.log_interval)
            op = self.m.getOperatingPoint()
            print(json.dumps(op, indent=2, default=str))
            self.csv.writerow([ str(x) for x in op.values()])
        self.ofh.close()


         
if __name__ == '__main__':
    b = BatteryTest()
    b.run()

