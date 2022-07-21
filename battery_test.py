#!/usr/bin/env python3

import time, csv, datetime, json, argparse, sys

import maynuo

def log(*args, **kwargs):
    print(*args, **kwargs)
    sys.stdout.flush()

class BatteryTest():

    def __init__(self):
        self.args = self.getArgs()
        self.m = maynuo.MaynuoLoad(self.args.port, self.args.speed, self.args.addr)
        self.m.inputOff()
        self.m.setReg('BATT', 0)
        self.ofh = open(self.args.filename, 'w')
        self.csv = csv.writer(self.ofh)

    def getArgs(self):
        parser = argparse.ArgumentParser(
            description='Maynuo Battery Tester',
            formatter_class= argparse.ArgumentDefaultsHelpFormatter
        )
        parser.add_argument(
            '--port', '-p',
            help='port to open',
            type=str,
            default='/dev/ttyUSB0'
        )
        parser.add_argument(
            '--speed', '-s',
            help='serial speed',
            type=int,
            default=9600
        )
        parser.add_argument(
            '--addr', '-a',
            help='instrument addares',
            type=int,
            default=1
        )
        parser.add_argument(
            '--volt-end', '-v',
            help='voltage at which to stop discharge test',
            type=float,
            default=3.00
        )
        parser.add_argument(
            '--current', '-c',
            help='constant current level for test',
            type=float,
            default=0.200
        )
        parser.add_argument(
            '--interval', '-i',
            help='how often to store to csv in seconds',
            type=float,
            default=30
        )
        parser.add_argument(
            '--filename', '-f',
            help='name of file to write',
            type=str,
            default= "battlog_" + datetime.datetime.now().isoformat() + ".csv"
        )

        return parser.parse_args()


    def run(self):
        self.m.battTest(self.args.current, self.args.volt_end)
        self.m.inputOn()
        time.sleep(self.args.interval)
        op = self.m.getOperatingPoint()
        log(json.dumps(op, indent=2, default=str))
        self.csv.writerow(op.keys())
        self.csv.writerow([ str(x) for x in op.values()])
        while op['v'] > self.args.volt_end and op['i'] > 0:
            time.sleep(self.args.interval)
            op = self.m.getOperatingPoint()
            log(json.dumps(op, indent=2, default=str))
            self.csv.writerow([ str(x) for x in op.values()])
            self.ofh.flush()
        self.ofh.close()


         
if __name__ == '__main__':
    b = BatteryTest()
    b.run()

