import io
import struct
import unittest
from arm_current_monitor.__main__ import decode_rt, Store
from arm_current_monitor.pcap import packets


def payload(t=100000000):
    a=['0',str(t),'1','100','2']
    for arm in range(2):
        a+=['7']
        for field in range(8):
            a += [str(1 if field==0 else (-.2*(j+1) if field==7 else 0)) for j in range(7)]
        a+=['0']*(49+14+7)
    a+=['2','6']+['0']*6+['1','0']
    return ('#'.join(a)+'#').encode()+b'\0'

class TelemetryTests(unittest.TestCase):
    def test_current_is_motor_tor_not_unused_motor_cur(self):
        r=decode_rt(payload(),100.)
        self.assertAlmostEqual(r['arms'][1]['current'][6],-1.4)
        self.assertEqual(r['arms'][0]['q'],[0]*7)
    def test_invalid_and_nonfinite(self):
        for p in [b'x',payload().replace(b'-0.2',b'nan'),payload().replace(b'2#7#',b'2#6#',1),payload().replace(b'-0.2',b'\0-0.2')]:
            with self.assertRaises((ValueError,IndexError)): decode_rt(p,100)
    def test_terminal_padding(self):
        self.assertEqual(decode_rt(payload()+b'\0'*20,100)['rt_us'],100000000)
    def test_reject_queued_and_duplicate_packets(self):
        s=Store();r=decode_rt(payload(),100)
        self.assertFalse(s.add(r,101));self.assertTrue(s.add(r,100.01))
        self.assertFalse(s.add(r,100.02));self.assertEqual(s.seq,1)
    def test_stale_rt_with_fresh_pcap_is_rejected(self):
        self.assertFalse(Store().add(decode_rt(payload(),110),110))
    def test_stale_stays_recorded_not_zero(self):
        s=Store();s.capture_alive=True;s.add(decode_rt(payload(),100),100);s.last_mono-=2
        d=s.snapshot();self.assertFalse(d['live']);self.assertEqual(d['hz'],0)
        self.assertAlmostEqual(d['rows'][0]['arms'][1]['current'][0],-.2)
    def test_ring_bound_and_incremental(self):
        s=Store(capacity=2)
        for i in range(4):s.add(decode_rt(payload(100000000+i*10000),100+i*.01),100+i*.01)
        self.assertEqual(len(s.rows),2);self.assertEqual(len(s.snapshot(3)['rows']),1)
    def test_pcap_destination_filter(self):
        p=payload();udp=struct.pack('!HHHH',45678,21101,len(p)+8,0)+p
        ip=bytearray(20);ip[0]=0x45;ip[9]=17;ip[2:4]=struct.pack('!H',20+len(udp));ip[12:20]=b'\x7f\0\0\1'*2
        packet=b'\0'*12+b'\x08\x00'+ip+udp
        data=struct.pack('<IHHIIII',0xa1b2c3d4,2,4,0,0,262144,1)+struct.pack('<IIII',100,0,len(packet),len(packet))+packet
        self.assertEqual(len(list(packets(io.BytesIO(data),21101))),1)
        self.assertEqual(len(list(packets(io.BytesIO(data),30241))),0)

if __name__=='__main__':unittest.main()
