import os
import pty
import select
import threading
import time
import unittest
import serial
from protocol import Decoder, parse_line, validate_parameter
from app import Vehicle

class ProtocolTests(unittest.TestCase):
    def test_actual_captured_29_columns_and_scaling(self):
        line='0,0,46,-46,-980,0,0,0,0,980,-980,0,250,0,0,'+','.join(['0']*14)
        decoder=Decoder()
        self.assertEqual(decoder.feed(line[:19].encode()),[])
        result=decoder.feed((line[19:]+'\r\n[OK] KP_L = 25.000\r\n').encode())
        self.assertEqual(result[0]['type'],'frame')
        self.assertEqual(len(result[0]['values']),29)
        self.assertAlmostEqual(result[0]['values'][2],.46)
        self.assertEqual(result[0]['values'][12],25)
        self.assertEqual(result[1],dict(type='ack',name='KP_L',value=25.0))

    def test_corrupt_and_partial_frames_never_become_waveforms(self):
        for line in [','.join(['0']*28),','.join(['NaN']+['0']*28),'9'*5000,','.join(['999999999999']+['0']*28)]:
            self.assertNotEqual(parse_line(line)['type'],'frame')
        self.assertEqual(parse_line('KP_L = 2.5e1  // speed KP')['value'],25)

    def test_commands_are_validated_before_serial_write(self):
        for name,value in [('run',1),('KP_L','nan'),('KI_L','1\nrun=1'),('base_speed',1.5),('pwm_limit',-1)]:
            with self.assertRaises(ValueError):
                validate_parameter(name,value)
        self.assertEqual(validate_parameter('kp_l',25)[2],'KP_L=25\n')

    def test_sensor_weights_have_independent_wire_names_and_bounds(self):
        for i in range(1,15):
            self.assertEqual(validate_parameter(f'weight{i}',.15)[2],f'weight{i}=0.15\n')
        for name,value in [('weight0',1),('weight15',1),('weight1',-1),('weight14',101),('weight7','nan')]:
            with self.assertRaises(ValueError): validate_parameter(name,value)
        self.assertEqual(parse_line('[OK] weight14 = 6.500')['name'],'weight14')
        self.assertEqual(parse_line('0,weight7 = 0.100 // sensor 7 weight')['name'],'weight7')
        self.assertEqual(parse_line('4angle20 = 0 // slot20 angle')['name'],'angle20')

    def test_sending_is_not_confirmation_and_timeouts_are_honest(self):
        class FakePort:
            is_open=True
            def write(self,data): return len(data)
        vehicle=Vehicle()
        vehicle.port=FakePort()
        vehicle.submit([dict(name='KP_L',value=25)])
        self.assertEqual(vehicle.commands[-1]['status'],'waiting')
        vehicle.pending['deadline']=0
        vehicle.timeout_check()
        self.assertEqual(vehicle.commands[-1]['status'],'unconfirmed')

    def test_real_serial_stream_with_pseudo_terminal(self):
        master,slave=pty.openpty()
        port=serial.Serial(os.ttyname(slave),115200,timeout=.05)
        vehicle=Vehicle()
        vehicle.port=port
        reader=threading.Thread(target=vehicle.read,args=(port,vehicle.generation),daemon=True)
        reader.start()
        vehicle.submit([dict(name='KP_L',value=25)])
        self.assertEqual(os.read(master,100),b'KP_L=25\n')
        self.assertEqual(vehicle.commands[-1]['status'],'waiting')
        frame=(','.join(['0']*29)+'\n').encode()
        os.write(master,frame[:13])
        os.write(master,frame[13:]+b'[OK] KP_L = 25.000\r\n')
        deadline=time.monotonic()+2
        while time.monotonic()<deadline and vehicle.commands[-1]['status']!='confirmed':
            time.sleep(.01)
        self.assertEqual(vehicle.commands[-1]['status'],'confirmed')
        self.assertEqual(vehicle.frame_id,1)
        self.assertEqual(vehicle.params['KP_L']['value'],25)
        vehicle.disconnect()
        reader.join(1)
        os.close(master)
        os.close(slave)

    def test_14_weight_batch_requires_each_serial_ack(self):
        master,slave=pty.openpty()
        port=serial.Serial(os.ttyname(slave),115200,timeout=.05)
        vehicle=Vehicle()
        vehicle.port=port
        reader=threading.Thread(target=vehicle.read,args=(port,vehicle.generation),daemon=True)
        reader.start()
        received=[]
        def car():
            buffer=bytearray()
            deadline=time.monotonic()+3
            while len(received)<14 and time.monotonic()<deadline:
                if not select.select([master],[],[],.1)[0]: continue
                buffer.extend(os.read(master,256))
                while b'\n' in buffer:
                    line,_,rest=buffer.partition(b'\n')
                    buffer=bytearray(rest)
                    name,value=line.decode().split('=')
                    received.append(name)
                    os.write(master,f'[OK] {name} = {float(value):.3f}\r\n'.encode())
        simulator=threading.Thread(target=car,daemon=True)
        simulator.start()
        vehicle.submit([dict(name=f'weight{i}',value=.1*i) for i in range(1,15)])
        simulator.join(4)
        deadline=time.monotonic()+1
        while time.monotonic()<deadline and vehicle.commands[-1]['status']!='confirmed': time.sleep(.01)
        self.assertEqual(received,[f'weight{i}' for i in range(1,15)])
        self.assertTrue(all(c['status']=='confirmed' for c in vehicle.commands))
        self.assertEqual(len(vehicle.params),14)
        vehicle.disconnect()
        reader.join(1)
        os.close(master)
        os.close(slave)

if __name__=='__main__':
    unittest.main()
