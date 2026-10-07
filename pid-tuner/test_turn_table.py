import os
import pty
import select
import threading
import time
import unittest
import serial
from app import Vehicle
from protocol import PARAMS, TURN_TABLE_INFO, parse_line, validate_parameter

class TurnTableTests(unittest.TestCase):
    def test_existing_firmware_names_signed_angles_and_bounds(self):
        table=[p for p in PARAMS if p.get('turn_table')]
        self.assertEqual(len(table),61)
        self.assertFalse(TURN_TABLE_INFO['boot_enabled'])
        self.assertEqual(TURN_TABLE_INFO['lookup_slots'],5)
        self.assertEqual(validate_parameter('ANGLE1',-75)[2],'angle1=-75\n')
        for name,value in [('angle20',-25.5),('turn20',0),('exit20',0),('entry_ms',10)]:
            canonical,number,_=validate_parameter(name,value)
            self.assertEqual(parse_line(f'[OK] {canonical} = {number}')['value'],number)
        for name,value in [('turn1',-1),('turn1',1.5),('angle1',181),('angle1',-181),
                           ('angle1','nan'),('exit1',-1),('exit1',60001),('entry_ms',0),('angle21',0)]:
            with self.assertRaises(ValueError):validate_parameter(name,value)

    def test_full_61_value_table_over_serial_requires_every_ack(self):
        master,slave=pty.openpty()
        port=serial.Serial(os.ttyname(slave),115200,timeout=.05)
        vehicle=Vehicle();vehicle.port=port
        reader=threading.Thread(target=vehicle.read,args=(port,vehicle.generation),daemon=True)
        reader.start()
        items=[dict(name=p['name'],value=p['default']) for p in PARAMS if p.get('turn_table')]
        received=[]
        def car():
            buffer=bytearray();deadline=time.monotonic()+5
            while len(received)<61 and time.monotonic()<deadline:
                if not select.select([master],[],[],.1)[0]:continue
                buffer.extend(os.read(master,1024))
                while b'\n' in buffer:
                    line,_,rest=buffer.partition(b'\n');buffer=bytearray(rest)
                    name,value=line.decode().split('=');received.append((name,float(value)))
                    os.write(master,f'[OK] {name} = {float(value):.3f}\r\n'.encode())
        simulator=threading.Thread(target=car,daemon=True);simulator.start()
        try:
            vehicle.submit(items);simulator.join(6)
            deadline=time.monotonic()+1
            while vehicle.pending and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual([n for n,v in received],[p['name'] for p in items])
            self.assertTrue(all(c['status']=='confirmed' for c in vehicle.commands))
            self.assertEqual(vehicle.params['angle1']['value'],-75)
            self.assertEqual(len(vehicle.params),61)
        finally:
            vehicle.disconnect();reader.join(1);os.close(master);os.close(slave)

    def test_partial_table_rejection_cancels_remaining_parameters(self):
        class FakePort:
            is_open=True
            def write(self,data):return len(data)
        vehicle=Vehicle();vehicle.port=FakePort()
        vehicle.submit([dict(name='turn1',value=7),dict(name='angle1',value=-60),dict(name='exit1',value=100)])
        vehicle.receive(parse_line('[OK] turn1 = 7'))
        vehicle.receive(parse_line('[ERR] invalid angle'))
        self.assertEqual([c['status'] for c in vehicle.commands],['confirmed','rejected','cancelled'])
        self.assertNotIn('angle1',vehicle.params)
        self.assertNotIn('exit1',vehicle.params)

    def test_query_response_blocks_writes_until_complete(self):
        vehicle=Vehicle()
        vehicle.query=dict(status='receiving',time=time.monotonic(),last=time.monotonic(),count=1)
        with self.assertRaises(ValueError):vehicle.submit([dict(name='angle1',value=-60)])
        self.assertFalse(vehicle.commands)

class TurnSwitchTests(unittest.TestCase):
    def connected_vehicle(self):
        class FakePort:
            is_open=True
            def __init__(self):self.writes=[]
            def write(self,data):self.writes.append(data);return len(data)
        vehicle=Vehicle();vehicle.port=FakePort()
        return vehicle

    def test_switch_boolean_wire_protocol_and_validation(self):
        for value in (0,1):
            self.assertEqual(validate_parameter('TURN_ENABLE',value)[2],f'turn_enable={value}\n')
            self.assertEqual(parse_line(f'0,turn_enable = {value} // on off')['value'],value)
        for value in (-1,2,.5,'nan','1\nrun=1'):
            with self.assertRaises(ValueError):validate_parameter('turn_enable',value)

    def test_legacy_firmware_cannot_enable_switch(self):
        vehicle=self.connected_vehicle()
        with self.assertRaises(ValueError):vehicle.set_turn_table_enabled(1)
        self.assertFalse(vehicle.port.writes)

    def test_enabled_only_after_reply_and_disabling_preserves_table(self):
        vehicle=self.connected_vehicle()
        for p in PARAMS:
            if p.get('turn_table'):vehicle.receive(dict(type='param',name=p['name'],value=p['default']))
        vehicle.receive(dict(type='param',name='turn_enable',value=0))
        vehicle.receive(dict(type='frame',raw=[0]*29,values=[0]*29))
        table={k:v['value'] for k,v in vehicle.params.items() if k!='turn_enable'}
        vehicle.set_turn_table_enabled(1)
        self.assertEqual(vehicle.params['turn_enable']['value'],0)
        self.assertEqual(vehicle.pending['status'],'waiting')
        vehicle.receive(parse_line('[OK] turn_enable = 1'))
        self.assertEqual(vehicle.params['turn_enable']['value'],1)
        # Switching off is allowed during running; it never zeroes table slots.
        frame=[0]*29;frame[11]=1
        vehicle.receive(dict(type='frame',raw=frame,values=frame))
        vehicle.set_turn_table_enabled(0)
        vehicle.receive(parse_line('[OK] turn_enable = 0'))
        self.assertEqual(vehicle.port.writes,[b'turn_enable=1\n',b'turn_enable=0\n'])
        self.assertEqual(table,{k:v['value'] for k,v in vehicle.params.items() if k!='turn_enable'})

    def test_enable_requires_recent_stopped_frame_and_read_table(self):
        for scenario in ('no_frame','running','stale','nonzero_pwm','missing_table'):
            vehicle=self.connected_vehicle()
            vehicle.receive(dict(type='param',name='turn_enable',value=0))
            if scenario!='missing_table':
                for p in PARAMS:
                    if p.get('turn_table'):vehicle.receive(dict(type='param',name=p['name'],value=p['default']))
            if scenario!='no_frame':
                frame=[0]*29
                if scenario=='running':frame[11]=1
                if scenario=='nonzero_pwm':frame[9]=100
                vehicle.receive(dict(type='frame',raw=frame,values=frame))
                if scenario=='stale':vehicle.frames[-1]['time']-=3
            with self.subTest(scenario=scenario):
                with self.assertRaises(ValueError):vehicle.set_turn_table_enabled(1)
                self.assertFalse(vehicle.port.writes)

if __name__=='__main__':unittest.main()
