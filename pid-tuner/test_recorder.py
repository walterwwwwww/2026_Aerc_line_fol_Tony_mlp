import csv
import io
import json
import tempfile
import threading
import urllib.request
import unittest
import zipfile
from collections import deque
from pathlib import Path

from app import Vehicle
import app
from protocol import CHANNELS
from recorder import Recorder
from protocol import Decoder

class RecorderTests(unittest.TestCase):
    def test_http_start_stop_and_download_zip(self):
        class FakePort:
            is_open=True
        with tempfile.TemporaryDirectory() as tmp:
            old_vehicle,old_token=app.vehicle,app.TOKEN
            app.vehicle=Vehicle(recording_root=tmp)
            app.vehicle.port=FakePort()
            app.vehicle.device='TEST virtual serial'
            app.TOKEN='test-recording-token'
            server=app.ThreadingHTTPServer(('127.0.0.1',0),app.Handler)
            thread=threading.Thread(target=server.serve_forever,daemon=True)
            thread.start()
            base=f'http://127.0.0.1:{server.server_address[1]}'
            def post(path,body):
                req=urllib.request.Request(base+path,data=json.dumps(body).encode(),headers={'Content-Type':'application/json','X-PID-Token':app.TOKEN},method='POST')
                return json.load(urllib.request.urlopen(req))
            try:
                self.assertTrue(post('/api/recording/start',{'note':'HTTP test'})['active'])
                raw=(','.join(str(i) for i in range(29))+'\n').encode()
                app.vehicle.recorder.bytes('rx',raw)
                for item in Decoder().feed(raw):app.vehicle.receive(item)
                saved=post('/api/recording/stop',{})
                with urllib.request.urlopen(base+saved['download_url']) as response:
                    self.assertEqual(response.headers['Content-Type'],'application/zip')
                    downloaded=response.read()
                with zipfile.ZipFile(io.BytesIO(downloaded)) as archive:
                    self.assertEqual(archive.read('serial-rx.bin'),raw)
                    self.assertEqual(json.loads(archive.read('manifest.json'))['frames'],1)
            finally:
                server.shutdown();server.server_close();thread.join(1)
                app.vehicle,app.TOKEN=old_vehicle,old_token

    def test_full_disk_capture_is_not_limited_by_display_ring(self):
        with tempfile.TemporaryDirectory() as tmp:
            vehicle=Vehicle(recording_root=tmp)
            vehicle.frames=deque(maxlen=3)
            vehicle.recorder.start({'params':{'KP_L':{'value':70,'time':1}}},CHANNELS,'左弯测试')
            data=b'partial\xff\x00\r\n'
            vehicle.recorder.bytes('rx',data[:4])
            vehicle.recorder.bytes('rx',data[4:])
            for i in range(25):
                raw=[i]*29
                vehicle.receive(dict(type='frame',raw=raw,values=[v*c[2] for v,c in zip(raw,CHANNELS)]))
            vehicle.receive(dict(type='ack',name='weight7',value=.4))
            result=vehicle.stop_recording()
            self.assertEqual(len(vehicle.frames),3)
            self.assertEqual(result['frames'],25)
            with zipfile.ZipFile(result['archive']) as archive:
                self.assertEqual(archive.read('serial-rx.bin'),data)
                rows=list(csv.reader(archive.read('frames.csv').decode('utf-8-sig').splitlines()))
                self.assertEqual(len(rows),26)
                self.assertEqual(len(rows[0]),61)
                self.assertEqual(rows[-1][3:32],['24']*29)
                chunks=[json.loads(line) for line in archive.read('serial-chunks.jsonl').decode().splitlines()]
                self.assertEqual([chunk['offset'] for chunk in chunks],[0,4])
                self.assertEqual(sum(chunk['length'] for chunk in chunks),len(data))
                manifest=json.loads(archive.read('manifest.json'))
                self.assertEqual(manifest['status'],'complete')
                self.assertEqual(manifest['frames'],25)
                self.assertEqual(manifest['end_snapshot']['params']['weight7']['value'],.4)
                events=[json.loads(line) for line in archive.read('events.jsonl').decode().splitlines()]
                self.assertTrue(any(event['type']=='parsed_serial' and event['data']['type']=='ack' for event in events))

    def test_transmitted_command_and_confirmation_are_recorded(self):
        class FakePort:
            is_open=True
            def write(self,data):return len(data)
        with tempfile.TemporaryDirectory() as tmp:
            vehicle=Vehicle(recording_root=tmp)
            vehicle.port=FakePort()
            vehicle.start_recording('改参')
            vehicle.submit([dict(name='weight7',value=.4)])
            vehicle.receive(dict(type='ack',name='weight7',value=.4))
            result=vehicle.stop_recording()
            with zipfile.ZipFile(result['archive']) as archive:
                self.assertEqual(archive.read('serial-tx.bin'),b'weight7=0.4\n')
                events=[json.loads(line) for line in archive.read('events.jsonl').decode().splitlines()]
                statuses=[event['data']['status'] for event in events if event['type']=='command_status']
                self.assertEqual(statuses,['queued','waiting','confirmed'])

    def test_duplicate_start_and_archive_path_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder=Recorder(tmp)
            recorder.start({},CHANNELS)
            with self.assertRaises(ValueError):recorder.start({},CHANNELS)
            with self.assertRaises(ValueError):recorder.archive('../app.py')
            result=recorder.stop({})
            self.assertTrue(recorder.archive(result['id']).is_file())
            self.assertFalse(recorder.status()['active'])

    def test_disk_error_does_not_break_vehicle_telemetry(self):
        class BrokenWriter:
            def writerow(self,_):raise OSError('disk full')
        with tempfile.TemporaryDirectory() as tmp:
            vehicle=Vehicle(recording_root=tmp)
            vehicle.recorder.start({},CHANNELS)
            vehicle.recorder.active['writer']=BrokenWriter()
            vehicle.receive(dict(type='frame',raw=[0]*29,values=[0]*29))
            self.assertEqual(vehicle.frame_id,1)
            self.assertEqual(len(vehicle.frames),1)
            status=vehicle.recorder.status()
            self.assertFalse(status['active'])
            self.assertIn('disk full',status['error'])

if __name__=='__main__':unittest.main()
