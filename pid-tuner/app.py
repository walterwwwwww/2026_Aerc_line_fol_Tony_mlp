# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial==3.5", "numpy"]
# ///
"""Local-only PID tuner. No cloud, no automatic start or parameter writes."""
import argparse
import errno
import csv
import io
import json
import math
import threading
import time
import urllib.parse
import urllib.request
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial
from serial.tools import list_ports
from protocol import CHANNELS, PARAMS, TURN_TABLE_INFO, Decoder, validate_parameter
from recorder import Recorder
from ml_trainer import OnlineWeightTrainer

ROOT = Path(__file__).resolve().parent

def vehicle_ports():
    return [p for p in list_ports.comports()
            if 'debug-console' not in p.device and 'Bluetooth-Incoming-Port' not in p.device]

class Vehicle:
    def __init__(self, recording_root=None):
        self.lock = threading.RLock()
        self.write_lock = threading.Lock()
        self.port = None
        self.generation = 0
        self.device = ''
        self.baud = 115200
        self.error = ''
        self.frames = deque(maxlen=18000)
        self.logs = deque(maxlen=300)
        self.commands = deque(maxlen=100)
        self.params = {}
        self.query = None
        self.pending = None
        self.queue = deque()
        self.frame_id = self.log_id = self.command_id = 0
        self.rx_bytes = 0
        self.arrivals = deque(maxlen=200)
        self.recorder=Recorder(recording_root or ROOT/'recordings')
        self.ml_trainer=OnlineWeightTrainer()

    def log(self, text, kind='info'):
        with self.lock:
            self.log_id += 1
            self.logs.append(dict(id=self.log_id, time=time.time(), text=text, kind=kind))
            self.recorder.event('host_log',dict(text=text,kind=kind))

    def recording_snapshot(self):
        with self.lock:
            return dict(device=self.device,baud=self.baud,connected=bool(self.port and self.port.is_open),
                params={k:dict(v) for k,v in self.params.items()},frame_id=self.frame_id,rx_bytes=self.rx_bytes,
                parameter_note='Last values actually confirmed by the car, with their confirmation timestamps; unqueried values are absent.',
                latest_frame=dict(self.frames[-1]) if self.frames else None)

    def start_recording(self, note=''):
        with self.lock:
            if not self.port or not self.port.is_open:
                raise ValueError('请先连接串口，再开始录制')
            return self.recorder.start(self.recording_snapshot(),CHANNELS,note)

    def stop_recording(self):
        return self.recorder.stop(self.recording_snapshot())

    def record_command(self, command):
        self.recorder.event('command_status',{k:v for k,v in command.items() if k!='deadline'})

    def cancel_queue(self, reason):
        for command in self.queue:
            command.update(status='cancelled',message=reason)
            self.record_command(command)
        self.queue.clear()

    def connect(self, device, baud):
        available = {p.device for p in vehicle_ports()}
        if device not in available:
            raise ValueError('串口不在当前设备列表中，请刷新设备')
        if int(baud) not in (9600,19200,38400,57600,115200,230400,460800,921600):
            raise ValueError('不支持的波特率')
        self.disconnect()
        port = serial.Serial(port=None, baudrate=int(baud), timeout=.1,
                             write_timeout=1, exclusive=True)
        port.dtr = False
        port.rts = False
        port.port = device
        port.open()
        with self.lock:
            self.port = port
            self.device, self.baud, self.error = device, int(baud), ''
            self.params = {}
            self.query = None
            self.arrivals.clear()
            self.frames.clear()
            generation = self.generation
        self.log(f'已连接 {device} · {baud} · 8N1')
        threading.Thread(target=self.read, args=(port,generation), daemon=True).start()

    def disconnect(self):
        with self.lock:
            self.generation += 1
            port, self.port = self.port, None
            self.cancel_queue('串口连接已断开')
            if self.pending:
                self.pending.update(status='unconfirmed', message='连接已断开，未确认生效')
                self.record_command(self.pending)
                self.pending = None
            self.query = None
        if port:
            port.close()
            self.log('串口已断开')

    def send(self, text):
        with self.write_lock:
            with self.lock:
                port = self.port
            if not port or not port.is_open:
                raise ValueError('串口尚未连接')
            self.recorder.event('tx_attempt',dict(text=text,byte_count=len(text)))
            written = port.write(text.encode('ascii'))
            self.recorder.bytes('tx',text.encode('ascii')[:written])
            if written != len(text):
                raise ValueError('串口命令未完整发送')
            self.log('发送：' + text.strip(), 'tx')

    def query_params(self):
        with self.lock:
            if self.pending or self.queue:
                raise ValueError('请等待当前参数发送完成')
            self.query = dict(status='waiting', time=time.monotonic(), count=0)
        try:
            self.send('?\n')
        except Exception:
            with self.lock:
                self.query = None
            raise

    def submit(self, items):
        if not isinstance(items,list) or not 1 <= len(items) <= 64:
            raise ValueError('每次发送 1 到 64 个参数')
        checked = [validate_parameter(p.get('name'),p.get('value')) for p in items]
        with self.lock:
            if self.pending or self.queue or (self.query and self.query['status'] in ('waiting','receiving')):
                raise ValueError('车端仍在处理上一条请求，请稍后再发')
            if not self.port:
                raise ValueError('串口尚未连接')
            for name,value,text in checked:
                self.command_id += 1
                command=dict(id=self.command_id,name=name,value=value,text=text,status='queued',time=time.time())
                self.commands.append(command)
                self.queue.append(command)
                self.record_command(command)
        self.advance()

    def set_turn_table_enabled(self, enabled):
        name,value,_=validate_parameter('turn_enable',enabled)
        with self.lock:
            known=self.params.get(name)
            if not self.port or not self.port.is_open:
                raise ValueError('请先连接串口')
            if not known or known['value'] not in (0,1):
                raise ValueError('车端尚未提供 turn_enable；请读取参数，旧固件需要重烧')
            if value==1:
                latest=self.frames[-1] if self.frames else None
                if not latest or time.time()-latest['time']>2 or latest['values'][11]!=0 or latest['values'][9]!=0 or latest['values'][10]!=0:
                    raise ValueError('开启前请停车，并等待收到新的停车状态波形')
                names=['entry_ms']+[f'{prefix}{i}' for i in range(1,6) for prefix in ('turn','angle','exit')]
                if not all(n in self.params for n in names):
                    raise ValueError('请先读取完整转向表，再开启')
                for n in names:validate_parameter(n,self.params[n]['value'])
            self.submit([dict(name=name,value=value)])

    def stop(self):
        with self.lock:
            if not self.port or not self.port.is_open:
                raise ValueError('串口尚未连接')
            self.cancel_queue('被停车请求取消')
            if self.pending:
                self.pending.update(status='unconfirmed',message='停车请求打断了参数发送')
                self.pending = None
            self.command_id += 1
            command=dict(id=self.command_id,name='!',value=0,text='!\n',status='waiting',time=time.time(),deadline=time.monotonic()+5)
            self.commands.append(command)
            self.pending = command
            self.record_command(command)
        try:
            self.send('!\n')
        except Exception as e:
            with self.lock:
                command.update(status='unconfirmed',message=str(e))
                self.record_command(command)
                self.pending=None
            raise

    def advance(self):
        with self.lock:
            if self.pending or not self.queue:
                return
            command = self.queue.popleft()
            command.update(status='waiting',deadline=time.monotonic()+5)
            self.pending = command
            self.record_command(command)
        try:
            self.send(command['text'])
        except Exception as e:
            with self.lock:
                command.update(status='unconfirmed',message=str(e))
                self.record_command(command)
                self.pending=None
                self.cancel_queue('前一条命令发送失败')
            self.log(str(e),'error')

    def receive(self, item):
        now = time.time()
        if item['type']!='frame':
            self.recorder.event('parsed_serial',item)
        with self.lock:
            kind=item['type']
            if kind=='frame':
                self.frame_id+=1
                frame=dict(id=self.frame_id,time=now,raw=item['raw'],values=item['values'])
                self.frames.append(frame)
                self.recorder.frame(frame)
                self.arrivals.append(time.monotonic())
                # feed the online ML weight trainer with the 14 raw reflectances
                # (columns 28..41, scale .01 -> 0..10, low = dark). It only
                # accepts frames while a ground-truth label is active.
                if len(item['raw']) >= 42:
                    try:
                        refl=[item['raw'][28+i]*0.01 for i in range(14)]
                        self.ml_trainer.feed(refl)
                    except (TypeError, ValueError):
                        pass
            elif kind in ('param','ack'):
                name=item['name']
                self.params[name]=dict(value=item['value'],time=now)
                if kind=='param' and self.query:
                    self.query['count']+=1
                    self.query['status']='receiving'
                    self.query['last']=time.monotonic()
                command=self.pending
                if kind=='ack' and command and command['name'].lower()==name.lower():
                    if math.isclose(float(command['value']),item['value'],rel_tol=1e-5,abs_tol=.00051):
                        command.update(status='confirmed',message='车端已确认')
                    else:
                        command.update(status='unconfirmed',message='回复值与提交值不一致')
                    self.record_command(command)
                    self.pending=None
                    if command['status']!='confirmed':
                        self.cancel_queue('车端回复值与请求不一致')
                self.log(f"{'确认' if kind=='ack' else '读取'}：{name} = {item['value']:g}",'rx')
            else:
                text=item['text']
                if text.startswith('[STOP]') and self.pending and self.pending['name']=='!':
                    self.pending.update(status='confirmed',message='车端已回复停车；仍需检查 PWM 归零')
                    self.record_command(self.pending)
                    self.pending=None
                elif text.startswith('[ERR]') and self.pending:
                    self.pending.update(status='rejected',message=text)
                    self.record_command(self.pending)
                    self.pending=None
                    self.cancel_queue('车端拒绝了前一条命令')
                self.log(text,'error' if text.startswith('[ERR]') else 'rx')
        self.advance()

    def timeout_check(self):
        now=time.monotonic()
        with self.lock:
            if self.pending and now>self.pending['deadline']:
                self.pending.update(status='unconfirmed',message='5 秒内未收到车端确认，不能认为已生效')
                self.record_command(self.pending)
                self.log(self.pending['name']+'：未收到车端确认','error')
                self.pending=None
                self.cancel_queue('前一条命令未获车端确认')
            if self.query and self.query['status']=='waiting' and now-self.query['time']>5:
                self.query['status']='timeout'
                self.log('参数查询没有回复；波形接收成功不代表车端能处理命令','error')
            if self.query and self.query['status']=='receiving' and now-self.query.get('last',now)>1:
                self.query['status']='complete'

    def read(self, port, generation):
        decoder=Decoder()
        try:
            while port.is_open and generation==self.generation:
                data=port.read(min(port.in_waiting or 1,8192))
                self.recorder.bytes('rx',data)
                with self.lock:
                    self.rx_bytes+=len(data)
                for item in decoder.feed(data):
                    self.receive(item)
                self.timeout_check()
        except (serial.SerialException,OSError) as e:
            with self.lock:
                if generation!=self.generation:
                    return
                self.error=str(e)
            self.log('串口读取失败：'+str(e),'error')
            self.disconnect()

    def state(self, after=0, log_after=0):
        with self.lock:
            frames=[f for f in self.frames if f['id']>after][-200:]
            arrivals=list(self.arrivals)
            rate=(len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 and arrivals[-1]>arrivals[0] else 0
            latest=self.frames[-1] if self.frames else None
            return dict(connected=bool(self.port and self.port.is_open),device=self.device,baud=self.baud,error=self.error,
                        rx_bytes=self.rx_bytes,frame_count=self.frame_id,rate=rate,
                        last_frame_age=time.time()-latest['time'] if latest else None,
                        latest=latest,frames=frames,params=dict(self.params),query=dict(self.query) if self.query else None,
                        commands=[{k:v for k,v in c.items() if k not in ('text','deadline')} for c in self.commands],
                        logs=[l for l in self.logs if l['id']>log_after],log_id=self.log_id,
                        recording=self.recorder.status())

vehicle=Vehicle()
TOKEN=''

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass
    def reply(self, status, body, kind='application/json; charset=utf-8'):
        if not isinstance(body,bytes):
            body=json.dumps(body,ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type',kind)
        self.send_header('Content-Length',str(len(body)))
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.end_headers()
        self.wfile.write(body)
    def do_GET(self):
        parsed=urllib.parse.urlparse(self.path)
        path=parsed.path
        if path=='/api/meta':
            return self.reply(200,dict(channels=[dict(id=i,name=c[0],unit=c[1],scale=c[2]) for i,c in enumerate(CHANNELS)],params=PARAMS,turn_table=TURN_TABLE_INFO,token=TOKEN))
        if path=='/api/ports':
            return self.reply(200,[dict(device=p.device,description=p.description) for p in vehicle_ports()])
        if path=='/api/state':
            return self.reply(200,vehicle.state())
        if path.startswith('/api/recordings/') and path.endswith('/download'):
            ident=path[len('/api/recordings/'):-len('/download')]
            try:
                archive=vehicle.recorder.archive(ident)
            except ValueError as e:
                return self.reply(404,dict(error=str(e)))
            self.send_response(200)
            self.send_header('Content-Type','application/zip')
            self.send_header('Content-Length',str(archive.stat().st_size))
            self.send_header('Content-Disposition',f'attachment; filename="{archive.name}"')
            self.send_header('Cache-Control','no-store')
            self.end_headers()
            try:
                with archive.open('rb') as f:
                    while chunk:=f.read(65536): self.wfile.write(chunk)
            except (BrokenPipeError,ConnectionResetError): pass
            return
        if path=='/api/events':
            self.send_response(200)
            self.send_header('Content-Type','text/event-stream')
            self.send_header('Cache-Control','no-cache')
            self.end_headers()
            cursor=log_cursor=0
            try:
                while True:
                    state=vehicle.state(cursor,log_cursor)
                    self.wfile.write(('data: '+json.dumps(state,ensure_ascii=False)+'\n\n').encode())
                    self.wfile.flush()
                    if state['frames']:
                        cursor=state['frames'][-1]['id']
                    log_cursor=state['log_id']
                    time.sleep(.15)
            except (BrokenPipeError,ConnectionResetError):
                return
        if path=='/api/export.csv':
            out=io.StringIO()
            writer=csv.writer(out)
            writer.writerow(['电脑接收时间 Unix 秒']+[f'CH{i} {c[0]} {c[1]}' for i,c in enumerate(CHANNELS)])
            with vehicle.lock:
                rows=list(vehicle.frames)
            for frame in rows:
                writer.writerow([frame['time']]+frame['values'])
            return self.reply(200,('\ufeff'+out.getvalue()).encode(),'text/csv; charset=utf-8')
        if path=='/api/ml':
            return self.reply(200,vehicle.ml_trainer.status())
        files={'/':'index.html','/app.js':'app.js','/style.css':'style.css','/weights.css':'weights.css'}
        if path in files:
            mime={'/':'text/html','/app.js':'text/javascript','/style.css':'text/css','/weights.css':'text/css'}[path]
            return self.reply(200,(ROOT/'static'/files[path]).read_bytes(),mime+'; charset=utf-8')
        self.reply(404,dict(error='未找到页面'))
    def do_POST(self):
        try:
            # Require a same-page secret: another website cannot silently send car commands.
            if self.headers.get('X-PID-Token')!=TOKEN:
                return self.reply(403,dict(error='请求未获本地页面授权'))
            length=int(self.headers.get('Content-Length','0'))
            if not 0<=length<=16384:
                raise ValueError('请求过大')
            body=json.loads(self.rfile.read(length) or b'{}')
            if self.path=='/api/connect':
                vehicle.connect(body['device'],body.get('baud',115200))
            elif self.path=='/api/disconnect':
                vehicle.disconnect()
            elif self.path=='/api/query':
                vehicle.query_params()
            elif self.path=='/api/parameters':
                if any(str(p.get('name','')).lower()=='turn_enable' for p in body['parameters']):
                    raise ValueError('请使用独立转向表开关')
                vehicle.submit(body['parameters'])
            elif self.path=='/api/stop':
                vehicle.stop()
            elif self.path=='/api/turn-table/switch':
                vehicle.set_turn_table_enabled(body['enabled'])
            elif self.path=='/api/turn-table/disable':
                vehicle.submit([dict(name=f'turn{i}',value=0) for i in range(1,21)])
            elif self.path=='/api/recording/start':
                return self.reply(200,vehicle.start_recording(body.get('note','')))
            elif self.path=='/api/recording/stop':
                return self.reply(200,vehicle.stop_recording())
            elif self.path=='/api/ml/start':
                vehicle.ml_trainer.start()
            elif self.path=='/api/ml/stop':
                vehicle.ml_trainer.stop()
            elif self.path=='/api/ml/reset':
                vehicle.ml_trainer.reset()
            elif self.path=='/api/ml/label':
                offset = body.get('offset')
                shape = body.get('shape', '')
                if offset is None:
                    vehicle.ml_trainer.clear_label()
                else:
                    vehicle.ml_trainer.set_label(float(offset), str(shape))
            elif self.path=='/api/ml/apply':
                # send the current ML weights to the car as weight1..weight14
                weights = vehicle.ml_trainer.status()['weights']
                vehicle.submit([dict(name=f'weight{i+1}', value=float(w))
                                for i, w in enumerate(weights)])
                return self.reply(200, dict(ok=True, weights=weights))
            else:
                return self.reply(404,dict(error='未知操作'))
            self.reply(200,dict(ok=True))
        except (ValueError,KeyError,serial.SerialException,OSError,json.JSONDecodeError) as e:
            self.reply(400,dict(error=str(e)))

def main():
    global TOKEN
    import secrets
    TOKEN=secrets.token_urlsafe(24)
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--serial-port')
    parser.add_argument('--open',action='store_true')
    args=parser.parse_args()
    url=f'http://127.0.0.1:{args.port}'
    try:
        server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    except OSError as error:
        if error.errno==errno.EADDRINUSE and args.open:
            try:
                with urllib.request.urlopen(url+'/api/meta',timeout=2) as response:
                    running=json.load(response)
                if len(running.get('channels',[]))==29:
                    print('上位机已经运行：'+url,flush=True)
                    webbrowser.open(url)
                    return
            except (OSError,ValueError):
                pass
        raise
    if args.serial_port:
        try:
            vehicle.connect(args.serial_port,115200)
        except (ValueError,serial.SerialException,OSError) as error:
            vehicle.error=str(error)
            vehicle.log('自动连接未成功：'+str(error),'error')
    print('PID 上位机：'+url,flush=True)
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if vehicle.recorder.status()['active']:
            try:vehicle.stop_recording()
            except (OSError,ValueError):pass
        vehicle.disconnect()
        server.server_close()

if __name__=='__main__':
    main()
