"""Continuous disk recording of received serial bytes, frames and events."""
import csv
import json
import os
import re
import threading
import time
import uuid
import zipfile
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

ID_PATTERN = re.compile(r"\d{8}-\d{6}-[a-f0-9]{8}")

class Recorder:
    def __init__(self, root):
        self.root=Path(root)
        self.lock=threading.RLock()
        self.active=None
        self.last=None
        self.error=''
        self.finalizing=False

    def start(self, metadata, channels, note=''):
        with self.lock:
            if self.active or self.finalizing:
                raise ValueError('已经在录制或正在保存')
            if not isinstance(note,str) or len(note)>200:
                raise ValueError('录制说明最多 200 个字符')
            self.root.mkdir(parents=True,exist_ok=True)
            ident=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]
            folder=self.root/ident
            folder.mkdir()
            handles={}
            try:
                handles['rx']=open(folder/'serial-rx.bin','wb')
                handles['tx']=open(folder/'serial-tx.bin','wb')
                handles['chunks']=open(folder/'serial-chunks.jsonl','w',encoding='utf-8')
                handles['events']=open(folder/'events.jsonl','w',encoding='utf-8')
                handles['csv']=open(folder/'frames.csv','w',encoding='utf-8-sig',newline='')
                writer=csv.writer(handles['csv'])
                writer.writerow(['frame_id','host_unix_time_s','recording_elapsed_s']+
                                [f'raw_CH{i}' for i in range(len(channels))]+
                                [f'CH{i} {c[0]} {c[1]}' for i,c in enumerate(channels)])
                start_ns=time.monotonic_ns()
                manifest=dict(format='linfol-recording-v1',id=ident,note=note,status='recording',
                    start_host_unix_time_s=time.time(),start_local_time=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
                    timebase='Computer receive timestamps; elapsed times use a monotonic clock.',
                    capture_scope='All serial RX bytes processed during recording, decoded 29-column frames, TX bytes and host events. Does not contain untransmitted MCU internals or raw sensor ADC.',
                    boundary_note='A first decoded frame can span the start boundary. Raw RX starts at the recording boundary; serial gaps are logged rather than filled.',
                    channels=[dict(id=i,name=c[0],unit=c[1],scale=c[2]) for i,c in enumerate(channels)],
                    start_snapshot=metadata)
                (folder/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
                self.active=dict(id=ident,folder=folder,handles=handles,writer=writer,
                    manifest=manifest,start_ns=start_ns,frames=0,rx_bytes=0,tx_bytes=0,events=0,last_flush_ns=start_ns)
                self.error=''
                self._event_unlocked('recording_started',{'note':note})
                return self.status()
            except Exception:
                self.active=None
                for handle in handles.values(): handle.close()
                raise

    def _elapsed(self, active):
        return (time.monotonic_ns()-active['start_ns'])/1e9

    def _flush(self, active):
        now=time.monotonic_ns()
        if now-active['last_flush_ns']>=1_000_000_000:
            for handle in active['handles'].values(): handle.flush()
            active['last_flush_ns']=now

    def _fail(self, exception):
        active=self.active
        self.active=None
        self.error='录制文件写入失败：'+str(exception)
        if active:
            for handle in active['handles'].values():
                try: handle.close()
                except OSError: pass
            active['manifest'].update(status='error',error=self.error,
                frames=active['frames'],rx_bytes=active['rx_bytes'],tx_bytes=active['tx_bytes'])
            try:
                (active['folder']/'manifest.json').write_text(json.dumps(active['manifest'],ensure_ascii=False,indent=2)+'\n')
            except OSError: pass
            self.last=dict(id=active['id'],frames=active['frames'],rx_bytes=active['rx_bytes'],
                           duration_s=self._elapsed(active),folder=str(active['folder']),download_url=None,status='error')

    def bytes(self, direction, data):
        if not data:return
        with self.lock:
            active=self.active
            if not active:return
            try:
                key='rx_bytes' if direction=='rx' else 'tx_bytes'
                offset=active[key]
                active['handles'][direction].write(data)
                entry=dict(type='serial_chunk',direction=direction,time=time.time(),elapsed_s=self._elapsed(active),offset=offset,length=len(data))
                active['handles']['chunks'].write(json.dumps(entry,ensure_ascii=False)+'\n')
                active[key]+=len(data)
                self._flush(active)
            except OSError as e:self._fail(e)

    def frame(self, frame):
        with self.lock:
            active=self.active
            if not active:return
            try:
                active['writer'].writerow([frame['id'],frame['time'],self._elapsed(active)]+frame['raw']+frame['values'])
                active['frames']+=1
                self._flush(active)
            except OSError as e:self._fail(e)

    def _event_unlocked(self, kind, data):
        active=self.active
        if not active:return
        entry=dict(type=kind,time=time.time(),elapsed_s=self._elapsed(active),data=data)
        active['handles']['events'].write(json.dumps(entry,ensure_ascii=False)+'\n')
        active['events']+=1
        self._flush(active)

    def event(self, kind, data):
        with self.lock:
            if not self.active:return
            try:self._event_unlocked(kind,data)
            except OSError as e:self._fail(e)

    def status(self):
        with self.lock:
            active=self.active
            return dict(active=bool(active),finalizing=self.finalizing,error=self.error,
                        session=dict(id=active['id'],frames=active['frames'],rx_bytes=active['rx_bytes'],tx_bytes=active['tx_bytes'],
                                     duration_s=self._elapsed(active),folder=str(active['folder'])) if active else None,
                        last=dict(self.last) if self.last else None)

    def stop(self, end_snapshot):
        with self.lock:
            active=self.active
            if not active:
                if self.last:return dict(self.last)
                raise ValueError('当前没有正在录制的会话')
            try:
                self._event_unlocked('recording_stopped',{})
                for handle in active['handles'].values():
                    handle.flush()
                    os.fsync(handle.fileno())
                    handle.close()
            except OSError as e:
                self._fail(e)
                raise ValueError(self.error) from e
            self.active=None
            self.finalizing=True
            duration=self._elapsed(active)
            manifest=active['manifest']
            manifest.update(status='complete',end_host_unix_time_s=time.time(),duration_s=duration,
                frames=active['frames'],rx_bytes=active['rx_bytes'],tx_bytes=active['tx_bytes'],events=active['events'],end_snapshot=end_snapshot)
        # Compression is outside the recording lock so serial reading stays responsive.
        try:
            (active['folder']/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
            archive=self.root/(active['id']+'.zip')
            temporary=self.root/(active['id']+'.zip.part')
            with zipfile.ZipFile(temporary,'w',compression=zipfile.ZIP_DEFLATED) as z:
                for file in sorted(active['folder'].iterdir()):z.write(file,arcname=file.name)
            temporary.replace(archive)
            result=dict(id=active['id'],frames=active['frames'],rx_bytes=active['rx_bytes'],duration_s=duration,
                        folder=str(active['folder']),archive=str(archive),archive_bytes=archive.stat().st_size,
                        download_url='/api/recordings/'+active['id']+'/download',status='complete')
            with self.lock:self.last=result
            return result
        except OSError as e:
            with self.lock:
                self.error='录制已落盘，但压缩保存失败：'+str(e)
                self.last=dict(id=active['id'],frames=active['frames'],rx_bytes=active['rx_bytes'],duration_s=duration,
                    folder=str(active['folder']),download_url=None,status='error')
            raise ValueError(self.error) from e
        finally:
            with self.lock:self.finalizing=False

    def archive(self, ident):
        if not ID_PATTERN.fullmatch(ident):raise ValueError('无效的录制编号')
        path=self.root/(ident+'.zip')
        if not path.is_file():raise ValueError('录制文件不存在或尚未保存完成')
        return path
