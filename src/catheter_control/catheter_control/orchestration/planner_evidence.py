"""Opt-in bounded worker evidence; never commands hardware or blocks on disk.

Capture/serialization overhead is intrusive. Normal execution creates no writer.
Trusted-local pickle records must not be loaded from untrusted sources.
"""
import json
import os
from pathlib import Path
import pickle
import queue
import threading


class PlannerEvidenceWriter:
    def __init__(self, directory, metadata, *, capacity=4, max_records=2048,
                 max_bytes=512*1024*1024):
        if capacity<=0 or max_records<=0 or max_bytes<=0:
            raise ValueError('positive evidence bounds required')
        self.directory=Path(directory)/f'worker_{os.getpid()}'
        self.directory.mkdir(parents=True,exist_ok=False)
        self.queue=queue.Queue(maxsize=capacity)
        self.max_records=max_records;self.max_bytes=max_bytes
        self.accepted=0;self.written=0;self.dropped=0;self.bytes=0;self.errors=[]
        self.directory.joinpath('metadata.pkl').write_bytes(pickle.dumps(metadata,protocol=5))
        self.thread=threading.Thread(target=self._write,name='planner-evidence',daemon=True)
        self.thread.start()

    def submit(self, record):
        if self.accepted>=self.max_records:
            self.dropped+=1;return
        payload=pickle.dumps(record,protocol=5)
        if self.bytes+len(payload)>self.max_bytes:
            self.dropped+=1;return
        try:self.queue.put_nowait((record['sequence'],payload))
        except queue.Full:self.dropped+=1;return
        self.accepted+=1;self.bytes+=len(payload)

    def _write(self):
        while True:
            item=self.queue.get()
            try:
                if item is None:return
                sequence,payload=item
                self.directory.joinpath(f'plan_{sequence:06d}.pkl').write_bytes(payload)
                self.written+=1
            except Exception as error:
                if len(self.errors)<8:self.errors.append(f'{type(error).__name__}: {error}')
            finally:self.queue.task_done()

    def close(self):
        # Called only at worker shutdown, never in the planning callback.
        try:
            self.queue.put_nowait(None)
        except queue.Full:
            # A slow disk must not prevent worker shutdown.
            try:
                self.queue.get_nowait()
                self.queue.task_done()
                self.dropped += 1
            except queue.Empty:
                pass
            self.queue.put_nowait(None)
        self.thread.join(timeout=2.)
        self.directory.joinpath('summary.json').write_text(json.dumps(dict(
            accepted=self.accepted,written=self.written,dropped=self.dropped,
            bytes=self.bytes,errors=self.errors,writer_finished=not self.thread.is_alive()),indent=2)+'\n')
