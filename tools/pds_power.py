"""Transient Windows power request scoped to a running PDS task.

No power plan or display settings are changed. Windows releases the request
when this thread exits; the context also explicitly releases it on completion.
"""
import contextlib,ctypes,os

@contextlib.contextmanager
def active_pds():
    kernel=None
    if os.name=='nt':
        kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        kernel.SetThreadExecutionState.argtypes=[ctypes.c_uint]
        kernel.SetThreadExecutionState.restype=ctypes.c_uint
        kernel.SetThreadExecutionState(0x80000001) # continuous + system, not display
    try:
        yield
    finally:
        if kernel is not None:kernel.SetThreadExecutionState(0x80000000)

if __name__=='__main__':
    import argparse,json
    from pathlib import Path
    p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    assert a.run.replace('_','').isalnum() and os.name=='nt'
    pid=json.loads((root/'build/pds'/a.run/'start.json').read_text())['pid']
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.OpenProcess.argtypes=[ctypes.c_uint,ctypes.c_int,ctypes.c_uint]
    kernel.OpenProcess.restype=ctypes.c_void_p
    kernel.WaitForSingleObject.argtypes=[ctypes.c_void_p,ctypes.c_uint]
    kernel.WaitForSingleObject.restype=ctypes.c_uint
    kernel.CloseHandle.argtypes=[ctypes.c_void_p]
    handle=kernel.OpenProcess(0x00100000,False,pid) # synchronization only
    if not handle:raise OSError(ctypes.get_last_error(),'Cannot observe PDS process')
    print('Transient system-awake request for PDS run',a.run,'PID',pid,flush=True)
    try:
        with active_pds():
            while True:
                state=kernel.WaitForSingleObject(handle,20000)
                if state==0:break
                if state!=258:raise OSError(ctypes.get_last_error(),'PDS wait failed')
    finally:kernel.CloseHandle(handle)
    print('PDS ended; transient power request released.',flush=True)
