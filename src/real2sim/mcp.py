"""Blender MCP transport. Target Blender must see scene/config/output paths."""
import socket,json

def execute(code,host='127.0.0.1',port=9876,timeout=120):
 payload=json.dumps({'type':'execute_code','params':{'code':code}}).encode()
 with socket.create_connection((host,port),timeout=timeout) as s:
  s.sendall(payload);buf=b''
  while True:
   block=s.recv(65536)
   if not block:raise RuntimeError('Blender MCP closed before complete JSON')
   buf+=block
   if len(buf)>64*1024*1024:raise RuntimeError('MCP response too large')
   try:result=json.loads(buf)
   except (json.JSONDecodeError,UnicodeDecodeError):continue
   if result.get('status')!='success':raise RuntimeError(result)
   return result
