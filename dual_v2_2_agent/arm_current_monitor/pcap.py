"""Passive IPv4/UDP PCAP decoder, adapted from hand_workspace_ar.state_relay."""
import struct
def packets(stream,destination_port=10165):
    def exact(n):
        data=bytearray()
        while len(data)<n:
            chunk=stream.read(n-len(data))
            if not chunk:
                if not data:return None
                raise ValueError('Truncated PCAP')
            data.extend(chunk)
        return bytes(data)
    header=exact(24)
    if header is None:return
    formats={b'\xd4\xc3\xb2\xa1':('<',1e6),b'\xa1\xb2\xc3\xd4':('>',1e6),b'\x4d\x3c\xb2\xa1':('<',1e9),b'\xa1\xb2\x3c\x4d':('>',1e9)}
    endian,scale=formats[header[:4]]
    link=struct.unpack(endian+'I',header[20:24])[0]
    if link not in (0,1,113,276):raise ValueError(f'Unsupported PCAP link type {link}')
    while True:
        h=exact(16)
        if h is None:return
        sec,sub,n,original=struct.unpack(endian+'IIII',h)
        if n>262144:raise ValueError('Oversized PCAP packet')
        p=exact(n)
        if p is None:raise ValueError('Truncated PCAP packet')
        offset={0:4,1:14,113:16,276:20}[link]
        if link==1 and p[12:14]!=b'\x08\x00':continue
        p=p[offset:]
        if len(p)<28 or p[0]>>4!=4 or p[9]!=17:continue
        ihl=(p[0]&15)*4
        total=struct.unpack('!H',p[2:4])[0]
        if ihl<20 or total>len(p) or total<ihl+8:continue
        if struct.unpack('!H',p[6:8])[0]&0x3fff:continue
        if p[12:16]!=b'\x7f\x00\x00\x01' or p[16:20]!=b'\x7f\x00\x00\x01':continue
        src,dst,length,_=struct.unpack('!HHHH',p[ihl:ihl+8])
        if dst!=destination_port or length<8 or ihl+length>total:continue
        yield sec+sub/scale,p[ihl+8:ihl+length]
