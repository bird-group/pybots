from common_packets.avia_packet import AVIaPacket


class TextPacket(AVIaPacket):

    def __init__(self,pkt=None):
        if pkt is None:
            super(TextPacket,self)
            self.pkt_type = 0xF0
        else:
            super(TextPacket,self)
            self.pktType(pkt.pktType())
            self.payload = pkt.payload

