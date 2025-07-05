#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 1871
#define ACK_TIMER 264
#define MAX_SEQ 127
#define NR_BUFS ((MAX_SEQ + 1) / 2)
#define INC(s) do { s = (s + 1) % (MAX_SEQ + 1); } while(0)

typedef enum { false, true } boolean;

struct FRAME {
    unsigned char kind; /* FRAME_DATA */
    unsigned char ack;
    unsigned char seq;
    unsigned char data[PKT_LEN];
    unsigned int  padding;
};

// as a sender
static unsigned char out_buf[NR_BUFS][PKT_LEN], nbuffered = 0;
static unsigned char ack_expected = 0;
static boolean phl_ready = false;

// as a receiver
static unsigned char in_buf[NR_BUFS][PKT_LEN];
static unsigned char arrived[NR_BUFS] = { 0 }, frame_expected = 0;
static boolean no_nak = true;

static boolean between(unsigned char a, unsigned char b, unsigned char c)
{
    return (((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a)));
}

static void put_frame(unsigned char* frame, int len)
{
    *(unsigned int*)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = false;
}

static void framing(unsigned char kind, unsigned char arg)
{
    struct FRAME s;

    s.kind = kind;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    if (kind == FRAME_DATA) {
        s.seq = arg;
        memcpy(s.data, out_buf[arg % NR_BUFS], PKT_LEN);
        dbg_frame("Send DATA %3d %3d, ID %d, Window %3d, PHL Queue %d\n",
            s.seq, s.ack, *(short*)s.data, (s.seq + MAX_SEQ + 1 - ack_expected) % NR_BUFS, phl_sq_len());
        put_frame((unsigned char*)&s, 3 + PKT_LEN);
        start_timer(arg, DATA_TIMER);
    }
    else {
        if (kind == FRAME_ACK) {
            dbg_frame("Send ACK  %3d\n", s.ack);
        }
        else { // NAK
            dbg_frame("Send NAK  %3d\n", s.ack);
            no_nak = false;
        }
        put_frame((unsigned char*)&s, 2);
    }

    stop_ack_timer();
}

int main(int argc, char** argv)
{
    int event, arg, nerror = 0, nresend = 0;
    struct FRAME f;
    int len = 0;

    protocol_init(argc, argv);
    lprintf("Designed by Chen Zirong, build: " __DATE__"  "__TIME__"\n");

    disable_network_layer();

    for (;;) {
        event = wait_for_event(&arg);

        switch (event) {
        case NETWORK_LAYER_READY:
            get_packet(out_buf[(ack_expected + nbuffered) % NR_BUFS]);
            framing(FRAME_DATA, (ack_expected + nbuffered) % (MAX_SEQ + 1));
            nbuffered++;
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = true;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char*)&f, sizeof f);
            if (len < 5 || crc32((unsigned char*)&f, len) != 0) {
                dbg_event("Receiver Error %d: Bad CRC Checksum\n", ++nerror);
                if (len > 6 && no_nak)
                    framing(FRAME_NAK, 0);
                break;
            }

            if (f.kind == FRAME_DATA) {
                dbg_frame("Recv DATA %3d %3d, ID %d, Window %3d %3d\n",
                    f.seq, f.ack, *(short*)f.data, (f.seq - frame_expected + MAX_SEQ + 1) % (MAX_SEQ + 1), (frame_expected + NR_BUFS) % (MAX_SEQ + 1));
                if (between(frame_expected, f.seq, (frame_expected + NR_BUFS) % (MAX_SEQ + 1)) && f.seq != frame_expected && no_nak)
                    framing(FRAME_NAK, 0);
                else
                    start_ack_timer(ACK_TIMER);
                if (between(frame_expected, f.seq, (frame_expected + NR_BUFS) % (MAX_SEQ + 1))
                    && (arrived[f.seq % NR_BUFS] == false)) {
                    arrived[f.seq % NR_BUFS] = 1;
                    memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN);
                    while (arrived[frame_expected % NR_BUFS]) {
                        put_packet(in_buf[frame_expected % NR_BUFS], len - 7);
                        no_nak = true;
                        arrived[frame_expected % NR_BUFS] = false;
                        INC(frame_expected);
                        start_ack_timer(ACK_TIMER);
                    }
                }
            }

            if (f.kind == FRAME_ACK) {
                dbg_frame("Recv ACK  %3d\n", f.ack);
            }

            if (f.kind == FRAME_NAK) {
                dbg_frame("Recv NAK  %3d\n", f.ack);
                if (between(ack_expected, (f.ack + 1) % (MAX_SEQ + 1), (ack_expected + nbuffered) % (MAX_SEQ + 1)))
                    framing(FRAME_DATA, (f.ack + 1) % (MAX_SEQ + 1));
            }

            while (between(ack_expected, f.ack, (ack_expected + nbuffered) % (MAX_SEQ + 1))) {
                nbuffered--;
                stop_timer(ack_expected);
                INC(ack_expected);
            }

            break;

        case DATA_TIMEOUT:
            dbg_event("Timeout DATA %3d, retransmit %d\n", arg, ++nresend);
            framing(FRAME_DATA, arg);
            if (arg != ack_expected)
                for (int i = (arg + 1) % (MAX_SEQ + 1); between(ack_expected, i, (ack_expected + nbuffered) % (MAX_SEQ + 1)); ) {
                    start_timer(i, DATA_TIMER);
                    INC(i);
                }
            break;

        case ACK_TIMEOUT:
            framing(FRAME_ACK, 0);
        }

        if (nbuffered < NR_BUFS && phl_ready)
            enable_network_layer();
        else
            disable_network_layer();
    }
}
